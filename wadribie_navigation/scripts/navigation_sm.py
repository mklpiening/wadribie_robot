#!/usr/bin/python

import rospy
import smach
import smach_ros
from mbf_msgs.msg import ExePathAction, ExePathResult
from mbf_msgs.msg import GetPathAction, GetPathResult
from mbf_msgs.msg import RecoveryAction, RecoveryResult
import threading
from geometry_msgs.msg import PoseStamped


class WaitForGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['received_goal', 'preempted'], input_keys=[], output_keys=['target_pose'])
        self.target_pose = PoseStamped()
        self.signal = threading.Event()

    def execute(self, user_data):
        self.signal.clear()

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        while not rospy.is_shutdown() and not self.signal.is_set() and not self.preempt_requested():
            self.signal.wait(1)

        user_data.target_pose = self.target_pose
        return 'received_goal'

    def goal_callback(self, pos):
        self.target_pose = pos
        self.signal.set()

class PlanToGoal(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['target_pose'],
            output_keys=['outcome', 'message', 'path', 'cost']
        )

        with self:
            self.userdata.recovery_behavior = None

            smach.StateMachine.add(
                'GET_PATH',
                smach_ros.SimpleActionState(
                    'move_base_flex/get_path',
                    GetPathAction,
                    goal_cb=PlanToGoal.planner_goal_cb,
                    result_cb=PlanToGoal.planner_result_cb),
                transitions={
                    'succeeded': 'succeeded',
                    'preempted': 'preempted',
                    'aborted': 'aborted'
                }
            )

    @staticmethod
    @smach.cb_interface(input_keys=['target_pose'])
    def planner_goal_cb(user_data, goal):
        goal.use_start_pose = False
        goal.tolerance = 0.05
        goal.target_pose = user_data.target_pose
        goal.planner = 'defaultPlanner'

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message', 'path', 'cost'],
        outcomes=['succeeded', 'preempted', 'aborted'])
    def planner_result_cb(user_data, status, result):
        user_data.message = result.message
        user_data.outcome = result.outcome
        user_data.path = result.path
        user_data.cost = result.cost

        if result.outcome == GetPathResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == GetPathResult.CANCELED:
            return 'preempted'
        else:
            return 'aborted'

class MoveToGoal(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted', 'failed'],
            input_keys=['path'],
            output_keys=[
                'outcome', 'message',
                'final_pose', 'dist_to_goal', 'angle_to_goal']
        )

        with self:

            self.userdata.recovery_behavior = None

            smach.StateMachine.add(
                'EXE_PATH',
                smach_ros.SimpleActionState(
                    'move_base_flex/exe_path',
                    ExePathAction,
                    goal_cb=MoveToGoal.controller_goal_cb,
                    result_cb=MoveToGoal.controller_result_cb),
                transitions={
                    'succeeded': 'succeeded',
                    'preempted': 'preempted',
                    'aborted': 'aborted',
                    'failed': 'failed',
                }
            )

    @staticmethod
    @smach.cb_interface(input_keys=['path'])
    def controller_goal_cb(user_data, goal):
        goal.path = user_data.path
        goal.controller = 'defaultController'

    @staticmethod
    @smach.cb_interface(
        output_keys=['path', 'outcome', 'message', 'final_pose', 'dist_to_goal', 'angle_to_goal'],
        outcomes=['succeeded', 'aborted', 'failed', 'preempted'])
    def controller_result_cb(user_data, status, result):
        outcome_map = {
            ExePathResult.COLLISION: 'COLLISION',
            ExePathResult.CANCELED: 'CANCELED',
            ExePathResult.BLOCKED_PATH: 'BLOCKED_PATH',
            ExePathResult.FAILURE: 'FAILURE',
            ExePathResult.INTERNAL_ERROR: 'INTERNAL_ERROR',
            ExePathResult.INVALID_PATH: 'INVALID_PATH',
            ExePathResult.MISSED_GOAL: 'MISSED_GOAL',
            ExePathResult.INVALID_PLUGIN: 'INVALID_PLUGIN',
            ExePathResult.MISSED_PATH: 'MISSED_PATH',
            ExePathResult.NO_VALID_CMD: 'NO_VALID_CMD',
            ExePathResult.NOT_INITIALIZED: 'NOT_INITIALIZED',
            ExePathResult.OSCILLATION: 'OSCILLATION',
            ExePathResult.PAT_EXCEEDED: 'PAT_EXCEEDED',
            ExePathResult.ROBOT_STUCK: 'ROBOT_SUCK',
            ExePathResult.TF_ERROR: 'TF_ERROR',
            ExePathResult.SUCCESS: 'SUCCESS',
        }

        controller_aborted_map = [
            ExePathResult.TF_ERROR,
            ExePathResult.INTERNAL_ERROR,
            ExePathResult.INVALID_PATH,
            ExePathResult.NOT_INITIALIZED,
        ]

        controller_failed_map = [
            ExePathResult.PAT_EXCEEDED,
            ExePathResult.BLOCKED_PATH,
            ExePathResult.FAILURE,
            ExePathResult.MISSED_PATH,
            ExePathResult.MISSED_GOAL,
            ExePathResult.NO_VALID_CMD,
            ExePathResult.OSCILLATION,
            ExePathResult.ROBOT_STUCK,
        ]

        user_data.outcome = result.outcome
        user_data.message = result.message
        user_data.final_pose = result.final_pose
        user_data.dist_to_goal = result.dist_to_goal
        user_data.angle_to_goal = result.angle_to_goal

        if result.outcome == ExePathResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == ExePathResult.CANCELED:
            return 'preempted'
        elif result.outcome in controller_failed_map:
            return 'failed'
        else:
            return 'aborted'

if __name__=='__main__':
    rospy.init_node('navigation')

    base_sm = smach.StateMachine(outcomes=['preempted', 'aborted'])

    with base_sm:
        smach.StateMachine.add(
                'WAIT_FOR_GOAL',
            WaitForGoal(),
            transitions={
                'received_goal': 'PLAN_TO_GOAL',
                'preempted': 'preempted',
            }
        )

        smach.StateMachine.add(
                'PLAN_TO_GOAL',
            PlanToGoal(),
            transitions={
                'succeeded': 'MOVE_TO_GOAL',
                'preempted': 'WAIT_FOR_GOAL',
                'aborted': 'WAIT_FOR_GOAL',
            }
        )

        smach.StateMachine.add(
                'MOVE_TO_GOAL',
            MoveToGoal(),
            transitions={
                'succeeded': 'WAIT_FOR_GOAL',
                'preempted': 'WAIT_FOR_GOAL',
                'aborted': 'WAIT_FOR_GOAL',
                'failed': 'WAIT_FOR_GOAL',
            }
        )

    outcome = base_sm.execute()
    rospy.spin()
