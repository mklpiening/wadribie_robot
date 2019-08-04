$fn=32;

hull() {
    cylinder(d=0.075, h=0.02);
    translate([0, 0.045, 0]) {
        cylinder(d=0.03, h=0.02);
    }
}

translate([0, 0, 0.02]) {
    cylinder(d=0.07, h=0.02); 
    translate([0, 0.045, 0]) {
        cylinder(d=0.017, h=0.002); 
    }
    difference() {
        hull() {
            cylinder(d=0.072, h=0.002);
            translate([0, 0.045, 0]) {
                cylinder(d=0.017, h=0.002);
            }
        }
        hull() {
            cylinder(d=0.068, h=0.003);
            translate([0, 0.045, 0]) {
                cylinder(d=0.013, h=0.003);
            }
        }
    }
}