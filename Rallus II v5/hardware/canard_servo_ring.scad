// Rallus II v3 — canard servo ring.
// Printed ring pressed into the top of the upper tube. Carries the 4 MG90S
// servos at 45 deg offsets (clear of the rail button plane and the camera
// window), outputs pointing UP toward the canard shaft bosses.
// Retention: pocket + zip-tie through the flange tabs (tolerant of servo
// dimension variance). Print: PETG or CF-nylon, 4 walls, 30 % infill.

include <dimensions.scad>;

ring_h = st_canard_bay_h;               // 38
boss_z = st_canard_shaft;               // shaft axis height in the tube

module servo_pocket_2d() {
    // Generous pocket around the body + tab rails; ties exit the sides.
    hull() {
        square([mg_body_l + 2, mg_body_w + 2], center = true);
        square([mg_tab_span + 4, mg_body_w + 2], center = true);
    }
}

module canard_servo_ring() {
    difference() {
        // Outer sleeve + top flange
        union() {
            cylinder(d = part_od, h = ring_h);
            translate([0, 0, ring_h - 3])
                cylinder(d = part_od, h = 3);
        }
        // Central wiring channel
        translate([0, 0, -1])
            cylinder(d = part_od - 2 * (mg_body_w + 7), h = ring_h + 2);

        // Four servo pockets, rotated to the canard angles. Pocket floor
        // sits the servo output flush with the ring top.
        for (a = ang_canards)
            rotate([0, 0, a])
                translate([0, -(mg_body_l + 2) / 2 - (part_od - 2 * (mg_body_w + 7)) / 2 - 0.5, 6])
                    linear_extrude(ring_h)
                        servo_pocket_2d();

        // Shaft bosses: 3.2 mm holes through the top at the canard radius.
        // Canard CP radius ~ tube_od/2 + 14 mm of span; shafts sit at the
        // tube wall, boss center = tube wall mid-thickness radius.
        shaft_r = (tube_od - tube_wall) / 2;
        for (a = ang_canards)
            rotate([0, 0, a])
                translate([shaft_r, 0, -1])
                    cylinder(d = shaft_hole_d, h = ring_h + 2);

        // Nose cone interface (v4): counterbore for the nose spigot
        // (5 deep) + two key-pin sockets 12 deep from the counterbore
        // floor at radius 17, angles 0/180.
        translate([0, 0, -1]) cylinder(d = 41.6, h = 6);
        translate([17, 0, 5]) cylinder(d = 4.2, h = 12);
        rotate([0, 0, 180]) translate([17, 0, 5]) cylinder(d = 4.2, h = 12);

        // Zip-tie slots through each tab area (side entry, 3 mm wide).
        for (a = ang_canards)
            rotate([0, 0, a])
                translate([0, -part_od / 2 - 1, 12])
                    rotate([90, 0, 0])
                        hull() {
                            cylinder(d = 3, h = 8);
                            translate([0, 8, 0]) cylinder(d = 3, h = 8);
                        }
    }
}

canard_servo_ring();
