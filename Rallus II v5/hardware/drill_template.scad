// Rallus II v3 — drill template for the aluminum upper tube.
// A developed (unrolled) view of every hole in the upper tube. Print the
// two halves, wrap them around the tube edge-to-edge, align the 0 deg seam
// with the rail button plane (use a rail button hole as the index pin),
// tape, center-punch, drill.
//
//   45/135/225/315 deg @ 12 mm  : 4x 3.2 mm canard shaft holes
//   90 deg @ 60 mm              : 26 x 26 mm camera window (drill + file)
//   270 deg @ 130 mm            : 3.2 mm baro vent (tape over lightly? NO:
//                                 this vent stays open; only the window is
//                                 taped)
//   180 deg @ 196.5 mm          : 9 x 5 mm switch slot
//   180 deg @ 198 mm            : 6 mm RFB jack hole
//   0 deg @ 140 mm              : 3 mm LED lightpipe (optional)
//
// Print 1:1, both halves. PETG is fine. Check a calibration cube first.

include <dimensions.scad>;

develop_w = tube_od * PI;               // 159.6 mm circumference
t = 1.2;                                // template thickness

module half_upper() {
    // stations 0..100 mm
    difference() {
        cube([develop_w / 2, 100, t]);
        for (a = ang_canards) {
            x = (a / 360) * develop_w;
            if (x < develop_w / 2)
                translate([x, st_canard_shaft, -1]) cylinder(d = shaft_hole_d, h = t + 2);
        }
        xw = (ang_window / 360) * develop_w;
        translate([xw - window_w / 2, 60 - window_h / 2, -1])
            cube([window_w, window_h, t + 2]);
        // optional LED lightpipe: half-hole on the 0 deg seam edge
        translate([0, 140, -1]) cylinder(d = led_pipe_d, h = t + 2);
        // seam alignment notch at 0 deg
        translate([develop_w / 2 - 2, 50, -1]) cube([4, 6, t + 2]);
    }
}

module half_lower() {
    // stations 100..210 mm, angles 180..360 deg. Local y = station - 100.
    // Local x = 0 is the 180 deg seam; x = develop_w/2 is the 0 deg seam.
    difference() {
        cube([develop_w / 2, 110, t]);   // stations 100..210
        // 270 deg vent: local x = (270-180)/360 * develop_w = develop_w/4
        translate([develop_w / 4, 130 - 100, -1])
            cylinder(d = vent_d, h = t + 2);
        // 180 deg: switch slot + RFB hole at the local x = 0 seam edge
        translate([-switch_slot_l / 2, 196.5 - 100, -1])
            cube([switch_slot_l, switch_slot_w, t + 2]);
        translate([0, 198 - 100, -1])
            cylinder(d = rfb_hole_d, h = t + 2);
        // seam alignment notches at both edges
        translate([develop_w / 2 - 2, 40, -1]) cube([4, 6, t + 2]);
        translate([-2, 40, -1]) cube([4, 6, t + 2]);
    }
}

// Hole angle -> x on the development: x = (angle / 360) * circumference,
// measured from the 0 deg seam (rail button plane). 180 deg and 270 deg
// both land beyond the mid-circumference, so they sit on the second half.

// Parts must not touch when rendered together (coplanar CGAL collapse).
half_upper();
translate([0, 120, 0]) half_lower();
