// Rallus II v3 — GoPro HERO5 Session camera cage.
// Sleeve pressed into the upper tube directly below the canard servo ring.
// The camera sits with its lens face against the tube wall, centered on the
// drilled window (the aperture rim drops into the window opening, so the
// cage self-indexes in rotation). Foam pads on the four non-lens faces
// provide shock mounting; the retention plate screws over the back.
// Print: PETG or CF-nylon, 4 walls, 25 % infill, the layer lines running
// around the sleeve (print oriented as modeled, window up).

include <dimensions.scad>;

cage_h = st_cam_bay_h;                  // 46
cav_w = cam_w + 2 * cam_clear + 2 * cam_foam;
cav_h = cam_h + 2 * cam_clear + 2 * cam_foam;
cav_d = cam_d + cam_clear + cam_foam + 1;   // lens face flush at wall

module camera_cage() {
    difference() {
        cylinder(d = part_od, h = cage_h);

        // Camera cavity, offset toward the window side (local +X)
        translate([part_od / 2 - cav_d, -(cav_w / 2) - 1, (cage_h - cav_h) / 2 - 1])
            cube([cav_d + 2, cav_w + 2, cav_h + 2]);

        // Window aperture through the sleeve wall, self-indexing rim
        translate([part_od / 2 - wall_print - 1, -window_w / 2, (cage_h - window_h) / 2])
            cube([wall_print + 2, window_w, window_h]);

        // Retention plate screw bosses: 4x M3 from the back
        for (sy = [-1, 1], sz = [-1, 1])
            translate([-part_od / 2 + 5, sy * (cav_w / 2 + 6),
                       cage_h / 2 + sz * (cav_h / 2 + 5)])
                rotate([0, 90, 0])
                    cylinder(d = 2.8, h = 10, $fn = 24);

        // Alignment flat on top against the servo ring (anti-rotation belt
        // and suspenders): a shallow notch at 0 deg.
        rotate([0, 0, 0])
            translate([-2, -part_od / 2 - 1, cage_h - 6])
                cube([4, 4, 8]);
    }
}

module retention_plate() {
    // Screws over the back of the camera cavity, M3 into the bosses.
    difference() {
        hull() {
            translate([0, -(cav_w / 2) - 1, 0]) cube([2.4, cav_w + 2, cav_h + 2]);
        }
        for (sy = [-1, 1], sz = [-1, 1])
            translate([0, sy * (cav_w / 2 + 6), (cav_h + 2) / 2 + sz * (cav_h / 2 + 5)])
                rotate([0, 90, 0])
                    cylinder(d = 3.2, h = 10, $fn = 24);
    }
}

camera_cage();
