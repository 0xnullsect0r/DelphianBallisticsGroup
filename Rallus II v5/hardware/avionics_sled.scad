// Rallus II v3 — avionics sled: dedicated places for everything electronic.
// Stack, from the camera bay down:
//   component deck  : Pico 2 on standoffs (USB toward the nose), 3-slot
//                     breakout caddy (MS5611 / W25Q128 / MAX31855), BEC
//                     strap posts, thermocouple probe clip
//   battery cradle  : 2S 450 LiPo on velcro, centered for CG
//   bottom ring     : switch body + RFB jack mounted radially so they poke
//                     through the tube wall, plus the pass-through for the
//                     chute shock cord
// Print: PETG or CF-nylon, 4 walls, 30 % infill. Print deck and cradle flat
// as modeled; ring standing.

include <dimensions.scad>;

deck_t = 3;
post_h = 42;                            // deck -> cradle standoffs (M3)
cradle_t = 3;
ring_t = 6;

module pico_footprints(deck_h) {
    // 4x M2.2 standoff bosses on the deck top face, USB end toward +Z(nose)
    for (sx = [-1, 1], sy = [-1, 1])
        translate([sx * pico_hole_x / 2, sy * pico_hole_y / 2, deck_h])
            difference() {
                cylinder(d = 6, h = pico_standoff_h, $fn = 24);
                translate([0, 0, 1])
                    cylinder(d = pico_hole_d, h = pico_standoff_h, $fn = 24);
            }
}

module caddy(deck_h) {
    // Three side-by-side slots on raised rails; boards friction-fit with a
    // dab of silicone. Slot opens upward for wire access.
    slot_pitch = caddy_slot_w + 3;
    total_w = caddy_n * slot_pitch;
    translate([-total_w / 2 + slot_pitch / 2, -pico_hole_x / 2 - 16, deck_h])
        for (i = [0 : caddy_n - 1])
            translate([i * slot_pitch, 0, 0])
                difference() {
                    cube([caddy_slot_w, caddy_slot_l, 8], center = false);
                    translate([2, 2, 3])
                        cube([caddy_slot_w - 4, caddy_slot_l - 4, 6]);
                }
}

module bec_posts(deck_h) {
    // Strap posts for the BEC, opposite the caddy.
    for (sx = [-1, 1])
        translate([sx * (bec_l / 2 + 2), pico_hole_x / 2 + 12, deck_h])
            difference() {
                cylinder(d = 8, h = 10, $fn = 24);
                translate([0, 0, -1]) cylinder(d = 3.4, h = 12, $fn = 24);
            }
}

module deck() {
    difference() {
        cylinder(d = part_od, h = deck_t);
        // Wire pass-throughs and weight relief
        for (a = [0, 120, 240])
            rotate([0, 0, a])
                translate([0, 0, -1])
                    cylinder(d = 10, h = deck_t + 2, $fn = 24);
        // M3 post holes (three, offset from the Pico/caddy side)
        for (a = [30, 150, 270])
            rotate([0, 0, a])
                translate([part_od / 2 - 6, 0, -1])
                    cylinder(d = 3.2, h = deck_t + 2, $fn = 24);
    }
    pico_footprints(deck_t);
    caddy(deck_t);
    bec_posts(deck_t);
    // Thermocouple probe clip: dedicated hook on the deck rim, vent side.
    rotate([0, 0, ang_vent + 180])
        translate([part_od / 2 - 4, 0, deck_t])
            difference() {
                cylinder(d = 6, h = 8, $fn = 24);
                translate([0, 0, 2]) cylinder(d = 2.6, h = 8, $fn = 24);
            }
}

module cradle() {
    difference() {
        cylinder(d = part_od, h = cradle_t);
        // Battery pocket + velcro strap slots running across
        translate([-bat_l / 2, -bat_w / 2, -1])
            cube([bat_l, bat_w, cradle_t + 2]);
        for (sx = [-1, 1])
            translate([sx * (bat_l / 2 + 4), -(bat_w / 2) - 6, -1])
                cube([6, bat_w + 12, cradle_t + 2]);
        // M3 post holes matching the deck
        for (a = [30, 150, 270])
            rotate([0, 0, a])
                translate([part_od / 2 - 6, 0, -1])
                    cylinder(d = 3.2, h = cradle_t + 2, $fn = 24);
    }
}

module bottom_ring() {
    // Switch on the arm side (180 deg), RFB jack beside it.
    difference() {
        cylinder(d = part_od, h = ring_t);
        translate([0, 0, -1]) cylinder(d = part_od - 2 * wall_print - 8, h = ring_t + 2);
        rotate([0, 0, ang_arm]) {
            translate([part_od / 2 - 3, -switch_body_w / 2, -1])
                cube([8, switch_body_w, ring_t + 2]);       // switch body
            translate([part_od / 2 - 3, switch_body_w / 2 + 6, -1])
                cylinder(d = rfb_hole_d, h = ring_t + 2, $fn = 24); // RFB jack
        }
        // Shock cord pass-through, center
        translate([0, 0, -1]) cylinder(d = 12, h = ring_t + 2, $fn = 32);
    }
}

module posts() {
    for (a = [30, 150, 270])
        rotate([0, 0, a])
            translate([part_od / 2 - 6, 0, 0])
                difference() {
                    cylinder(d = 7, h = post_h, $fn = 24);
                    translate([0, 0, -1]) cylinder(d = 3.2, h = post_h + 2, $fn = 24);
                }
}

// Exploded view for inspection; print each module separately.
deck();
translate([0, 0, post_h]) cradle();
translate([0, 0, post_h + cradle_t + 20]) bottom_ring();
%posts();
