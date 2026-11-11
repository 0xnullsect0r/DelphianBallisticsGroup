// Rallus II v3 — shared dimensions for the upper-tube bay hardware.
// Units: mm. VERIFY marked numbers against your actual parts before
// printing; everything downstream is parametric off these.

// ---- upper tube (aluminum) ----
tube_od = 50.8;
tube_wall = 2.0;
tube_id = tube_od - 2 * tube_wall;      // 46.8

// slip-fit diameter for printed parts inside the tube (0.4 mm diametral
// clearance; bump to 0.6 if your printer runs large)
fit_clear = 0.4;
part_od = tube_id - fit_clear;

// ---- stations along the upper tube, from the top (nose shoulder) ----
st_canard_shaft = 12;     // canard shaft axis below tube top
st_canard_bay_h = 38;     // servo ring height
st_cam_bay      = st_canard_bay_h;       // camera bay starts here
st_cam_bay_h    = 46;
st_sled_bay     = st_canard_bay_h + st_cam_bay_h;          // 84
st_sled_bay_h   = 94;
st_bay_end      = st_sled_bay + st_sled_bay_h;             // 178

// ---- angular layout (0 deg = rail button plane) ----
ang_window = 90;                        // camera window + lens
ang_canards = [45, 135, 225, 315];      // shafts clear of rail + window
ang_vent = 270;                         // baro vent, sled bay
ang_arm = 180;                          // RFB jack + switch side
ang_led = 0;                            // LED lightpipe

// ---- canards / shafts ----
shaft_d = 3.0;
shaft_hole_d = 3.2;                     // drill through tube wall

// ---- MG90S servos (VERIFY against your units) ----
mg_body_l = 22.8;
mg_body_w = 12.2;
mg_body_h = 23.0;                       // below flange
mg_tab_span = 32.5;                     // overall length incl. flange tabs
mg_hole_pitch = 28.0;                   // tab hole spacing, long axis
mg_hole_d = 2.5;

// ---- GoPro HERO5 Session (VERIFY: ~37.4 x 37.4 x 35.9 cube) ----
cam_w = 37.4;
cam_h = 37.4;
cam_d = 35.9;
cam_clear = 1.2;                        // cavity clearance per side
cam_foam = 1.5;                         // foam pad thickness on 4 faces
window_w = 26;                          // tube window opening
window_h = 26;

// ---- Pico 2 (VERIFY holes against the board) ----
pico_l = 51.0;
pico_w = 21.0;
pico_hole_x = 47.0;                     // 4-hole pitch
pico_hole_y = 20.3;
pico_hole_d = 2.2;
pico_standoff_h = 6;

// ---- breakout caddy slots (GY modules vary; generous) ----
caddy_slot_l = 26;
caddy_slot_w = 18;
caddy_n = 3;                            // MS5611, W25Q128, MAX31855

// ---- battery: 2S 450 mAh LiPo (VERIFY) ----
bat_l = 35;
bat_w = 20;
bat_h = 30;

// ---- BEC ----
bec_l = 28;
bec_w = 16;

// ---- switch + RFB ----
switch_slot_l = 9;                      // tube wall slot for slide knob
switch_slot_w = 5;
switch_body_l = 21;
switch_body_w = 11;
rfb_hole_d = 6.0;                       // 3.5 mm jack clearance

// ---- misc ----
vent_d = 3.2;                           // 1/8 in baro vent
led_pipe_d = 3.0;
wall_print = 2.4;                       // default printed wall

$fn = 64;
