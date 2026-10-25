// =====================================================================
// Gas meter magnetometer case v6: XIAO on top, antenna bulkhead in the top wall
// (rectangular, or tapered with tapered = true)
// GY-273 (QMC5883P/L) + Seeed XIAO ESP32-C6 for the back diaphragm cover
// of an Elster/Honeywell AC-250, stuck on with 3M VHB foam tape.
//
// - The face against the meter is only as big as the sensor board needs.
// - The sensor lies chip-side DOWN on a 0.6 mm membrane in the back wall.
// - Above it the walls flare out, and the XIAO lies flat just under the
//   lid, ~17 mm above the sensor chip (keeps Wi-Fi noise out).
// - Four screw posts (self-tapping screws) sit inside the wider top; heads are counterbored flush.
// - Mounted with the power cable (-X wall) pointing down, the +X end faces up.
//   The antenna's RP-SMA bulkhead sits in a turret on the lid at that end, so
//   the antenna screws on pointing straight up and the pigtail drops straight
//   down to the XIAO's U.FL socket (which faces the lid). Its hex flange sits in
//   a pocket, which keeps it from turning. Only the lid changes; the base is v5.
//   (antenna_mount = "wall" puts it through the +X wall of the base instead.)
//
// Coordinates: X = across, Y = along, Z = depth (Z=0 is the taped back face).
// Print: base back-face down, lid outside-face down. ASA or PETG, 0.2 mm
// layers. The flare and the XIAO support tabs are <= 45 degrees: no supports.
//
//   openscad -D 'part="base"' -o base_top.stl gas_meter_case_top.scad
// part = "base" | "lid" | "assembly" | "closed" | "section" | "fittest" | "antennatest" | "plugs" | "plugtest"
// =====================================================================

part = "assembly";

/* ---------- Shape ---------- */
tapered   = false;  // true: small back that flares out to the top; false: plain rectangular box

/* ---------- Shell ---------- */
wall      = 2.0;
floor_t   = 1.9;
membrane  = 0.6;
lid_t     = 2.4;
corner_r  = 3;

/* ---------- Sensor board: GY-273, chip-side down, long axis along Y ---------- */
s_w       = 13.0;   s_l = 18.0;   s_pcb = 1.6;
s_hole_d  = 3.0;    s_hole_edge = 2.6;
s_chip    = 7.5;    // chip centre from the hole end
ledge     = 4.0;    comp_h = 1.3;
res_x0    = 1.5;    res_w = 5.0;
peg_d     = 2.2;

/* ---------- XIAO ESP32-C6, flat under the lid ---------- */
x_l       = 21.0;   // along X; USB-C at the +X end
x_w       = 17.8;   // along Y
x_pcb     = 1.0;
usb_p     = 1.2;    // USB-C overhang past the PCB edge
usb_gap   = 1.8;    // USB-C end to the +X wall (keeps it clear of the lid lip)
x_top_clr = 0.6;    // USB-C (3.2 mm tall) to the underside of the lid
tab_under = 1.5;    // how far the support tabs reach under the PCB

/* ---------- Common ---------- */
clr = 0.3;  rib_t = 1.2;  rib_h = 1.4;

/* ---------- Cable ---------- */
cable_d   = 3.5;

/* ---------- Antenna bulkhead (RP-SMA female, 1/4"-36), +X wall ----------
   ASSUMED until measured: 6.35 mm thread, 8 mm hex flange behind the wall,
   8 mm nut + lock washer in front, ~9.5 mm of thread. */
antenna    = true;
antenna_mount = "cap";     // "cap" (tall box lid) | "turret" (on the lid) | "wall" (through the +X wall of the base)
bh_hole_d  = 6.7;    // clearance for the 6.35 mm thread
bh_flat    = 0;      // > 0: D-flat at this distance from the hole centre (only if the thread has a flat)
bh_hex_af  = 8.0;    // hex flange across flats (inside)
bh_hex_t   = 2.0;    // hex flange thickness
bh_hex_clr = 0.25;
bh_pocket  = 1.4;    // how deep the hex pocket is (stops the connector from turning)
bh_boss_d  = 12.0;   // ring around the pocket, on the inside of the wall
bh_panel   = 2.0;    // wall thickness the thread passes through (thinner = more thread outside)
bh_spot_d  = 11.0;   // outside counterbore for nut + washer, used when bh_panel < wall
bh_ferrule = [3.6, 6.0];  // crimp sleeve behind the flange (dia, length)
bh_thread  = 9.5;    // thread length in front of the flange
tur_wall   = 1.6;    // turret side/back walls and roof

/* ---------- Cap lid (antenna_mount = "cap") ----------
   The lid is a hollow box with the same outline as the base. The bulkhead goes
   through its +X end wall and the pigtail coils inside it. The screws sit at
   the bottom of deep pockets (same 11.5 mm screws, same seats as before). */
cap_roof    = 2.0;
cap_wall    = 2.0;
pocket_d    = 5.0;   // screw pocket (driver bit)
pk_flare_d  = 9.0;   // pocket widens to this at the top ...
pk_flare_h  = 3.0;   // ... over this height
tube_wall   = 1.2;   // wall around each screw pocket
/* ---------- Plugs for the screw pockets ----------
   A cone that sits flush in the pocket's flare, and a ribbed stem that holds by
   friction. A screwdriver slot on top to twist and lever it out. Print top down.
   PETG works; TPU seals better. Tune plug_fit with the plug test (1/2/3 dots). */
plug_fit    = 0.10;  // rib interference on the diameter (mm)
plug_stem   = 4.0;   // stem length below the cone
plug_slot   = [1.0, 1.2];  // screwdriver slot width, depth
cap_under_head = 2.4; // plastic under each screw head (the flat lid has 1.2); the screw
                      // then reaches 1.2 mm less into the post, still ~9 mm of thread
driver_reach = 13;   // straight reach of the driver bit; 15 with the flare

/* ---------- Lid / screws: self-tapping, ~M2, round head 3.5 x 1.9 mm ---------- */
screw_len = 11.5;  // length under the head
pilot_d   = 1.8;   // pilot hole in the posts (prints a little undersized; the screw cuts its thread)
screw_d   = 2.4;   // clearance hole in the lid
head_h    = 1.9;   cb_d = 4.0;  cb_depth = head_h + 0.2;  under_head = 1.2;
boss_h    = max(0, cb_depth + under_head - lid_t);
col_d     = 5.6;
lip_t     = 1.2;   lip_h = 3;   lip_clr = 0.25;

$fn = 48;

/* ---------- Derived ---------- */
sp_w = s_w + 2*clr;   sp_l = s_l + 2*clr;
col_o = col_d/2 - 0.8;
post_reach = col_o + col_d/2;                                   // post extent from a wall
in_wt = clr + x_l + usb_p + usb_gap;                            // top interior
in_lt = x_w + 2*clr + 2*(post_reach + 0.4);
in_w = tapered ? sp_w + 2*rib_t : in_wt;                        // bottom interior
in_l = tapered ? sp_l + 2*rib_t : in_lt;
out_w = in_w + 2*wall;          out_l = in_l + 2*wall;          // back footprint
fx = (in_wt - in_w)/2;          fy = (in_lt - in_l)/2;          // flare per side
flare_h = max(fx, fy);
post_len = (screw_len - under_head) + 1.0;
z_min = floor_t + s_pcb + 1.0 + 1.7;                            // keep the sensor zone straight
out_h = z_min + flare_h + post_len + boss_h;
post_top = out_h - boss_h;
z_f1 = post_top - post_len;     z_f0 = z_f1 - flare_h;
z_pb = out_h - x_top_clr - 3.2 - x_pcb;                         // XIAO PCB underside
in_h = out_h - floor_t;

ix0 = wall;  iy0 = wall;
tx0 = ix0 - fx;  ty0 = iy0 - fy;                                // top interior origin
sp_x0 = ix0 + (in_w - sp_w)/2;  sp_y0 = iy0 + (in_l - sp_l)/2;
bx = sp_x0 + clr;  by = sp_y0 + clr;
holes = [[bx + s_hole_edge, by + s_hole_edge], [bx + s_w - s_hole_edge, by + s_hole_edge]];
chip  = [bx + s_w/2, by + s_chip];
xx0 = tx0 + clr;                         // XIAO PCB start (X)
xy0 = ty0 + (in_lt - x_w)/2;             // XIAO PCB start (Y)
cols = [[tx0 + col_o,         ty0 + col_o],
        [tx0 + in_wt - col_o, ty0 + col_o],
        [tx0 + col_o,         ty0 + in_lt - col_o],
        [tx0 + in_wt - col_o, ty0 + in_lt - col_o]];
cable_y = iy0 + in_l/2;
bh_y   = ty0 + in_lt/2;                              // centred along Y
bh_z   = (z_f1 + z_pb)/2 + 0.2;                      // between sensor zone and XIAO
bh_xw  = tx0 + in_wt;                                // inner face of the +X wall
// turret on the lid: front face flush with the +X end of the case
bh_r      = (bh_hex_af/2 + bh_hex_clr) / cos(30);    // flange corner radius
tur_front = bh_panel + bh_pocket;                    // front wall incl. the hex pocket
tur_x1    = out_w + fx;                              // +X end of the case
tur_tz    = out_h + lid_t + bh_hex_af/2 + bh_hex_clr + 0.3;   // connector axis
tur_cav_h = 2*bh_r + 0.4;                            // room to turn the flange while fitting it
tur_cav_w = 2*bh_r + 0.4;
// the whole rigid connector (flange + crimp + thread) has to fit inside before
// it is pushed out through the front wall
tur_cav_l = bh_hex_t + bh_ferrule[1] + bh_thread - bh_pocket + 0.8;
tur_x0    = tur_x1 - tur_front - tur_cav_l - tur_wall;
tur_top   = tur_tz + tur_cav_h/2 + tur_wall;
tur_w     = tur_cav_w + 2*tur_wall;
lip_in_x  = tx0 + in_wt - lip_clr - lip_t;           // inner face of the lid lip at +X
// cap lid: the flange must stay above the lip (which reaches below z = out_h)
cap_tz    = out_h + 0.2 + bh_hex_af/2 + bh_hex_clr + 1.0;  // connector axis
cap_top   = cap_tz + bh_r + 0.4 + cap_roof;
seat_z    = post_top + cap_under_head;                // screw head seat in the cap
// the flare may not break through the outside: the posts are only col_o + wall from the faces
pk_flare_eff = min(pk_flare_d, 2*(col_o + wall - 0.8));
cap_xw    = out_w + fx - cap_wall;                    // inner face of the +X end wall
if (antenna && antenna_mount == "cap") {
    echo(str("Cap lid: ", cap_top - out_h, " mm tall (case ", cap_top, " mm deep); connector axis ", cap_tz - out_h, " mm above the seam"));
    echo(str("Screw pockets: ", cap_top - seat_z, " mm deep to the screw head (driver reach ", driver_reach, " mm straight); flare at the top ", pocket_d, " -> ", pk_flare_eff, " mm"));
    echo(str("Under each screw head: ", cap_under_head, " mm; screw thread into the post: ", screw_len - cap_under_head, " mm"));
}
if (antenna && antenna_mount == "turret") echo(str("Antenna turret: ", tur_x1 - tur_x0, " x ", tur_w, " mm, ", tur_top - out_h - lid_t, " mm above the lid; axis ", tur_tz - out_h - lid_t, " mm above the lid"));
cable_z = (z_f1 + z_pb)/2 - 0.5;

echo(str("Back footprint: ", out_w, " x ", out_l, " mm"));
echo(str("Top: ", in_wt + 2*wall, " x ", in_lt + 2*wall, " mm; height incl. lid: ", out_h + lid_t, " mm"));
echo(str("Flare z ", z_f0, " -> ", z_f1, "; XIAO PCB underside z = ", z_pb, "; chip-to-XIAO ~", z_pb - (floor_t - 0.9), " mm"));

/* ---------- Helpers ---------- */
module rrect2d(w, l, r) { translate([r, r]) offset(r) square([w - 2*r, l - 2*r]); }

module pocket_ribs(x0, y0, w, l, z0) {
    seg = 0.35;
    for (side = [0, 1]) {
        for (e = [0, 1])
            translate([x0 - rib_t + side*(w + rib_t), y0 + e*l*(1 - seg), z0]) cube([rib_t, l*seg, rib_h]);
        for (e = [0, 1])
            translate([x0 - rib_t + e*(w*(1 - seg) + rib_t), y0 - rib_t + side*(l + rib_t), z0])
                cube([w*seg + rib_t, rib_t, rib_h]);
    }
}

module outer_solid() {
    hull() {
        linear_extrude(z_f0) rrect2d(out_w, out_l, corner_r);
        translate([-fx, -fy, z_f1]) linear_extrude(out_h - z_f1) rrect2d(out_w + 2*fx, out_l + 2*fy, corner_r);
    }
}
module inner_solid() {
    hull() {
        translate([ix0, iy0, floor_t]) cube([in_w, in_l, z_f0 - floor_t]);
        translate([tx0, ty0, z_f1]) cube([in_wt, in_lt, out_h - z_f1 + 1]);
    }
}
module posts() {
    intersection() {
        for (c = cols) translate([c[0], c[1], tapered ? z_f0 : 0]) cylinder(d = col_d, h = post_top - (tapered ? z_f0 : 0));
        outer_solid();
    }
}
// tabs on the posts that carry the XIAO's four corners (45-degree undersides)
module xiao_tabs() {
    for (side = [0, 1]) {
        y_in  = side == 0 ? ty0 + post_reach - 0.6 : ty0 + in_lt - post_reach + 0.6;
        y_out = side == 0 ? xy0 + tab_under        : xy0 + x_w - tab_under;
        d = abs(y_out - y_in);
        for (xr = [[tx0 + 1.0, tx0 + post_reach - 0.6], [tx0 + in_wt - post_reach + 0.6, tx0 + in_wt - 1.0]])
            translate([xr[0], 0, 0]) rotate([90, 0, 90]) linear_extrude(xr[1] - xr[0])
                polygon([[y_in, z_pb], [y_out, z_pb], [y_in, z_pb - d]]);
    }
}

/* ---------- Base ---------- */
module base() {
    difference() {
        union() {
            difference() { outer_solid(); inner_solid(); }
            posts();
            xiao_tabs();
            pocket_ribs(sp_x0, sp_y0, sp_w, sp_l, floor_t);
            for (h = holes) translate([h[0], h[1], floor_t - 0.01]) cylinder(d = peg_d, h = s_pcb + 1.0);
            if (antenna && antenna_mount == "wall") bh_boss();
        }
        // sensor component recess + resistor slot, down to the membrane
        translate([sp_x0 - 0.01, by + ledge, membrane]) cube([sp_w + 0.02, sp_l - clr - ledge + 0.01, comp_h + 0.5]);
        translate([bx + s_w/2 - res_w/2, by + res_x0, membrane]) cube([res_w, ledge - res_x0 + 0.01, comp_h + 0.5]);
        // pilot holes for the self-tapping screws
        for (c = cols) translate([c[0], c[1], post_top - (screw_len - under_head) - 0.5])
            cylinder(d = pilot_d, h = screw_len + 1);
        // cable hole in the -X wall, just below the XIAO
        translate([tx0 - wall - 1, cable_y, cable_z]) rotate([0, 90, 0]) cylinder(d = cable_d, h = wall + 2);
        if (antenna && antenna_mount == "wall") bh_cut();
    }
}

/* ---------- Cap lid ---------- */
module cap_in_place() {
    h = cap_top - out_h;
    difference() {
        union() {
            // shell
            difference() {
                translate([-fx, -fy, out_h]) linear_extrude(h) rrect2d(out_w + 2*fx, out_l + 2*fy, corner_r);
                translate([-fx + cap_wall, -fy + cap_wall, out_h - 0.01])
                    linear_extrude(h - cap_roof + 0.01) rrect2d(out_w + 2*fx - 2*cap_wall, out_l + 2*fy - 2*cap_wall, max(0.5, corner_r - cap_wall));
            }
            // locating lip inside the base (same as the flat lid)
            translate([0, 0, out_h - lip_h]) linear_extrude(lip_h + 0.01) lip_2d();
            // The lip sits inside the base walls, so it is inset from the cap's own
            // walls. Flare the cap walls (and the screw tubes) inward at 45 degrees
            // down to the seam so the lip is joined to them and prints without
            // overhangs (the cap prints roof down, seam on top).
            cap_seam_flare();
            // screw tubes from the roof down to the seats, and the bosses that sit on the posts
            intersection() {
                for (c = cols) translate([c[0], c[1], out_h]) cylinder(d = pocket_d + 2*tube_wall, h = h);
                translate([-fx, -fy, out_h]) linear_extrude(h) rrect2d(out_w + 2*fx, out_l + 2*fy, corner_r);
            }
            intersection() {
                for (c = cols) translate([c[0], c[1], post_top]) cylinder(d = col_d, h = out_h - post_top + 0.01);
                translate([tx0 + lip_clr, ty0 + lip_clr, post_top - 1]) cube([in_wt - 2*lip_clr, in_lt - 2*lip_clr, boss_h + 2]);
            }
            // boss with the hex pocket on the inside of the +X end wall, kept above the lip
            intersection() {
                translate([cap_xw - bh_pocket + 0.01, bh_y, cap_tz]) rotate([0, 90, 0]) cylinder(d = bh_boss_d, h = bh_pocket);
                translate([cap_xw - 5, ty0, out_h]) cube([5.01, in_lt, h]);
            }
        }
        // screw: clearance hole, head seat, pocket, flare
        for (c = cols) translate(c) {
            translate([0, 0, post_top - 1]) cylinder(d = screw_d, h = seat_z - post_top + 1.01);
            translate([0, 0, seat_z]) cylinder(d = pocket_d, h = cap_top - seat_z + 1);
            translate([0, 0, cap_top - pk_flare_h]) cylinder(d1 = pocket_d, d2 = pk_flare_eff, h = pk_flare_h + 0.01);
        }
        // hex pocket and thread hole
        translate([cap_xw - bh_pocket - 0.01, bh_y, cap_tz]) rotate([0, 90, 0]) rotate([0, 0, 30])
            cylinder(d = (bh_hex_af + 2*bh_hex_clr) / cos(30), h = bh_pocket + 0.02, $fn = 6);
        translate([cap_xw - bh_pocket - 1, bh_y, cap_tz]) rotate([0, 90, 0]) intersection() {
            cylinder(d = bh_hole_d, h = cap_wall + bh_pocket + 2);
            if (bh_flat > 0) translate([-bh_hole_d, -bh_flat, 0]) cube([2*bh_hole_d, bh_hole_d, cap_wall + bh_pocket + 2]);
            else cylinder(d = bh_hole_d + 1, h = cap_wall + bh_pocket + 2);
        }
        // thinner panel from the outside if asked for
        if (bh_panel < cap_wall)
            translate([cap_xw + bh_panel, bh_y, cap_tz]) rotate([0, 90, 0]) cylinder(d = bh_spot_d, h = cap_wall);
    }
}

module cap_seam_flare() {
    d = cap_wall + lip_clr + lip_t;            // lip inner face, from the outside
    f = d - cap_wall + 0.2;                    // flare height for 45 degrees
    W = out_w + 2*fx;  L = out_l + 2*fy;
    intersection() {
        translate([-fx, -fy, out_h]) linear_extrude(f) rrect2d(W, L, corner_r);
        difference() {
            translate([-fx - 1, -fy - 1, out_h - 0.01]) cube([W + 2, L + 2, f + 0.02]);
            hull() {
                translate([-fx + d, -fy + d, out_h - 0.02]) linear_extrude(0.01) rrect2d(W - 2*d, L - 2*d, max(0.5, corner_r - d + 1));
                translate([-fx + cap_wall - 0.2, -fy + cap_wall - 0.2, out_h + f]) linear_extrude(0.01)
                    rrect2d(W - 2*cap_wall + 0.4, L - 2*cap_wall + 0.4, max(0.5, corner_r - cap_wall));
            }
        }
    }
    // the lip's rings around the posts reach further in than the screw tubes
    r_ring = col_d/2 + lip_clr + lip_t + 0.05;  // a hair past the ring, so the surfaces don't coincide
    r_tube = pocket_d/2 + tube_wall;
    if (r_ring > r_tube) intersection() {
        for (c = cols) translate([c[0], c[1], out_h]) cylinder(r1 = r_ring, r2 = r_tube - 0.1, h = r_ring - r_tube + 0.1);
        translate([-fx, -fy, out_h]) linear_extrude(f) rrect2d(W, L, corner_r);
    }
}

/* ---------- Pocket plugs ---------- */
// built top down (z = 0 is the top face, flush with the lid)
module plug(fit = plug_fit, dots = 0) {
    cl = 0.05;                        // cone clearance: seats on the flare
    core = pocket_d - 0.4;            // stem core, ribs do the gripping
    rib_r = 0.45;
    rib_c = (pocket_d + fit) / 2 - rib_r;   // rib centre radius so the tips reach (pocket_d + fit)/2
    difference() {
        union() {
            cylinder(d1 = pk_flare_eff - 2*cl, d2 = pocket_d - 2*cl, h = pk_flare_h);
            translate([0, 0, pk_flare_h - 0.01]) {
                cylinder(d = core, h = plug_stem - 0.4);
                translate([0, 0, plug_stem - 0.41]) cylinder(d1 = core, d2 = core - 0.8, h = 0.4);
                // three ribs with a lead-in at the bottom
                for (a = [0, 120, 240]) rotate([0, 0, a]) translate([rib_c, 0, 0]) hull() {
                    cylinder(r = rib_r, h = plug_stem - 1.2, $fn = 16);
                    translate([-0.35, 0, 0]) cylinder(r = rib_r * 0.6, h = plug_stem - 0.6, $fn = 16);
                }
            }
        }
        // screwdriver slot across the top
        translate([-(pk_flare_eff + 1)/2, -plug_slot[0]/2, -0.01]) cube([pk_flare_eff + 1, plug_slot[0], plug_slot[1] + 0.01]);
        // fit-test marks: dots in the top face
        for (i = [0 : dots - 1]) if (dots > 0)
            rotate([0, 0, 90]) translate([(i - (dots - 1)/2) * 0.9 + 0, 1.8, -0.01]) cylinder(d = 0.6, h = 0.3, $fn = 12);
    }
}
module plugs() {
    for (i = [0 : 3]) translate([i * (pk_flare_eff + 3), 0, 0]) plug();
}
module plug_test() {
    for (i = [0 : 2]) translate([i * (pk_flare_eff + 3), 0, 0]) plug(fit = [0.0, 0.10, 0.20][i], dots = i + 1);
}

/* ---------- Antenna turret on the lid ---------- */
module turret_solid() {
    r = 1.5;
    translate([0, bh_y - tur_w/2, out_h + lid_t - 0.01]) hull() {
        for (x = [tur_x0 + r, tur_x1 - r], y = [r, tur_w - r])
            translate([x, y, 0]) cylinder(r = r, h = tur_top - out_h - lid_t - r + 0.01);
        for (x = [tur_x0 + r, tur_x1 - r], y = [r, tur_w - r])
            translate([x, y, tur_top - out_h - lid_t - r]) sphere(r = r);
    }
}
module turret_cut() {
    xf = tur_x1 - tur_front;          // inner face of the front wall (incl. pocket)
    // cavity above the lid
    translate([tur_x0 + tur_wall, bh_y - tur_cav_w/2, out_h + lid_t - 0.02])
        cube([xf - tur_x0 - tur_wall, tur_cav_w, tur_tz + tur_cav_h/2 - out_h - lid_t + 0.02]);
    // opening down through the lid into the case, stopping short of the lip
    translate([tur_x0 + tur_wall, bh_y - tur_cav_w/2, out_h - 1])
        cube([min(xf, lip_in_x - 0.3) - tur_x0 - tur_wall, tur_cav_w, lid_t + 1.1]);
    // hex pocket (flats top and bottom) and thread hole through the front wall
    translate([xf - 0.01, bh_y, tur_tz]) rotate([0, 90, 0]) rotate([0, 0, 30])
        cylinder(d = (bh_hex_af + 2*bh_hex_clr) / cos(30), h = bh_pocket + 0.01, $fn = 6);
    translate([xf, bh_y, tur_tz]) rotate([0, 90, 0]) intersection() {
        cylinder(d = bh_hole_d, h = tur_front + 1);
        if (bh_flat > 0) translate([-bh_hole_d, -bh_flat, 0]) cube([2*bh_hole_d, bh_hole_d, tur_front + 1]);
        else cylinder(d = bh_hole_d + 1, h = tur_front + 1);
    }
}

/* ---------- Antenna bulkhead in the +X wall (antenna_mount = "wall") ---------- */
module bh_boss() {
    // ring on the inner face of the +X wall, clipped to stay below the XIAO/USB-C
    intersection() {
        translate([bh_xw - bh_pocket + 0.01, bh_y, bh_z]) rotate([0, 90, 0]) cylinder(d = bh_boss_d, h = bh_pocket);
        translate([bh_xw - 5, ty0, floor_t]) cube([5, in_lt, z_pb - 0.4 - floor_t]);
    }
}
module bh_cut() {
    // thread hole (optionally D-shaped)
    translate([bh_xw - bh_pocket - 1, bh_y, bh_z]) rotate([0, 90, 0]) intersection() {
        cylinder(d = bh_hole_d, h = wall + bh_pocket + 2);
        if (bh_flat > 0) translate([-bh_hole_d, -bh_flat, 0]) cube([2*bh_hole_d, bh_hole_d, wall + bh_pocket + 2]);
        else cylinder(d = bh_hole_d + 1, h = wall + bh_pocket + 2);
    }
    // hex pocket for the flange (flats top and bottom)
    translate([bh_xw - bh_pocket - 0.01, bh_y, bh_z]) rotate([0, 90, 0]) rotate([0, 0, 30])
        cylinder(d = (bh_hex_af + 2*bh_hex_clr) / cos(30), h = bh_pocket + 0.02, $fn = 6);
    // thinner panel from the outside if asked for
    if (bh_panel < wall)
        translate([bh_xw + bh_panel, bh_y, bh_z]) rotate([0, 90, 0]) cylinder(d = bh_spot_d, h = wall);
}
module bh_dummy() {
    turret = antenna_mount == "turret";
    cap = antenna_mount == "cap";
    // flange sits against the inner face of the wall
    x_in = turret ? tur_x1 - bh_panel - bh_hex_t : cap ? cap_xw + cap_wall - bh_panel - bh_hex_t : bh_xw - bh_hex_t;
    x_out = turret ? tur_x1 : cap ? cap_xw + cap_wall : bh_xw + wall;
    az = turret ? tur_tz : cap ? cap_tz : bh_z;
    translate([0, bh_y, az]) {
        color("Gold") {
            translate([x_in, 0, 0]) rotate([0, 90, 0]) rotate([0, 0, 30]) cylinder(d = bh_hex_af / cos(30), h = bh_hex_t, $fn = 6);
            translate([x_in + bh_hex_t, 0, 0]) rotate([0, 90, 0]) cylinder(d = 6.35, h = 9.5);
            translate([x_in - bh_ferrule[1], 0, 0]) rotate([0, 90, 0]) cylinder(d = bh_ferrule[0], h = bh_ferrule[1]);
            translate([x_out, 0, 0]) rotate([0, 90, 0]) rotate([0, 0, 30]) cylinder(d = 8 / cos(30), h = 2.0, $fn = 6);  // nut
        }
        color("Black") translate([x_out + 2.5, 0, 0]) rotate([0, 90, 0]) cylinder(d = 10, h = 30);  // antenna base (cut short)
        // pigtail: out of the crimp, then down towards the XIAO
        color("DimGray") translate([x_in - bh_ferrule[1] - 3, 0, 0]) rotate([0, 90, 0]) cylinder(d = 1.13, h = 3);
        color("DimGray") translate([x_in - bh_ferrule[1] - 3, 0, -(turret || cap ? az - z_pb - 3.5 : 3)]) cylinder(d = 1.13, h = turret || cap ? az - z_pb - 3.5 : 3);
        // a flat coil of the spare pigtail in the cap
        if (cap) color("DimGray") translate([x_in - bh_ferrule[1] - 11, 0, -(az - out_h) + 3]) rotate_extrude($fn = 48) translate([7.5, 0]) circle(d = 1.2);
    }
}

/* ---------- Lid ---------- */
module lip_2d() {
    inner = [in_wt - 2*lip_clr, in_lt - 2*lip_clr];
    o = [tx0 + lip_clr, ty0 + lip_clr];
    post_clear = col_d/2 + lip_clr;
    intersection() {
        translate(o) square(inner);
        difference() {
            union() {
                translate(o) difference() {
                    square(inner);
                    translate([lip_t, lip_t]) square([inner[0] - 2*lip_t, inner[1] - 2*lip_t]);
                }
                for (c = cols) translate(c) circle(r = post_clear + lip_t);
            }
            for (c = cols) translate(c) circle(r = post_clear);
        }
    }
}

module lid_in_place() {
    if (antenna && antenna_mount == "cap") cap_in_place();
    else flat_lid_in_place();
}
module flat_lid_in_place() {
    difference() {
        union() {
            if (antenna && antenna_mount == "turret") turret_solid();
            translate([-fx, -fy, out_h]) linear_extrude(lid_t) rrect2d(out_w + 2*fx, out_l + 2*fy, corner_r);
            translate([0, 0, out_h - lip_h]) linear_extrude(lip_h + 0.01) lip_2d();
            intersection() {
                for (c = cols) translate([c[0], c[1], post_top]) cylinder(d = col_d, h = boss_h + 0.01);
                translate([tx0 + lip_clr, ty0 + lip_clr, post_top - 1]) cube([in_wt - 2*lip_clr, in_lt - 2*lip_clr, boss_h + 2]);
            }
        }
        if (antenna && antenna_mount == "turret") turret_cut();
        for (c = cols) translate(c) {
            translate([0, 0, post_top - 1]) cylinder(d = screw_d, h = boss_h + lid_t + 2);
            translate([0, 0, out_h + lid_t - cb_depth]) cylinder(d = cb_d, h = cb_depth + 1);
        }
    }
}
// Without the turret the lid prints outside-face down. With the turret it prints
// the right way up, standing on its lip: the plate bridges between the lip
// walls (~24 mm), which PETG/ASA handle with good cooling.
module lid() {
    if (antenna && antenna_mount == "turret") translate([0, 0, -(out_h - lip_h)]) lid_in_place();
    else if (antenna && antenna_mount == "cap") translate([0, out_l, cap_top]) rotate([180, 0, 0]) lid_in_place();
    else translate([0, out_l, out_h + lid_t]) rotate([180, 0, 0]) lid_in_place();
}

/* ---------- Dummy boards ---------- */
module dummies() {
    color("RoyalBlue") translate([bx, by, floor_t]) difference() {
        cube([s_w, s_l, s_pcb]);
        for (h = holes) translate([h[0] - bx, h[1] - by, -1]) cylinder(d = s_hole_d, h = 4);
    }
    color("Black") translate([chip[0] - 1.5, chip[1] - 1.5, floor_t - 0.9]) cube([3, 3, 0.9]);
    color("DarkSlateGray") translate([xx0, xy0, z_pb]) cube([x_l, x_w, x_pcb]);
    color("Silver") translate([xx0 + 3, xy0 + 2.6, z_pb + x_pcb]) cube([13, 12.5, 2.4]);
    color("Silver") translate([xx0 + x_l - 7.3 + usb_p, xy0 + x_w/2 - 4.5, z_pb + x_pcb]) cube([7.3, 9, 3.2]);
    color("Black") translate([tx0 - wall - 6, cable_y, cable_z]) rotate([0, 90, 0]) cylinder(d = cable_d - 0.2, h = 14);
    if (antenna && antenna_mount == "wall") bh_dummy();
}

/* ---------- Fit test ---------- */
// 1) the bottom (sensor pocket, pegs, resistor slot, membrane)
// 2) the top ring (posts with pilot holes, XIAO tabs, cable hole), printed
//    standing on its lower edge right next to it
module fit_test() {
    intersection() {
        base();
        translate([-50, -50, -1]) cube([100, 100, z_min + 0.5]);
    }
    fz = (antenna && antenna_mount == "wall") ? min(z_f1 + 0.3, floor_t + s_pcb + 1.1) : z_f1 + 0.3;
    translate([out_w + 2*fx + 6, 0, -fz]) intersection() {
        base();
        translate([-50, -50, fz]) cube([100, 100, post_top - fz + 1]);
    }
}

// Just the +X wall with the bulkhead hole and pocket: a 10-minute print to
// check the connector before printing the whole case. Prints lying on its
// outside face.
module antenna_test() {
    if (antenna_mount == "cap")
        // the +X end of the cap with the connector hole, printed roof down like the cap
        translate([0, 0, cap_top]) rotate([180, 0, 0]) intersection() {
            lid_in_place();
            translate([cap_xw - 6, bh_y - 9, out_h]) cube([cap_wall + 6.01, 18, cap_top - out_h + 0.01]);
        }
    else if (antenna_mount == "turret")
        // the turret with the lid plate under it (no lip, so it prints flat on the plate)
        translate([0, 0, -out_h]) intersection() {
            lid_in_place();
            translate([tur_x0 - 2, bh_y - tur_w/2 - 2, out_h]) cube([tur_x1 - tur_x0 + 2.01, tur_w + 4, 40]);
        }
    else
        rotate([0, -90, 0]) translate([-(bh_xw + wall), 0, 0]) intersection() {
            base();
            translate([bh_xw - 3, bh_y - 9, bh_z - 8]) cube([wall + 3.01, 18, 16]);
        }
}

/* ---------- Output ---------- */
if (part == "base") base();
else if (part == "lid") lid();
else if (part == "fittest") fit_test();
else if (part == "antennatest") antenna_test();
else if (part == "plugs") plugs();
else if (part == "plugtest") plug_test();
else if (part == "closed") {
    color("WhiteSmoke") base();
    color("Gainsboro") lid_in_place();
    if (antenna && antenna_mount != "wall") bh_dummy();
}
else if (part == "section") {
    intersection() {
        union() { color("WhiteSmoke") base(); color("LightGray") lid_in_place(); dummies();
                  if (antenna && antenna_mount != "wall") bh_dummy(); }
        translate([-50, antenna ? bh_y : chip[1], -10]) cube([100, 100, 100]);
    }
}
else {
    color("WhiteSmoke") base();
    dummies();
    color("LightGray", 0.35) translate([0, 0, 15]) lid_in_place();
    if (antenna && antenna_mount != "wall") translate([0, 0, 15]) bh_dummy();
}
