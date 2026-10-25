// =====================================================================
// Gas meter magnetometer case v5: XIAO on top (rectangular, or tapered with tapered = true)
// GY-273 (QMC5883P/L) + Seeed XIAO ESP32-C6 for the back diaphragm cover
// of an Elster/Honeywell AC-250, stuck on with 3M VHB foam tape.
//
// - The face against the meter is only as big as the sensor board needs.
// - The sensor lies chip-side DOWN on a 0.6 mm membrane in the back wall.
// - Above it the walls flare out, and the XIAO lies flat just under the
//   lid, ~17 mm above the sensor chip (keeps Wi-Fi noise out).
// - Four screw posts (self-tapping screws) sit inside the wider top; heads are counterbored flush.
//
// Coordinates: X = across, Y = along, Z = depth (Z=0 is the taped back face).
// Print: base back-face down, lid outside-face down. ASA or PETG, 0.2 mm
// layers. The flare and the XIAO support tabs are <= 45 degrees: no supports.
//
//   openscad -D 'part="base"' -o base_top.stl gas_meter_case_top.scad
// part = "base" | "lid" | "assembly" | "section" | "fittest"
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
        }
        // sensor component recess + resistor slot, down to the membrane
        translate([sp_x0 - 0.01, by + ledge, membrane]) cube([sp_w + 0.02, sp_l - clr - ledge + 0.01, comp_h + 0.5]);
        translate([bx + s_w/2 - res_w/2, by + res_x0, membrane]) cube([res_w, ledge - res_x0 + 0.01, comp_h + 0.5]);
        // pilot holes for the self-tapping screws
        for (c = cols) translate([c[0], c[1], post_top - (screw_len - under_head) - 0.5])
            cylinder(d = pilot_d, h = screw_len + 1);
        // cable hole in the -X wall, just below the XIAO
        translate([tx0 - wall - 1, cable_y, cable_z]) rotate([0, 90, 0]) cylinder(d = cable_d, h = wall + 2);
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
    difference() {
        union() {
            translate([-fx, -fy, out_h]) linear_extrude(lid_t) rrect2d(out_w + 2*fx, out_l + 2*fy, corner_r);
            translate([0, 0, out_h - lip_h]) linear_extrude(lip_h + 0.01) lip_2d();
            intersection() {
                for (c = cols) translate([c[0], c[1], post_top]) cylinder(d = col_d, h = boss_h + 0.01);
                translate([tx0 + lip_clr, ty0 + lip_clr, post_top - 1]) cube([in_wt - 2*lip_clr, in_lt - 2*lip_clr, boss_h + 2]);
            }
        }
        for (c = cols) translate(c) {
            translate([0, 0, post_top - 1]) cylinder(d = screw_d, h = boss_h + lid_t + 2);
            translate([0, 0, out_h + lid_t - cb_depth]) cylinder(d = cb_d, h = cb_depth + 1);
        }
    }
}
module lid() { translate([0, out_l, out_h + lid_t]) rotate([180, 0, 0]) lid_in_place(); }

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
    translate([out_w + 2*fx + 6, 0, -(z_f1 + 0.3)]) intersection() {
        base();
        translate([-50, -50, z_f1 + 0.3]) cube([100, 100, post_top - z_f1 + 1]);
    }
}

/* ---------- Output ---------- */
if (part == "base") base();
else if (part == "lid") lid();
else if (part == "fittest") fit_test();
else if (part == "section") {
    intersection() {
        union() { color("WhiteSmoke") base(); color("LightGray") lid_in_place(); dummies(); }
        translate([-50, chip[1], -10]) cube([100, 100, 100]);
    }
}
else {
    color("WhiteSmoke") base();
    dummies();
    color("LightGray", 0.35) translate([0, 0, 15]) lid_in_place();
}
