/*
NERO 7F -> Wuji Hand2 Beta1 universal thin metal-core candidate V2.

Coordinate convention (millimetres):
  - Hand2 OEM four-capsule plate outer face is Z = 0.
  - +Z points from the wrist toward the Hand2 fingers.
  - The NERO cup mouth is behind the Hand2 face at cup_mouth_z.

This file is intentionally independent from the V1 print-fit core and the
D405 bracket.  It is a metal-intent geometry candidate which must first be
3D-printed and checked on unpowered hardware.  The retained Ø39.60 plug is a
prototype fit value, not a machining release dimension.

Examples:
  openscad -o core_universal.stl \
    -D 'part="core_universal"' nero_hand2_beta1_universal_metal_v2.scad
  openscad -o assembly_preview.stl \
    -D 'part="assembly_preview"' nero_hand2_beta1_universal_metal_v2.scad
*/

$fn = 72;

part = "core_universal";

// ---------------- Frozen NERO cup interface ----------------

nero_cup_nominal_inner_d = 40.0;
nero_cup_nominal_outer_d = 44.0;
nero_plug_d = 39.60;
nero_plug_flat_x = 18.00;
nero_plug_length = 7.80;
nero_plug_lead_length = 0.90;
nero_plug_lead_radial_clearance = 0.35;
nero_plug_root_overlap = 0.20;
nero_radial_hole_from_mouth = 3.50;
nero_radial_m3_print_pilot_d = 2.70;
nero_radial_hole_min_lead_side_material = 2.50;

// This narrow proud annulus is the datum land for the real cup's thin stop
// shoulder.  It establishes axial seating; it is not deleted or thickened by
// the universal frame redesign.
stop_land_outer_d = 44.50;
stop_land_height = 0.40;

// The real cup and the pinned vendor STL both place the D-flat at +X.  The
// male plug therefore uses the same zero-clocked chord; it cannot rotate once
// inserted.  The four radial fasteners are independently clocked 45 degrees.
nero_plug_flat_clocking_deg = 0.0;
nero_radial_hole_clocking_deg = 45.0;

// Freeze V2's body/rail cable opening while correcting only the male plug.
// This value is intentionally independent from the plug D-flat clocking.
core_cable_notch_clocking_deg = -90.0;

// ---------------- Frozen Hand2 Beta1 four-capsule interface ----------------

hand_capsule_center_x = 20.1525;
hand_capsule_center_y = 20.1525;
hand_capsule_center_span = 8.0;
hand_capsule_key_width = 5.80;
hand_capsule_key_depth = 1.80;
hand_capsule_key_lead = 0.30;
hand_capsule_key_lead_clearance = 0.20;
hand_capsule_key_root_overlap = 0.30;
hand_m3_clearance_d = 3.40;

// A 90-degree flat-head M3 seat replaces V1's deep cylindrical counterbore.
// Confirm the actual fastener standard and available thread depth before metal.
hand_m3_countersink_d = 6.40;
hand_m3_countersink_angle = 90.0;
hand_m3_countersink_depth =
    (hand_m3_countersink_d - hand_m3_clearance_d)
        / (2 * tan(hand_m3_countersink_angle / 2));

// ---------------- Universal thin frame ----------------

core_thickness = 3.50;
core_center_hub_d = 42.0;
core_corner_boss_d = 16.0;
core_diagonal_rib_width = 7.0;
core_cable_bore_d = 20.0;
core_cable_notch_width = 10.0;
core_cable_notch_start_x = 22.0;
core_cable_notch_length = 9.5;

// A rounded square ring makes all four attachment directions equivalent.
// The ring is 8 mm wide at each side and remains the same 3.5 mm thickness as
// the body; there is no legacy side-specific camera lug.
frame_outer_size = 62.0;
frame_inner_size = 46.0;
frame_outer_radius = 8.0;
frame_inner_radius = 5.0;

// Two axial M3 clearance holes per side.  They lie outside the NERO cup and
// remain accessible from either face.  No threads are assumed in 3.5 mm stock.
accessory_hole_side_offset = 27.0;
accessory_hole_half_pitch = 10.0;
accessory_m3_clearance_d = 3.40;

cup_mouth_z = -core_thickness - stop_land_height;
nero_radial_hole_z = cup_mouth_z - nero_radial_hole_from_mouth;

assert(core_thickness >= 3.0 && core_thickness <= 5.0,
    "thin-core candidate thickness must remain in the 3-5 mm study range");
assert(core_thickness - hand_m3_countersink_depth >= 1.8,
    "M3 countersink leaves less than 1.8 mm of cylindrical bearing length");
assert(nero_plug_d < nero_cup_nominal_inner_d,
    "prototype NERO plug must retain positive diametral clearance");
assert((stop_land_outer_d - nero_plug_d) / 2 >= 2.0,
    "thin-shoulder datum land is too narrow");
assert(nero_plug_length > nero_radial_hole_from_mouth
        + nero_radial_m3_print_pilot_d / 2,
    "NERO radial pilot breaks through the plug lead end");
assert(nero_plug_length - nero_radial_hole_from_mouth
        - nero_radial_m3_print_pilot_d / 2
        >= nero_radial_hole_min_lead_side_material,
    "NERO radial pilot leaves less than 2.5 mm material toward the lead end");
assert((nero_plug_d - core_cable_bore_d) / 2 >= 6.0,
    "NERO plug radial wall is too thin around cable bore");
assert(frame_outer_size > frame_inner_size,
    "perimeter frame must have positive rail width");
assert((frame_outer_size - frame_inner_size) / 2
        >= accessory_m3_clearance_d + 4.0,
    "accessory rail is too narrow for an M3 through hole");
assert(accessory_hole_side_offset + accessory_m3_clearance_d / 2
        <= frame_outer_size / 2 - 2.0,
    "accessory M3 hole lacks 2 mm outer edge distance");

// ---------------- Reusable geometry ----------------

module rounded_rect_2d(size_xy = [10, 10], radius = 2) {
    offset(r = radius)
        square([size_xy[0] - 2 * radius, size_xy[1] - 2 * radius], center = true);
}

module capsule_2d(center_span = 8, width = 5.8) {
    hull() {
        translate([-center_span / 2, 0]) circle(d = width);
        translate([ center_span / 2, 0]) circle(d = width);
    }
}

module d_profile_2d(diameter = 40, flat_x = 18) {
    radius = diameter / 2;
    intersection() {
        circle(d = diameter);
        translate([-radius - 1, -radius - 1])
            square([flat_x + radius + 1, 2 * radius + 2]);
    }
}

module perimeter_frame_2d() {
    difference() {
        rounded_rect_2d([frame_outer_size, frame_outer_size], frame_outer_radius);
        rounded_rect_2d([frame_inner_size, frame_inner_size], frame_inner_radius);
    }
}

module universal_skeleton_2d() {
    union() {
        // Central metal hub transfers plug bending loads into four diagonal
        // paths.  It is only slightly larger than the Ø40 nominal cup.
        circle(d = core_center_hub_d);
        perimeter_frame_2d();

        for (xs = [-1, 1], ys = [-1, 1]) {
            // Local material remains only around each capsule/M3 interface.
            translate([xs * hand_capsule_center_x, ys * hand_capsule_center_y])
                circle(d = core_corner_boss_d);

            // Four equal diagonal ribs make one non-handed load path.
            hull() {
                translate([xs * 13.5, ys * 13.5])
                    circle(d = core_diagonal_rib_width);
                translate([xs * hand_capsule_center_x, ys * hand_capsule_center_y])
                    circle(d = core_diagonal_rib_width);
            }
        }
    }
}

module hand_capsule_key(xs = 1, ys = 1) {
    key_angle = xs * ys * 45;
    lead_scale = [
        (hand_capsule_center_span + hand_capsule_key_width
            - 2 * hand_capsule_key_lead_clearance)
            / (hand_capsule_center_span + hand_capsule_key_width),
        (hand_capsule_key_width - 2 * hand_capsule_key_lead_clearance)
            / hand_capsule_key_width
    ];

    translate([
        xs * hand_capsule_center_x,
        ys * hand_capsule_center_y,
        -hand_capsule_key_root_overlap
    ])
        rotate([0, 0, key_angle]) {
            linear_extrude(
                height = hand_capsule_key_depth - hand_capsule_key_lead
                    + hand_capsule_key_root_overlap
            )
                capsule_2d(hand_capsule_center_span, hand_capsule_key_width);
            translate([
                0,
                0,
                hand_capsule_key_depth - hand_capsule_key_lead
                    + hand_capsule_key_root_overlap
            ])
                linear_extrude(height = hand_capsule_key_lead, scale = lead_scale)
                    capsule_2d(hand_capsule_center_span, hand_capsule_key_width);
        }
}

module nero_plug_solid() {
    lead_d = nero_plug_d - 2 * nero_plug_lead_radial_clearance;
    lead_flat_x = nero_plug_flat_x - nero_plug_lead_radial_clearance;
    lead_scale = nero_plug_d / lead_d;

    translate([0, 0, cup_mouth_z - nero_plug_length]) {
        linear_extrude(height = nero_plug_lead_length, scale = lead_scale)
            rotate([0, 0, nero_plug_flat_clocking_deg])
                d_profile_2d(lead_d, lead_flat_x);
        translate([0, 0, nero_plug_lead_length])
            linear_extrude(
                height = nero_plug_length - nero_plug_lead_length
                    + nero_plug_root_overlap
            )
                rotate([0, 0, nero_plug_flat_clocking_deg])
                    d_profile_2d(nero_plug_d, nero_plug_flat_x);
    }
}

module nero_stop_land_solid() {
    translate([0, 0, cup_mouth_z])
        cylinder(h = stop_land_height, d = stop_land_outer_d);
}

module universal_core_raw() {
    union() {
        translate([0, 0, -core_thickness])
            linear_extrude(height = core_thickness)
                universal_skeleton_2d();
        nero_stop_land_solid();
        nero_plug_solid();
        for (xs = [-1, 1], ys = [-1, 1]) hand_capsule_key(xs, ys);
    }
}

// ---------------- Interface cuts ----------------

module hand_main_screw_holes() {
    for (xs = [-1, 1], ys = [-1, 1]) {
        translate([
            xs * hand_capsule_center_x,
            ys * hand_capsule_center_y,
            -core_thickness - 0.2
        ])
            cylinder(
                h = core_thickness + hand_capsule_key_depth + 0.4,
                d = hand_m3_clearance_d
            );

        // Large diameter at the NERO side, narrowing toward Hand2.
        translate([
            xs * hand_capsule_center_x,
            ys * hand_capsule_center_y,
            -core_thickness - 0.01
        ])
            cylinder(
                h = hand_m3_countersink_depth + 0.02,
                d1 = hand_m3_countersink_d,
                d2 = hand_m3_clearance_d
            );
    }
}

module accessory_m3_holes() {
    for (side_sign = [-1, 1], pitch_sign = [-1, 1]) {
        translate([
            pitch_sign * accessory_hole_half_pitch,
            side_sign * accessory_hole_side_offset,
            -core_thickness - 0.2
        ])
            cylinder(h = core_thickness + 0.4, d = accessory_m3_clearance_d);
        translate([
            side_sign * accessory_hole_side_offset,
            pitch_sign * accessory_hole_half_pitch,
            -core_thickness - 0.2
        ])
            cylinder(h = core_thickness + 0.4, d = accessory_m3_clearance_d);
    }
}

module nero_radial_m3_pilots() {
    translate([0, 0, nero_radial_hole_z]) {
        rotate([0, 0, nero_radial_hole_clocking_deg]) {
            rotate([0, 90, 0])
                cylinder(h = nero_plug_d + 4, d = nero_radial_m3_print_pilot_d, center = true);
            rotate([90, 0, 0])
                cylinder(h = nero_plug_d + 4, d = nero_radial_m3_print_pilot_d, center = true);
        }
    }
}

module core_cable_reliefs() {
    translate([0, 0, cup_mouth_z - nero_plug_length - 0.2])
        cylinder(
            h = nero_plug_length + core_thickness + stop_land_height
                + hand_capsule_key_depth + 0.4,
            d = core_cable_bore_d
        );

    // The center opening continues through one side rail in the same physical
    // frozen V2 direction.  Both M3 holes on that side remain intact.
    rotate([0, 0, core_cable_notch_clocking_deg])
        translate([
            core_cable_notch_start_x,
            -core_cable_notch_width / 2,
            -core_thickness - 0.1
        ])
            cube([
                core_cable_notch_length,
                core_cable_notch_width,
                core_thickness + 0.2
            ]);
}

module core_universal() {
    difference() {
        universal_core_raw();
        hand_main_screw_holes();
        accessory_m3_holes();
        nero_radial_m3_pilots();
        core_cable_reliefs();
    }
}

// ---------------- Inspection-only references ----------------

module reference_hand_outline_2d() {
    union() {
        rounded_rect_2d([38.0, 40.0], 6.0);
        for (xs = [-1, 1], ys = [-1, 1]) {
            hull() {
                translate([xs * hand_capsule_center_x, ys * hand_capsule_center_y])
                    circle(d = 19.60);
                translate([xs * 13.0, ys * 13.0]) circle(d = 13.0);
            }
        }
    }
}

module hand_plate_reference() {
    difference() {
        linear_extrude(height = 4.0) reference_hand_outline_2d();
        for (xs = [-1, 1], ys = [-1, 1])
            translate([xs * hand_capsule_center_x, ys * hand_capsule_center_y, -0.1])
                rotate([0, 0, xs * ys * 45])
                    linear_extrude(height = 4.2)
                        capsule_2d(8.4, 6.2);
        translate([0, 0, -0.1])
            linear_extrude(height = 4.2)
                rounded_rect_2d([10.0, 34.0], 2.0);
    }
}

module nero_cup_reference() {
    source_mouth_z = 12.0;
    translate([0, 0, cup_mouth_z - source_mouth_z])
        scale([1000, 1000, 1000])
            import("../../../third_party/src/agx_arm_urdf_nero_gripper_flange/nero/meshes/gripper_flange.stl");
}

module assembly_preview(exploded = 0) {
    color([0.16, 0.22, 0.30]) nero_cup_reference();
    color([0.92, 0.58, 0.12]) core_universal();
    translate([0, 0, exploded]) {
        color([0.52, 0.56, 0.62]) hand_plate_reference();
        color([0.20, 0.34, 0.66, 0.20])
            translate([-17, -34, 4]) cube([34, 68, 78]);
    }
}

module section_preview() {
    intersection() {
        assembly_preview(0);
        translate([-40, -0.01, -30]) cube([80, 40.01, 120]);
    }
}

// ---------------- Export selector ----------------

if (part == "core_universal") core_universal();
else if (part == "hand_plate_reference") hand_plate_reference();
else if (part == "assembly_preview") assembly_preview(0);
else if (part == "exploded_preview") assembly_preview(16);
else if (part == "section_preview") section_preview();
else assert(false, str("unknown part selector: ", part));
