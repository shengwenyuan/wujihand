/*
NERO + Wuji Hand2 Beta1 + Orbbec Gemini 305 wrist-camera mount

Purpose
  One-piece, support-friendly wrist mount for the Wuji Hand2 Beta1 wrist
  flange and the Gemini 305 rear M3 interface.

Coordinate convention
  XY is the Wuji metal-flange face. +Z points from the robot wrist toward
  the hand/fingers. X is palm thickness; Y is hand width.

Source dimensions and revision boundary (millimetres)
  - The pinned Hand2 adapter asset gives outer flange-hole centres near
    X/Y = +/-20.15. The current official Beta1 whole-assembly STEP shows the
    same approximately 40-mm outer-fastener pattern, while the adjustment
    slots retain tolerance for the small revision/measurement difference.
    Wuji does not currently publish that adapter as a standalone Beta1 STEP.
  - Official Gemini 305 drawing/CAD V1.1: 42 x 42 x 23 nominal body, rear
    2 x M3 at 20 pitch, maximum screw insertion 4.8. The rear-hole row and
    camera image horizontal axis both run along local Y in this model; with
    the validated right-hand roll, the Type-C side is local +Y.

Hardware intent
  - Robot side: replace only the two same-side flange-plate screws with
    longer M3 socket-head screws and washers. The NERO motor-output six-screw
    pattern must not be disturbed.
  - Camera side: 2 x M3x6 socket-head screws. With the default 3.2-mm plate,
    nominal camera insertion is 2.8 mm, below the 4.8-mm maximum.

IMPORTANT
  Public CAD can differ from a production revision. Before mounting, power
  off the robot, compare the printed paper/cheap prototype to the real metal
  flange, and verify screw diameter, thread depth, engagement and clearance.
  Never force a screw that bottoms out. Re-run robot collision checks after
  fitting the camera and cable.
*/

$fn = 56;

// ---------------- User parameters ----------------

// Public right-hand adapter default: camera on the -X (dorsal) side.
// Change to +1 to mirror the complete bracket to the other X side.
dorsal_sign = -1;                 // -1 or +1

// Negative lifts the optical axis slightly away from the hand so only a small
// dorsal/finger reference remains at the lower edge of the Color image.
camera_tilt_deg = -2;

// Metal flange interface from the pinned adapter, cross-checked against the
// current official Beta1 whole-assembly STEP. Slots retain fit adjustment.
flange_hole_x = 20.15;
flange_hole_y = 20.15;
robot_slot_width = 3.6;           // M3 clearance
robot_slot_center_span = 5.5;     // adjustment along diagonal

// Printed base.
base_thickness = 4.0;
base_pad_diameter = 16.0;
base_outer_rail_x = 9.0;
base_outer_rail_y = 58.0;
adapter_keepout_half_x = 14.0;    // leaves clearance to public Beta1 adapter

// Gemini 305 rear plate.
camera_plate_x = 34.0;
camera_plate_y = 30.0;
camera_plate_thickness = 3.2;
camera_plate_corner_r = 3.0;
camera_hole_pitch = 20.0;
camera_hole_diameter = 3.4;       // M3 clearance

// Camera position relative to the Wuji flange face.
camera_center_dorsal_offset = 52.0;
camera_rear_plane_z = 28.0;

// Frame stiffness. Increase to 3.8 for soft materials or heavy cabling.
strut_radius = 3.4;

// Preview-only keep-outs; they never enter STL export.
show_reference_preview = true;

// ---------------- Derived values ----------------

mount_x = dorsal_sign * flange_hole_x;
outer_rail_center_x = dorsal_sign * 30.0;
camera_center_x = dorsal_sign * camera_center_dorsal_offset;
camera_rotation_y = -dorsal_sign * camera_tilt_deg;

// ---------------- Reusable geometry ----------------

module rounded_rect_2d(size_xy = [10, 10], radius = 2) {
    offset(r = radius)
        square([size_xy[0] - 2 * radius, size_xy[1] - 2 * radius], center = true);
}

module rounded_plate_xy(size_xyz = [10, 10, 2], radius = 2) {
    linear_extrude(height = size_xyz[2])
        rounded_rect_2d([size_xyz[0], size_xyz[1]], radius);
}

module capsule_2d(center_span = 5, diameter = 3.6) {
    hull() {
        translate([-center_span / 2, 0]) circle(d = diameter);
        translate([ center_span / 2, 0]) circle(d = diameter);
    }
}

module flange_slot(y_sign = 1) {
    // Mirror the diagonal direction together with the chosen dorsal side.
    slot_angle = dorsal_sign * y_sign * 45;
    translate([mount_x, y_sign * flange_hole_y, -0.1])
        rotate([0, 0, slot_angle])
            linear_extrude(height = base_thickness + 0.2)
                capsule_2d(robot_slot_center_span, robot_slot_width);
}

module base_outline_2d() {
    union() {
        translate([outer_rail_center_x, 0])
            rounded_rect_2d([base_outer_rail_x, base_outer_rail_y], 3.5);

        // Two lobe pads and short webs stay on the accessible outer flange.
        for (ys = [-1, 1]) {
            hull() {
                translate([mount_x, ys * flange_hole_y])
                    circle(d = base_pad_diameter);
                translate([outer_rail_center_x, ys * flange_hole_y])
                    circle(d = 9.0);
            }
        }
    }
}

module flange_base() {
    difference() {
        linear_extrude(height = base_thickness)
            base_outline_2d();

        for (ys = [-1, 1]) flange_slot(ys);

        // Shave the pad's inner edge away from the green Hand2 adapter body.
        if (dorsal_sign < 0)
            translate([-adapter_keepout_half_x, -40, -0.1])
                cube([80, 80, base_thickness + 0.2]);
        else
            translate([-80, -40, -0.1])
                cube([80 + adapter_keepout_half_x, 80, base_thickness + 0.2]);
    }
}

module camera_transform() {
    translate([camera_center_x, 0, camera_rear_plane_z])
        rotate([0, camera_rotation_y, 0])
            children();
}

module camera_plate() {
    difference() {
        camera_transform()
            rounded_plate_xy(
                [camera_plate_x, camera_plate_y, camera_plate_thickness],
                camera_plate_corner_r
            );

        camera_transform()
            for (ys = [-1, 1])
                translate([0, ys * camera_hole_pitch / 2, -0.1])
                    cylinder(
                        h = camera_plate_thickness + 0.2,
                        d = camera_hole_diameter
                    );
    }
}

module ball_strut(point_a, point_b, radius = 3.4) {
    hull() {
        translate(point_a) sphere(r = radius);
        translate(point_b) sphere(r = radius);
    }
}

module support_frame() {
    base_x = outer_rail_center_x;
    top_x = camera_center_x;
    base_z = base_thickness;
    top_z = camera_rear_plane_z + 0.8;
    mid_x = (base_x + top_x) / 2;
    mid_z = (base_z + top_z) / 2;

    difference() {
        union() {
            // Widely spaced twin stays resist camera yaw and cable torque.
            for (ys = [-1, 1])
                ball_strut(
                    [base_x, ys * 17.0, base_z],
                    [top_x,  ys * 11.5, top_z],
                    strut_radius
                );

            // Cross brace raises the first torsional mode without a solid wall.
            ball_strut(
                [mid_x, -12.5, mid_z],
                [mid_x,  12.5, mid_z],
                strut_radius * 0.82
            );
        }

        // Ball-ended stays fuse into the plate but must not protrude through
        // its camera-contact face into the Gemini rear housing.
        camera_transform()
            translate([-80, -80, camera_plate_thickness - 0.02])
                cube([160, 160, 200]);
    }
}

module printable_mount() {
    union() {
        flange_base();
        support_frame();
        camera_plate();
    }
}

// ---------------- Preview-only reference envelopes ----------------

module reference_preview() {
    // Approximate 60-mm Wuji metal flange below the printed base.
    %color([0.55, 0.58, 0.62, 0.35])
        translate([0, 0, -8]) cylinder(h = 8, d = 60);

    // Conservative hand/adapter keep-out; actual public CAD is more rounded.
    %color([0.15, 0.35, 0.75, 0.25])
        translate([-17, -34, 0]) cube([34, 68, 45]);

    // Gemini 305 body: rear face starts after the printed camera plate.
    %color([0.08, 0.08, 0.08, 0.35])
        camera_transform()
            translate([0, 0, camera_plate_thickness + 23 / 2])
                cube([42, 42, 23], center = true);

    // Optical-axis guide, 150 mm forward from the camera front.
    %color([0.9, 0.2, 0.1, 0.55])
        camera_transform()
            translate([0, 0, camera_plate_thickness + 23])
                cylinder(h = 150, d = 1.2);
}

printable_mount();
if (show_reference_preview) reference_preview();
