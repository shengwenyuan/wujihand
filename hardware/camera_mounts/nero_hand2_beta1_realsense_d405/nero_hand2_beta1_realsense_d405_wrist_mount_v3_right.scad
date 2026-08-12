/*
NERO + Wuji Hand2 Beta1 + RealSense D405 wrist-camera mount v3, right hand

The v3 physical-fit prototype keeps the accepted v2 Hand2 flange interface
but changes the camera end of the structure:
  - both D405 screw holes retain a circular, planar bearing land on each face;
  - the support frame lands beside the screw holes rather than on their axes;
  - the camera moves farther dorsal/thumb-outboard to protect thumb motion;
  - the optical axis favours the palmar grasp workspace instead of the thumb.

XY is the metal-flange face.  -X is dorsal, +Y is the right-thumb side and +Z
runs from the wrist toward the fingertips.  All dimensions are millimetres.

This file defines only the right-hand prototype.  It does not replace or
modify v2 and is not yet a production, payload, thermal or collision release.
*/

$fn = 64;

// ---------------- Design placement ----------------

// Relative to v2 (-55, 90, 30), the D405 rear plane moves 10 mm farther
// dorsal, 30 mm farther outboard and 6 mm farther toward the fingertips.
camera_center_x = -65.0;
camera_center_y = 120.0;
camera_rear_plane_z = 36.0;

// This pose puts the optical axis through the palmar grasp workspace near
// [30, 0, 115], rather than centring the right-thumb sweep.
camera_azimuth_deg = -55.0;
camera_tilt_deg = 63.0;

// Optical preview only.  It is a design/framing contract, not a physical
// RealSense calibration or a claim about the D405 lens.
design_horizontal_fov_deg = 110.0;
design_aspect_ratio = 4 / 3;

// ---------------- Hand2 flange interface retained from v2 ----------------

flange_hole_x = 20.1525;
flange_hole_y = 20.1525;
robot_screw_hole_diameter = 3.4;
flange_key_width = 5.8;
flange_key_center_span = 8.0;
flange_key_depth = 1.8;
flange_key_lead = 0.3;
flange_key_lead_clearance = 0.2;
flange_key_root_overlap = 0.3;

base_thickness = 4.0;
base_pad_diameter = 16.0;
base_outer_rail_x = 58.0;
base_outer_rail_y = 9.0;
base_outer_rail_center_y = 30.0;
adapter_keepout_max_y = 14.0;
nero_terminal_keepout_half_x = 18.0;
nero_terminal_keepout_max_y = 18.7;

// ---------------- RealSense D405 interface ----------------

camera_body_x = 42.0;
camera_body_y = 42.0;
camera_body_depth = 23.0;
camera_plate_x = 34.0;
camera_plate_y = 30.0;
camera_plate_thickness = 3.2;
camera_plate_corner_r = 3.0;
camera_hole_pitch = 20.0;
camera_hole_diameter = 3.4;

// A 10 mm diameter planar land accommodates an M3 head or small washer.
// Support material is explicitly removed throughout this cylinder; the
// camera plate alone therefore defines both bearing faces around each hole.
camera_screw_flat_land_radius = 5.0;

// Longer v3 stays use a slightly larger primary radius.  Every stay lands at
// camera-local X=+12 mm, more than 12 mm from either screw axis.
primary_strut_radius = 3.8;
cross_strut_radius = 3.1;
camera_support_anchor_x = 12.0;
camera_support_half_span = 12.0;
camera_body_clearance = 0.25;
camera_route_x = camera_body_x / 2 + primary_strut_radius + 1.7;
camera_route_z = -4.5;
camera_plate_anchor_z = 0.8;

show_reference_preview = true;

// ---------------- Derived design gates ----------------

camera_rotation_y = camera_tilt_deg;
nearest_screw_to_anchor_mm = sqrt(
    camera_support_anchor_x * camera_support_anchor_x
        + (camera_support_half_span - camera_hole_pitch / 2)
            * (camera_support_half_span - camera_hole_pitch / 2)
);

assert(
    design_horizontal_fov_deg >= 110 && design_horizontal_fov_deg <= 130,
    "v3 design HFOV must remain within the agreed 110-130 degree range"
);
assert(
    nearest_screw_to_anchor_mm
        > camera_screw_flat_land_radius + primary_strut_radius + 1.0,
    "camera support anchor violates the screw-head planar-land margin"
);

function camera_point(point) = let(
    pitched_x = point[0] * cos(camera_rotation_y)
        + point[2] * sin(camera_rotation_y),
    pitched_z = -point[0] * sin(camera_rotation_y)
        + point[2] * cos(camera_rotation_y)
) [
    camera_center_x
        + pitched_x * cos(camera_azimuth_deg)
        - point[1] * sin(camera_azimuth_deg),
    camera_center_y
        + pitched_x * sin(camera_azimuth_deg)
        + point[1] * cos(camera_azimuth_deg),
    camera_rear_plane_z + pitched_z
];

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

module camera_transform() {
    translate([camera_center_x, camera_center_y, camera_rear_plane_z])
        rotate([0, 0, camera_azimuth_deg])
            rotate([0, camera_rotation_y, 0])
                children();
}

module flange_key(x_sign = 1) {
    key_angle = x_sign * 45;
    lead_scale = [
        (flange_key_center_span + flange_key_width
            - 2 * flange_key_lead_clearance)
            / (flange_key_center_span + flange_key_width),
        (flange_key_width - 2 * flange_key_lead_clearance)
            / flange_key_width
    ];

    translate([
        x_sign * flange_hole_x,
        flange_hole_y,
        -flange_key_root_overlap
    ])
        rotate([0, 0, key_angle]) {
            linear_extrude(
                height = flange_key_depth - flange_key_lead
                    + flange_key_root_overlap
            )
                capsule_2d(flange_key_center_span, flange_key_width);
            translate([
                0,
                0,
                flange_key_depth - flange_key_lead + flange_key_root_overlap
            ])
                linear_extrude(height = flange_key_lead, scale = lead_scale)
                    capsule_2d(flange_key_center_span, flange_key_width);
        }
}

module base_outline_2d() {
    union() {
        translate([0, base_outer_rail_center_y])
            rounded_rect_2d([base_outer_rail_x, base_outer_rail_y], 3.5);
        for (xs = [-1, 1]) {
            hull() {
                translate([xs * flange_hole_x, flange_hole_y])
                    circle(d = base_pad_diameter);
                translate([xs * 22.0, base_outer_rail_center_y])
                    circle(d = base_outer_rail_y);
            }
        }
    }
}

module flange_base_solid() {
    union() {
        difference() {
            translate([0, 0, -base_thickness])
                linear_extrude(height = base_thickness)
                    base_outline_2d();
            translate([-50, -50, -base_thickness - 0.1])
                cube([100, 50 + adapter_keepout_max_y, base_thickness + 0.2]);
            translate([
                -nero_terminal_keepout_half_x,
                adapter_keepout_max_y - 0.1,
                -base_thickness - 0.1
            ])
                cube([
                    2 * nero_terminal_keepout_half_x,
                    nero_terminal_keepout_max_y - adapter_keepout_max_y + 0.2,
                    base_thickness + 0.2
                ]);
        }
        for (xs = [-1, 1]) flange_key(xs);
    }
}

module camera_plate_solid() {
    camera_transform()
        rounded_plate_xy(
            [camera_plate_x, camera_plate_y, camera_plate_thickness],
            camera_plate_corner_r
        );
}

module ball_strut(point_a, point_b, radius) {
    hull() {
        translate(point_a) sphere(r = radius);
        translate(point_b) sphere(r = radius);
    }
}

module routed_strut(point_a, camera_anchor_y, radius) {
    route_point = camera_point([
        camera_route_x,
        camera_anchor_y,
        camera_route_z
    ]);
    plate_point = camera_point([
        camera_support_anchor_x,
        camera_anchor_y,
        camera_plate_anchor_z
    ]);

    ball_strut(point_a, route_point, radius);
    ball_strut(route_point, plate_point, radius);
}

module camera_body_keepout() {
    camera_transform()
        translate([
            -camera_body_x / 2 - camera_body_clearance,
            -camera_body_y / 2 - camera_body_clearance,
            camera_plate_thickness - 0.02
        ])
            cube([
                camera_body_x + 2 * camera_body_clearance,
                camera_body_y + 2 * camera_body_clearance,
                camera_body_depth + 0.04
            ]);
}

module camera_screw_flat_land_keepouts() {
    camera_transform()
        for (ys = [-1, 1])
            translate([0, ys * camera_hole_pitch / 2, -12])
                cylinder(h = camera_plate_thickness + 24, r = camera_screw_flat_land_radius);
}

module support_frame_solid() {
    difference() {
        union() {
            routed_strut(
                [-22.0, 31.0, -base_thickness],
                -camera_support_half_span,
                primary_strut_radius
            );
            routed_strut(
                [22.0, 31.0, -base_thickness],
                camera_support_half_span,
                primary_strut_radius
            );
            routed_strut(
                [-22.0, 32.0, -base_thickness],
                camera_support_half_span,
                cross_strut_radius
            );
            routed_strut(
                [22.0, 32.0, -base_thickness],
                -camera_support_half_span,
                cross_strut_radius
            );
        }
        camera_body_keepout();
        camera_screw_flat_land_keepouts();
    }
}

module robot_screw_holes() {
    for (xs = [-1, 1])
        translate([
            xs * flange_hole_x,
            flange_hole_y,
            -base_thickness - 0.1
        ])
            cylinder(
                h = base_thickness + flange_key_depth + 0.2,
                d = robot_screw_hole_diameter
            );
}

module camera_screw_holes() {
    camera_transform()
        for (ys = [-1, 1])
            translate([0, ys * camera_hole_pitch / 2, -8])
                cylinder(h = camera_plate_thickness + 16, d = camera_hole_diameter);
}

module printable_mount_v3_right() {
    difference() {
        union() {
            flange_base_solid();
            support_frame_solid();
            camera_plate_solid();
        }
        robot_screw_holes();
        camera_screw_holes();
    }
}

// ---------------- Preview-only design envelopes ----------------

module preview_segment(point_a, point_b, radius = 0.7) {
    %color([0.90, 0.22, 0.12, 0.62])
        ball_strut(point_a, point_b, radius);
}

module optical_frustum_preview() {
    optical_origin = [0, 9.0, camera_plate_thickness + camera_body_depth + 1.0];
    preview_depth = 105.0;
    half_width = preview_depth * tan(design_horizontal_fov_deg / 2);
    half_height = half_width / design_aspect_ratio;

    camera_transform() {
        preview_segment(optical_origin, [0, 9.0, optical_origin[2] + 155.0], 0.8);
        for (sx = [-1, 1], sy = [-1, 1])
            preview_segment(
                optical_origin,
                [sx * half_height, 9.0 + sy * half_width, optical_origin[2] + preview_depth],
                0.45
            );
    }
}

module reference_preview() {
    // Approximate wrist/palm body used by v2, retained for visual continuity.
    %color([0.55, 0.58, 0.62, 0.30]) cylinder(h = 5, d = 60);
    %color([0.15, 0.35, 0.75, 0.22])
        translate([-17, -34, 0]) cube([34, 68, 45]);

    // Conservative right-thumb motion envelope.  It is intentionally larger
    // than one static pose and is a design keepout, not a collision asset.
    %color([0.92, 0.18, 0.12, 0.22])
        hull() {
            translate([8, 28, 22]) sphere(r = 13);
            translate([30, 72, 45]) sphere(r = 13);
            translate([18, 68, 82]) sphere(r = 13);
        }

    %color([0.08, 0.08, 0.08, 0.38])
        camera_transform()
            translate([0, 0, camera_plate_thickness + camera_body_depth / 2])
                cube([camera_body_x, camera_body_y, camera_body_depth], center = true);

    // Palmar grasp-workspace target used to choose the v3 optical pose.
    %color([0.12, 0.72, 0.28, 0.38])
        translate([30, 0, 115]) sphere(r = 12);

    optical_frustum_preview();
}

printable_mount_v3_right();
if (show_reference_preview) reference_preview();
