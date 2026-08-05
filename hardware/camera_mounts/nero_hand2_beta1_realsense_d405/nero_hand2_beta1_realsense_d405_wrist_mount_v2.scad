/*
NERO + Wuji Hand2 Beta1 + RealSense D405 wrist-camera mount v2

This simulation-qualified prototype uses the two Hand2 flange capsules nearest
the right thumb (+Y side), unlike v1's dorsal pair.  XY is the metal-flange
face; +Z runs from wrist toward fingertips.

Nominal source dimensions (millimetres)
  - Pinned Hand2 Beta1 right asset: capsule centres at
    X = +/-20.1525, Y = +20.1525.  The right-hand URDF places the thumb CMC
    on +Y.  Each 14.2 x 6.2 x 2.0 pocket receives a clearance-fit key.
  - RealSense D400 Series Datasheet 337029-017 Rev 023, Figure 10-14:
    D405 body 42 x 42 x 23 nominal, rear 2 x M3 at 20 pitch, maximum rear M3
    insertion 4 mm.  An M3x6 screw through this 3.2 mm plate gives 2.8 mm
    nominal insertion.

The official datasheet recommends a thermally conductive 6000-series
aluminium bracket and a large mating surface.  This one-piece SCAD geometry
is suitable for simulation and a low-cost fit prototype, not thermal or
dynamic qualification.  Confirm the production flange, screw engagement,
cable bend, payload and collision envelope before powered use.
*/

$fn = 56;

// Export "right" as the accepted canonical geometry.  Export "left" as a
// separate, baked Y-reflection; never mirror this mesh with a negative USD
// scale because that would corrupt winding, normals and collision semantics.
mount_side = "right";

// ---------------- Placement parameters ----------------

// Right-hand base convention: -X is dorsal, +Y is the thumb/inside edge,
// and +Z runs toward the fingertips.  The v2 camera remains on the thumb
// side instead of drifting back to a dorsal, wrist-centred viewpoint.
camera_center_x = -55.0;
camera_center_y = 90.0;
camera_rear_plane_z = 30.0;
camera_azimuth_deg = -55.0;
camera_tilt_deg = 58.0;

// ---------------- Hand2 thumb-side interface ----------------

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

strut_radius = 3.4;
camera_support_half_span = 11.5;
camera_body_clearance = 0.15;
camera_route_x = camera_body_x / 2 + strut_radius + 1.6;
camera_route_z = -4.0;
show_reference_preview = true;

// ---------------- Derived values ----------------

camera_rotation_y = camera_tilt_deg;

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

module ball_strut(point_a, point_b, radius = strut_radius) {
    hull() {
        translate(point_a) sphere(r = radius);
        translate(point_b) sphere(r = radius);
    }
}

module routed_strut(point_a, camera_anchor_y, radius = strut_radius) {
    route_point = camera_point([
        camera_route_x,
        camera_anchor_y,
        camera_route_z
    ]);
    plate_point = camera_point([0.0, camera_anchor_y, 0.8]);

    // The base is in front of the D405 rear plane for this accepted optical
    // pose.  Route around the body edge before turning behind the rear plate;
    // a direct stay would pass through the camera housing.
    ball_strut(point_a, route_point, radius);
    ball_strut(route_point, plate_point, radius);
}

module support_frame_solid() {
    // This is the v1 twin-stay architecture turned toward the thumb side.
    // Two primary members carry the plate; the lighter crossed pair resists
    // yaw and cable torque without turning the frame into a solid wall.
    difference() {
        union() {
            routed_strut(
                [-22.0, 31.0, -base_thickness],
                -camera_support_half_span
            );
            routed_strut(
                [ 22.0, 31.0, -base_thickness],
                camera_support_half_span
            );
            routed_strut(
                [-22.0, 32.0, -base_thickness],
                camera_support_half_span,
                strut_radius * 0.82
            );
            routed_strut(
                [ 22.0, 32.0, -base_thickness],
                -camera_support_half_span,
                strut_radius * 0.82
            );
        }

        // Remove only the physical D405 body envelope.  The previous
        // half-space subtraction also deleted the middle of every stay.
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

module printable_mount() {
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

module side_specific_printable_mount() {
    assert(
        mount_side == "right" || mount_side == "left",
        "mount_side must be right or left"
    );
    if (mount_side == "right") {
        printable_mount();
    } else {
        mirror([0, 1, 0]) printable_mount();
    }
}

// ---------------- Preview-only envelopes ----------------

module reference_preview() {
    %color([0.55, 0.58, 0.62, 0.35]) cylinder(h = 5, d = 60);
    %color([0.15, 0.35, 0.75, 0.25])
        translate([-17, -34, 0]) cube([34, 68, 45]);
    %color([0.08, 0.08, 0.08, 0.38])
        camera_transform()
            translate([0, 0, camera_plate_thickness + camera_body_depth / 2])
                cube([camera_body_x, camera_body_y, camera_body_depth], center = true);
    %color([0.9, 0.2, 0.1, 0.55])
        camera_transform()
            translate([0, 9.0, camera_plate_thickness + camera_body_depth + 1])
                cylinder(h = 150, d = 1.2);
}

side_specific_printable_mount();
if (show_reference_preview) {
    if (mount_side == "right") {
        reference_preview();
    } else {
        mirror([0, 1, 0]) reference_preview();
    }
}
