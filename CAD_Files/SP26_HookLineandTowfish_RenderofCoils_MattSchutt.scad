// ===== PARAMETERS =====
cube_size = 100;
wall_thickness = 4;
wire_radius = 0.6;

turns = 30;               // ↑ more turns = tighter vertical spacing
segments_per_side = 40;   // ↑ smoother path (was 15)
gap = 20;

// ===== MAIN =====
union() {
    translate([-(cube_size/2 + gap/2), 0, 0])
        assembly();

    translate([(cube_size/2 + gap/2), 0, 0])
        assembly();
}

// ===== SINGLE UNIT =====
module assembly() {
    cube_ring();
    copper_coil();
}

// ===== HOLLOW CUBE =====
module cube_ring() {
    color([0.8,0.8,0.8])
    difference() {
        cube([cube_size, cube_size, cube_size], center=true);

        cube([cube_size - 2*wall_thickness,
              cube_size - 2*wall_thickness,
              cube_size + 2], center=true);
    }
}

// ===== COIL =====
module copper_coil() {
    color([0.72, 0.45, 0.2]) {
        pts = path_points();

        for (i = [0:len(pts)-2]) {
            segment(pts[i], pts[i+1]);
        }
    }
}

// ===== PATH =====
function path_points() =
    [ for (i = [0 : turns * 4 * segments_per_side])
        let(
            t = i / segments_per_side,
            side = floor(t) % 4,
            local_t = t - floor(t),

            z = -cube_size/2 + cube_size * i / (turns * 4 * segments_per_side),
            offset = cube_size/2 + wire_radius,

            x = side == 0 ? lerp(-offset, offset, local_t) :
                side == 1 ? offset :
                side == 2 ? lerp(offset, -offset, local_t) :
                            -offset,

            y = side == 0 ? offset :
                side == 1 ? lerp(offset, -offset, local_t) :
                side == 2 ? -offset :
                            lerp(-offset, offset, local_t)
        )
        [x, y, z]
    ];

// ===== SEGMENT =====
module segment(p1, p2) {
    v = [p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]];
    len = norm(v);

    if (len > 0)
        translate(p1)
            rotate(a = acos(v[2]/len)*180/PI,
                   v = [-v[1], v[0], 0])
            cylinder(h=len, r=wire_radius, $fn=24); // smoother cylinder
}

// ===== LERP =====
function lerp(a,b,t) = a + (b-a)*t;