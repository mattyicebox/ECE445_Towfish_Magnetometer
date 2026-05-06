// 80 mm x 80 mm x 100 mm hollow box with a centered circular hole in the top.
// Wall and top/bottom thickness are 4 mm.

outer_x = 80;
outer_y = 80;
height = 100;
wall = 4;
top_hole_diameter = 20;

difference() {
    cube([outer_x, outer_y, height], center = false);

    // Hollow interior, leaving a 4 mm floor and 4 mm top.
    translate([wall, wall, wall])
        cube([
            outer_x - 2 * wall,
            outer_y - 2 * wall,
            height - 2 * wall
        ], center = false);

    // Centered hole through the top face.
    translate([outer_x / 2, outer_y / 2, height - wall - 1])
        cylinder(h = wall + 2, d = top_hole_diameter, $fn = 96);
}
