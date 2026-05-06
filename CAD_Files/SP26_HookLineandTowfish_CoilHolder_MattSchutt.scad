// 100 mm x 100 mm x 100 mm hollow square ring with 4 mm walls.
// Open at the top and bottom.

outer_size = 100;
height = 100;
wall = 4;

difference() {
    cube([outer_size, outer_size, height], center = false);

    translate([wall, wall, -1])
        cube([
            outer_size - 2 * wall,
            outer_size - 2 * wall,
            height + 2
        ], center = false);
}
