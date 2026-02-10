$fn = 1000;
$fa = 0.5;
$fs = $preview ? 0.1 : 0.1; // if necessary, increase from 0.2 to 0.5 to speedup rendering

module triangle(width, height, depth) {
    rotate([0, 90, 0]) rotate([0, 0, 90]) linear_extrude(depth) polygon(points=[[0, 0], [width, 0], [0, height]]);
}

module create_spoke(width, depth, height, chamfer_height) {
    translate([0, depth / 2, 0]) rotate([0, 0, 270]) translate([0, 0, height - chamfer_height]) triangle(depth, chamfer_height, width);
    cube([depth, width, height - chamfer_height]);
}

/*cube(5);
translate([0, 0, 5]) triangle(10, 35, 5);*/
module create_supported_hole(inner_diam, outer_diam, height) {
    union() {
        translate([0, 0, 0.4]) difference() {
            cylinder(h=height-0.4, d1=outer_diam, d2=outer_diam);
            translate([0, 0, -0.2]) cylinder(h=height, d1=inner_diam, d2=inner_diam);
        };
    
        intersection() {
            union() {
                translate([-outer_diam/2.0, inner_diam/2.0, 0]) cube([outer_diam, (outer_diam - inner_diam) / 2.0, 0.4]);
                translate([-outer_diam/2.0, -3.0/2.0 * inner_diam, 0]) cube([outer_diam, (outer_diam - inner_diam) / 2.0, 0.4]);
            
                translate([inner_diam/2.0, -outer_diam/2.0, 0.2]) cube([(outer_diam - inner_diam) / 2.0, outer_diam, 0.2]);
                translate([-outer_diam/2.0, -3.0/2.0 * inner_diam, 0.2]) cube([(outer_diam - inner_diam) / 2.0, outer_diam, 0.2]);
            };
            cylinder(h=1, d1=outer_diam, d2=outer_diam);
        };
    };
}

//create_supported_hole(3.3, 10, 5);
include <./libs/bosl2/std.scad>
include <./libs/bosl2/screw_drive.scad>

planetary_output_shaft_diam = 10.8;

tolerance = 0.2;

adapter_height = 30.0; // The total height of the adapter including the base
adapter_diam = 27.2; // The measured diameter of the blind tube
screw_hole_diam = 3.3; // M3 hole + tolerance
screw_head_diam = 5.7; // M3 Screw head diameter + tolerance
screw_head_wall = 2.0; // How much space should be between screw head and output shaft

torx_bit_height = 5.0;
torx_bit_chamfer_height = 5.0;
torx_bit_z_pos = adapter_height - 12;

// head is around 3mm tall

base_height = 4.0; // The height of the base.
base_offset = 1; // How much the base radius differs from the blind tube diam

// This defines the spokes that grab the blind tube.

spoke_num = 10; // The number of spokes to add.
spoke_width = 2.0; // How wide the spokes are.
spoke_inset = 3; // How deep they are
spoke_chamfer_height = 10;

allowed_torx_sizes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 27, 30, 40, 45, 50, 55, 60,  70, 80, 90, 100];

function min_key(list) = (len(list) == 1) ? list[0] : let(head = list[0], tail = min_key(list_tail(list))) (head[0] < tail[0]) ? head : tail;

function find_best_torx_size(available, max_diam) =
    min_key([for (i = available) if (max_diam - torx_diam(i) > 0.0) [max_diam - torx_diam(i), i]])[1];

best_torx_size = find_best_torx_size(allowed_torx_sizes, planetary_output_shaft_diam);

echo(str("Using Torx Size: ", best_torx_size));
translate([0, 0, 5]) torx_mask(50, 10);
cylinder(h=5, d1=10.5, d2=10.5);
