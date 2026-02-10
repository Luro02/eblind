$fa = 1;
$fs = $preview ? 1.0 : 0.2; // if necessary, increase from 0.2 to 0.5 to speedup rendering

include <BOSL2/std.scad>;
include <BOSL2/screw_drive.scad>;

adapter_height = 30.0; // The total height of the adapter including the base
adapter_diam = 27.2; // The measured diameter of the blind tube
screw_hole_diam = 3.3; // M3 hole + tolerance
screw_head_diam = 5.7; // M3 Screw head diameter + tolerance
screw_head_wall = 2.0; // How much space should be between screw head and output shaft

base_height = 4.0; // The height of the base.
base_offset = 1; // How much the base radius differs from the blind tube diam

// This defines the spokes that grab the blind tube.

spoke_num = 10; // The number of spokes to add.
spoke_width = 2.0; // How wide the spokes are.
spoke_inset = 3; // How deep they are
spoke_chamfer_height = 10;

// These are internal variables that shouldn't be changed.
// If you change them, the adapter might not fit on my models
// anymore.
planetary_output_shaft_diam = 10.8;

torx_bit_height = 10.0;
torx_bit_chamfer_height = 3.0;
// head is about 3mm tall + 1mm tolerance
torx_bit_z_pos = 30.0 - torx_bit_height - screw_head_wall - 4;
best_torx_size = 50;

module create_supported_hole(inner_diam, outer_diam, height) {
    union() {
        translate([0, 0, 0.4]) difference() {
            cylinder(h=height-0.4, d1=outer_diam, d2=outer_diam);
            translate([0, 0, -0.2]) cylinder(h=height, d1=inner_diam, d2=inner_diam);
        };
    
        intersection() {
            union() {
                translate([-outer_diam/2.0, inner_diam/2.0, 0]) cube([outer_diam, (outer_diam - inner_diam) / 2.0, 0.4]);
                translate([-outer_diam/2.0, -outer_diam/2.0, 0]) cube([outer_diam, (outer_diam - inner_diam) / 2.0, 0.4]);
            
                translate([inner_diam/2.0, -outer_diam/2.0, 0.2]) cube([(outer_diam - inner_diam) / 2.0, outer_diam, 0.2]);
                translate([-outer_diam/2.0, -outer_diam/2.0, 0.2]) cube([(outer_diam - inner_diam) / 2.0, outer_diam, 0.2]);
            };
            cylinder(h=1, d1=outer_diam, d2=outer_diam);
        };
    };
}

module create_torx_receiver(torx_size, torx_height, chamfer_height, outer_diam) {
    // By subtracting the torx bit shape from the created cylinder, we will obtain the desired form
    difference() {
        // this creates a shape that looks like this:
        //
        //  ____
        // | __ |
        // |/  \|
        union() {
            // first create a cylinder where the torx bit should be:
            translate([0, 0, chamfer_height]) cylinder(h=torx_height, d1=outer_diam, d2=outer_diam);
            // this creates a cylinder that looks like this |/ \|
            difference() {
                cylinder(h=chamfer_height, d1=outer_diam, d2=outer_diam);
                cylinder(h=chamfer_height, d1=outer_diam, d2=torx_info(torx_size)[1]);
            };
        };
        torx_mask(torx_size, torx_height + chamfer_height);
    };
}


module triangle(width, height, depth) {
    rotate([0, 90, 0]) rotate([0, 0, 90]) linear_extrude(depth) polygon(points=[[0, 0], [width, 0], [0, height]]);
}

module create_spoke(width, depth, height, chamfer_height) {
    union() {
        translate([0, width, 0]) rotate([0, 0, 270]) translate([0, 0, height - chamfer_height]) triangle(depth, chamfer_height, width);
        cube([depth, width, height - chamfer_height]);
    }
}

module create_blind_shaft(inner_diam, outer_diam, height, base_height=5.0, base_offset=3.0, spoke_inset=5.0, spoke_width=3.0, spokes=5) {
    base_diam = outer_diam + base_offset * 2;
    inner_cylinder_diam = base_diam - 2 * spoke_inset - base_offset * 2;
    difference() {
        union() {
            cylinder(h=base_height, d1=base_diam, d2=base_diam);
            translate([0, 0, base_height]) cylinder(h=height - base_height, d1=inner_cylinder_diam, d2=inner_cylinder_diam);
        };
        translate([0, 0, -1]) cylinder(height + 2, d1=inner_diam, d2=inner_diam);
    }

    spoke_height = height - base_height;
    
    // create the spokes:
    for(angle = [0 : 360/spokes : 360]) {
        rotate([0,0,angle])
            translate([inner_cylinder_diam / 2.0 - 0.1, -spoke_width / 2.0, base_height])
                create_spoke(spoke_width, spoke_inset, spoke_height, spoke_chamfer_height);
    }
}

union() {
    translate([0, 0, torx_bit_z_pos - torx_bit_chamfer_height]) create_torx_receiver(best_torx_size, torx_bit_height, torx_bit_chamfer_height, planetary_output_shaft_diam);

    torx_bit_z_end = torx_bit_z_pos + torx_bit_height;
    translate([0, 0, torx_bit_z_end]) difference() {
        create_supported_hole(screw_hole_diam, planetary_output_shaft_diam, adapter_height - torx_bit_z_end);
        translate([0, 0, screw_head_wall])
        cylinder(h=(adapter_height - screw_head_wall), d1=screw_head_diam, d2=screw_head_diam);
    };
    create_blind_shaft(planetary_output_shaft_diam, adapter_diam, adapter_height, base_height=base_height, spoke_inset=spoke_inset, spoke_width=spoke_width, spokes=spoke_num, base_offset=base_offset);
}
