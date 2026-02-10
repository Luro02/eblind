$fa = 1;
$fs = $preview ? 1.0 : 0.2; // if necessary, increase from 0.2 to 0.5 to speedup rendering

include <./libs/bosl2/std.scad>
include <./libs/bosl2/screw_drive.scad>

allowed_torx_sizes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 27, 30, 40, 45, 50, 55, 60,  70, 80, 90, 100];

function min_key(list) = (len(list) == 1) ? list[0] : let(head = list[0], tail = min_key(list_tail(list))) (head[0] < tail[0]) ? head : tail;

function find_best_torx_size(available, max_diam) =
    min_key([for (i = available) if (max_diam - torx_diam(i) > 0.0) [max_diam - torx_diam(i), i]])[1];

shaft_diam = 10.8;

best_torx_size = find_best_torx_size(allowed_torx_sizes, shaft_diam);

echo(str("Using Torx Size: ", best_torx_size));
