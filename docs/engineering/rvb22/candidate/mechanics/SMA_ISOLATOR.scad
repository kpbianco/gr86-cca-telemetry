// Recovered RF-ISO01. Unclad insulating FR4; SMA metal must never bridge to vehicle metal.
$fn=64;
difference(){translate([-21,-12,0])cube([42,24,3]);translate([0,0,-1])cylinder(d=6.5,h=5);for(x=[-16,16])translate([x,0,-1])cylinder(d=3.4,h=5);}
// Vehicle metal:20mmminimum opening around axis, not merely6.5mmSMAhole.
// SMA metalOD<=10mm;axisdisplacement<=0.5mm;M3headOD<=7mm.
