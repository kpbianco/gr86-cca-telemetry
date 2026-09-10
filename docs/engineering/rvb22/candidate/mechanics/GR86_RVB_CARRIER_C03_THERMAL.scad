// C03: revised right chassis mounts; C02 retained as historical model input. PCB bottomZ0. This is source geometry, not native solid/actual vehicle fit.
$fn=64;
include <MECH_GEOMETRY.scad>
part="assembly";
board_t=1.60;sleeve_t=1.90;base_top=-6;base_bottom=-9;
under_module_clearance=7; //X-case C206 is outside the carrier; no material added here.
module base(){difference(){union(){
 //MECH22-31: local left-spine reinforcement; global fitbox unchanged.
 translate([-8,28,base_bottom])cube([8,12,3]);
 translate([-7,-15,base_bottom])cube([73.5,73,3]);
 for(p=mounts)translate([p[0]-3.5,p[1]-3.5,base_top])cube([7,7,6]);
 translate([-13,-12,base_bottom])cube([8,11,3]);
 translate([-13,42.5,base_bottom])cube([8,12.5,3]);
 }
 translate([13.5,-9.5,base_bottom-1])linear_extrude(5)offset(r=1)translate([1,1])square([39,7]);
 translate([13.5,41.8,base_bottom-1])linear_extrude(5)offset(r=1)translate([1,1])square([39,7]);
 difference(){
 translate([0,2,base_bottom-1])linear_extrude(5)offset(r=2)translate([2,2])square([55.5,33]);
 //Preserve2mm sidewebs beside wider right slots; insert staysbelowT01Y28.8.
 translate([58.5,30,base_bottom-1])cube([8,12,5]);
 }
 for(p=mounts)translate([p[0],p[1],base_bottom-1])cylinder(d=2.2,h=12);
 for(p=[[-9,-6],[-9,47.5],[57,-11.5],[57,54.5]])translate([p[0],p[1],base_bottom-1])cylinder(d=3.4,h=5);
 for(p=[[-4,31],[-4,37],[62.5,33],[62.5,39]])translate([p[0],p[1],base_bottom-1])linear_extrude(5)hull(){translate([-1.15,0])circle(r=.8);translate([1.15,0])circle(r=.8);}
}}
module sleeve(p){translate([p[0],p[1],0])difference(){cylinder(d=4,h=sleeve_t);translate([0,0,-1])cylinder(d=2.2,h=sleeve_t+2);}}
module washer(p){translate([p[0],p[1],sleeve_t])difference(){cylinder(d=6,h=.5);translate([0,0,-1])cylinder(d=2.2,h=3);}}
if(part=="base"||part=="assembly")color("ivory")base();
if(part=="hardware"||part=="assembly")for(p=mounts){color("silver")sleeve(p);color("silver")washer(p);}
if(part=="assembly"){
 color([0,.4,.2,.5])difference(){linear_extrude(board_t)polygon(outline);for(p=mounts)translate([p[0],p[1],-1])cylinder(d=4.4,h=4);}
 color([1,.4,0,.1])translate([0,0,board_t])cube([94.454766,41.288863,16]);
 color([1,0,0,.1])translate([73.254766,-1,-15])cube([36,48,35]);
 for(p=mounts)color([.2,.2,1,.1])translate([p[0],p[1],3.9])cylinder(d=6,h=20);
}
