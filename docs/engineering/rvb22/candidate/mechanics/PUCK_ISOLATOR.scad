// Recovered GPS-ISO02. Actual SKU/rigid-base/radome/cable-exit fit remains conditional.
$fn=64;part="assembly";puck=[41.2,38.5,13.3];base_t=2;frame_z=16.5;frame_t=2;
posts=[[-26.5,-20],[-26.5,20],[26.5,-20],[26.5,20]];
module base(){difference(){union(){
 translate([-32,-26,0])cube([64,52,base_t]);
 for(p=posts)translate([p[0],p[1],base_t])cylinder(d=6,h=frame_z-base_t);
 for(x=[-22.6,21.1])for(y=[-17,10])translate([x,y,base_t])cube([1.5,7,frame_z-base_t]);
 for(y=[-21.25,19.75])for(x=[-18,11])translate([x,y,base_t])cube([7,1.5,frame_z-base_t]);
 }
 for(x=[-28,28])translate([x,0,-1])cylinder(d=3.4,h=4);
 for(p=posts)translate([p[0],p[1],-1])cylinder(d=2.2,h=frame_z+2);
}}
module frame(){difference(){translate([-32,-26,frame_z])cube([64,52,frame_t]);translate([-18.5,-17,frame_z-1])cube([37,34,frame_t+2]);for(p=posts)translate([p[0],p[1],frame_z-1])cylinder(d=2.2,h=frame_t+2);}}
if(part=="base"||part=="assembly")color("ivory")base();
if(part=="frame"||part=="assembly")color("ivory")frame();
if(part=="assembly")color([.2,.3,.7,.2])translate([-puck[0]/2,-puck[1]/2,base_t])cube(puck);
// All insulationunclad FR4. No metal over37x34mmaperture.4M2headsOD<=4mm.
