// Service and installed harness envelopes only; translucent volumes are not fabricated parts.
// Primary left: preserve twist, maximum6mm round bundle,≤1.3mmstrap; maxstack7.3mm.
color([0,0.4,1,.2])translate([-7,25,-16.3])cube([7,17,7.3]);
// Side parked T18R head volumes; do not put3.7mmheads beneath6mmbundle.
for(y=[31,37])color([0,0.8,1,.3])translate([-12,y-2.6,-12.9])cube([5.2,5.2,3.9]);
// Right coax service loop only; SMA bulkhead carries external coax pull.
for(y=[33,39]){
 color([0,.4,1,.2])translate([61.5,y-.75,-11.8])cube([3,1.5,2.8]);
 color([0,.8,1,.3])translate([60.4,y-2.6,-12.9])cube([5.2,5.2,3.9]);
}
