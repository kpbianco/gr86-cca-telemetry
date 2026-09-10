// Installation template ONLY. Coordinates are local to the actual mated plug wire-exit plane.
// Local +X is the natural wire-exit direction; it is not a PCB or carrier coordinate.
// Transform this template using the actual mating plug before checking dashboard/tool/cable clashes.
// No tie, twist, bend or significant terminal bias is permitted in the first25.4mm.
// Controlled free length25.4..30mm; following bend radius>=30mm; total harness wire path<=300mm.
// This envelope is external to the basic carrier bounding box and does not prove installed fit.
$fn=64;
module f1_relaxed_exit_envelope(length_mm=30,bundle_diameter_mm=6) {
 assert(length_mm>=25.4 && length_mm<=30);
 assert(bundle_diameter_mm>0 && bundle_diameter_mm<=6);
 color([1,.65,0,.25]) rotate([0,90,0]) cylinder(h=length_mm,r=bundle_diameter_mm/2);
}
// Deliberately no fixed installation placement: actual plug exit/axis must be supplied.
