# I14 — programming contact clearance correction

The native I13 run completed with zero ERC/DRC findings, all 153 fitted references in STEP, unchanged source inputs, and a successful firmware build. Independent export verification also matched 1,827 pad/via flashes and all 334 plated plus 10 unplated holes. This is source/export evidence, not a tested assembly.

Manufacturer-specific review then found eight explicit-copper proximity findings at J201 that the generic 0.15 mm rule did not represent. I14 reroutes pins 1, 3, 4 and 6 outward, removes the close pin-3 ground via, moves the pin-4 via, and adds the manufacturer's 0.508 mm foreign-copper clearance as a native rule. All footprints, pad nets, RF routes, logo and other circuit routing remain unchanged. Native fill must demonstrate the pin-3 ground connection and validate the dedicated rule.

Full-load 4.815 W thermal margins, package temperatures, full connector envelopes and the 290-criterion register remain under review. The likely Adafruit 851 cable document is limited to 60 C and conflicts with its product page about SMA polarity; the purchased model is not yet positively identified. A compatible higher-temperature cable option is being dimensioned for final review. No manufacturing release or zero-overall assertion is made.
