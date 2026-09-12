# I24 coordinated engineering revision

PR #45 is merged at `f5d13fce5fcfc23a914f7da39e5bb448c56ed6b7`.
The recovered I23 baseline is 139 closed, 147 open and four not applicable
original criteria. MECH-03 was reopened in I23; the I22 mechanical acceptance
is historical. All 290 criteria and 363 prior redlines are preserved.

The final I24 native candidate relocates two complete oil inner-layer chains
between existing through-vias and adjusts LED_CAN around the remaining CAN_TX
reference gaps. Fifteen segments are replaced with twelve. Oil excitation is
6.705 mm shorter, raw oil signal length is unchanged, and the LED chain gains
2.009 mm. No component, via, RF geometry, outline or logo changes.
Independent source clearance/connectivity, all seven track keepouts and the
protected/raw voltage rules pass. Native refill and affected power/thermal
revalidation are pending; source-only evidence is not native acceptance.

Coordinated work also replaces the invalid bonded foil/free-span construction
with realizable terminals, free spans, landing pockets and supports. Mechanical
geometry is not accepted until the coupled force, heat path, component, cable,
RF and service clearances have been checked. Thermal source/contact topology
is being audited against the full 4.815 W, 65 C air and 70 C landings case.
The selected Adafruit 851/960 accessories remain unchanged. The linked 851
60 C operating limit does not cover the design environment.

The native workflow retains its pinned toolchain and all existing gates; its
same-repository branch allowlist now also includes this continuation branch.
This is an engineering candidate. No fabrication, flashing or hardware action
is authorized or represented as completed. Final evidence and register update
follow after native results and independent engineering review.
