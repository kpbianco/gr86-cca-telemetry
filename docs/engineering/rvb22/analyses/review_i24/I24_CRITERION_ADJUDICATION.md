# I24 criterion adjudication

Status: **one narrow re-closure; no release claim**

## Accounting basis

I22 ended at 140 closed / 146 open / 4 not applicable. I23 correctly reopened `MECH-03` after discovering that the earlier C05/W02/T03 mechanical construction used invalid assumptions, giving the effective I23 baseline of **139 closed / 147 open / 4 not applicable**.

I24 replaces that invalid construction with the C06/W03/T04 mechanical trial and an independent geometry review. This adjudication evaluates only whether that corrected evidence answers the original wording of `MECH-03`; it does not use the narrow criterion to waive broader installation, material, vibration or thermal acceptance.

## MECH-03 — CLOSED at original scope

Original question:

> Are bottom-side components allowed by standoff height and enclosure clearance?

Original evidence requirement:

> Worst-case tolerance stack includes component heights, solder protrusion, warp and fasteners.

I24's corrected independent review checks the source-effective component poses and the corrected populated/mated geometry using the C06/W03/T04 construction. It reports **9,982 component/service/cable/hot-shoe relationships with zero non-mating interference**. The minimum modeled gaps include 0.101 mm at U401, 0.140 mm for cable versus relieved post, and 0.160 mm for the moving hot shoe versus carrier base. The refinement check from 0.10 to 0.05 mm changes the maximum displacement by less than 0.000018 mm, and all 199 footprint poses match the source.

That evidence directly answers the original `MECH-03` clearance/tolerance-stack question after the I23 correction, so `MECH-03` is re-closed.

### Explicit exclusions

This re-closure does **not** assert any of the following:

- physical installed fit;
- final carrier material qualification;
- retained preload, hot creep or locking-process qualification;
- impact, fatigue or vibration acceptance;
- package/junction thermal acceptance;
- GPS pigtail retention/service acceptance;
- thermal-shoe feasibility or acceptance.

Those remain under their original open criteria (`MECH-01`, `MECH-06`, `MECH-08`, `MECH-09`, `GPS-14`, `REG-02`, `THERM-02` and related rows).

## I24 accounting after this adjudication

- Closed: **140**
- Open: **146**
- Not applicable: **4**
- Total: **290**

This restores the numerical I22 count after the justified I23 reopening; it is **not** represented as net numerical progress versus I22. The evidence basis is newer and corrected.

## Other I24 dispositions that remain open

- `GND-02`: remains open. Six nearest-adjacent-plane CAN_TX interruptions remain even though there are zero simultaneous gaps across both ground planes. Alternate reference availability is not silently converted into acceptable return transfer.
- `REG-02` / `THERM-02`: remain open. The current three-contact mechanical cooling path does not close the board-spreading/package problem; U201 local extraction is only a feasibility direction until the contact stack and exact thermal model are validated.
- `GPS-09` / `GPS-14`: remain open. The Samtec high-temperature pigtail candidate removes the known Adafruit 851 temperature incompatibility from the selected design path, but electrical bias/hot-plug and exact retention/service acceptance still require evidence.
- `WCA-07`: remains open. The present LED network has bounded current loading but no guaranteed low-current/hot optical visibility minimum at the exact operating point.

No physical test, supplier DFM result or fabrication release is claimed by this adjudication.
