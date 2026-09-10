# I24 high-temperature GPS pigtail disposition

Status: **CANDIDATE SUBSTITUTION SELECTED FOR VALIDATION — NOT RELEASED**

## Problem

The user-confirmed Adafruit 851 adapter is incompatible with the retained 65 °C design environment. The linked C934-001 datasheet gives an operating range of -10 to +60 °C. The retained passive thermal case has bulk air at 65 °C, 70 °C landings, nonnegative internal heat generation and no active refrigeration. No contact/spreader improvement can make a cable with a 60 °C operating ceiling universally compliant with a 65 °C minimum environmental boundary.

This is a requirements/component-selection conflict, not a thermal-solver uncertainty. Adafruit 851 remains useful as historical/prototype hardware but is rejected as the Rev B release baseline for the 65 °C case.

## Selected validation candidate

**Samtec MH113-MH1RP-01SB1-0150** is selected for the I24 validation branch because it preserves the electrical/mechanical interface class while removing the temperature conflict:

- End 1: MHF1 type right-angle plug / U.FL-type interface.
- End 2: `01SB1`, standard-polarity SMA straight jack, sealed bulkhead.
- Overall cable length: 150 mm nominal.
- Cable: MH113, 1.13 mm OD miniature coax.
- Impedance: 50 ohm.
- Frequency range: up to 6 GHz for the family/configuration.
- Operating temperature: -40 to +90 °C.
- Minimum cable bend radius: 6.8 mm.
- The exact SB4 bulkhead end drawing identifies a sealed SMA jack for MH113 cable, 8.00 mm body hex, 11.00 mm nut hex, 10.67 mm from mating reference plane in the primary view, 12.60 mm reference overall connector dimension, and a recommended panel cutout for up to 3.80 mm panel thickness. Exact toleranced envelope shall come from the current Samtec drawings/3D model rather than dimensions scaled from a rendering.

Adafruit product 960 terminates in a **standard SMA** connector, so the selected standard-polarity SMA jack is the correct interface family; an RP-SMA cable is not substituted.

## Source set

- Adafruit 851 product page: https://www.adafruit.com/product/851
- Adafruit 851 C934-001 datasheet: https://cdn-shop.adafruit.com/product-files/851/C934-001_datasheet.pdf
- Adafruit 960 product page: https://www.adafruit.com/product/960
- Samtec MH113 family page: https://www.samtec.com/products/mh113-mh1rp-mh1rp-0150
- Samtec MH113 series print: https://suddendocs.samtec.com/prints/mhxxx-xxxxx-xxxxxx-xxxx-mkt.pdf
- Samtec `SMA-J-C-X-ST-SB4` end drawing: https://suddendocs.samtec.com/prints/sma-j-c-x-st-sb4-mkt.pdf
- Exact distributor listing: https://www.digikey.com/en/products/detail/samtec-inc/MH113-MH1RP-01SB1-0150/6691530

## Criterion effect

This substitution removes the known **851 temperature-applicability blocker** from the design path. It does **not** by itself close the original criteria:

- `GPS-09` remains open for guaranteed delivered antenna bias/current and open/short/hot-plug behavior over the applicable temperature range.
- `GPS-14` remains open until the exact MHF1/SMA mechanical envelope, panel retention, strain relief, cable route and service procedure are checked against the current carrier/enclosure without loading the PCB socket as the strain anchor.
- `CFG-09` remains open until the actual installed antenna/harness/phone configuration is recorded.
- The board-level thermal criteria remain open independently of this accessory correction.

## Required I25 mechanical update

Replace the 851 cable solid/envelope in the next mechanical candidate with the exact Samtec assembly envelope. Use 1.13 mm cable OD, 6.8 mm minimum bend radius, the current Samtec MHF1 plug drawing, the `SMA-J-C-X-ST-SB4` bulkhead drawing, and the 150 mm assembly-length tolerance from the series print. Re-run cable motion, connector service, RF metal setback, panel cutout and strain-anchor checks before accepting the substitution.

No PCB CAD change is required for this disposition.
