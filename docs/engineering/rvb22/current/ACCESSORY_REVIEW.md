# I21 Adafruit accessory check

The user selected an Adafruit U.FL-to-SMA adapter and external GPS puck by product type. Product IDs 851 and 960 remain inferred, pending the actual item labels or order links.

The current [851 product page](https://www.adafruit.com/product/851) describes a 150 mm RG178 cable with a panel SMA connector. Its linked [C934-001 drawing](https://cdn-shop.adafruit.com/product-files/851/C934-001_datasheet.pdf) still specifies −10 to +60°C operation and labels the interface RP-SMA. That conflicts with the product-page connector description, and its stated operating range does not cover the model's 65°C dashboard bulk air. This discrepancy was checked again on 2026-09-10.

The [960 product page](https://www.adafruit.com/product/960) and linked [GPS-01 specification](https://cdn-shop.adafruit.com/datasheets/GPS-01.pdf) describe a GPS L1 active puck with a five-metre cable and SMA male connector. The specification gives 2.3–5.5 V and −30 to +85°C operation. Listed antenna currents are typical values, not guaranteed maxima.

Resolve the actual purchased IDs and applicable adapter drawing/temperature rating before accepting that accessory for the declared environment. The board's RF network and mated-envelope calculations remain conditional on their stated dimensions, bias and parasitics. No replacement accessory has been adopted.
