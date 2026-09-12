# I24 WCA-07 optical disposition

Status: **OPEN — no justified desktop-only closure**

The fitted status LED family is Kingbright `APT1608SGC`. Its current datasheet specifies a minimum luminous intensity of 5 mcd and typical 12 mcd at **20 mA**. The existing 1 kΩ GPIO-driven branches bound electrical loading, but the board does not establish 20 mA LED current; therefore the datasheet's guaranteed 20 mA intensity cannot be transferred to the actual operating point.

A low-current same-size candidate, Kingbright `APT1608LZGCK`, was also screened. It specifies minimum 50 mcd / typical 100 mcd at 2 mA and forward voltage 2.2 V minimum, 2.65 V typical, 3.0 V maximum at 2 mA. This is promising optically but is not adopted as a paper-only fix: the ESP32-S3 datasheet specifies VOH >= 0.8*VDD (2.64 V at 3.3 V) under its stated DC characteristic, while the 40 mA high-level source-current figure is typical rather than a guaranteed minimum-current source specification. The available specifications therefore do not prove 2 mA through the LED over the full required electrical/temperature corners.

Disposition: retain `WCA-07` OPEN. A later design change may use a guaranteed-current LED driver/sink or an LED/resistor combination with a supplier-guaranteed intensity at a provable current. Otherwise close this criterion by measured worst-case hot/low-rail visibility on the actual assembly. Do not alter the present LED BOM solely to manufacture a desktop closure.

Sources:
- https://www.kingbrightusa.com/images/catalog/spec/apt1608sgc.pdf
- https://www.kingbrightusa.com/images/catalog/spec/apt1608lzgck.pdf
- https://documentation.espressif.com/esp32_s3_datasheet_en.pdf

No PCB/BOM change is made by this disposition.
