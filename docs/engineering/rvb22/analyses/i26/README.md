# I26-A — three desktop dispositions completed; GND-02 still actionable

Date: 2026-09-12. Continue PR46 / `codex/rvb24-coordinated-closure`.
Starting HEAD: `48b10ee09697689ce6ad608b00116bef4b950925`. Revision-bound source recovery: `98e6657faf77dbdbe85091d139f52c96a33a2203`.

**Not zero actionable work. Not fabrication release.** No CAD, BOM, firmware, cooling hardware or installed configuration was changed. No physical testing, supplier acceptance, flashing, CAN transmission or vehicle operation was performed.

## Controlled disposition

| Criterion | Desktop outcome | Remaining |
|---|---|---|
| WCA-07 | Seven LED chains electrically bounded; explicitly conditional optical inference complete | Actual hot/cold current, temperature, photometry and dashboard recognition |
| REG-02 | Reconciled to I25/C04 coupled release model | Current, efficiency/loss and thermal confirmation |
| THERM-02 | I25 model complete to its stated conditional desktop limit | Actual local air, component temperatures and hot-spot correlation |
| GND-02 | All 54 historical findings reconciled; active-net scope widened | Return-transfer, etched-corridor and source/receiver margin proof |

All 290 original questions, required evidence and literal closure states are retained: 140 closed, 146 open, four N/A. The 146 literal open rows are not 146 PCB defects. Computed prehardware grouping: 141 desktop/source/documentary, 30 conditional-model, 114 hardware/supplier/installation, four N/A and **one actionable desktop criterion**.

## Return-path result

`HISTORICAL_54.csv` retains all original UUIDs, nets, coordinates and historical gaps. `return_review.py` checks those geometries against hash-locked native filled copper and records individual findings, exact reference layers and nearby ground-via identities.

**48 historical interruptions were removed by the prior coordinated copper revision. Six remain on CAN_TX_MCU upstream of the R301 DNP link**, totaling 1.341746 mm. They are not populated receive-path defects. Zero incremental detour from a removed historical void does not mean zero trace/via loop inductance. The retained 12-segment RF reference analysis remains separate.

The expanded scope correctly includes `GPS_TX_MODULE_BUFFERED` (GPIO18 receive), rather than confusing it with `GPS_TX_MCU` (GPIO17 command output). It also includes CAN_RXD before R302 and reset, boot and indicator/control paths.

Two active CAN_RXD B.Cu/In2 findings total 1.918029 mm: UUID `d3091ee3-2292-4281-a911-91513c53927e` at (21.8,10.475)..(21.8,11.5), and UUID `f00c8413-4b97-4145-b77b-21abb2261648` at (22.5,12.2)..(25.175,12.2). They are before the fitted 1 kohm MCU input resistor. Other findings are two CAN_TXD, fourteen ESP_EN, three ESP_GPIO0 and seven GPS_SEARCH_BUFFER; their applicability depends on actual source/receiver roles, not bit frequency alone.

Sixty signal-via transfers are individually enumerated. An exploratory 0.1 mm eroded-copper search found 53 candidate paired corridors and seven unresolved finite searches. **Not 53 passes and not seven defects.** Endpoint snapping, etched width, via-cap geometry and source/receiver margins remain unverified. The thin-filament inductance and selected 10/20/40 mA, 0.5/1/2/5 ns sensitivities are not guaranteed source limits, extracted impedance or settled-waveform proof.

Next: validate actual return corridors and source/receiver overshoot, clamp and settling margins; resolve finite-search failures; coordinate only justified routing/stitching changes, then rerun the complete affected review. Do not reroute merely to erase an isolated TX-stub count or close GND-02 because another ground plane exists.

## LED and thermal bounds

All seven fitted LEDs are APT1608SGC with 1 kohm CRCW06031K00FKEA limiters: six GPIO indicators D601..D606 plus D401 from U404. Actual source route, limiter population and active-high polarity are checked.

At 3.6 V, zero LED Vf, 1% resistor tolerance, 2% temperature allowance and an additional 2% service-drift allowance: **3.786302 mA maximum per LED**, 22.717807 mA for six GPIOs and 26.504108 mA total. LED self-heating is bounded at 3.407671 mW, not the 13.630684 mW maximum resistor/supply allocation.

Forty-eight typical-curve cases predict approximately 0.999844..1.904399 mA. Conditionally scaling the 20 mA optical bin gives 0.164849 mcd axial; **the guaranteed low-current/hot optical minimum remains zero**. Typical ESP32 drive current and high-impedance VOH are not treated as guaranteed loaded hot-output specifications. Retain 1 kohm; lower resistance reduces hot-current margin without creating a guaranteed optical minimum.

Extracted I25 fine-mesh LED neighborhoods plus the 5.616330 C mesh allowance span **79.96..83.63 C**. These are board regions, not local air or LED junction temperatures. The 85 C local-air and 110 C junction limits remain physical gates; the datasheet thermal resistance requiring 16 mm2 pads is not applied to smaller pads as a guarantee.

REG-02 and THERM-02 now use the **2.940859375 W** coupled release source, not the historical 4.815 W stress vector. Retain 160 MHz, +3 dBm BLE, Wi-Fi disabled, 120 notifications/s, <=0.45 A sustained main profile and >=80% efficiency or measured losses no worse than C04. The fine-grid board maximum is 107.266651 C, with the non-converged one-step screening value 112.882981 C. Neither is ESP32 junction or immediate-air temperature.

C02 remains unadopted. U201 immediate air must remain <=85 C at 65 C bulk air. Adafruit851 remains a separate <=60 C accessory-zone requirement. Generated `QUALIFICATION_GATES.json` includes measurement setup, uncertainty treatment, quantitative criteria and firmware/assembly/installation/PCB failure dispositions. It does not claim a fresh audit of every external qualification procedure.

## Verification

Local: 32 I26 tests, 31 existing regression tests and 12 C04 release-profile checks passed. The corrected current 290-row review entry point was executed. CAD and firmware remain hash-identical to the reviewed release design.

Hosted release-native run 34707369723 at 98e6657 passed. A separate unadopted feedback trial, run34707369729, failed on a silkscreen-over-copper warning for text405. That trial is not the release PCB; the warning is not waived and does not justify changing the release board.

`prepare_publication.py verify` retrieves hash-locked native copper and I25 thermal artifacts, regenerates results, applies the scoped legacy-review correction, reconciles the register and runs tests. `prepare-commit` creates only an **unreferenced** scoped Git commit after verification; it never updates a branch, merges, releases or operates hardware. The workflow preserves the prepared commit and complete logs for explicit review before a non-force branch update.

Native filled PCB SHA256: `11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1`. Source PCB SHA256: `5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8`. Changing these inputs invalidates the analysis; never edit hashes to manufacture a pass.
