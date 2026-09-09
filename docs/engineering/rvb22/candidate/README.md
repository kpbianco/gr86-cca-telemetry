# RVB22 I07 native compatibility candidate

The cad directory contains I07: the complete I06/I04 PCB records are preserved, with line breaks added to satisfy KiCad's line-reader limit, plus two C532 local labels promoted to the existing global GND/OIL_EFUSE_OUT nets. The full PCB parsed tree is identical to I06; no component, trace, via, zone or outline object changed. Native netlist parity and refill/ERC/DRC remain the required verification for this iteration.

PCB SHA256: 0cefceab108aeaa3db48dbdb003e0e42bef438f44c7a464f5e880ef82fe10f33. See verification/I07_SOURCE_DELTA.json and verification/I07_CANDIDATE_MANIFEST.json for the exact source change. The earlier I06 manifest and source-geometry reports are historical evidence; the parsed-tree identity establishes their unchanged geometry scope.

Firmware source is unchanged and has now compiled successfully using ArduinoCLI1.3.1, ESP32core3.3.6 and NimBLE2.3.6. The verified application SHA256 is5733442b4f66b6ec2ec60ce9e3fc9823fb709a0ddd5bf643f10bdd0027a11880. This is a target build, not live execution or flashing. See verification/TARGET_BUILD_VERIFICATION.json.

The actual hosted native route replaces the earlier missing-runtime condition. Its failed setup/path/parser attempts and fixes remain documented. I07 is being evaluated by the same pinned job; it is not a fabrication release. Historical FINAL_GATES and original criterion counts are pending reconciliation to the completed native evidence.

The C03 carrier, T01 contact, sensor/ASC harness and RF setup are unchanged. T02 now models four board layers and actual barrel locations, with provisional pours and explicit mesh sensitivity. Package, exact native fill, installed thermal, accessory and supplier conditions remain. No ordering, merge or hardware operation is performed by this review job.

Run the portable checker from the repository root with an already provisioned pinned environment:

```bash
python docs/engineering/rvb22/candidate/run_native_candidate.py \
  --cad-dir docs/engineering/rvb22/candidate/cad \
  --firmware-dir docs/engineering/rvb22/candidate/firmware \
  --output native_I07_review \
  --pcbnew-python /path/to/python-with-pcbnew \
  --arduino-cli /path/to/arduino-cli \
  --arduino-config /path/to/arduino-cli.yaml \
  --nimble-dir /path/to/NimBLE-Arduino --component-step
```
