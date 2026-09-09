#!/usr/bin/env bash
set -euo pipefail
firmware_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cli=${ARDUINO_CLI:-arduino-cli}
cli_args=()
if [[ -n ${ARDUINO_CLI_CONFIG:-} ]]; then
  cli_args+=(--config-file "$ARDUINO_CLI_CONFIG")
fi
fqbn='esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=8M,PartitionScheme=default_8MB,DebugLevel=none,PSRAM=enabled,LoopCore=1,EventsCore=1,EraseFlash=none,UploadSpeed=115200'
"$cli" "${cli_args[@]}" compile --fqbn "$fqbn" --warnings all \
  --build-path "${FIRMWARE_BUILD_DIR:-/tmp/gr86_revb_build}" \
  --output-dir "$firmware_dir/images" "$firmware_dir/cca_telemetry"
# Arduino --output-dir omits boot_app0; the installed pinned core contains it.
if [[ -n ${ARDUINO_DATA_DIR:-} ]]; then
  cp "$ARDUINO_DATA_DIR/packages/esp32/hardware/esp32/3.3.6/tools/partitions/boot_app0.bin" \
     "$firmware_dir/images/boot_app0.bin"
fi
