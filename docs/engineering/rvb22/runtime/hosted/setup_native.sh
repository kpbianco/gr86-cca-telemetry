#!/usr/bin/env bash
set -euo pipefail
: "${RUNNER_TEMP:?GitHub runner temporary directory is required}"
phase="${1:?Specify cad or firmware}"
[[ "$phase" == cad || "$phase" == firmware ]]
build_root="${RUNNER_TEMP}/rvb22-native-tools"
mkdir -p "$build_root/bin" "$build_root/config" native_setup
exec > >(tee "native_setup/install-${phase}.log") 2>&1
if [[ "$phase" == cad ]]; then
sudo add-apt-repository --yes ppa:kicad/kicad-9.0-releases
sudo apt-get update
sudo apt-get install --yes --no-install-recommends kicad kicad-symbols kicad-footprints kicad-packages3d xvfb xauth
kicad-cli version
/usr/bin/python3 -c 'import pcbnew; print(pcbnew.GetBuildVersion())'
exit 0
fi
cd "$build_root"
curl --fail --location --retry 3 --output arduino-cli_1.3.1_Linux_64bit.tar.gz https://github.com/arduino/arduino-cli/releases/download/v1.3.1/arduino-cli_1.3.1_Linux_64bit.tar.gz
curl --fail --location --retry 3 --output 1.3.1-checksums.txt https://github.com/arduino/arduino-cli/releases/download/v1.3.1/1.3.1-checksums.txt
/usr/bin/python3 - <<'PY'
import hashlib,pathlib
name='arduino-cli_1.3.1_Linux_64bit.tar.gz'
rows=[r.split() for r in pathlib.Path('1.3.1-checksums.txt').read_text().splitlines() if r.strip()]
expected=[r[0] for r in rows if len(r)==2 and r[1].lstrip('*')==name]
assert len(expected)==1, 'Exactly one official checksum required'
actual=hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest()
assert actual==expected[0], 'Arduino CLI archive checksum mismatch'
print('Arduino CLI 1.3.1 archive checksum verified:',actual)
PY
tar -xzf arduino-cli_1.3.1_Linux_64bit.tar.gz -C bin arduino-cli
cli="$build_root/bin/arduino-cli"
config="$build_root/config/arduino-cli.yaml"
"$cli" config init --dest-dir "$build_root/config"
"$cli" --config-file "$config" config set directories.data "$build_root/data"
"$cli" --config-file "$config" config set directories.user "$build_root/user"
"$cli" --config-file "$config" config set directories.downloads "$build_root/downloads"
"$cli" --config-file "$config" config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
"$cli" --config-file "$config" config set network.connection_timeout 1200s
"$cli" --config-file "$config" core update-index
"$cli" --config-file "$config" core install esp32:esp32@3.3.6
"$cli" --config-file "$config" lib install NimBLE-Arduino@2.3.6
"$cli" version
"$cli" --config-file "$config" core list
