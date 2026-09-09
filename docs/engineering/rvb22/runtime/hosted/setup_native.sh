#!/usr/bin/env bash
set -euo pipefail
: "${RUNNER_TEMP:?GitHub runner temporary directory is required}"
phase="${1:?Specify cad or firmware}"
[[ "$phase" == cad || "$phase" == firmware ]]
build_root="${RUNNER_TEMP}/rvb22-native-tools"
mkdir -p "$build_root/bin" "$build_root/config" native_setup
exec > >(tee "native_setup/install-${phase}.log") 2>&1
if [[ "$phase" == cad ]]; then
sudo add-apt-repository --yes --no-update ppa:kicad/kicad-9.0-releases
# The hosted image also contains unrelated vendor repositories. Scope this job's
# APT reads to Ubuntu and KiCad; retain normal signed-index/hash verification.
mkdir -p "$build_root/apt-sources"
/usr/bin/python3 - "$build_root/apt-sources" <<'PYAPT'
from pathlib import Path
import shutil,sys
out=Path(sys.argv[1]);sources=Path('/etc/apt/sources.list.d')
ubuntu=sources/'ubuntu.sources'
assert ubuntu.is_file(), 'Expected Ubuntu distribution source definition'
shutil.copy2(ubuntu,out/ubuntu.name)
selected=[]
for p in sources.iterdir():
 if p.suffix not in ['.sources','.list']:continue
 if 'ppa.launchpadcontent.net/kicad/kicad-9.0-releases/ubuntu' in p.read_text():
  shutil.copy2(p,out/p.name);selected.append(p.name)
assert len(selected)==1, selected
print('Scoped package sources:', ubuntu.name, *selected)
PYAPT
apt_scope=(-o Dir::Etc::sourcelist=/dev/null -o "Dir::Etc::sourceparts=$build_root/apt-sources")
sudo apt-get "${apt_scope[@]}" update
sudo apt-get "${apt_scope[@]}" install --yes --no-install-recommends kicad kicad-symbols kicad-footprints kicad-packages3d xvfb xauth
kicad-cli version
/usr/bin/python3 -c 'import pcbnew; print(pcbnew.GetBuildVersion())'
# Bind legacy source model aliases to the installed, version-recorded package.
test -d /usr/share/kicad/3dmodels
for model_version in 6 7 8 9; do
  printf 'KICAD%s_3DMODEL_DIR=/usr/share/kicad/3dmodels\n' "$model_version" >> "$GITHUB_ENV"
done
dpkg-query -W kicad-packages3d > native_setup/model-package.txt
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
