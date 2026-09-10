#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,re,subprocess,sys,os
candidate=Path(__file__).resolve().parents[1]
root=Path(os.environ.get('CCA_BASELINE_ROOT', str(candidate.parents[2]/'baseline/GR86_CCA_RevB'))).resolve()
s=(candidate/'cca_telemetry/cca_telemetry.ino').read_text()
results=[]
def check(name,ok):
 results.append({'check':name,'passed':bool(ok)})
for pin,value in {'CAN_TX_GPIO':5,'CAN_RX_GPIO':4,'GPS_RX_GPIO':18,'GPS_TX_GPIO':17,'OIL_ADC_PIN':1,'OIL_EXC_ADC_PIN':2}.items():
 check(pin, bool(re.search(r'\b'+pin+r'\s*=\s*'+str(value)+r';',s)))
check('PPS GPIO16', '#define GPS_PPS_GPIO 16' in (candidate/'cca_telemetry/config.h').read_text())
check('one listen-only controller mode',s.count('TWAI_MODE_LISTEN_ONLY')==1 and 'TWAI_MODE_NORMAL' not in s)
check('no CAN transmit API in source','twai_transmit' not in s)
check('zero TX queue','general.tx_queue_len = 0;' in s)
check('500k classical timing','TWAI_TIMING_CONFIG_500KBITS()' in s)
check('old source snapshot preserved',all(hashlib.sha256((root/x['path']).read_bytes()).hexdigest()==x['sha256'] for x in json.loads((root/'firmware/CANDIDATE_MANIFEST.json').read_text())['files']))
command_sources=s+'\n'+(candidate/'cca_telemetry/src/gps_mode_config.h').read_text()
for cmd in re.findall(r'"(\$[^"\n]*\*[0-9A-F]{2}\\r\\n)"',command_sources):
 cmd=cmd.replace('\\r\\n','');body,cs=cmd[1:].split('*');v=0
 for c in body:v^=ord(c)
 check('UART command checksum '+body, v==int(cs,16))
images=candidate/'images'
if not (images/'cca_telemetry.ino.merged.bin').exists():
 report={'status':'SOURCE_CHECKS_PASS_TARGET_UNBUILT' if all(x['passed'] for x in results) else 'FAIL','checks':results,'target_build':'PENDING; no candidate image exists, so no binary release checks passed.'}
 print(json.dumps(report,indent=2))
 sys.exit(0 if '--source-only' in sys.argv and all(x['passed'] for x in results) else 2)
merged=(images/'cca_telemetry.ino.merged.bin').read_bytes()
check('factory image exactly8MiB',len(merged)==8*1024*1024)
for name,offset in [('cca_telemetry.ino.bootloader.bin',0),('cca_telemetry.ino.partitions.bin',0x8000),('boot_app0.bin',0xe000),('cca_telemetry.ino.bin',0x10000)]:
 data=(images/name).read_bytes();check('merged offset '+name, merged[offset:offset+len(data)]==data)
check('embedded build identity',b'2.1.4-revb-recovered-20260909' in (images/'cca_telemetry.ino.bin').read_bytes())
symbols=subprocess.run(['nm','-g',str(images/'cca_telemetry.ino.elf')],check=True,capture_output=True,text=True).stdout
check('ELF contains receive', bool(re.search(r'\bT twai_receive$',symbols,re.M)))
check('ELF has no transmit symbol', not re.search(r'\btwai_transmit',symbols))
report={'status':'PASS' if all(x['passed'] for x in results) else 'FAIL','checks':results}
print(json.dumps(report,indent=2))
sys.exit(0 if report['status']=='PASS' else 1)
