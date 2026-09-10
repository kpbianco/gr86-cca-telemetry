#include "runtime_config.h"
#include "nvs_cfg.h"
#include <Preferences.h>
#include <cstring>
namespace runtime_config {
bool load(Snapshot& out){
  out=Snapshot{}; // Missing, truncated, incompatible or corrupt -> explicit defaults.
  seal(out);
  Preferences prefs;
  if(!prefs.begin("cca_cfg_b",true))return false;
  Snapshot stored{};
  const size_t length=prefs.getBytesLength("runtime_cfg");
  const size_t read=length==sizeof(stored)?prefs.getBytes("runtime_cfg",&stored,sizeof(stored)):0;
  prefs.end();
  if(read!=sizeof(stored)||!valid(stored))return false;
  out=stored;return true;
}
bool save(Snapshot value){
  seal(value);
  if(!valid(value))return false;
  Preferences prefs;
  if(!prefs.begin("cca_cfg_b",false))return false;
  ScopedNvsWrite guard;
  // One NVS record holds the complete configuration; never merge partial keys.
  const size_t written=prefs.putBytes("runtime_cfg",&value,sizeof(value));
  Snapshot readback{};
  const size_t length=prefs.getBytesLength("runtime_cfg");
  const size_t read=length==sizeof(readback)?prefs.getBytes("runtime_cfg",&readback,sizeof(readback)):0;
  prefs.end();
  return written==sizeof(value)&&read==sizeof(readback)&&valid(readback)&&
         std::memcmp(&value,&readback,sizeof(value))==0;
}
}
