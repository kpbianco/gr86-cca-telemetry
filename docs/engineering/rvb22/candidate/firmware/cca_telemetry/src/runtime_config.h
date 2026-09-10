#pragma once
#include <cstdint>
#include <cstddef>
namespace runtime_config {
constexpr uint32_t kVersion=1;
constexpr size_t kMaxDividers=64;
struct __attribute__((packed)) Divider {uint16_t pid;uint8_t divider;};
struct __attribute__((packed)) Snapshot {
  uint32_t version=kVersion;
  uint8_t profileEnabled=1;
  uint8_t reserved=0;
  uint16_t oilPeriodMs=20;
  uint16_t count=0;
  uint16_t reserved2=0;
  Divider items[kMaxDividers]={};
  uint32_t crc=0;
};
inline uint32_t checksum(const Snapshot& c) {
  uint32_t crc=0xFFFFFFFFu;
  const auto* data=reinterpret_cast<const unsigned char*>(&c);
  for(size_t i=0;i<offsetof(Snapshot,crc);++i){
    crc^=data[i];
    for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^(0xEDB88320u&(0u-(crc&1u)));
  }
  return crc^0xFFFFFFFFu;
}
inline void seal(Snapshot& c){c.crc=checksum(c);}
inline bool valid(const Snapshot& c){
  if(c.version!=kVersion||c.profileEnabled>1||c.reserved||c.reserved2||
     c.oilPeriodMs<10||c.oilPeriodMs>2000||c.count>kMaxDividers||c.crc!=checksum(c))return false;
  for(size_t i=0;i<kMaxDividers;++i){
    if(i<c.count){
      if(c.items[i].pid>0x7FF||c.items[i].divider==0)return false;
      for(size_t j=0;j<i;++j)if(c.items[j].pid==c.items[i].pid)return false;
    }else if(c.items[i].pid||c.items[i].divider)return false;
  }
  return true;
}
bool load(Snapshot& out);
bool save(Snapshot value);
}
