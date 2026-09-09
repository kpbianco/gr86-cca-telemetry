#include "../cca_telemetry/src/io_budget.h"
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <deque>
using cca::io::TokenBucket;
int main() {
 uint64_t assertions=0;
 for(uint32_t origin: {0u,1u,0xffffff00u}) for(uint32_t step: {1u,2u,5u,10u,17u,100u}) {
  TokenBucket b(24,120);b.reset(origin);uint64_t granted=0;
  while(b.take(origin)){++granted;}
  assert(granted==24);++assertions;
  for(uint32_t dt=step;dt<=1000000;dt+=step){
   while(b.take(origin+dt))++granted;
   assert(granted<=24u+uint64_t(dt)*120u/1000u);++assertions;
   assert(b.credit()<1000u);++assertions;
  }
 }
 TokenBucket exact(24,120);exact.reset(0);while(exact.take(0)){}
 uint32_t replenished=0;for(uint32_t t=1;t<=10000;++t)while(exact.take(t))++replenished;
 assert(replenished==1200);++assertions;
 // Random periods, saturation/idle and rollover; conservative cumulative envelope.
 std::mt19937 rng(0x86b19);TokenBucket b(24,120);uint32_t now=0xffff0000u;b.reset(now);uint64_t time=0,grants=0;
 for(int i=0;i<200000;++i){uint32_t dt=rng()%1500;time+=dt;now+=dt;for(unsigned j=0,n=rng()%50;j<n;++j)if(b.take(now))++grants;assert(grants<=24+time*120/1000);++assertions;}
 // Exact bounded queue versus independent deque oracle under pressure/wrap.
 cca::io::ByteQueue<127> queue;std::deque<uint8_t> oracle;uint64_t dropped=0;size_t high=0;
 for(int i=0;i<200000;++i){
  if(rng()%2){std::vector<uint8_t> bytes(rng()%250);for(auto&c:bytes)c=uint8_t(rng());size_t accepted=std::min<size_t>(bytes.size(),127-oracle.size());assert(queue.append(bytes.data(),bytes.size())==accepted);++assertions;for(size_t j=0;j<accepted;++j)oracle.push_back(bytes[j]);dropped+=bytes.size()-accepted;high=std::max(high,oracle.size());}
  else{size_t n=std::min<size_t>(rng()%80,queue.contiguousSize());const uint8_t*p=queue.contiguousData();for(size_t j=0;j<n;++j){assert(p[j]==oracle.front());++assertions;oracle.pop_front();}queue.consume(n);}
  assert(queue.size()==oracle.size()&&queue.dropped()==dropped&&queue.highWater()==high);++assertions;
 }
 printf("VERIFIED assertions=%llu exact_replenished_10s=%u\n",(unsigned long long)assertions,replenished);
}
