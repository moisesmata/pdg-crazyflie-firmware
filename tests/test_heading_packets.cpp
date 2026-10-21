#include <cassert>
#include "crazyflie_cpp/crtp.h"

template<class T> void check(unsigned command) {
  T p(0x12,0.25f,4.0f);
  assert(p.port()==8 && p.channel()==0 && p.payloadSize()==15);
  // Compare every payload byte to Python/cflib '<BBff?f' protocol layout.
  const unsigned char expected[]={static_cast<unsigned char>(command),0x12,0,0,0x80,0x3e,0,0,0,0,1,0,0,0x80,0x40};
  assert(std::memcmp(p.payload(),expected,sizeof(expected))==0);
}
int main() {
  check<crtpCommanderHighLevelTakeoffRequest>(7);
  check<crtpCommanderHighLevelLandRequest>(8);
}
