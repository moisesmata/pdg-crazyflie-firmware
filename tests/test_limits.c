#include <assert.h>
#include <math.h>
#include "pdg_limits.h"
int main(void) {
  bool clipped;
  assert(pdgLimitCollective(50000,false,10000,&clipped)==50000 && !clipped);
  assert(pdgLimitCollective(50000,true,40000,&clipped)==40000 && clipped);
  assert(pdgLimitCollective(0,true,0,&clipped)==0 && !clipped);
  assert(pdgLimitCollective(NAN,false,10000,&clipped)==0);
  assert(pdgLimitCollective(-1,false,10000,&clipped)==0);
  assert(pdgLimitCollective(10000,true,NAN,&clipped)==0 && clipped);
  assert(pdgLimitCollective(50000,true,70000,&clipped)==50000 && !clipped);
  return 0;
}
