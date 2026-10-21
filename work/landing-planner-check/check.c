#include <assert.h>
#include <math.h>
#include "planner.h"
int main(void) {
 struct planner old, held;
 plan_init(&old); plan_init(&held);
 plan_land(&old,mkvec(0,0,.5),.2,.004,.2,4,0);
 plan_land(&held,mkvec(0,0,.5),.2,.004,.2,4,0);
 plan_current_goal(&old,4.1);
 assert(plan_is_stopped(&old));
 assert(plan_go_to(&held,false,false,mkvec(0,0,.004),.2,2,2)==0);
 struct traj_eval e=plan_current_goal(&held,8);
 assert(!plan_is_stopped(&held));
 assert(held.state==TRAJECTORY_STATE_FLYING);
 assert(fabsf(e.pos.z-.004)<.0001);
 assert(fabsf(e.yaw-.2)<.0001);
 return 0;
}
