// 根据控制器快照推导对外暴露的生命周期状态。
#ifndef ROBOT_DOMAIN_STATE_MACHINE_H_
#define ROBOT_DOMAIN_STATE_MACHINE_H_

#include "robot/domain/types.h"

namespace robot {

RobotLifecycleState DeriveLifecycleState(const RobotSnapshot& snapshot);
const char* LifecycleStateName(RobotLifecycleState state);

}  // namespace robot

#endif  // ROBOT_DOMAIN_STATE_MACHINE_H_
