// gRPC 传输层启动入口，负责认证配置和服务生命周期，不承载业务规则。
#ifndef ROBOT_TRANSPORT_GRPC_SERVER_H_
#define ROBOT_TRANSPORT_GRPC_SERVER_H_

#include <csignal>
#include <string>

#include "robot/service/robot_service.h"
#include "robot/domain/status.h"
#include "robot_manager/transport/grpc_options.h"

namespace robot {

Status RunGrpcServer(const GrpcServerOptions& options,
                     const std::string& robot_id, RobotService* service,
                     const volatile std::sig_atomic_t* stop_requested);

}  // namespace robot

#endif  // ROBOT_TRANSPORT_GRPC_SERVER_H_
