// 机器人连接、运动、状态和安全策略等领域基础数据类型。
#ifndef ROBOT_DOMAIN_TYPES_H_
#define ROBOT_DOMAIN_TYPES_H_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot {

enum class RobotLifecycleState {
  kDisconnected,
  kConnecting,
  kStandby,
  kReady,
  kExecuting,
  kPaused,
  kFault,
  kEmergencyStop,
  kUnknown,
};

enum class ControllerMode {
  kManual,
  kManualFast,
  kAutomatic,
  kUnknown,
};

enum class StopMode {
  kControlled,
  kQuick,
  kPowerOffRequest,
};

enum class JogFrame {
  kJoint,
  kRobot,
  kTool,
  kUser,
  kWorld,
};

enum class JogDirection { kNegative = -1, kPositive = 1 };

struct ConnectionOptions {
  std::string robot_id;
  std::string address;
  // Opaque vendor model identifier. The selected adapter validates it.
  std::string expected_model;
  // Zero means the adapter decides; non-zero requests an exact axis count.
  std::size_t expected_axis_count = 0;
  bool verify_sdk_version = true;
  bool sdk_debug_logging = false;
};

struct JointPosition {
  std::vector<double> degrees;
};

struct CartesianPose {
  // 位置单位为 mm，姿态欧拉角单位为 degree；configuration 与 turn 计数
  // 保留控制器逆解分支信息，避免同一笛卡尔位姿落到不同关节构型。
  double x_mm = 0.0;
  double y_mm = 0.0;
  double z_mm = 0.0;
  double a_deg = 0.0;
  double b_deg = 0.0;
  double c_deg = 0.0;
  int configuration = 0;
  int joint_1_turn = 0;
  int joint_4_turn = 0;
  int joint_6_turn = 0;
};

struct JointMotionProfile {
  int speed_percent = 10;
  double blend = 0.0;
};

struct CartesianMotionProfile {
  int speed_mm_per_second = 100;
  double blend_mm = 0.0;
};

struct JointTarget {
  JointPosition position;
  std::string tool_name = "tool0";
  std::string work_object_name = "wobj0";
};

struct CartesianTarget {
  CartesianPose pose;
  std::string tool_name = "tool0";
  std::string work_object_name = "wobj0";
};

struct CircularTarget {
  CartesianPose via;
  CartesianPose target;
  std::string tool_name = "tool0";
  std::string work_object_name = "wobj0";
};

struct JogCommand {
  JogFrame frame = JogFrame::kJoint;
  std::uint32_t axis = 1;
  JogDirection direction = JogDirection::kPositive;
  bool start = true;
};

struct Alarm {
  int code = 0;
  int severity = 0;
  std::string occurred_at;
  std::string message;
};

struct RobotSnapshot {
  // 状态由轮询线程生成后整体复制给读者。observed_at 使用 steady_clock，
  // 可安全计算新鲜度，不受系统时间校准影响。
  std::uint64_t sequence = 0;
  std::chrono::steady_clock::time_point observed_at;
  RobotLifecycleState lifecycle = RobotLifecycleState::kUnknown;
  ControllerMode controller_mode = ControllerMode::kUnknown;
  bool connected = false;
  bool api_control = false;
  bool servo_on = false;
  bool emergency_stop = false;
  bool alarm_active = false;
  bool moving = false;
  bool paused = false;
  bool program_running = false;
  unsigned int speed_ratio = 0;
  double tcp_speed_mm_per_second = 0.0;
  std::string controller_model;
  std::size_t axis_count = 0;
  std::string active_tool;
  std::string active_work_object;
  JointPosition joints;
  CartesianPose tcp_pose;
};

struct JointLimit {
  double minimum_deg = 0.0;
  double maximum_deg = 0.0;
};

struct CartesianWorkspace {
  double minimum_x_mm = 0.0;
  double maximum_x_mm = 0.0;
  double minimum_y_mm = 0.0;
  double maximum_y_mm = 0.0;
  double minimum_z_mm = 0.0;
  double maximum_z_mm = 0.0;
};

struct SafetyPolicy {
  // 这些是站点核验后的软件保护边界，不替代急停、围栏、STO 等硬件安全回路。
  bool motion_enabled = false;
  bool program_execution_enabled = false;
  bool joint_limits_verified = false;
  bool cartesian_workspace_verified = false;
  int maximum_joint_speed_percent = 20;
  int maximum_linear_speed_mm_per_second = 500;
  std::vector<JointLimit> joint_limits;
  CartesianWorkspace cartesian_workspace;
  std::string cartesian_workspace_frame = "wobj0";
  std::vector<std::uint32_t> writable_digital_outputs;
  std::vector<std::string> approved_programs;
};

}  // namespace robot

#endif  // ROBOT_DOMAIN_TYPES_H_
