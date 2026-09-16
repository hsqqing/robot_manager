# robot-manager 上层应用集成说明

本文说明上层应用如何使用 `robot-manager` 控制机械臂，以及一条请求在服务内部的完整调用过程。

本文面向以下应用：

- HMI、示教界面和操作面板；
- MES、WMS、产线任务编排服务；
- 视觉定位、抓取和上下料应用；
- 自动化测试程序；
- 机器人状态监控和报警系统。

`robot-manager` 对外提供 gRPC + Protobuf 接口。上层应用不应直接调用 EFORT SDK，也不应与机器人控制器建立第二条控制连接。所有控制命令都应该经过本服务，以便统一处理控制权、状态校验、软限位、速度限制、命令历史和停止逻辑。

## 1. 系统组成

```text
上层应用 / HMI / MES
        │
        │ gRPC + Protobuf
        ▼
robot-manager
        │
        ├─ 控制租约
        ├─ 命令参数和安全校验
        ├─ 命令队列和异步执行
        ├─ 状态轮询和状态流
        └─ IRobotDriver
              │
              └─ EfortDriver
                    │
                    └─ EFORT C++ SDK
                          │
                          └─ 机器人控制器
```

完整接口定义位于 [`api/proto/robot/v1/robot_control.proto`](../api/proto/robot/v1/robot_control.proto)。当前服务实例对应一台机器人，一个实例只接受配置文件中 `robot.id` 对应的 `robot_id`。

## 2. 服务能力概览

| 能力              | gRPC 接口                                    | 是否需要控制租约 |
| ----------------- | -------------------------------------------- | ---------------- |
| 获取控制租约      | `AcquireControlLease`                      | 否               |
| 续租              | `RenewControlLease`                        | 当前租约身份     |
| 释放租约          | `ReleaseControlLease`                      | 当前租约身份     |
| 获取机器人状态    | `GetState`                                 | 否               |
| 持续订阅状态      | `StreamState`                              | 否               |
| 读取报警          | `ReadAlarms`                               | 否               |
| 读取数字输入/输出 | `ReadDigitalInput` / `ReadDigitalOutput` | 否               |
| 提交控制命令      | `SubmitCommand`                            | 大多数命令需要   |
| 查询命令结果      | `GetCommand`                               | 否               |

通过 `SubmitCommand` 支持以下命令：

- 获取/释放机器人 API 控制权；
- 上电、下电、清除普通报警；
- 设置全局速度；
- MoveJ、MoveL、MoveC；
- Hold、Resume、Stop；
- Jog 点动；
- 写数字输出；
- 加载、启动、停止控制器程序。

## 3. 启动服务

### 3.1 Mock 模式

Mock 模式用于不连接真实控制器的本地开发和单元测试：

```bash
cmake --preset linux-mock-debug
cmake --build --preset linux-mock-debug
ctest --preset linux-mock-debug

./build/linux-mock-debug/robot-manager \
  --config config/robots/mock.yaml
```

`linux-mock-debug` 预设关闭了 gRPC，因此该模式主要用于观察状态和运行测试，不适合作为本文 Python gRPC 示例的服务端。

### 3.2 EFORT + gRPC 模式

生产构建使用：

```bash
cmake --preset linux-efort-release
cmake --build --preset linux-efort-release
```

配置文件可以从对应型号的示例复制：

```bash
sudo cp config/robots/er7_900.yaml \
  /etc/robot-manager/robot.yaml
```

然后修改至少以下内容：

```yaml
schema_version: 1

service:
  instance_id: er7-900-01-edge
  grpc_listen: 127.0.0.1:50051
  state_poll_ms: 50
  state_stale_after_ms: 500
  command_queue_capacity: 16
  command_history_capacity: 4096

robot:
  id: er7-900-01
  driver: efort
  address: 192.168.1.13
  model: ER7-900

control_lease:
  ttl_ms: 3000
  jog_heartbeat_timeout_ms: 300

limits:
  motion_enabled: false
  program_execution_enabled: false
  maximum_joint_speed_percent: 20
  maximum_linear_speed_mm_per_second: 500

security:
  grpc:
    allow_insecure_loopback: true
```

开发环境可以使用 `allow_insecure_loopback: true`，但此时监听地址必须是回环地址，例如 `127.0.0.1:50051`。生产环境应关闭明文通信并配置 mTLS，见[第 12 节](#12-mtls-生产环境)。

启动：

```bash
./build/linux-efort-release/robot-manager \
  --config /etc/robot-manager/robot.yaml
```

服务启动时的内部流程是：

```text
main
  └─ LoadServiceConfig
       └─ CreateDriver(mock 或 efort)
            └─ CommandExecutor::Start
                 ├─ IRobotDriver::Connect
                 ├─ IRobotDriver::ReadState
                 ├─ 启动 WorkerLoop
                 └─ 启动 PollLoop
                      └─ RunGrpcServer
```

EFORT 驱动连接时会执行 `ConnectRobot`，读取 `GetCurrentRobotType`，并将实际控制器型号与配置中的 `robot.model` 比较。型号不匹配时服务启动失败。

### 3.3 systemd 部署

安装完成后可以使用：

```bash
sudo systemctl enable robot-manager
sudo systemctl start robot-manager
sudo systemctl status robot-manager
journalctl -u robot-manager -f
```

服务单元位于 [`deploy/systemd/robot-manager.service`](../deploy/systemd/robot-manager.service)。默认启动命令是：

```text
/usr/local/bin/robot-manager --config /etc/robot-manager/robot.yaml
```

## 4. 上层应用必须遵守的调用规则

### 4.1 每条命令都要有幂等键

`SubmitCommand` 要求：

- `client_id` 非空；
- `idempotency_key` 非空；
- `timeout_ms` 为正数，省略时服务使用 30000 ms。

幂等键的作用是让客户端可以安全重试网络请求。相同 `client_id + idempotency_key` 且内容相同的请求会返回原来的 `command_id`，不会重复执行。相同幂等键但请求内容不同会返回 `ALREADY_EXISTS`。

建议格式：

```text
{业务实例}-{动作}-{递增序号}
```

例如：

```text
order-20260813-0001-movej-0007
```

Jog 的心跳请求必须使用新的幂等键。重复发送相同幂等键不会刷新点动 watchdog。

### 4.2 控制命令通常需要控制租约

控制租约用于防止两个上层应用同时操作同一台机器人。默认租约有效期为 3000 ms。

应用必须：

1. 调用 `AcquireControlLease`；
2. 保存返回的 `lease_id`；
3. 周期性调用 `RenewControlLease`；
4. 所有需要租约的命令带上 `lease_id`；
5. 工作结束后调用 `ReleaseControlLease`。

如果租约过期：

- 新命令会被拒绝；
- 已排队的该租约命令会被取消；
- 正在执行的运动会由服务请求受控停止；
- Jog 会因为租约或 heartbeat 超时而停止。

### 4.3 接受命令不等于动作完成

`SubmitCommand` 成功只表示命令进入了服务队列：

```text
SubmitCommand → command_id
```

上层应用必须使用 `GetCommand` 查询最终状态。MoveJ、MoveL、MoveC 和 `start_program` 会等状态轮询确认动作完成后才变为 `SUCCEEDED`。

### 4.4 不要并发提交多个运动命令

服务同一时间只允许一个运动命令处于排队或执行状态。第二个 MoveJ/MoveL/MoveC/StartProgram 会返回 `RESOURCE_EXHAUSTED`。

`Stop` 会被放到队列最前面，并且可以在运动期间执行。应用不应通过快速连续提交多个目标点来实现轨迹跟踪；需要轨迹规划时，应由上层生成经验证的轨迹，并按服务允许的命令模型执行。

## 5. 推荐的标准控制流程

```text
连接 gRPC
   ↓
GetState，确认服务和机器人状态
   ↓
AcquireControlLease
   ↓
SubmitCommand(acquire_control)
   ↓
SubmitCommand(power_on)
   ↓
SubmitCommand(set_global_speed，可选)
   ↓
SubmitCommand(move_j / move_l / move_c)
   ↓
GetCommand，等待 SUCCEEDED/FAILED/STOPPED/TIMED_OUT
   ↓
必要时 SubmitCommand(stop)
   ↓
SubmitCommand(power_off，可选)
   ↓
SubmitCommand(release_control)
   ↓
ReleaseControlLease
```

实际运行中应独立启动一个状态监控任务，并持续续租。状态变为 `FAULT`、`EMERGENCY_STOP`、`UNKNOWN` 或 `state_stale=true` 时，上层应停止发送新运动命令，并根据现场安全策略处理。

## 6. Python 客户端准备

### 6.1 安装依赖

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install grpcio grpcio-tools protobuf
```

### 6.2 生成 Python gRPC 代码

在项目根目录执行：

```bash
mkdir -p /tmp/robot_manager_python
python -m grpc_tools.protoc \
  -I api/proto \
  --python_out=/tmp/robot_manager_python \
  --grpc_python_out=/tmp/robot_manager_python \
  api/proto/robot/v1/robot_control.proto
```

生成的文件包括：

```text
/tmp/robot_manager_python/robot/v1/robot_control_pb2.py
/tmp/robot_manager_python/robot/v1/robot_control_pb2_grpc.py
```

客户端运行时需要把生成目录加入 `PYTHONPATH`：

```bash
export PYTHONPATH=/tmp/robot_manager_python:$PYTHONPATH
```

项目没有把生成的语言绑定提交到源码库；不同语言的客户端应根据 proto 文件在自己的构建流程中生成代码。

## 7. Python 完整示例：获取控制权并执行 MoveJ

下面的示例连接本机明文 gRPC 服务，完成：

1. 查询状态；
2. 获取租约；
3. 获取机器人 API 控制权；
4. 上电；
5. 设置全局速度；
6. 执行 MoveJ；
7. 查询命令结果；
8. 停止并释放资源。

注意，仓库中的示例配置默认设置 `motion_enabled: false`，并且没有填写现场关节限位，因此直接使用默认配置运行到 MoveJ 时会被服务拒绝。这是预期的安全行为。只有在现场完成限位、工具、坐标系和低速验收后，才可以按以下形式配置并运行运动命令：

```yaml
limits:
  motion_enabled: true
  joint_limits_deg:
    - -170:170
    - -100:100
    - -150:150
    - -180:180
    - -120:120
    - -360:360
```

上面的数值仅表示配置格式，不能直接作为任何现场的安全限位。真实项目必须替换为经过确认的值。

示例文件可命名为 `robot_client_example.py`：

```python
import sys
import time

import grpc

from robot.v1 import robot_control_pb2 as pb
from robot.v1 import robot_control_pb2_grpc as pb_grpc


ENDPOINT = "127.0.0.1:50051"
ROBOT_ID = "er7-900-01"
CLIENT_ID = "hmi-01"


def wait_command(stub, command_id, timeout_seconds=40):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        response = stub.GetCommand(
            pb.GetCommandRequest(
                robot_id=ROBOT_ID,
                command_id=command_id,
            ),
            timeout=5,
        )
        state_name = pb.CommandState.Name(response.state)
        print(
            f"command={response.command_id} state={state_name} "
            f"message={response.error_message!r}"
        )

        if response.state in (
            pb.COMMAND_STATE_SUCCEEDED,
            pb.COMMAND_STATE_FAILED,
            pb.COMMAND_STATE_CANCELLED,
            pb.COMMAND_STATE_TIMED_OUT,
            pb.COMMAND_STATE_STOPPED,
        ):
            return response
        time.sleep(0.1)
    raise TimeoutError(f"command {command_id} did not finish")


def submit_command(stub, lease_id, key, **command_field):
    request = pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key=key,
        timeout_ms=30000,
        **command_field,
    )
    return stub.SubmitCommand(request, timeout=5)


def wait_state(stub, predicate, timeout_seconds=5):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        state = stub.GetState(
            pb.GetStateRequest(robot_id=ROBOT_ID),
            timeout=2,
        )
        if predicate(state):
            return state
        time.sleep(0.1)
    raise TimeoutError("robot state did not reach the expected condition")


def main():
    channel = grpc.insecure_channel(ENDPOINT)
    stub = pb_grpc.RobotControlServiceStub(channel)
    lease_id = None

    try:
        state = stub.GetState(pb.GetStateRequest(robot_id=ROBOT_ID), timeout=5)
        print(
            f"robot={state.robot_id} model={state.controller_model} "
            f"lifecycle={pb.LifecycleState.Name(state.lifecycle)} "
            f"connected={state.connected} servo_on={state.servo_on} "
            f"moving={state.moving} stale={state.state_stale}"
        )
        if not state.connected or state.state_stale:
            raise RuntimeError("robot is disconnected or state is stale")

        lease = stub.AcquireControlLease(
            pb.AcquireControlLeaseRequest(
                robot_id=ROBOT_ID,
                client_id=CLIENT_ID,
            ),
            timeout=5,
        )
        lease_id = lease.lease_id
        print(f"lease={lease_id} expires_in_ms={lease.expires_in_ms}")

        accepted = submit_command(
            stub,
            lease_id,
            "hmi-01-acquire-control-0001",
            acquire_control=pb.NoPayload(),
        )
        result = wait_command(stub, accepted.command_id)
        if result.state != pb.COMMAND_STATE_SUCCEEDED:
            raise RuntimeError(
                f"acquire_control failed: {result.error_message}"
            )
        # Command completion and the next state-poll update are separate
        # events. Wait until the published snapshot shows API control.
        wait_state(stub, lambda current: current.api_control)

        accepted = submit_command(
            stub,
            lease_id,
            "hmi-01-power-on-0001",
            power_on=pb.NoPayload(),
        )
        result = wait_command(stub, accepted.command_id)
        if result.state != pb.COMMAND_STATE_SUCCEEDED:
            raise RuntimeError(f"power_on failed: {result.error_message}")
        wait_state(stub, lambda current: current.servo_on)

        accepted = submit_command(
            stub,
            lease_id,
            "hmi-01-speed-0001",
            set_global_speed=pb.SetGlobalSpeed(ratio=10),
        )
        result = wait_command(stub, accepted.command_id)
        if result.state != pb.COMMAND_STATE_SUCCEEDED:
            raise RuntimeError(f"set speed failed: {result.error_message}")

        move = pb.MoveJ(
            target=pb.JointPosition(
                degrees=[0.0, -20.0, 30.0, 0.0, 45.0, 0.0]
            ),
            speed_percent=10,
            blend=0.0,
            tool_name="tool0",
            work_object_name="wobj0",
        )
        accepted = submit_command(
            stub,
            lease_id,
            "hmi-01-movej-0001",
            move_j=move,
        )
        result = wait_command(stub, accepted.command_id, timeout_seconds=60)
        if result.state != pb.COMMAND_STATE_SUCCEEDED:
            raise RuntimeError(f"MoveJ failed: {result.error_message}")

        print("MoveJ completed")

    except grpc.RpcError as error:
        print(
            f"gRPC error: code={error.code()} details={error.details()}",
            file=sys.stderr,
        )
        raise
    finally:
        if lease_id is not None:
            # If the process is interrupted during motion, stop first. The
            # service also stops active motion when the lease is released, but
            # an explicit stop makes the cleanup intent clear to the client.
            try:
                current = stub.GetState(
                    pb.GetStateRequest(robot_id=ROBOT_ID), timeout=2
                )
                if current.moving or current.program_running:
                    accepted = submit_command(
                        stub,
                        lease_id,
                        "hmi-01-cleanup-stop-0001",
                        stop=pb.Stop(mode=pb.STOP_MODE_CONTROLLED),
                    )
                    wait_command(stub, accepted.command_id, timeout_seconds=10)
            except grpc.RpcError:
                pass
            try:
                accepted = submit_command(
                    stub,
                    lease_id,
                    "hmi-01-release-control-command-0001",
                    release_control=pb.NoPayload(),
                )
                wait_command(stub, accepted.command_id, timeout_seconds=10)
            except grpc.RpcError:
                pass
            try:
                stub.ReleaseControlLease(
                    pb.ReleaseControlLeaseRequest(
                        robot_id=ROBOT_ID,
                        client_id=CLIENT_ID,
                        lease_id=lease_id,
                    ),
                    timeout=5,
                )
            except grpc.RpcError:
                pass
        channel.close()


if __name__ == "__main__":
    main()
```

运行：

```bash
PYTHONPATH=/tmp/robot_manager_python:$PYTHONPATH \
python robot_client_example.py
```

注意：示例中的关节角度只是接口格式示例，不代表任何现场可以安全执行的目标。真实机器人必须使用现场验证过的目标、工具、工件坐标系和关节限位。

## 8. Python 示例：控制租约续期

租约默认只有 3 秒。长时间任务不能只在开始时获取一次租约，应在后台线程或异步任务中续租：

```python
import threading
import time


def renew_loop(stub, robot_id, client_id, lease_id, stop_event):
    # 续租间隔应明显小于 ttl_ms，例如 ttl=3000 ms 时每 1000 ms 续租。
    while not stop_event.wait(1.0):
        try:
            lease = stub.RenewControlLease(
                pb.RenewControlLeaseRequest(
                    robot_id=robot_id,
                    client_id=client_id,
                    lease_id=lease_id,
                ),
                timeout=2,
            )
            print(f"lease renewed, expires_in_ms={lease.expires_in_ms}")
        except grpc.RpcError as error:
            print(f"lease renewal failed: {error.details()}")
            # 续租失败时不应继续发送运动命令。
            stop_event.set()


# 获取 lease 后启动：
stop_event = threading.Event()
renew_thread = threading.Thread(
    target=renew_loop,
    args=(stub, ROBOT_ID, CLIENT_ID, lease_id, stop_event),
    daemon=True,
)
renew_thread.start()

# 任务结束：
stop_event.set()
renew_thread.join(timeout=2)
```

实际应用应把续租失败视为控制流程失败，并根据状态确认机器人是否已停止。

## 9. 各类命令示例

以下代码假设已经创建：

```python
channel = grpc.insecure_channel("127.0.0.1:50051")
stub = pb_grpc.RobotControlServiceStub(channel)
lease_id = "从 AcquireControlLease 返回的 lease_id"
```

### 9.1 获取和释放 API 控制权

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-acquire-001",
        timeout_ms=10000,
        acquire_control=pb.NoPayload(),
    ),
    timeout=5,
)
result = wait_command(stub, accepted.command_id)
```

释放控制权：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-release-001",
        timeout_ms=10000,
        release_control=pb.NoPayload(),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)

stub.ReleaseControlLease(
    pb.ReleaseControlLeaseRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
    ),
    timeout=5,
)
```

### 9.2 上电和下电

```python
# 上电
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-power-on-001",
        timeout_ms=10000,
        power_on=pb.NoPayload(),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)

# 下电
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-power-off-001",
        timeout_ms=10000,
        power_off=pb.NoPayload(),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)
```

### 9.3 设置全局速度

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-speed-001",
        timeout_ms=10000,
        set_global_speed=pb.SetGlobalSpeed(ratio=10),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)
```

最终速度必须同时满足：服务配置中的最大速度、EFORT SDK 允许的 `1..100` 范围以及现场安全要求。

### 9.4 MoveJ

```python
move = pb.MoveJ(
    target=pb.JointPosition(
        degrees=[10.0, -15.0, 25.0, 0.0, 40.0, 5.0]
    ),
    speed_percent=10,
    blend=0.0,
    tool_name="tool0",
    work_object_name="wobj0",
)

accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-movej-002",
        timeout_ms=60000,
        move_j=move,
    ),
    timeout=5,
)
result = wait_command(stub, accepted.command_id, timeout_seconds=70)
```

MoveJ 要求：

- `degrees` 必须正好包含 6 个值；
- 角度单位为度；
- `speed_percent` 为百分比；
- `blend` 必须大于等于 0；
- 六个关节目标必须在配置的 `joint_limits_deg` 范围内；
- 机器人必须处于自动模式、Ready、伺服开启且没有报警或急停。

### 9.5 MoveL

```python
move = pb.MoveL(
    target=pb.CartesianPose(
        x_mm=450.0,
        y_mm=0.0,
        z_mm=550.0,
        a_deg=180.0,
        b_deg=0.0,
        c_deg=180.0,
        configuration=0,
        joint_1_turn=0,
        joint_4_turn=0,
        joint_6_turn=0,
    ),
    speed_mm_per_second=50,
    blend_mm=0.0,
    tool_name="tool0",
    work_object_name="wobj0",
)

accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-movel-001",
        timeout_ms=60000,
        move_l=move,
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id, timeout_seconds=70)
```

MoveL 要求配置：

```yaml
limits:
  motion_enabled: true
  joint_limits_deg:
    - -170:170
    - -100:100
    - -150:150
    - -180:180
    - -120:120
    - -360:360
  cartesian_workspace:
    frame: wobj0
    x_mm: -1000:1000
    y_mm: -1000:1000
    z_mm: 0:1500
```

`work_object_name` 必须与 `cartesian_workspace.frame` 完全一致。服务只检查配置的 XYZ 工作空间，不把 SDK 的可达性检查当作安全围栏；实际工作空间必须由现场确认。

### 9.6 MoveC

```python
move = pb.MoveC(
    via=pb.CartesianPose(
        x_mm=400.0,
        y_mm=100.0,
        z_mm=500.0,
        a_deg=180.0,
        b_deg=0.0,
        c_deg=180.0,
    ),
    target=pb.CartesianPose(
        x_mm=450.0,
        y_mm=0.0,
        z_mm=500.0,
        a_deg=180.0,
        b_deg=0.0,
        c_deg=180.0,
    ),
    speed_mm_per_second=50,
    blend_mm=0.0,
    tool_name="tool0",
    work_object_name="wobj0",
)

accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-movec-001",
        timeout_ms=60000,
        move_c=move,
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id, timeout_seconds=70)
```

`via` 和 `target` 都必须在配置的 Cartesian workspace 内，并且会分别经过 EFORT SDK 的目标检查。

### 9.7 Hold、Resume 和 Stop

Hold：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-hold-001",
        timeout_ms=10000,
        hold=pb.NoPayload(),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)
```

Resume：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-resume-001",
        timeout_ms=10000,
        resume=pb.NoPayload(),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)
```

受控停止：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-stop-001",
        timeout_ms=10000,
        stop=pb.Stop(mode=pb.STOP_MODE_CONTROLLED),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)
```

停止命令不要求有效控制租约，但仍要求机器人已连接。它会优先插入队列。EFORT SDK V2.8 没有独立 Quick Stop API，因此：

- `STOP_MODE_CONTROLLED` 使用 `MOVEHOLD` + `MOVECLEAR`；
- `STOP_MODE_POWER_OFF_REQUEST` 执行下电请求；
- `STOP_MODE_QUICK` 会被拒绝。

服务不是安全控制器。急停、STO、安全门和安全光栅必须通过认证的安全硬件实现。

### 9.8 Jog 点动

Jog 需要：

- 有效控制租约；
- API 控制权；
- 机器人处于手动模式；
- 伺服开启；
- 没有急停或报警；
- 已开启运动策略和关节限位；
- 持续发送 heartbeat。

开始点动：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-jog-0001",
        timeout_ms=10000,
        jog=pb.Jog(
            frame=pb.JOG_FRAME_JOINT,
            axis=1,
            direction=1,
            start=True,
        ),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)
```

继续点动时必须使用新幂等键，例如 `client-001-jog-0002`。默认 watchdog 为 300 ms，因此应用应根据网络和系统调度情况，以小于 300 ms 的周期发送新的逻辑命令。

停止点动：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-jog-stop-0001",
        timeout_ms=10000,
        jog=pb.Jog(
            frame=pb.JOG_FRAME_JOINT,
            axis=1,
            direction=1,
            start=False,
        ),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)
```

点动 watchdog 超时后，服务会自动排入内部 Stop。

### 9.9 数字 I/O

读取输入：

```python
value = stub.ReadDigitalInput(
    pb.ReadDigitalIoRequest(robot_id=ROBOT_ID, index=0),
    timeout=5,
)
print(f"DIN[{value.index}]={value.value}")
```

读取输出：

```python
value = stub.ReadDigitalOutput(
    pb.ReadDigitalIoRequest(robot_id=ROBOT_ID, index=0),
    timeout=5,
)
print(f"DOUT[{value.index}]={value.value}")
```

EFORT I/O 索引范围是 `0..175`。写输出前必须在配置中加入白名单：

```yaml
limits:
  writable_digital_outputs:
    - 0
    - 1
```

写输出：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-dout-0001",
        timeout_ms=10000,
        write_digital_output=pb.WriteDigitalOutput(index=0, value=True),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)
```

### 9.10 控制器程序

程序执行是独立的安全策略，必须显式打开并配置批准列表：

```yaml
limits:
  program_execution_enabled: true
  approved_programs:
    - loading-cycle-v1
```

加载程序：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-load-program-0001",
        timeout_ms=30000,
        load_program=pb.Program(name="loading-cycle-v1"),
    ),
    timeout=5,
)
result = wait_command(stub, accepted.command_id)
```

只有当前服务进程成功加载过的批准程序才允许启动：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-start-program-0001",
        timeout_ms=120000,
        start_program=pb.NoPayload(),
    ),
    timeout=5,
)
result = wait_command(stub, accepted.command_id, timeout_seconds=130)
```

停止程序：

```python
accepted = stub.SubmitCommand(
    pb.CommandRequest(
        robot_id=ROBOT_ID,
        client_id=CLIENT_ID,
        lease_id=lease_id,
        idempotency_key="client-001-stop-program-0001",
        timeout_ms=10000,
        stop_program=pb.NoPayload(),
    ),
    timeout=5,
)
wait_command(stub, accepted.command_id)
```

服务不会逐点分析控制器内部程序，因此只能依靠现场审核后的程序白名单。

## 10. 状态读取和状态订阅

### 10.1 单次读取

```python
state = stub.GetState(
    pb.GetStateRequest(robot_id=ROBOT_ID),
    timeout=5,
)

print("lifecycle:", pb.LifecycleState.Name(state.lifecycle))
print("controller mode:", pb.ControllerMode.Name(state.controller_mode))
print("connected:", state.connected)
print("api control:", state.api_control)
print("servo:", state.servo_on)
print("emergency stop:", state.emergency_stop)
print("alarm:", state.alarm_active)
print("moving:", state.moving)
print("paused:", state.paused)
print("speed ratio:", state.speed_ratio)
print("state sequence:", state.sequence)
print("state stale:", state.state_stale)
print("state age ms:", state.state_age_ms)
print("joints:", list(state.joints.degrees))
```

常见生命周期状态：

| 状态               | 含义                                         |
| ------------------ | -------------------------------------------- |
| `DISCONNECTED`   | 未连接控制器                                 |
| `STANDBY`        | 已连接但还未达到运动准备状态                 |
| `READY`          | 可以接受新的运动命令，仍需满足租约和策略检查 |
| `EXECUTING`      | 正在运动或执行程序                           |
| `PAUSED`         | 运动被 Hold                                  |
| `FAULT`          | 普通报警或控制器故障                         |
| `EMERGENCY_STOP` | 急停有效                                     |
| `UNKNOWN`        | 状态读取失败或已过期                         |

`state_stale=true` 时，不应把状态当作安全的实时状态使用。默认状态超过 500 ms 未更新会进入过期处理。

### 10.2 状态流

```python
stream = stub.StreamState(
    pb.StreamStateRequest(
        robot_id=ROBOT_ID,
        maximum_rate_hz=10,
    ),
    timeout=None,
)

try:
    for state in stream:
        print(
            f"seq={state.sequence} "
            f"lifecycle={pb.LifecycleState.Name(state.lifecycle)} "
            f"moving={state.moving} stale={state.state_stale}"
        )
except grpc.RpcError as error:
    print(f"state stream ended: {error.code()} {error.details()}")
```

服务端会把请求的 `maximum_rate_hz` 限制在 `1..50`，不填时默认为 20 Hz。状态流适合 HMI 和监控，不应替代安全硬件信号。

### 10.3 报警

```python
alarms = stub.ReadAlarms(
    pb.GetStateRequest(robot_id=ROBOT_ID),
    timeout=5,
)
for alarm in alarms.alarms:
    print(alarm.code, alarm.severity, alarm.message, alarm.occurred_at)
```

清除普通报警使用 `clear_fault` 命令，但急停仍然有效时服务会拒绝清除。清除报警不等于完成安全复位，也不等于自动恢复运动。

## 11. 使用 grpcurl 手工调试

gRPC 服务没有启用 reflection，因此 `grpcurl` 需要显式指定 proto 文件和 import 路径。

查询状态：

```bash
grpcurl -plaintext \
  -import-path api/proto \
  -proto robot/v1/robot_control.proto \
  -d '{"robot_id":"er7-900-01"}' \
  127.0.0.1:50051 robot.v1.RobotControlService/GetState
```

获取租约：

```bash
grpcurl -plaintext \
  -import-path api/proto \
  -proto robot/v1/robot_control.proto \
  -d '{"robot_id":"er7-900-01","client_id":"grpcurl"}' \
  127.0.0.1:50051 robot.v1.RobotControlService/AcquireControlLease
```

提交受控停止：

```bash
grpcurl -plaintext \
  -import-path api/proto \
  -proto robot/v1/robot_control.proto \
  -d '{
    "robot_id":"er7-900-01",
    "client_id":"grpcurl",
    "idempotency_key":"grpcurl-stop-001",
    "timeout_ms":10000,
    "stop":{"mode":"STOP_MODE_CONTROLLED"}
  }' \
  127.0.0.1:50051 robot.v1.RobotControlService/SubmitCommand
```

`grpcurl` 适合接口联通性和状态读取测试，不建议用于生产运动控制，因为它不负责租约续期、命令重试、状态监控和异常停止。

## 12. mTLS 生产环境

生产配置应关闭明文 gRPC：

```yaml
security:
  grpc:
    allow_insecure_loopback: false
    client_ca_file: /etc/robot-manager/pki/clients-ca.pem
    server_certificate_chain_file: /etc/robot-manager/pki/server.pem
    server_private_key_file: /etc/robot-manager/pki/server-key.pem
    allowed_client_common_names:
      - robot-hmi
      - robot-orchestrator
```

服务启动时会要求以下字段都存在：

- 客户端 CA；
- 服务端证书链；
- 服务端私钥；
- 非空客户端 Common Name 白名单。

Python 客户端连接 mTLS 服务：

```python
with open("clients-ca.pem", "rb") as file:
    root_certificates = file.read()
with open("robot-orchestrator-cert.pem", "rb") as file:
    certificate_chain = file.read()
with open("robot-orchestrator-key.pem", "rb") as file:
    private_key = file.read()

credentials = grpc.ssl_channel_credentials(
    root_certificates=root_certificates,
    private_key=private_key,
    certificate_chain=certificate_chain,
)
channel = grpc.secure_channel("robot-host:50051", credentials)
stub = pb_grpc.RobotControlServiceStub(channel)
```

客户端证书的 Common Name 必须在 `allowed_client_common_names` 中，否则服务返回 `PERMISSION_DENIED`。mTLS 只解决通信身份和加密问题，不能替代控制租约和现场安全回路。

## 13. 错误处理

### 13.1 gRPC 状态码

服务将内部状态映射为 gRPC 状态码：

| gRPC 状态码             | 常见原因                           | 客户端处理建议                       |
| ----------------------- | ---------------------------------- | ------------------------------------ |
| `INVALID_ARGUMENT`    | 参数格式、关节数量、速度或坐标错误 | 修正请求，不要盲目重试               |
| `FAILED_PRECONDITION` | 状态、配置或安全策略不满足         | 读取状态和配置后处理                 |
| `PERMISSION_DENIED`   | 租约无效、I/O/程序不在白名单       | 停止当前流程，重新获取授权或人工处理 |
| `NOT_FOUND`           | robot_id 或 command_id 不存在      | 检查配置和命令生命周期               |
| `ALREADY_EXISTS`      | 租约已被占用或幂等键复用           | 查询原命令，不要创建新动作           |
| `RESOURCE_EXHAUSTED`  | 队列已满或已有运动命令             | 等待当前命令完成                     |
| `UNAVAILABLE`         | 服务、控制器或状态不可用           | 退避重试，并检查状态                 |
| `DEADLINE_EXCEEDED`   | RPC 或运动命令超时                 | 查询命令和状态，确认是否已停止       |
| `FAILED_PRECONDITION` | 控制器故障或 Quick Stop 不支持     | 读取报警，按现场流程处理             |
| `INTERNAL`            | 服务内部异常或驱动抛出异常         | 记录日志并转人工/运维处理            |

### 13.2 RPC 重试原则

对于 `SubmitCommand`：

1. 使用同一个 `client_id`；
2. 使用同一个 `idempotency_key`；
3. 保持请求内容完全一致；
4. 重新调用后以返回的 `command_id` 为准；
5. 用 `GetCommand` 查询最终状态。

不要在不确定第一次请求是否到达服务时生成新的幂等键，否则可能导致同一动作被提交两次。

对于租约续期失败，不应继续发送运动命令。先查询 `GetState`，确认当前运动状态，再决定是否重建租约或转人工处理。

## 14. 一条 MoveJ 命令在服务内部如何执行

```text
Python/C++/其他上层客户端
        │
        ▼
RobotControlService::SubmitCommand
        │
        ├─ 校验 mTLS 身份
        ├─ 校验 robot_id
        └─ FromProtoCommand
              │
              ▼
CommandExecutor::Submit
        │
        ├─ 校验 client_id / idempotency_key / timeout
        ├─ 检查幂等键是否已使用
        ├─ 检查控制租约
        ├─ CommandGuard::Validate
        │    ├─ motion_enabled
        │    ├─ joint_limits_verified
        │    ├─ Ready / Automatic / servo_on
        │    ├─ 无报警、急停、已有运动
        │    ├─ 六个关节值在配置范围内
        │    └─ 速度和 blend 合法
        └─ 放入命令队列，返回 command_id
              │
              ▼
CommandExecutor::WorkerLoop
        │
        ├─ 再次读取状态并再次校验
        ├─ IRobotDriver::MoveJ
        └─ EfortDriver::MoveJ
              │
              ├─ EFORT CheckTarget(RobotJoint)
              └─ EFORT MJOINT
                    │
                    ▼
                  控制器

PollLoop ── GetMoveState / ReadState ──► 更新 RobotSnapshot
                                      └─► MoveJ 完成后命令变为 SUCCEEDED
```

这种设计意味着：

- 网络线程不会直接调用厂商 SDK；
- SDK 写操作由执行器串行化；
- 运动命令不会因为 SDK 返回“已接受”就立即报告完成；
- 状态异常、租约过期或命令超时会触发受控停止逻辑。

## 15. 生产上线检查清单

### 配置和通信

- [ ] `schema_version` 为 `1`；
- [ ] `robot.id`、IP 地址和型号正确；
- [ ] 实际控制器型号与配置匹配；
- [ ] 生产环境关闭明文 gRPC；
- [ ] mTLS 证书、私钥和客户端 CN 白名单正确；
- [ ] 服务账号权限和 SDK 动态库路径正确。

### 运动安全

- [ ] 六个关节限位已经由现场确认；
- [ ] `motion_enabled` 只在现场验证后开启；
- [ ] MoveL/MoveC workspace 和 frame 已现场确认；
- [ ] 工具和工件坐标系已验证；
- [ ] 最大速度已按工艺和安全风险设置；
- [ ] 急停、STO、安全门、光栅由认证硬件实现；
- [ ] 已验证受控停止和控制器断线行为。

### 上层应用

- [ ] 使用唯一且稳定的 `client_id`；
- [ ] 每条命令使用唯一幂等键；
- [ ] 实现租约续期；
- [ ] 以 `GetCommand` 确认异步命令最终状态；
- [ ] 订阅状态并处理 `UNKNOWN`、`FAULT`、`EMERGENCY_STOP` 和 stale；
- [ ] RPC 超时后按照幂等规则重试；
- [ ] 进程退出、网络断开和租约失效时执行停止/收尾流程；
- [ ] 不把 gRPC 明文开发配置带入生产网络。

## 16. 相关源码和文档

- 接口定义：[`api/proto/robot/v1/robot_control.proto`](../api/proto/robot/v1/robot_control.proto)
- 服务启动入口：[`apps/robot_manager/main.cc`](../apps/robot_manager/main.cc)
- 应用执行器：[`include/robot/control/command_executor.h`](../include/robot/control/command_executor.h)
- 命令执行实现：[`src/control/command_executor.cc`](../src/control/command_executor.cc)
- 安全校验：[`src/control/command_guard.cc`](../src/control/command_guard.cc)
- 驱动抽象：[`include/robot/control/robot_driver.h`](../include/robot/control/robot_driver.h)
- EFORT 适配器：[`src/adapters/efort/efort_driver.cc`](../src/adapters/efort/efort_driver.cc)
- EFORT 集成说明：[`docs/sdk_integration.md`](sdk_integration.md)
- 配置示例：[`config/robots/er7_900.yaml`](../config/robots/er7_900.yaml)
