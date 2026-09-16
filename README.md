# robot-manager

`robot-manager` is a Linux C++17 control service for EFORT robots. The current
implementation supports ER150-2700, ER7-900, and ER155-3200 through the same
EFORT C++ SDK while keeping the upper-layer API independent of SDK types.

## Framework and extensions

`robot_core` contains the domain model, command executor, control leases and
safety checks. It depends only on the C++ standard library and threads.
Drivers implement `IRobotDriver`; transports call `RobotService`.

| Target | Responsibility | Build option |
| --- | --- | --- |
| `robot_core` | Reusable control framework | Always built |
| `robot_mock` | In-memory driver | `ROBOT_ENABLE_MOCK` (default ON) |
| `robot_efort` | EFORT SDK driver | `ROBOT_ENABLE_EFORT` (default OFF) |
| `robot_grpc` | gRPC transport and protocol mapping | `ROBOT_ENABLE_GRPC` (default OFF) |
| `robot_config` | YAML deployment configuration | Built with apps or tests |

Each adapter owns its CMake target under `src/adapters/<name>/`. Applications
select and compose extensions; the framework never creates concrete drivers.
Add a driver by implementing `IRobotDriver` in a separate library linked to
`robot_core`, then inject it into `CommandExecutor`. A new transport takes a
`RobotService` and does not need to know the driver or executor implementation.
Extensions are linked at build time; dynamic plugin loading is not required.

The shared `include/robot/driver/` directory contains only `robot_driver.h`,
whose interface type is `IRobotDriver`. Concrete driver headers are private to
their adapter modules under `src/adapters/<name>/`; only the application that
composes an adapter adds that module's private include directory. Linking only
`robot_core` does not expose vendor or mock headers. EFORT SDK headers are a
private dependency of `robot_efort`; its SDK libraries still participate in
final executable linking.

This separation concerns implementation dependencies. The current domain and
wire contracts still assume six axes and the supported EFORT model catalog;
supporting other robot families may require changes to those contracts.

To build only the framework, without YAML, mock, vendor SDK or network libraries:

```sh
cmake -S . -B build/core -DROBOT_BUILD_APPS=OFF -DROBOT_BUILD_TESTS=OFF \
  -DROBOT_ENABLE_MOCK=OFF -DROBOT_ENABLE_EFORT=OFF -DROBOT_ENABLE_GRPC=OFF
cmake --build build/core
```

## Safety boundary

This service is not a safety-rated controller. Emergency stop, guarding, STO,
and other safety functions must remain in certified hardware. Real motion is
disabled by default. The daemon refuses to enable motion unless six site-
verified joint limits are configured.

## Linux build

The bundled `libEftSdk.so` is an x86-64 Linux ELF library built with GCC 5.4.
Build this C++17 service on x86-64 Linux with GCC 9+ or Clang 10+ and the GCC
5+ `std::__cxx11` ABI; GCC 5.4 itself does not provide the required C++17
standard-library facilities.

The framework requires CMake 3.22+ and a C++17 compiler. The gRPC extension
additionally requires Protobuf and gRPC.
Package names vary by distribution.

The build accepts both the official gRPC CMake package and Linux distribution
packages that provide gRPC through headers, `libgrpc++`, and
`grpc_cpp_plugin` only (including Ubuntu 20.04).

For example, on Ubuntu/Debian the package set is typically:

```sh
sudo apt-get install build-essential cmake ninja-build protobuf-compiler \
  libprotobuf-dev libgrpc++-dev protobuf-compiler-grpc
```

```sh
./deploy/scripts/validate_linux.sh
```

To build and test without the physical SDK:

```sh
cmake --preset linux-mock-debug
cmake --build --preset linux-mock-debug
ctest --preset linux-mock-debug
```

## Vendored gRPC build

The production preset uses the gRPC v1.78.1 toolchain installed under
`3rdparty/grpc/install`. Clone the pinned source and its shallow submodules:

```sh
git clone --recurse-submodules -b v1.78.1 --depth 1 --shallow-submodules \
  https://github.com/grpc/grpc 3rdparty/grpc
./deploy/scripts/build_grpc.sh
```

`build_grpc.sh` stores intermediate files in `3rdparty/grpc/build` and installs
headers, libraries, CMake package files, `protoc`, and `grpc_cpp_plugin` in
`3rdparty/grpc/install`. `build_linux.sh` invokes this step automatically for
the EFORT release build.

查看构建版本、Git 元数据和编译器信息：

```sh
./build/linux-mock-debug/robot-manager --version
./build/linux-mock-debug/robotctl version
```

默认版本为 `0.1.0-release`。配置时可通过
`-DROBOT_VERSION_STAGE=dev`、`beta`、`rc` 或 `release` 覆盖阶段；版本头文件由
CMake 写入构建目录的 `generated/robot/version.h`，不会进入源码树。
例如，仓库提供的 `linux-mock-dev` 预设会生成 `0.1.0-dev`：

```sh
cmake --preset linux-mock-dev
cmake --build --preset linux-mock-dev
./build/linux-mock-dev/robot-manager --version
```

For a local process-only check, use the Mock configuration:

```sh
./build/linux-mock-debug/robot-manager \
  --config config/robots/mock.yaml
```

## Configuration

The service reads a versioned YAML file. By default its path is
`/etc/robot-manager/robot.yaml`; pass `--config <path>` for another location.
Copy one model profile, then set the real controller address, all six joint
limits, allowed digital outputs, and speed limits. MoveL and MoveC additionally
remain blocked until the site XYZ workspace is configured. Do not enable motion
until the values have been reviewed on site. The workspace `frame` defines the
coordinate system of its bounds, and Cartesian commands using another work
object are rejected.

Controller-program execution is independently disabled. Set
`limits.program_execution_enabled: true` only for programs reviewed and
accepted on the target controller, and add each allowed program to
`limits.approved_programs`.

```sh
sudo cp config/robots/er7_900.yaml /etc/robot-manager/robot.yaml
```

The three model examples differ only in model identity and site configuration;
all use `libEftSdk.so`. At connection time the service calls
`GetCurrentRobotType` and rejects a configured/actual model mismatch.

The gRPC schema is in `api/proto/robot/v1/robot_control.proto`. Production
gRPC requires mTLS: configure `security.grpc.client_ca_file`,
`security.grpc.server_certificate_chain_file`,
`security.grpc.server_private_key_file`, and a nonempty
`security.grpc.allowed_client_common_names` allowlist. Plaintext gRPC is
development-only, must bind to loopback, and requires
`security.grpc.allow_insecure_loopback: true` explicitly.

The configuration parser accepts the documented YAML mapping/list subset only,
requires `schema_version: 1`, and rejects unknown, duplicate, missing, or
wrongly typed settings. Only a program in the site-approved list that was
successfully loaded by the current service process can start.

上层应用集成、调用顺序、Python gRPC 示例、命令状态轮询、租约续期、I/O、
程序控制、mTLS 和生产上线检查清单见
[`docs/application_integration_zh.md`](docs/application_integration_zh.md)。

## SDK packaging note

The supplied directory contains an x86-64 `libEftSdk.so`, but the versioned
`liblog4cpp.so.2.9*` files are AArch64. The x86-64 SDK depends on the unversioned
`liblog4cpp.so`, so the CMake install and verification script intentionally
package only architecture-matched files. Do not copy the whole SDK `lib`
directory into the production runtime path.
