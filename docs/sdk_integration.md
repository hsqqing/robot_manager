# EFORT SDK V2.8 integration notes

## Validated source material

The adapter is based on the bundled `C++SDK使用手册_V2.8.pdf`,
`EfortSdk.h`, `SdkConstDef.h`, and `SdkStructDef.h`.

The manual is V2.8.0 dated 2026-01-16 and states that the Linux SDK was built
with GCC 5.4.0 and may be consumed by GCC 5.4 or newer. `SdkConstDef.h` reports
SDK version 280, while `EfortSdk.h` and `SdkStructDef.h` still identify
themselves as V2.7.4. This source package mismatch must be confirmed with EFORT
before a production release is frozen.

The service itself uses `std::optional`, `std::variant`, and other C++17
library facilities, so its supported build floor is GCC 9 or Clang 10 rather
than the SDK producer's GCC 5.4 compiler.

## Linux ABI

The primary runtime files have been inspected as ELF binaries:

| File | Architecture | Used by deployment |
| --- | --- | --- |
| `libEftSdk.so` | x86-64 | yes |
| `liblog4cpp.so` | x86-64 | yes |
| `librlibcpp.bcc.so.1` | x86-64 | yes |
| `librlibcpp.tool.so.1` | x86-64 | yes |
| `liblog4cpp.so.2.9*` | AArch64 | no |

`libEftSdk.so` directly requests the unversioned `liblog4cpp.so`, so the
production installer deliberately excludes the AArch64 versioned files.
After every SDK update, re-check the ELF architecture of each runtime file
and the dependency set of `libEftSdk.so` before shipping.

The four selected libraries are installed together under
`/usr/local/lib/robot-manager` by the default deployment. The executable has a
relative install RPATH, and the supplied systemd unit also sets
`LD_LIBRARY_PATH` to that private directory so transitive SDK dependencies are
resolved consistently. Adjust both paths when using a non-default install
prefix.

The SDK exports C++ standard-library types such as `std::string` and
`std::vector`; therefore compiler ABI compatibility matters. The supplied
library contains the GCC 5+ `std::__cxx11` ABI symbols. Do not set
`_GLIBCXX_USE_CXX11_ABI=0` for this build.

## Supported robot models

All three deployed models use the same driver and six primary ER axes:

| Model | Axes | Rated payload | Nominal reach | Runtime validation |
| --- | ---: | ---: | ---: | --- |
| ER150-2700 | 6 | 150 kg | 2700 mm | `GetCurrentRobotType` |
| ER7-900 | 6 | 7 kg | 900 mm | `GetCurrentRobotType` |
| ER155-3200 | 6 | 155 kg | 3200 mm | `GetCurrentRobotType` |

Payload and reach are model metadata, not software motion limits. The SDK
package does not provide verified joint limits for these three installations.
The service therefore does not guess them: motion is disabled until all six
site limits are supplied and `limits.motion_enabled: true` is explicitly set.
MoveL and MoveC also require a verified, site-specific XYZ workspace; SDK
reachability validation is not treated as a cell-safety envelope.
The workspace is tied to one configured controller work-object frame, and
Cartesian requests using another frame are rejected rather than compared in
the wrong coordinate system.
Controller-program execution additionally requires
`limits.program_execution_enabled: true` and a nonempty
`limits.approved_programs` allowlist. The service may start only an approved
program that it successfully loaded during its current process lifetime. These
controls are independent because an already-loaded controller program cannot
be validated point by point by the service.

## Implemented API mapping

| Service capability | EFORT SDK call |
| --- | --- |
| Connect/disconnect | `ConnectRobot`, `DisconnectRobot`, `IsConnected` |
| Validate model | `GetCurrentRobotType` |
| API control | `EnableApiControl`, `IsApiControl` |
| Servo power | `PowerOn`, `PowerOff` |
| Global speed | `SetGlobalSpeed` |
| Clear ordinary alarm | `ClearAlarm` after `GetCurrentEmgStatus` |
| Joint motion | `CheckTarget(RobotJoint)` then nonblocking `MJOINT` |
| Linear motion | frame selection, `CheckTarget(RobotPos)`, then `MLIN` |
| Circular motion | two `CheckTarget` calls, then `MCIRC` |
| Hold/resume | `MOVEHOLD`, `MOVERESUME` |
| Controlled stop | `MOVEHOLD` when moving, then `MOVECLEAR` |
| Jog | `SetJogMode`, `Jog1..6Plus/Minus` |
| Program | `LoadProgram`, `StartProgram`, `StopProgram` |
| State | `GetRobotStatusData`, emergency/servo/alarm/move queries,
  `GetJointPos`, `GetBaseCoordinatePos2` |
| Alarm details | `GetAlarmData(ALA_CURRENT)` |
| Digital I/O | `ReadDIn`, `ReadDOut`, `WriteDOut` |

EFORT V2.8 does not expose a distinct quick-stop API in the supplied header.
The adapter rejects `QUICK_STOP` instead of pretending that `MOVEHOLD` has a
certified quick-stop meaning. Hardware emergency stop and STO remain outside
this service.

## Units and motion completion

- `MJOINT` receives six joint values in degrees and speed in the range 1-100%.
- `RobotPos` and `PointC` use millimetres and Euler angles in degrees.
- `MLIN` and `MCIRC` use TCP speed; `MCIRC` explicitly documents mm/s.
- Negative zones select blocking SDK behavior. The service accepts only
  nonnegative blend values and uses nonblocking calls so Stop can be processed.
- Command acceptance is not command completion. The executor observes
  `GetMoveState` through `ReadState` and only then transitions an asynchronous
  command to a terminal state.
- EFORT digital input/output indexes are 0-175. Writes additionally require a
  site allowlist.

## Known validation still requiring hardware

The following items cannot be proven on the current Windows editing host and
must be completed on the Linux HIL station:

1. Link and load all four x86-64 runtime libraries on the production Linux
   image.
2. Confirm controller firmware compatibility with SDK constants V2.8 and the
   V2.7.4-labelled headers.
3. Record the exact strings returned by `GetCurrentRobotType` for all three
   robots and extend aliases only if required.
4. Measure `ConnectRobot`, `GetRobotStatusData`, `GetMoveState`, and Stop worst
   case latency.
5. Verify `MOVECLEAR` behavior after hold and after already-completed motion.
6. Validate tool/work-object selection and Euler-angle conventions on each
   controller.
7. Fill site joint limits, speed limits, writable I/O, and perform low-speed
   acceptance in a cleared work cell.
