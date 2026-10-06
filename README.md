# Piper Known-Object Control

Piper X 已知物体的一次性抓取控制工具，从比赛工程 `Piper_Control` 提取。接收 FoundationPose 的位姿和模型标识，经手眼变换、目标冻结、IK 与路径预检查后，执行预抓取、靠近、闭合夹爪及沿基座 +Z 提升 50 mm，结束后保持物体。

当前是**设备相关的实验控制程序**，包含已测设备的夹爪策略和若干物体配置。默认运行演练，不发送控制指令。控制逻辑仍有待完善的行为，见 `docs/KNOWN_ISSUES.md`；整理完成不能等同于换台机械臂后直接可用。

## 与其他模块的关系

```text
FoundationPose → 相机坐标位姿 + 模型标识
TCP 反馈 + 本机手眼外参 → handeye_pose_bridge → 基座物体位姿
物体位姿 + 关节 / 机械臂 / 夹爪反馈 → grasp_controller
                               ↓
                  AGX ROS 驱动 → CAN → Piper X
```

AGX ROS 驱动是唯一 CAN 控制进程。这里保留的 `vendor/piper_ros2_control` 是纯夹爪反馈/启动策略，不启动另一套 SDK 驱动。

## 下载后先运行纯软件检查

在本仓库根目录：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-test.txt
python -m pytest -v
```

测试覆盖抓取几何、正逆运动学、夹爪响应、反馈检查、运动稳定判断和标定文件检查。不需要 ROS、相机或机械臂，使用 `tests/fixtures/` 的历史 Piper X URDF；该文件只是数值回归素材，不代替本机安装的机器人描述包。来源及许可证已附带。

`tests/external/` 内另保留针对原本修改过的 pyAgxArm CAN 发送逻辑的回归测试，默认不收集。该 SDK 补丁没有随这个代码包复制，不能将这些测试视为通用 SDK 必然支持的行为。

## 运行依赖

- Ubuntu / Python 3.10 / ROS 2 Humble 基线，使用系统 Python。
- `rclpy`、`geometry_msgs`、`sensor_msgs`、`std_msgs`、`visualization_msgs`、`ament_index_python`，以及 NumPy、SciPy、PyYAML。
- 已安装构建的 AGX ROS 驱动工作空间，提供 `agx_arm_ctrl`、`agx_arm_msgs`、`agx_arm_description`。
- 与机器人一致的 Piper X URDF、关节名字、固件协议和夹爪规格。
- 运行中的视觉模块，以及你这台机器人/相机组合的手眼标定。

测试依赖清单不包含 ROS 和厂商驱动。驱动来源可参考 [agilexrobotics/agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros)，原工程实际使用版本、补丁差异见来源文档。没有把外部整个驱动工作空间、机器人网格或 SDK 混进本仓库。

在每个控制终端设置：

```bash
export AGX_ARM_WS=/实际路径/agx_arm_ws
source scripts/env.sh
```

也可直接设置 `AGX_ARM_SETUP=/实际路径/install/setup.bash`，ROS 安装不是默认路径时设置 `ROS_SETUP`。没有设置 AGX 路径时脚本明确报错，不再猜测作者电脑目录。

## 准备本机标定和物体配置

```bash
cp config/online_handeye.example.yaml config/online_handeye.yaml
cp config/gripper_calibration.example.yaml config/gripper_calibration.yaml
```

手眼模板中的单位矩阵是占位符；填入实际 `tcp_T_camera`，核对基座、TCP 与光学坐标系，确认后把 `calibration_confirmed` 改为 `true`。夹爪模板中的数值来自原设备，必须重新核对行程、反馈、零点、启动宽度和 homing 行为，完成后同样确认。

这两个本机文件被 `.gitignore` 排除；原实测记录保留在 `examples/device_profile/` 供追溯，不能当作任何机器人通用标定。标记确认只是防止误用模板，不会自动测量或证明标定正确。

`config/object_002_grasp.yaml`、`object_003_grasp.yaml`、`object_005_grasp.yaml` 保留原先的不同模型标识。修改 `object.foundationpose_model_id` 使其与视觉发布的 `model_id` 一致；重新检查工作空间、预抓取距离、夹爪开合宽度和路径限制。旧 object_004 配置实际上仍指向 object_003，已转存为历史重复示例，不作为支持的第四个物体配置。

## 先启动反馈、视觉与演练

CAN 激活由外部驱动完成。默认驱动启动只打开反馈，整理版不自动使能控制：

```bash
CAN_PORT=can0 bash scripts/start_piper.sh
```

先按视觉仓库 README 启动 D405、ROS I/O 和 GPU worker，确认框与位姿正确，然后在控制仓库另一个已配置的终端启动手眼桥：

```bash
bash scripts/start_handeye.sh
```

桥按图像时间戳选择最近的 TCP 反馈，拒绝超出时间窗、零时间戳和明显坐标系不匹配的输入。它不做插值；移动中的机械臂需要更严格的同步评估。默认模板时间窗 0.10 秒，历史设备为 0.25 秒，不要为绕过同步问题盲目放宽。

演练一个物体：

```bash
bash scripts/start_grasp_003.sh
```

默认等待反馈、稳定目标和模型标识，冻结目标、运行 IK 和路径检查，并发布检查用的目标与状态；应出现 `DRY RUN COMPLETE`。不连接正确设备/ROS 话题时会等待或报错，这是预期行为。可用 `start_grasp_002.sh` / `start_grasp_005.sh` 选择其他历史配置。

检查结果：

```bash
ros2 topic echo /piper_control/grasp_status --once
ros2 topic echo /piper_control/grasp_target_pose --once
ros2 topic echo /piper_control/pregrasp_joint_target --once
```

`check_ready.sh` 是原完整执行环境的检查，仍要求机械臂处于允许控制的模式；在默认反馈模式下可能失败，不应为消除这类提示贸然开启控制。

## 在本机重新验证后执行

实际执行有两个显式开关：先重启外部驱动，允许其控制和自动使能；然后给控制器 `--execute`。执行前必须核对本机参数并完成演练。

```bash
# 这一条会允许驱动使能和控制真实机器人
CAN_PORT=can0 SPEED_PERCENT=10 bash scripts/start_piper.sh --enable-control

# 另一终端：允许一次抓取，包含夹爪启动归一化
bash scripts/start_grasp_003.sh --execute
```

控制器按状态执行夹爪启动检查/80 mm 归一化、稳定目标获取、IK 预检查、95 mm 日常张开、关节空间预抓取、线性靠近、闭合与接触判断、50 mm 线性抬升。**夹爪归一化可能发生在视觉和 IK 验证之前**，此前旧 README 的流程叙述不准确，整理版按实际代码说明。

抓取目标使用“相机朝向物体中心，保持指定相机距离”的历史几何，再通过外参还原 TCP；它不是通用夹爪抓取点求解，物体形状和相机安装变化后应单独验证。提升固定为 50 mm，结束保持，不自动返回或放下。

遇到异常会进入 ABORTED 并停止后续命令；**这不是硬件急停，也不会取消已经交给驱动的运动**。本次保留该行为并列为待解决问题。IK 和路径采样只检查关节限制与 TCP 工作空间，不包含障碍物或自碰撞检测。

## ROS 接口

| 类别 | 话题 |
|---|---|
| 视觉输入 | `/foundationpose/object_pose_base`、`object_pose_json`、`recovery_required` |
| 机器人反馈 | `/feedback/tcp_pose`、`joint_states`、`arm_status`、`gripper_status` |
| 执行输出 | `/control/move_j`、`move_l`、`joint_states` |
| 检查输出 | `/piper_control/frozen_object_pose`、`grasp_target_pose`、`pregrasp_joint_target`、`grasp_status` |

位姿消息用 `PoseStamped`，关节消息用 `JointState`；夹爪与机械臂反馈用外部 `agx_arm_msgs`。具体话题配置在物体 YAML 中。

## ROS 模拟与工具

`tests/ros_dry_run.py` 等脚本提供合成 ROS 反馈。它们需要 AGX 消息及 URDF 安装，且需要本机配置文件存在；不属于普通 GitHub CPU 测试。执行模拟会发布控制话题，必须在与硬件完全隔离的测试域中运行。

```bash
source scripts/env.sh
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/piper_dryrun_logs \
  python3 tests/ros_dry_run.py
```

`pose_receiver.py` 只接收目标、发布检查用消息及保存 JSON；`tools/gripper_feedback_probe.py` 只记录反馈。`tools/gripper_recalibrate.py` 是历史 SDK 零点恢复工具，需要 pyAgxArm，必须与占用 CAN 的驱动互斥，并会写夹爪零点；不属于日常启动流程。

## 目录与发布

```text
*.py                   抓取状态机、几何、IK、稳定判断与手眼桥
config/                 三种物体配置和本机标定模板
examples/device_profile/ 原设备历史记录
scripts/                环境加载、反馈启动、演练与执行入口
vendor/                 已提取的纯夹爪策略
tools/                  反馈记录与历史恢复工具
tests/                  CPU 回归、数值 URDF 素材、ROS 模拟
docs/                   原 README、来源和已知问题
```

上传这个目录即可，原内嵌 FoundationPose、诊断 CSV、历史计划文件和缓存已排除。GitHub Actions 只运行纯软件测试。厂商 URDF 的 MIT 许可证保留在 fixtures，团队提供的夹爪策略来源在 vendor 文档中说明；本次未代替你为个人或团队代码选择新许可证。
