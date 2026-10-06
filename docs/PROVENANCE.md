# 来源与整理记录

本包从比赛工程 `Piper_Control/` 提取。内嵌的 FoundationPose 已拆成另一个独立包，历史日志、个人环境与计划文件不包含在这里。原说明保留为 `ORIGINAL_README.md`。

## 保留的代码

保留抓取状态机、手眼桥、位姿接收、数值 FK/IK、相机距离抓取几何、运动稳定判断、夹爪反馈策略、物体配置与模拟脚本。`vendor/piper_ros2_control/` 中纯夹爪策略来自团队提供的代码，保留原样；不包含其独立 CAN 驱动。没有替个人或团队代码新增整体开源许可证。

## 外部厂商依赖

- AGX ROS：https://github.com/agilexrobotics/agx_arm_ros
- 原工程本地驱动仓库基准提交：`77882f4305c5e169b4542b408ac9f6755bdfa7c8`。原环境包含本地调整；不能假设任意上游版本具有完全相同的反馈和控制行为。
- `tests/fixtures/piper_x_description.urdf` 来自原 AGX 工作空间 `agx_arm_description/agx_arm_urdf/piper_x/urdf/`。MIT 许可证保留为 `tests/fixtures/LICENSE.agx_arm_ros`。
- URDF 仅用于读取六个关节的数值参数。其引用的视觉网格未附带，不能作为完整 RViz 或厂商机器人描述包。
- `tests/external/` 针对原本地 pyAgxArm 发送补丁，不随默认测试收集；本包没有复制 SDK 或该补丁。

## 本次整理修改

- 环境路径改为显式 `AGX_ARM_WS` / `AGX_ARM_SETUP`，驱动启动默认仅反馈，需要 `--enable-control` 才设置控制和自动使能。
- 原设备标定移到历史示例；新增本机配置模板，实际执行要求手眼与夹爪文件均有 `calibration_confirmed: true`。
- 增加标定矩阵与基座一致性检查；手眼桥拒绝零时间戳和明显坐标系不匹配的数据。
- 将 FK/IK 测试改为自带数值素材，增加配置检查测试和 CPU CI。
- 原 object_004 实际重复 object_003，转入历史示例。

此次没有重写完整抓取策略，没有实机执行，没有验证新机器的驱动/固件组合。整理时通过 35 项纯软件测试，已知控制行为见 `KNOWN_ISSUES.md`。
