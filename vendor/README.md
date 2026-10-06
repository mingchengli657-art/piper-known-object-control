# Vendored gripper safety core

`piper_ros2_control/gripper_execution.py` is copied unchanged from the
team-provided `piper_ros2_control` package.  Only this pure, hardware-independent
profile and feedback gate is imported by `grasp_controller.py`. The companion
`gripper_startup.py` contains the team's pure startup policy, also retained
unchanged. These modules do not contain a CAN node.

The teammate package's CAN/SDK nodes are deliberately not started here because
`agx_arm_ctrl` remains the single owner of `can0`.
