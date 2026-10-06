#!/usr/bin/env python3
"""ROS simulation proving frozen gripper feedback cannot cause a lift."""

from ros_execute_sim import main


if __name__ == "__main__":
    main(bad_feedback=True)
