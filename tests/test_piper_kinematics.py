from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from grasp_geometry import lifted_target, make_camera_standoff_target
from piper_kinematics import PiperKinematics


ROOT = Path(__file__).resolve().parents[1]


def load_case():
    config = yaml.safe_load((ROOT / "config/object_003_grasp.yaml").read_text())
    robot = config["robot"]
    urdf = ROOT / "tests/fixtures/piper_x_description.urdf"
    kinematics = PiperKinematics.from_urdf(urdf, robot["joint_names"])
    joints = np.array(
        [
            0.2298598625,
            0.4161388536,
            0.0147654855,
            -0.2394591734,
            0.2666339498,
            0.0500909495,
        ]
    )
    base_T_tcp = np.eye(4)
    base_T_tcp[:3, 3] = [0.128301, 0.020480, 0.189938]
    base_T_tcp[:3, :3] = Rotation.from_quat(
        [-0.5083728128, 0.5401803611, -0.4779179105, 0.4704856337]
    ).as_matrix()
    base_T_object = np.eye(4)
    base_T_object[:3, 3] = [0.6061, 0.2055, 0.082]
    handeye = yaml.safe_load((ROOT / "examples/device_profile/online_handeye.yaml").read_text())
    tcp_T_camera = np.asarray(handeye["tcp_T_camera"], dtype=float)
    return config, kinematics, joints, base_T_tcp, base_T_object, tcp_T_camera


def test_forward_kinematics_matches_live_feedback():
    _, kinematics, joints, base_T_tcp, _, _ = load_case()
    actual = kinematics.forward(joints)
    assert np.linalg.norm(actual[:3, 3] - base_T_tcp[:3, 3]) < 0.001
    error = Rotation.from_matrix(
        actual[:3, :3].T @ base_T_tcp[:3, :3]
    ).magnitude()
    assert np.degrees(error) < 0.1


def test_rejected_cartesian_case_has_valid_continuous_joint_plan():
    config, kinematics, joints, tcp, obj, tcp_T_camera = load_case()
    pregrasp = make_camera_standoff_target(tcp, obj, tcp_T_camera, 0.15)
    grasp = make_camera_standoff_target(tcp, obj, tcp_T_camera, 0.10)
    lift = lifted_target(grasp, 0.05)
    plan = kinematics.plan_grasp_motion(
        joints,
        pregrasp,
        grasp,
        lift,
        config["planning"],
        config["safety"]["workspace_m"],
    )
    assert plan.minimum_margin_deg > 15.0
    assert plan.minimum_tcp_z_m > 0.05
    assert np.allclose(
        np.degrees(plan.pregrasp_joints),
        [17.97, 133.02, -60.34, -68.39, 0.32, 11.28],
        atol=0.25,
    )
