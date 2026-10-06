"""Load local device calibration without importing ROS or accessing hardware."""
from pathlib import Path
import numpy as np
import yaml
from grasp_geometry import as_transform


def _mapping(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"configuration must be a mapping: {path}")
    return data


def load_grasp_configuration(path):
    path = Path(path).expanduser().resolve()
    cfg = _mapping(path)
    def relative_file(key):
        value = Path(str(cfg[key])).expanduser()
        return value if value.is_absolute() else path.parent / value
    gripper = _mapping(relative_file("gripper_calibration"))
    handeye = _mapping(relative_file("handeye_config"))
    if handeye.get("base_frame", "base_link") != cfg.get("base_frame", "base_link"):
        raise ValueError("base_frame differs between grasp and hand-eye configurations")
    transform = as_transform(np.asarray(handeye["tcp_T_camera"], dtype=float))
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5):
        raise ValueError("hand-eye rotation must be a proper rigid rotation")
    cfg["_gripper_calibration"] = gripper
    cfg["_calibrations_confirmed"] = (handeye.get("calibration_confirmed") is True and gripper.get("calibration_confirmed") is True)
    return cfg, transform
