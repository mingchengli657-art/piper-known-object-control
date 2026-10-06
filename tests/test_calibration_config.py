from pathlib import Path
import numpy as np
import pytest
import yaml
from calibration_config import load_grasp_configuration

ROOT = Path(__file__).resolve().parents[1]


def local_profile(tmp_path, confirmed=False):
    cfg = yaml.safe_load((ROOT / 'config/object_003_grasp.yaml').read_text())
    for name in ('online_handeye', 'gripper_calibration'):
        value = yaml.safe_load((ROOT / 'examples/device_profile' / f'{name}.yaml').read_text())
        value['calibration_confirmed'] = confirmed
        (tmp_path / f'{name}.yaml').write_text(yaml.safe_dump(value))
    (tmp_path / 'grasp.yaml').write_text(yaml.safe_dump(cfg))
    return tmp_path / 'grasp.yaml'


def test_historical_profile_does_not_authorize_execution(tmp_path):
    cfg, matrix = load_grasp_configuration(local_profile(tmp_path))
    assert not cfg['_calibrations_confirmed']
    assert matrix.shape == (4,4)


def test_local_calibrations_can_be_confirmed(tmp_path):
    cfg, _ = load_grasp_configuration(local_profile(tmp_path, confirmed=True))
    assert cfg['_calibrations_confirmed']


def test_invalid_local_transform_is_rejected(tmp_path):
    path = local_profile(tmp_path)
    handeye = yaml.safe_load((tmp_path/'online_handeye.yaml').read_text())
    handeye['tcp_T_camera'] = np.zeros((4,4)).tolist()
    (tmp_path/'online_handeye.yaml').write_text(yaml.safe_dump(handeye))
    with pytest.raises(ValueError):
        load_grasp_configuration(path)


def test_mismatched_base_frames_are_rejected(tmp_path):
    path = local_profile(tmp_path)
    handeye = yaml.safe_load((tmp_path/'online_handeye.yaml').read_text())
    handeye['base_frame'] = 'different_base'
    (tmp_path/'online_handeye.yaml').write_text(yaml.safe_dump(handeye))
    with pytest.raises(ValueError, match='base_frame'):
        load_grasp_configuration(path)
