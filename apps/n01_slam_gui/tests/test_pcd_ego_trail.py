import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from filter_pcd_ego_trail import filter_points, read_trajectory


def test_historical_ego_filter_removes_trail_and_keeps_wall(tmp_path):
    poly = np.array([[-.3, -.2], [.3, -.2], [.3, .2], [-.3, .2]])
    poses = [(0., 0., 0., 0.), (.5, 0., 0., 0.), (1., 0., 0., 0.)]
    trail = np.column_stack((np.linspace(0, 1, 30), np.zeros(30), np.zeros(30)))
    wall = np.column_stack((np.full(60, 3.0), np.linspace(-1, 1, 60), np.zeros(60)))
    kept, removed, cells = filter_points(
        np.vstack((trail, wall)), poses, poly, .06, .10, .80, .90)
    assert removed >= 30
    assert (kept[:, 0] > 2.5).sum() == 60
    assert cells > 0


def test_filter_requires_trajectory():
    with pytest.raises(ValueError, match="trajectory"):
        filter_points(np.zeros((1, 3)), [], np.array([[0, 0], [1, 0], [1, 1]]))


def test_trajectory_csv_quaternion_and_header(tmp_path):
    p = tmp_path / "traj.csv"
    p.write_text("stamp,x,y,z,qx,qy,qz,qw\n1,0,0,0,0,0,0,1\n2,1,2,0,0,0,0,1\n")
    poses = read_trajectory(p)
    assert len(poses) == 2
    assert poses[1][:3] == (1.0, 2.0, 0.0)
