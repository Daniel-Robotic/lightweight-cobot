import numpy as np
import pytest
from iiwa_planning.direct_trajectory import prepare_trajectory

LIMITS = [[-2, 2, 1, 1, 2]]


def test_dense_points_are_smooth_and_retimed():
    t, q, v, a, scale = prepare_trajectory([0, .1, .2], [[0], [.5], [1]], LIMITS)
    assert scale > 1
    np.testing.assert_allclose(q, [[0], [.5], [1]])
    assert abs(v[1][0]) > 0  # do not stop at every CSV waypoint
    np.testing.assert_allclose([v[0], v[-1], a[0], a[-1]], 0, atol=1e-9)
    assert t[-1] > .2


@pytest.mark.parametrize('times,positions', [([0, 0], [[0], [1]]), ([0, 1], [[0], [float('nan')]]), ([0, 1], [[0], [3]])])
def test_bad_trajectory_is_rejected(times, positions):
    with pytest.raises((ValueError, RuntimeError)):
        prepare_trajectory(times, positions, LIMITS)


def test_time_scaling_preserves_quintic_between_waypoints():
    from scipy.interpolate import BPoly
    times, positions, velocities, accelerations, scale = prepare_trajectory([0,.1,.2], [[0],[.5],[1]], LIMITS)
    curve = BPoly.from_derivatives(times, [[q,v,a] for q,v,a in zip(positions,velocities,accelerations)])
    grid = np.linspace(times[0],times[-1],10001)
    for order,cap in [(1,1),(2,1),(3,2)]:
        assert np.max(np.abs(curve(grid,nu=order))) <= cap * 1.000001
