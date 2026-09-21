"""Prepare position-only waypoints for the JTC quintic interpolator, off the RT thread."""
import numpy as np
from scipy.interpolate import make_interp_spline
from _iiwa_spline_limits import retime


def prepare_trajectory(times, positions, limits):
    """Preserve waypoints, fit a C2+ rest-to-rest curve, bound its complete interpolation.

    `times` starts at zero. Limits contain [lower, upper, velocity, acceleration, jerk].
    Returns (times, positions, velocities, accelerations, uniform_time_scale).
    Timing may increase; intermediate path can differ from position-only linear interpolation.
    No collision checking is performed by this direct-command API.
    """
    t, q = np.asarray(times, dtype=float), np.asarray(positions, dtype=float)
    if t.ndim != 1 or q.ndim != 2 or len(t) < 2 or len(t) != len(q):
        raise ValueError('At least two complete trajectory points are required')
    if not np.isfinite(t).all() or not np.isfinite(q).all() or t[0] != 0 or np.any(np.diff(t) < 1e-6):
        raise ValueError('Finite waypoints and strictly increasing times starting at zero are required')
    if q.shape[1] != len(limits):
        raise ValueError('Joint count does not match limits')
    zero = np.zeros(q.shape[1])
    # With two derivative constraints per endpoint, knots coincide with all input times.
    # Thus JTC quintics reproduce these spline segments from their endpoint p/v/a.
    spline = make_interp_spline(t, q, k=5, bc_type=([(1, zero), (2, zero)], [(1, zero), (2, zero)]))
    return retime(t.tolist(), q.tolist(), spline(t, 1).tolist(), spline(t, 2).tolist(), limits)
