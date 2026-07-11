"""
Intended-velocity sources -- the travel the floor must cancel to keep the person
centered. M4 uses 'commanded' (a scripted v_cmd(t), deterministic ground truth);
M5 adds the live-sensor + model-fused estimate. 'gait' recovers speed/heading from
the foot motion itself (used as the predictor-derived reference).
"""

import math
import numpy as np


def commanded_path(t, turn_t=3.0, turn_dur=0.6):
    """Default scripted intended velocity (m/s): walk +x, then a SHARP turn to +y.

    The quick direction change is deliberate: it penalizes sensor latency (live
    lags through the turn) and rewards anticipation (the model leads), so the
    fusion weighting actually matters. Deterministic -> clean ground truth.
    """
    speed = 0.9
    if t < turn_t:
        ang = 0.0
    elif t < turn_t + turn_dur:
        ang = (math.pi / 2.0) * (t - turn_t) / turn_dur
    else:
        ang = math.pi / 2.0
    return speed * np.array([math.cos(ang), math.sin(ang)])


class IntendedVelocityEstimator:
    def __init__(self, source="commanded", cmd_fn=None):
        self.source = source
        self.cmd_fn = cmd_fn or commanded_path

    def commanded(self, t):
        return np.asarray(self.cmd_fn(t), float)

    @staticmethod
    def from_gait(foot_rel_stance, prev_rel_stance, dt):
        """Intended travel implied by the stance foot moving backward under the
        body: v_intended = -d(foot_rel_stance)/dt."""
        if prev_rel_stance is None or dt <= 0:
            return np.zeros(2)
        return -(np.asarray(foot_rel_stance) - np.asarray(prev_rel_stance)) / dt

    def estimate(self, t, **kw):
        if self.source == "commanded":
            return self.commanded(t)
        if self.source == "gait":
            return self.from_gait(kw["foot_rel_stance"], kw.get("prev_rel_stance"),
                                  kw.get("dt", 0.01))
        raise ValueError(f"unknown source {self.source!r}")
