"""
Weighted fusion of the live-sensor estimate and the model's anticipatory estimate
-- the heart of the project: the floor controller's "predicted travel path" is no
longer live-only; it is a tunable blend of live data and the trained model.

    v_fused = w_live * v_live + w_model * v_model        (weights normalized)

Trade-off the weights expose:
  * live  -> accurate but LAGS (sensor latency): late at turns.
  * model -> ANTICIPATES (leads) but carries model error.
A blend beats either alone; the best weight is intermediate (see the M5 sweep).
"""

import numpy as np


class VelocityFusion:
    def __init__(self, w_live=0.5, w_model=0.5):
        s = w_live + w_model
        self.w_live = w_live / s if s > 0 else 0.5
        self.w_model = w_model / s if s > 0 else 0.5

    def fuse(self, v_live, v_model):
        return self.w_live * np.asarray(v_live, float) + \
               self.w_model * np.asarray(v_model, float)
