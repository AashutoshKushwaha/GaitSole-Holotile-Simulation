"""
Synthetic 'live' sensor for the intended-velocity estimate.

Stands in for real tracking hardware (camera/lidar/IMU) until that's wired up: it
observes the true signal but returns it DELAYED (sensor + processing latency) and
NOISY -- the reactive estimate. Its weakness is lag: at a direction change it
reports the turn late. The model term (anticipatory) compensates; fusing the two
with tunable weights is the project's thesis.
"""

from collections import deque

import numpy as np


class LiveSensor:
    def __init__(self, latency_frames=4, noise_std=0.04, seed=0):
        self.latency = int(latency_frames)
        self.noise_std = noise_std
        self.rng = np.random.default_rng(seed)
        self.buf = deque(maxlen=self.latency + 1)

    def measure(self, true_vec):
        """Push the current truth, return the delayed + noisy estimate."""
        v = np.asarray(true_vec, float)
        self.buf.append(v.copy())
        delayed = self.buf[0]                       # oldest = latency frames ago
        return delayed + self.rng.normal(0.0, self.noise_std, size=v.shape)
