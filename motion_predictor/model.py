"""
The predictor network.

A deliberately LIGHTWEIGHT model: an MLP over the flattened observation window
(siMLPe-style "back to MLP" philosophy -- a simple all-MLP net is competitive
with heavy GCN/transformer/diffusion models on short-horizon human motion
prediction, while running in well under a millisecond on CPU). That speed is
the whole point: "minimum latency" prediction.

Three output heads share the trunk:
  * pose head     -> next-frame joint-angle/pelvis RESIDUALS  (H_OUT x 12)
  * rootvel head  -> root horizontal velocity                 (H_OUT x 2)
  * kinetics head -> foot force + moment                      (H_OUT x 12)

Everything is in NORMALIZED space; data.invert_stats() maps back to physical
units (radians, m/s, body-weight, metres) at inference.
"""

import torch
import torch.nn as nn

import config as C


class MotionPredictor(nn.Module):
    def __init__(self, in_dim=C.IN_DIM, t_in=C.T_IN, h_out=C.H_OUT,
                 hidden=C.HIDDEN, n_layers=C.N_LAYERS, dropout=C.DROPOUT,
                 pose_dim=C.OUT_POSE_DIM, rootvel_dim=C.OUT_ROOTVEL_DIM,
                 kin_dim=C.OUT_KIN_DIM):
        super().__init__()
        self.t_in = t_in
        self.h_out = h_out
        self.pose_dim = pose_dim
        self.rootvel_dim = rootvel_dim
        self.kin_dim = kin_dim

        flat = in_dim * t_in
        trunk = [nn.Linear(flat, hidden), nn.GELU()]
        for _ in range(n_layers - 1):
            trunk += [nn.Linear(hidden, hidden), nn.GELU()]
            if dropout > 0:
                trunk += [nn.Dropout(dropout)]
        self.trunk = nn.Sequential(*trunk)

        self.head_pose = nn.Linear(hidden, h_out * pose_dim)
        self.head_rootvel = nn.Linear(hidden, h_out * rootvel_dim)
        self.head_kin = nn.Linear(hidden, h_out * kin_dim)

    def forward(self, x):
        # x: [B, T_IN, IN_DIM]
        b = x.shape[0]
        h = self.trunk(x.reshape(b, -1))
        return {
            "pose": self.head_pose(h).reshape(b, self.h_out, self.pose_dim),
            "rootvel": self.head_rootvel(h).reshape(b, self.h_out, self.rootvel_dim),
            "kin": self.head_kin(h).reshape(b, self.h_out, self.kin_dim),
        }

    @torch.no_grad()
    def predict_one(self, window):
        """Single-sample inference helper for the streaming demo.
        window: [T_IN, IN_DIM] (already normalized) tensor. Returns the dict
        with batch dim removed."""
        self.eval()
        out = self.forward(window.unsqueeze(0))
        return {k: v.squeeze(0) for k, v in out.items()}


def count_params(model):
    return sum(p.numel() for p in model.parameters())
