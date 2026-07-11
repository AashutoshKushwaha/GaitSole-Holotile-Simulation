using System;
using UnityEngine;
using HoloTile;

namespace HoloTile.Control
{
    /// <summary>Inspector-tunable walker / disc limits (runtime overrides for M4 demo).</summary>
    [Serializable]
    public class WalkerTuning
    {
        [Tooltip("Intended walking speed (m/s). Lower = slower gait and lower disc demand.")]
        [Range(0.05f, 1.0f)]
        public float walkSpeed = HolotileConfig.WalkSpeed;

        [Tooltip("Max disk spin rate ω (rad/s). Surface speed ≈ ω × DRAG_PER_OMEGA.")]
        [Range(5f, 150f)]
        public float spinMaxRadPerSec = HolotileConfig.SpinMax;

        [Tooltip("Position gain for keep-centered controller. Lower = gentler disc response.")]
        [Range(1f, 12f)]
        public float controllerKp = HolotileConfig.FloorControllerKp;

        public float MaxSurfaceSpeed => spinMaxRadPerSec * HolotileConfig.DragPerOmega;

        public static WalkerTuning Default => new WalkerTuning();
    }
}
