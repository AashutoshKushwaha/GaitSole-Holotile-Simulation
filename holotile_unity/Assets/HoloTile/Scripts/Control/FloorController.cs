using UnityEngine;
using HoloTile;
using HoloTile.Math;

namespace HoloTile.Control
{
    /// <summary>
    /// Patent keep-centered logic — drives stance foot via disk surface velocity.
    /// Mirrors holotile_sim/floor_controller.py.
    /// </summary>
    public class FloorController
    {
        readonly float _dt;
        float _kP;
        float _spinMax;
        readonly Vector2 _center;

        float _prevAzimuthRad;
        float _prevSpinRadPerSec;

        public float PrevAzimuthRad => _prevAzimuthRad;
        public float PrevSpinRadPerSec => _prevSpinRadPerSec;

        public FloorController(float dt, float kP = HolotileConfig.FloorControllerKp,
            float spinMax = HolotileConfig.SpinMax, Vector2? center = null)
        {
            _dt = dt;
            _kP = kP;
            _spinMax = spinMax;
            _center = center ?? Vector2.zero;
        }

        public void ApplyTuning(WalkerTuning tuning)
        {
            _kP = tuning.controllerKp;
            _spinMax = tuning.spinMaxRadPerSec;
        }

        /// <summary>
        /// Drive stance foot so pelvis stays at centre.
        /// target = center + footRelStance; V = footRelVel + k_p * (target - footXz).
        /// </summary>
        public TileCommand CommandStance(Vector2 footXz, Vector2 footRelStance, Vector2 footRelVel,
            out Vector3 desiredVelocity)
        {
            Vector2 target = _center + footRelStance;
            Vector2 v = footRelVel + _kP * (target - footXz);
            float maxSpeed = _spinMax * HolotileConfig.DragPerOmega;
            if (v.sqrMagnitude > maxSpeed * maxSpeed)
                v = v.normalized * maxSpeed;
            desiredVelocity = new Vector3(v.x, 0f, v.y);

            DiskKinematics.CommandFromVelocity(desiredVelocity, _prevAzimuthRad, _spinMax,
                out float azimuthRad, out float spinRadPerSec);

            azimuthRad = DiskKinematics.SlewAngle(_prevAzimuthRad, azimuthRad,
                HolotileConfig.AziSlewRadPerSec * _dt);
            spinRadPerSec = DiskKinematics.SlewSpin(_prevSpinRadPerSec, spinRadPerSec,
                HolotileConfig.SpinSlewPerSec * _dt);

            _prevAzimuthRad = azimuthRad;
            _prevSpinRadPerSec = spinRadPerSec;
            return new TileCommand(azimuthRad, spinRadPerSec);
        }

        public TileCommand Idle()
        {
            _prevSpinRadPerSec = 0f;
            return new TileCommand(_prevAzimuthRad, 0f);
        }
    }
}
