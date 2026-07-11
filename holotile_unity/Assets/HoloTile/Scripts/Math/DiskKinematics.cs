using UnityEngine;
using HoloTile;

namespace HoloTile.Math
{
    /// <summary>
    /// Pure kinematics for the patent disk-stack embodiment [FIG 11-15, §0087-0092].
    ///
    /// State per disk assembly (per operating period [Claim 1]):
    ///   alpha = azimuth (rad) — swashplate rotation about vertical Y; sets push direction.
    ///   omega = spin rate (rad/s) — rotation about tilted axis; sets push speed.
    ///
    /// All functions are side-effect free. Unity horizontal plane is XZ, up is Y.
    /// </summary>
    public static class DiskKinematics
    {
        // -----------------------------------------------------------------
        // Spin axis
        // -----------------------------------------------------------------

        /// <summary>
        /// Unit spin axis in world frame after azimuth alpha is applied.
        /// Tilt theta is fixed; axis lies in a vertical plane through push direction.
        ///
        /// MuJoCo holotile_sim body-frame axis (sin θ, 0, cos θ) rotated by azimuth
        /// about Z-up maps to Unity: (sin θ cos α, cos θ, sin θ sin α).
        /// </summary>
        public static Vector3 SpinAxisWorld(float azimuthRad)
        {
            float s = Mathf.Sin(HolotileConfig.TiltRad);
            float c = Mathf.Cos(HolotileConfig.TiltRad);
            float ca = Mathf.Cos(azimuthRad);
            float sa = Mathf.Sin(azimuthRad);
            return new Vector3(s * ca, c, s * sa).normalized;
        }

        // -----------------------------------------------------------------
        // Raised rim contact point
        // -----------------------------------------------------------------

        /// <summary>
        /// World-space offset from disk centre to the raised rim contact point
        /// (highest point on the contact sphere / rim), before spin phase.
        ///
        /// With tilt about local X after azimuth, the high point is along +Y
        /// in the tilted disk frame, at radius R from centre.
        /// </summary>
        public static Vector3 RimOffsetWorld(float azimuthRad)
        {
            float r = HolotileConfig.DiskRadius;
            float theta = HolotileConfig.TiltRad;
            float ca = Mathf.Cos(azimuthRad);
            float sa = Mathf.Sin(azimuthRad);
            // Rim "up" in world: azimuth rotates the tilt plane.
            return new Vector3(-r * Mathf.Sin(theta) * sa, r * Mathf.Cos(theta), -r * Mathf.Sin(theta) * ca);
        }

        /// <summary>
        /// Rim contact world position given disk centre, azimuth, and spin phase phi.
        /// phi = integral of omega dt; rim orbits about spin axis.
        /// </summary>
        public static Vector3 RimContactPosition(Vector3 diskCenterWorld, float azimuthRad,
            float spinPhaseRad)
        {
            Vector3 offset = RimOffsetWorld(azimuthRad);
            Vector3 axis = SpinAxisWorld(azimuthRad);
            return diskCenterWorld + Quaternion.AngleAxis(spinPhaseRad * Mathf.Rad2Deg, axis) * offset;
        }

        // -----------------------------------------------------------------
        // Surface velocity (what the floor imparts at the contact point)
        // -----------------------------------------------------------------

        /// <summary>
        /// Horizontal surface velocity (Y component = 0) at the raised contact point.
        ///
        /// v = omega x r_rim  projected to XZ.
        /// Equivalent closed form (holotile_sim parity):
        ///   speed = omega * DRAG_PER_OMEGA
        ///   v = speed * (sin alpha, 0, -cos alpha)
        /// </summary>
        public static Vector3 SurfaceVelocity(float azimuthRad, float spinRadPerSec)
        {
            float speed = spinRadPerSec * HolotileConfig.DragPerOmega;
            return new Vector3(speed * Mathf.Sin(azimuthRad), 0f, -speed * Mathf.Cos(azimuthRad));
        }

        /// <summary>Speed magnitude of surface velocity (m/s).</summary>
        public static float SurfaceSpeed(float spinRadPerSec)
        {
            return Mathf.Abs(spinRadPerSec) * HolotileConfig.DragPerOmega;
        }

        // -----------------------------------------------------------------
        // Inverse: desired velocity -> disk command
        // -----------------------------------------------------------------

        /// <summary>
        /// Invert SurfaceVelocity: find (azimuth, spin) for a desired horizontal velocity.
        /// Mirrors holotile_sim floor_controller.surface_to_command.
        /// </summary>
        public static void CommandFromVelocity(Vector3 desiredVelocityXZ, float prevAzimuthRad,
            float spinMaxRadPerSec, out float azimuthRad, out float spinRadPerSec)
        {
            desiredVelocityXZ.y = 0f;
            float mag = desiredVelocityXZ.magnitude;
            if (mag < 1e-4f)
            {
                azimuthRad = prevAzimuthRad;
                spinRadPerSec = 0f;
                return;
            }

            spinRadPerSec = Mathf.Clamp(mag / HolotileConfig.DragPerOmega, -spinMaxRadPerSec,
                spinMaxRadPerSec);
            azimuthRad = Mathf.Atan2(desiredVelocityXZ.x, -desiredVelocityXZ.z);
        }

        /// <summary>Uses <see cref="HolotileConfig.SpinMax"/>.</summary>
        public static void CommandFromVelocity(Vector3 desiredVelocityXZ, float prevAzimuthRad,
            out float azimuthRad, out float spinRadPerSec)
        {
            CommandFromVelocity(desiredVelocityXZ, prevAzimuthRad, HolotileConfig.SpinMax,
                out azimuthRad, out spinRadPerSec);
        }

        // -----------------------------------------------------------------
        // Actuator slew limits (Phase C; available for smooth demo transitions)
        // -----------------------------------------------------------------

        public static float SlewAngle(float prevRad, float targetRad, float maxStepRad)
        {
            float delta = Mathf.DeltaAngle(prevRad * Mathf.Rad2Deg, targetRad * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            delta = Mathf.Clamp(delta, -maxStepRad, maxStepRad);
            return prevRad + delta;
        }

        public static float SlewSpin(float prev, float target, float maxStep)
        {
            return Mathf.Clamp(target, prev - maxStep, prev + maxStep);
        }

        // -----------------------------------------------------------------
        // Transform helpers for mechanism hierarchy
        // -----------------------------------------------------------------

        /// <summary>Swashplate: rotation about world Y by azimuth (degrees).</summary>
        public static Quaternion AzimuthRotation(float azimuthRad)
        {
            return Quaternion.Euler(0f, azimuthRad * Mathf.Rad2Deg, 0f);
        }

        /// <summary>Fixed tilt: rotate disk plane by theta about local X (degrees).</summary>
        public static Quaternion TiltRotation()
        {
            return Quaternion.Euler(HolotileConfig.TiltDeg, 0f, 0f);
        }

        /// <summary>Spin: rotation about tilted axis by spin phase (degrees).</summary>
        public static Quaternion SpinRotation(float azimuthRad, float spinPhaseRad)
        {
            Vector3 axis = SpinAxisWorld(azimuthRad);
            return Quaternion.AngleAxis(spinPhaseRad * Mathf.Rad2Deg, axis);
        }
    }
}
