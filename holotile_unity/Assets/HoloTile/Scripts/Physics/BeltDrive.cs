using UnityEngine;
using HoloTile;

namespace HoloTile.PhysicsSim
{
    /// <summary>
    /// Moving-surface (belt) friction — mirrors holotile_sim/sim_world._apply_belt_drive.
    /// Phase C: drives a foot Rigidbody toward the tile's commanded surface velocity.
    /// </summary>
    public static class BeltDrive
    {
        public static void Apply(Rigidbody body, Vector3 surfaceVelocity, float normalForce,
            Vector3 externalForceXZ, float beltK = -1f, float beltMu = -1f)
        {
            if (body == null) return;

            if (beltK < 0f) beltK = HolotileConfig.BeltK;
            if (beltMu < 0f) beltMu = HolotileConfig.BeltMu;

            Vector3 vel = body.linearVelocity;
            Vector3 slip = new Vector3(vel.x - surfaceVelocity.x, 0f, vel.z - surfaceVelocity.z);
            Vector3 force = -beltK * slip;
            float fmag = new Vector2(force.x, force.z).magnitude;
            float fmax = beltMu * Mathf.Max(normalForce, 0f);
            if (fmag > fmax && fmag > 1e-9f)
                force *= fmax / fmag;

            force += new Vector3(externalForceXZ.x, 0f, externalForceXZ.z);
            body.AddForce(force, ForceMode.Force);
        }

        public static float EstimateNormalForce(Rigidbody body, bool grounded)
        {
            if (!grounded || body == null)
                return 0f;
            return body.mass * Mathf.Abs(Physics.gravity.y);
        }
    }
}
