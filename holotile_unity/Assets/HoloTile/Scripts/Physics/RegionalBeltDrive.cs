using UnityEngine;
using HoloTile;

namespace HoloTile.PhysicsSim
{
    public static class RegionalBeltDrive
    {
        public static void ApplyAtPoint(Rigidbody body, Vector3 worldPoint, Vector3 surfaceVelocity,
            float normalForce)
        {
            if (body == null || normalForce <= 0f)
                return;

            Vector3 vel = body.GetPointVelocity(worldPoint);
            Vector3 slip = new Vector3(vel.x - surfaceVelocity.x, 0f, vel.z - surfaceVelocity.z);
            Vector3 force = -HolotileConfig.RegionBeltK * slip;
            float fmag = new Vector2(force.x, force.z).magnitude;
            float fmax = HolotileConfig.RegionBeltMu * normalForce;
            if (fmag > fmax && fmag > 1e-9f)
                force *= fmax / fmag;

            body.AddForceAtPosition(force, worldPoint, ForceMode.Force);
        }
    }
}
