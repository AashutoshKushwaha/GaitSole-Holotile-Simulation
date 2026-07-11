using UnityEngine;
using HoloTile;
using HoloTile.Math;

namespace HoloTile.OpenSim
{
    /// <summary>
    /// Map OpenSim 4-region GRF to local disc surface velocity commands.
    /// OpenSim 2D: Fx = horizontal, Fy = vertical. Unity floor = XZ.
    /// </summary>
    public static class RegionForceMapper
    {
        public static Vector3 SurfaceVelocityFromGrf(RegionGrf grf, float weightScale = 1f)
        {
            // Tangential command: carry foot opposite horizontal GRF / slip direction.
            var horizontal = new Vector3(grf.Force.x, 0f, 0f);
            float normal = Mathf.Max(grf.Force.y, 0f);
            if (horizontal.sqrMagnitude < 1e-3f && normal < 1f)
                return Vector3.zero;

            float speed = Mathf.Clamp(horizontal.magnitude / 400f, 0f, HolotileConfig.MaxSurfaceSpeed);
            if (speed < 1e-4f)
                return Vector3.zero;

            return -horizontal.normalized * speed * weightScale;
        }

        public static TileCommand BlendRegionalCommands(Vector3[] patchVelocities, float prevAzimuth)
        {
            Vector3 sum = Vector3.zero;
            int n = 0;
            foreach (var v in patchVelocities)
            {
                if (v.sqrMagnitude < 1e-6f) continue;
                sum += v;
                n++;
            }

            if (n == 0)
                return new TileCommand(prevAzimuth, 0f);

            Vector3 avg = sum / n;
            DiskKinematics.CommandFromVelocity(avg, prevAzimuth, HolotileConfig.SpinMax,
                out float azi, out float spin);
            return new TileCommand(azi, spin);
        }
    }
}
