using UnityEngine;
using HoloTile;
using HoloTile.Math;
using HoloTile.Mechanism;

namespace HoloTile.OpenSim
{
    /// <summary>
    /// Per-tile disc commands from 4-region GRF samples under each foot.
    /// </summary>
    public static class RegionalTileCommands
    {
        public static void ApplyToFloor(FloorGrid floor, WalkFrame frame, SkeletonPose pose,
            TileCommand fallbackCenter, float spinMax)
        {
            floor.SetAllTilesIdle(fallbackCenter.AzimuthRad);

            ApplyFootRegions(floor, frame, pose, side: 0, pose.AnkleR, pose.ToeR,
                -HolotileConfig.HipHalfWidth, spinMax);
            ApplyFootRegions(floor, frame, pose, side: 1, pose.AnkleL, pose.ToeL,
                HolotileConfig.HipHalfWidth, spinMax);

            // Centering command on tiles under each stance foot.
            if (FootLoad(frame, 0) > HolotileConfig.StanceGrfThreshold)
                floor.SetTileAtWorld(pose.AnkleR, fallbackCenter);
            if (FootLoad(frame, 1) > HolotileConfig.StanceGrfThreshold)
                floor.SetTileAtWorld(pose.AnkleL, BlendCommands(fallbackCenter, frame, 1, spinMax));
        }

        static void ApplyFootRegions(FloorGrid floor, WalkFrame frame, SkeletonPose pose, int side,
            Vector3 ankle, Vector3 toe, float lateralZ, float spinMax)
        {
            for (int r = 0; r < 4; r++)
            {
                var grf = frame.Regions[side, r];
                Vector3 v = RegionForceMapper.SurfaceVelocityFromGrf(grf);
                if (v.sqrMagnitude < 1e-5f)
                    continue;

                Vector3 world = OpenSimWalkFK.RegionWorldPoint(ankle, toe, r, lateralZ);
                DiskKinematics.CommandFromVelocity(v, 0f, spinMax, out float az, out float spin);
                floor.SetTileAtWorld(world, new TileCommand(az, spin));
            }
        }

        static TileCommand BlendCommands(TileCommand center, WalkFrame frame, int side, float spinMax)
        {
            Vector3 sum = center.SurfaceVelocity;
            int n = 1;
            for (int r = 0; r < 4; r++)
            {
                Vector3 v = RegionForceMapper.SurfaceVelocityFromGrf(frame.Regions[side, r]);
                if (v.sqrMagnitude < 1e-5f) continue;
                sum += v;
                n++;
            }

            DiskKinematics.CommandFromVelocity(sum / n, center.AzimuthRad, spinMax, out float az, out float spin);
            return new TileCommand(az, spin);
        }

        public static float FootLoad(WalkFrame frame, int side)
        {
            float sum = 0f;
            for (int r = 0; r < 4; r++)
                sum += Mathf.Max(0f, frame.Regions[side, r].Force.y);
            return sum;
        }

        public static void FillPatchVelocities(WalkFrame frame, int side, Vector3[] outVelocities)
        {
            for (int r = 0; r < 4; r++)
                outVelocities[r] = RegionForceMapper.SurfaceVelocityFromGrf(frame.Regions[side, r]);
        }
    }
}
