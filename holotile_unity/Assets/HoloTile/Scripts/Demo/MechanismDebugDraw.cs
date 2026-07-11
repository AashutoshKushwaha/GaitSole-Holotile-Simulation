using UnityEngine;
using HoloTile;
using HoloTile.Math;
using HoloTile.Mechanism;

namespace HoloTile.Demo
{
    /// <summary>
    /// Debug gizmos for Phase D: tilt axis, surface velocity arrow, rim contact points.
    /// </summary>
    public static class MechanismDebugDraw
    {
        public static void DrawFloor(FloorGrid floor)
        {
            if (floor == null) return;

            foreach (var kv in floor.Tiles)
            {
                var tile = kv.Value;
                DrawTile(tile);
            }
        }

        static void DrawTile(ActiveTile tile)
        {
            Vector3 centre = tile.CenterWorld + Vector3.up * HolotileConfig.SupportY;
            var cmd = tile.Command;
            Vector3 vel = cmd.SurfaceVelocity;

            // Surface velocity arrow at tile centre.
            Gizmos.color = HolotileConfig.VelocityColor;
            if (vel.sqrMagnitude > 1e-8f)
                Gizmos.DrawRay(centre, vel * 0.5f);

            // Sample first disk for tilt axis + rim.
            if (tile.Disks.Count == 0) return;
            var disk = tile.Disks[0];
            Vector3 diskCentre = disk.transform.position;
            Vector3 axis = DiskKinematics.SpinAxisWorld(cmd.AzimuthRad);

            Gizmos.color = HolotileConfig.TiltAxisColor;
            Gizmos.DrawRay(diskCentre, axis * HolotileConfig.DiskRadius * 0.8f);

            Gizmos.color = HolotileConfig.RimColor;
            Gizmos.DrawSphere(disk.RimWorldPosition(), HolotileConfig.DiskRadius * 0.06f);
        }
    }
}
