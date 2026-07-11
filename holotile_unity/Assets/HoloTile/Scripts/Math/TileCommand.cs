using System;
using UnityEngine;
using HoloTile;

namespace HoloTile.Math
{
    /// <summary>
    /// Per-tile operating command [Claim 1]: orient disks (azimuth) then spin.
    /// All disks in one active tile share the same command [§0014].
    /// </summary>
    [Serializable]
    public struct TileCommand
    {
        /// <summary>Azimuth alpha (rad). Swashplate angle; sets push direction.</summary>
        public float AzimuthRad;

        /// <summary>Spin rate omega (rad/s). Sets push speed.</summary>
        public float SpinRadPerSec;

        public TileCommand(float azimuthRad, float spinRadPerSec)
        {
            AzimuthRad = azimuthRad;
            SpinRadPerSec = spinRadPerSec;
        }

        public static TileCommand Zero => new TileCommand(0f, 0f);

        public Vector3 SurfaceVelocity => DiskKinematics.SurfaceVelocity(AzimuthRad, SpinRadPerSec);

        public float SurfaceSpeed => DiskKinematics.SurfaceSpeed(SpinRadPerSec);

        public void ClampSpin()
        {
            SpinRadPerSec = Mathf.Clamp(SpinRadPerSec, -HolotileConfig.SpinMax, HolotileConfig.SpinMax);
        }

        public override string ToString()
        {
            return $"TileCommand(azi={AzimuthRad * Mathf.Rad2Deg:F1} deg, " +
                   $"spin={SpinRadPerSec:F1} rad/s, |v|={SurfaceSpeed:F3} m/s)";
        }
    }
}
