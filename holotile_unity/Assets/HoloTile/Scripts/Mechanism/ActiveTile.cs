using System.Collections.Generic;
using UnityEngine;
using HoloTile;
using HoloTile.Math;

namespace HoloTile.Mechanism
{
    /// <summary>
    /// [§0046, FIG 3] One active tile — array of disk assemblies + structural pad.
    ///
    /// Physics function:
    ///   During each operating period the controller:
    ///     1) Orients all disk assemblies (shared azimuth alpha)
    ///     2) Spins all disk assemblies (shared omega)
    ///   All disks in a tile are driven identically [§0014, §0088].
    /// </summary>
    public class ActiveTile : MonoBehaviour
    {
        readonly List<DiskAssembly> _disks = new List<DiskAssembly>();
        StructuralPad _pad;
        TileCommand _command = TileCommand.Zero;

        public Vector2Int Index { get; private set; }
        public Vector3 CenterWorld { get; private set; }
        public TileCommand Command => _command;
        public IReadOnlyList<DiskAssembly> Disks => _disks;

        public void Build(Vector2Int index, Vector3 centerWorld, Transform parent, bool physicsSupport = false)
        {
            Index = index;
            CenterWorld = centerWorld;
            transform.SetParent(parent, false);
            transform.position = centerWorld;
            gameObject.name = $"ActiveTile_{index.x}_{index.y}";

            _pad = gameObject.AddComponent<StructuralPad>();
            _pad.Build(transform, physicsSupport);

            int n = HolotileConfig.DisksPerTile;
            float pitch = HolotileConfig.DiskPitch;
            float d0 = -(n - 1) * 0.5f * pitch;
            float centerY = HolotileConfig.HemisphereCenterY;

            for (int ix = 0; ix < n; ix++)
            {
                for (int iz = 0; iz < n; iz++)
                {
                    var local = new Vector3(d0 + ix * pitch, centerY, d0 + iz * pitch);
                    var go = new GameObject($"DiskAssembly_{ix}_{iz}");
                    var asm = go.AddComponent<DiskAssembly>();
                    asm.Build(transform, local);
                    _disks.Add(asm);
                }
            }
        }

        /// <summary>Apply shared tile command to every disk assembly.</summary>
        public void SetCommand(TileCommand command)
        {
            command.ClampSpin();
            _command = command;
            foreach (var disk in _disks)
                disk.SetCommand(command);
        }

        public void SetCommand(float azimuthRad, float spinRadPerSec)
        {
            SetCommand(new TileCommand(azimuthRad, spinRadPerSec));
        }

        public Vector3 SurfaceVelocity => _command.SurfaceVelocity;

        public void Tick(float dt)
        {
            foreach (var disk in _disks)
                disk.Tick(dt);
        }
    }
}
