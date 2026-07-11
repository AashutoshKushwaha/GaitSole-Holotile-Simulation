using System.Collections.Generic;
using UnityEngine;
using HoloTile;
using HoloTile.Mechanism;

namespace HoloTile.PhysicsSim
{
    /// <summary>
    /// Phase C: frictionless tile pads + analytic belt drive on test foot puck(s).
    /// Mirrors holotile_sim sim_world._apply_belt_drive + step_driven loop.
    /// </summary>
    public class FloorPhysicsController : MonoBehaviour
    {
        FloorGrid _grid;
        readonly List<FootPuck> _pucks = new List<FootPuck>();
        bool _walkerBelt;

        public IReadOnlyList<FootPuck> Pucks => _pucks;

        public void Initialize(FloorGrid grid, bool spawnTestPuck, bool walkerBelt = false)
        {
            _grid = grid;
            _walkerBelt = walkerBelt;
            if (spawnTestPuck)
                SpawnPuck("0", Vector3.zero);
        }

        public FootPuck SpawnPuck(string id, Vector3 worldXZ)
        {
            var go = new GameObject();
            go.transform.SetParent(transform, false);
            var puck = go.AddComponent<FootPuck>();
            float y = HolotileConfig.SupportY
                + HolotileConfig.PuckHalfExtents.y
                + HolotileConfig.PuckStartClear;
            puck.Initialize(id, new Vector3(worldXZ.x, y, worldXZ.z));
            _pucks.Add(puck);
            return puck;
        }

        public void SpawnWalkerFeet(out FootPuck footLeft, out FootPuck footRight)
        {
            _pucks.Clear();
            footLeft = SpawnPuck("l", new Vector3(0f, 0f, HolotileConfig.HipHalfWidth));
            footRight = SpawnPuck("r", new Vector3(0f, 0f, -HolotileConfig.HipHalfWidth));
        }

        public void ApplyAll()
        {
            if (_grid == null)
                return;

            foreach (var puck in _pucks)
                puck.ApplyBeltDrive(_grid, _walkerBelt);
        }

        [SerializeField] bool _autoDrive = true;

        public bool AutoDrive
        {
            get => _autoDrive;
            set => _autoDrive = value;
        }

        void FixedUpdate()
        {
            if (_autoDrive)
                ApplyAll();
        }
    }
}
