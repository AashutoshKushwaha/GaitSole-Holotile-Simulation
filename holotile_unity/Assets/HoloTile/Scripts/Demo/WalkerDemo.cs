using UnityEngine;
using HoloTile;
using HoloTile.Control;
using HoloTile.Math;
using HoloTile.Mechanism;
using HoloTile.PhysicsSim;

namespace HoloTile.Demo
{
    /// <summary>
    /// M4 entry point: walking person kept centered via floor controller + belt physics.
    /// Mirrors holotile_sim/run_control.py.
    ///
    /// Attach to an empty GameObject (instead of MechanismDemo). Requires belt physics.
    /// </summary>
    [DefaultExecutionOrder(-50)]
    public class WalkerDemo : MonoBehaviour
    {
        [Header("Floor")]
        [SerializeField] int tilesX = HolotileConfig.WalkerDemoTiles;
        [SerializeField] int tilesY = HolotileConfig.WalkerDemoTiles;

        [Header("Controller")]
        [Tooltip("When off, the person walks off the floor (compare demo).")]
        [SerializeField] bool controllerEnabled = true;
        [SerializeField] bool runOnPlay = true;

        [Header("Disc / gait tuning")]
        [SerializeField] WalkerTuning tuning = new WalkerTuning();
        [Tooltip("Update sliders while playing (no rebuild needed).")]
        [SerializeField] bool liveTuneInPlayMode = true;

        [Header("Debug")]
        [SerializeField] bool showDebugGizmos = true;
        [SerializeField] bool logDriftEachSecond = true;

        FloorGrid _floor;
        FloorPhysicsController _physics;
        WalkerSim _walker;
        FootPuck _footLeft;
        FootPuck _footRight;
        bool _running;
        bool _hasPhysicsStep;
        float _logTimer;
        int _settleRemaining;
        SimulationMode _prevSimulationMode;

        public FloorGrid Floor => _floor;
        public WalkerSim Walker => _walker;

        void Start()
        {
            if (runOnPlay)
                BuildAndRun();
        }

        void OnDestroy()
        {
            Physics.simulationMode = _prevSimulationMode;
        }

        [ContextMenu("Build Walker Demo")]
        public void BuildAndRun()
        {
            if (_floor != null)
                Destroy(_floor.gameObject);
            if (_physics != null)
                Destroy(_physics.gameObject);

            Time.fixedDeltaTime = HolotileConfig.ControlDt;
            _prevSimulationMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;

            var root = new GameObject("HoloTileWalker");
            root.transform.SetParent(transform, false);
            _floor = root.AddComponent<FloorGrid>();
            _floor.Build(tilesX, tilesY, physicsSupport: true);

            var physGo = new GameObject("FloorPhysics");
            physGo.transform.SetParent(root.transform, false);
            _physics = physGo.AddComponent<FloorPhysicsController>();
            _physics.Initialize(_floor, spawnTestPuck: false, walkerBelt: true);
            _physics.AutoDrive = false;
            _physics.SpawnWalkerFeet(out _footLeft, out _footRight);

            _walker = new WalkerSim(HolotileConfig.ControlDt, tuning)
            {
                ControllerEnabled = controllerEnabled
            };
            _walker.ResetDriftStats();
            _settleRemaining = HolotileConfig.WalkerSettleSteps;
            _hasPhysicsStep = false;
            _running = true;

            SetupCamera();
            Debug.Log($"[HoloTile] Walker demo: {tilesX}x{tilesY} floor, controller " +
                      $"{(controllerEnabled ? "ON" : "OFF")}. " +
                      "Hybrid belt model (not literal disk contact). Walk +X then turn +Z.");
        }

        void FixedUpdate()
        {
            if (!_running || _floor == null || _physics == null || _walker == null)
                return;

            float dt = HolotileConfig.ControlDt;

            if (_settleRemaining > 0)
            {
                _floor.SetAllTiles(TileCommand.Zero);
                _floor.Tick(dt);
                StepPhysics();
                _settleRemaining--;
                if (_settleRemaining == 0)
                    _walker.BeginWalking(_footLeft, _footRight);
                return;
            }

            if (_hasPhysicsStep)
                _walker.PostPhysicsTick(_footLeft, _footRight, dt);

            if (liveTuneInPlayMode)
                _walker.ApplyTuning(tuning);

            TileCommand cmd = _walker.PrePhysicsTick(_footLeft, _footRight, dt);
            _floor.SetAllTiles(cmd);
            _floor.Tick(dt);
            StepPhysics();
            _hasPhysicsStep = true;

            if (logDriftEachSecond)
            {
                _logTimer += dt;
                if (_logTimer >= 1f)
                {
                    _logTimer = 0f;
                    Vector2 p = _walker.PelvisXz;
                    Debug.Log($"[HoloTile] t={_walker.Time:F1}s pelvis=({p.x:F3},{p.y:F3}) m  " +
                              $"max drift={_walker.MaxDrift:F3} m  {cmd}");
                }
            }
        }

        void StepPhysics()
        {
            int substeps = HolotileConfig.PhysicsSubsteps;
            float subDt = HolotileConfig.ControlDt / substeps;
            for (int i = 0; i < substeps; i++)
            {
                _physics.ApplyAll();
                Physics.Simulate(subDt);
            }
        }

        void SetupCamera()
        {
            var cam = Camera.main;
            if (cam == null) return;

            float extent = Mathf.Max(tilesX, tilesY) * (HolotileConfig.TileSize + HolotileConfig.TileGap);
            cam.transform.position = new Vector3(extent * 0.55f, extent * 0.65f, -extent * 0.85f);
            cam.transform.LookAt(new Vector3(0f, HolotileConfig.SupportY + 0.85f, 0f));
            cam.nearClipPlane = 0.01f;
            cam.fieldOfView = 50f;
        }

        void OnDrawGizmos()
        {
            if (!showDebugGizmos || _walker == null)
                return;

            Vector2 pelvis = _walker.PelvisXz;
            float y = HolotileConfig.SupportY + 0.85f;
            Vector3 pelvisWorld = new Vector3(pelvis.x, y, pelvis.y);

            Gizmos.color = HolotileConfig.PelvisColor;
            Gizmos.DrawSphere(pelvisWorld, 0.05f);

            Gizmos.color = HolotileConfig.CenterMarkerColor;
            Gizmos.DrawLine(new Vector3(-0.08f, y, 0f), new Vector3(0.08f, y, 0f));
            Gizmos.DrawLine(new Vector3(0f, y, -0.08f), new Vector3(0f, y, 0.08f));

            if (_floor != null)
                MechanismDebugDraw.DrawFloor(_floor);
        }
    }
}
