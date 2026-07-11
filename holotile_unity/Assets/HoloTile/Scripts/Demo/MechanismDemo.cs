using UnityEngine;
using HoloTile;
using HoloTile.Math;
using HoloTile.Mechanism;
using HoloTile.PhysicsSim;

namespace HoloTile.Demo
{
    /// <summary>
    /// Phase D entry point: build the patent disk-stack mechanism and drive it.
    ///
    /// Modes:
    ///   Scripted — azimuth rotates at DemoAzimuthHz, constant spin (like run_demo.py).
    ///   Manual   — use inspector AzimuthDeg / SpinRadPerSec sliders.
    ///
    /// Attach to an empty GameObject in a blank scene. No prefabs required.
    /// </summary>
    public class MechanismDemo : MonoBehaviour
    {
        public enum DriveMode
        {
            Scripted,
            Manual
        }

        [Header("Floor size")]
        [Tooltip("Start with 1x1 for M1 proof; set 8x8 for full modular floor.")]
        [SerializeField] int tilesX = HolotileConfig.DefaultTilesX;
        [SerializeField] int tilesY = HolotileConfig.DefaultTilesY;

        [Header("Drive")]
        [SerializeField] DriveMode driveMode = DriveMode.Scripted;
        [SerializeField] bool runOnPlay = true;

        [Header("Manual command (DriveMode = Manual)")]
        [SerializeField] float azimuthDeg;
        [SerializeField] float spinRadPerSec = HolotileConfig.DemoSpinRadPerSec;

        [Header("Scripted command (DriveMode = Scripted)")]
        [SerializeField] float scriptedAzimuthHz = HolotileConfig.DemoAzimuthHz;
        [SerializeField] float scriptedSpinRadPerSec = HolotileConfig.DemoSpinRadPerSec;

        [Header("Debug")]
        [SerializeField] bool showDebugGizmos = true;
        [SerializeField] bool logCommandEachSecond;

        [Header("Phase C — belt physics")]
        [Tooltip("Frictionless tile pads + moving-surface belt drive on a test foot puck.")]
        [SerializeField] bool enableBeltPhysics;
        [Tooltip("Spawn a red shoe puck at floor centre (needs room to translate — use 3×3+ tiles).")]
        [SerializeField] bool spawnTestPuck = true;

        FloorGrid _floor;
        FloorPhysicsController _physics;
        float _elapsed;
        float _logTimer;

        public FloorGrid Floor => _floor;

        void Start()
        {
            if (runOnPlay)
                BuildAndRun();
        }

        [ContextMenu("Build Floor")]
        public void BuildAndRun()
        {
            if (_floor != null)
                Destroy(_floor.gameObject);
            if (_physics != null)
                Destroy(_physics.gameObject);

            if (enableBeltPhysics)
                Time.fixedDeltaTime = HolotileConfig.ControlDt;

            var root = new GameObject("HoloTileMechanism");
            root.transform.SetParent(transform, false);
            _floor = root.AddComponent<FloorGrid>();
            _floor.Build(tilesX, tilesY, enableBeltPhysics);

            if (enableBeltPhysics)
            {
                var physGo = new GameObject("FloorPhysics");
                physGo.transform.SetParent(root.transform, false);
                _physics = physGo.AddComponent<FloorPhysicsController>();
                _physics.Initialize(_floor, spawnTestPuck);
            }

            SetupCamera();
            string phase = enableBeltPhysics ? "Phase C (belt physics)" : "Phase D (kinematics)";
            Debug.Log($"[HoloTile] Built {tilesX}x{tilesY} floor ({phase}). " +
                      $"Tilt={HolotileConfig.TiltDeg} deg, R={HolotileConfig.DiskRadius:F4} m, " +
                      $"DRAG_PER_OMEGA={HolotileConfig.DragPerOmega:F4} m/s per rad/s");
        }

        void Update()
        {
            if (_floor == null) return;

            float dt = Time.deltaTime;
            _elapsed += dt;

            TileCommand cmd = driveMode == DriveMode.Scripted
                ? ScriptedCommand(_elapsed, scriptedAzimuthHz, scriptedSpinRadPerSec)
                : new TileCommand(azimuthDeg * Mathf.Deg2Rad, spinRadPerSec);

            _floor.SetAllTiles(cmd);
            _floor.Tick(dt);

            if (logCommandEachSecond)
            {
                _logTimer += dt;
                if (_logTimer >= 1f)
                {
                    _logTimer = 0f;
                    Debug.Log($"[HoloTile] {cmd}");
                }
            }
        }

        static TileCommand ScriptedCommand(float t, float azimuthHz, float spin)
        {
            float alpha = 2f * Mathf.PI * azimuthHz * t;
            return new TileCommand(alpha, spin);
        }

        void SetupCamera()
        {
            var cam = Camera.main;
            if (cam == null) return;

            float extent = Mathf.Max(tilesX, tilesY) * (HolotileConfig.TileSize + HolotileConfig.TileGap);
            float lookY = HolotileConfig.SupportY + 0.005f;

            // Oblique view: mostly see the flush plate; bumps visible as subtle motion.
            cam.transform.position = new Vector3(extent * 0.85f, extent * 0.55f, -extent * 0.75f);
            cam.transform.LookAt(new Vector3(0f, lookY, 0f));
            cam.nearClipPlane = 0.005f;
            cam.fieldOfView = 45f;
        }

        void OnDrawGizmos()
        {
            if (!showDebugGizmos || _floor == null) return;
            MechanismDebugDraw.DrawFloor(_floor);
        }
    }
}
