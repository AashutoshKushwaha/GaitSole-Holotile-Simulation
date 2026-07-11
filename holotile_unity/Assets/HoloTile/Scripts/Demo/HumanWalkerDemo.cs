using UnityEngine;
using HoloTile;
using HoloTile.Control;
using HoloTile.Math;
using HoloTile.Mechanism;
using HoloTile.OpenSim;
using HoloTile.PhysicsSim;

namespace HoloTile.Demo
{
    /// <summary>
    /// Phase M6: OpenSim walk replay + 4-region disc commands + clothed character.
    /// Press Play to loop the exported walk_GRF cycle on the HoloTile floor.
    /// </summary>
    [DefaultExecutionOrder(-50)]
    public class HumanWalkerDemo : MonoBehaviour
    {
        [Header("Floor")]
        [SerializeField] int tilesX = 5;
        [SerializeField] int tilesY = 5;

        [Header("Replay")]
        [SerializeField] bool loopReplay = true;
        [Tooltip("Play forward then reverse to close the half-cycle STO without a pose pop.")]
        [SerializeField] bool pingPongLoop = true;
        [SerializeField] bool centerPelvis = true;
        [SerializeField] bool useRegionalGrfCommands = true;
        [SerializeField] bool physicsCoupled = true;
        [Tooltip("1 = OpenSim cycle speed. Lower to slow walk (e.g. 0.6).")]
        [Range(0.25f, 2f)]
        [SerializeField] float replayTimeScale = 0.65f;
        [SerializeField] WalkerTuning tuning = new WalkerTuning();

        [Header("Optional high-poly character")]
        [Tooltip("Drag an FBX/Prefab from Project window onto this field.")]
        [SerializeField] GameObject customCharacterPrefab;

        [Header("Debug")]
        [SerializeField] bool logEachSecond = true;

        FloorGrid _floor;
        OpenSimReplayData _replay;
        ProceduralCharacterMesh _character;
        ImportedCharacterDriver _importedDriver;
        GameObject _customCharInstance;
        RegionalFoot _footL;
        RegionalFoot _footR;
        FloorController _floorCtrl;
        readonly Vector3[] _patchVelL = new Vector3[4];
        readonly Vector3[] _patchVelR = new Vector3[4];
        float _time;
        float _logTimer;
        float _pelvisStartX;
        SimulationMode _prevSim;

        void Start() => BuildAndPlay();

        void OnDestroy() => Physics.simulationMode = _prevSim;

        [ContextMenu("Build Human Walker")]
        public void BuildAndPlay()
        {
            _prevSim = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            Time.fixedDeltaTime = HolotileConfig.ControlDt;

            if (_floor != null)
                Destroy(_floor.gameObject);

            _replay = OpenSimReplayData.LoadFromStreamingAssets();
            _floorCtrl = new FloorController(HolotileConfig.ControlDt, tuning.controllerKp, tuning.spinMaxRadPerSec);

            var root = new GameObject("HoloTileHumanWalker");
            root.transform.SetParent(transform, false);
            _floor = root.AddComponent<FloorGrid>();
            _floor.Build(tilesX, tilesY, physicsSupport: true);

            var charGo = new GameObject("Character");
            charGo.transform.SetParent(root.transform, false);
            if (customCharacterPrefab != null)
            {
                _importedDriver = charGo.AddComponent<ImportedCharacterDriver>();
                _customCharInstance = Instantiate(customCharacterPrefab, charGo.transform);
                var walkClip = ImportedCharacterDriver.ResolveWalkClipFromPrefab(customCharacterPrefab);
                _importedDriver.Setup(_customCharInstance, _replay.HeightM, walkClip);
            }
            else
            {
                _character = charGo.AddComponent<ProceduralCharacterMesh>();
                _character.Build();
            }

            var footParent = new GameObject("RegionalFeet");
            footParent.transform.SetParent(root.transform, false);
            _footL = new GameObject("FootL").AddComponent<RegionalFoot>();
            _footR = new GameObject("FootR").AddComponent<RegionalFoot>();
            _footL.transform.SetParent(footParent.transform, false);
            _footR.transform.SetParent(footParent.transform, false);

            WalkFrame f0 = _replay.Sample(0f, loopReplay, pingPongLoop);
            _pelvisStartX = f0.Pose.Pelvis.x;
            _footL.Build("l", f0.Pose.AnkleL, hideCollider: true);
            _footR.Build("r", f0.Pose.AnkleR, hideCollider: true);

            if (physicsCoupled)
            {
                _footL.EnablePhysics();
                _footR.EnablePhysics();
            }

            SetupCamera();
            _time = 0f;
            float cycle = _replay.FullCycleDuration(replayTimeScale, pingPongLoop);
            Debug.Log($"[HoloTile] HumanWalkerDemo: {_replay.FrameCount} frames, segment={_replay.Duration:F2}s, " +
                      $"fullCycle={cycle:F2}s, pingPong={pingPongLoop}, physicsCoupled={physicsCoupled}.");
        }

        void FixedUpdate()
        {
            if (_floor == null || _replay == null)
                return;

            float dt = HolotileConfig.ControlDt * replayTimeScale;
            _floorCtrl.ApplyTuning(tuning);

            WalkFrame frame = _replay.Sample(_time, loopReplay, pingPongLoop);
            SkeletonPose pose = CenterPose(frame.Pose);

            RegionalTileCommands.FillPatchVelocities(frame, 1, _patchVelL);
            RegionalTileCommands.FillPatchVelocities(frame, 0, _patchVelR);
            _footL.SetPatchVelocities(_patchVelL);
            _footR.SetPatchVelocities(_patchVelR);

            // Custom character: animate first, then pucks track the visible leg bones exactly.
            if (_importedDriver != null)
            {
                _importedDriver.ApplyPose(pose, _time, _replay.Duration, loopReplay, pingPongLoop);

                if (_importedDriver.TryGetFootTargets(out Vector3 aL, out Vector3 tL, out Vector3 aR, out Vector3 tR))
                {
                    float restY = FootPuck.RestFootCenterY;
                    pose.AnkleL = new Vector3(aL.x, restY, aL.z);
                    pose.ToeL = new Vector3(tL.x, restY, tL.z);
                    pose.AnkleR = new Vector3(aR.x, restY, aR.z);
                    pose.ToeR = new Vector3(tR.x, restY, tR.z);
                }
            }
            else if (_character != null)
            {
                _character.ApplyPose(pose);
            }

            Vector2 pelvisVel = EstimatePelvisVelocity();
            Vector2 footLXz = new Vector2(pose.AnkleL.x, pose.AnkleL.z);
            Vector2 footRXz = new Vector2(pose.AnkleR.x, pose.AnkleR.z);

            TileCommand cmdL = _floorCtrl.CommandStance(footLXz, Vector2.zero, -pelvisVel, out _);
            TileCommand cmdR = _floorCtrl.CommandStance(footRXz, Vector2.zero, -pelvisVel, out _);
            TileCommand centerCmd = AverageStanceCommand(cmdL, cmdR);

            if (useRegionalGrfCommands)
                RegionalTileCommands.ApplyToFloor(_floor, frame, pose, centerCmd, tuning.spinMaxRadPerSec);
            else
                _floor.SetAllTiles(centerCmd);

            _floor.Tick(HolotileConfig.ControlDt);

            if (physicsCoupled && _importedDriver == null)
            {
                float loadL = RegionalTileCommands.FootLoad(frame, 1);
                float loadR = RegionalTileCommands.FootLoad(frame, 0);
                _footL.DrivePhysicsStep(pose.AnkleL, pose.ToeL, loadL);
                _footR.DrivePhysicsStep(pose.AnkleR, pose.ToeR, loadR);
                StepPhysics();
            }
            else
            {
                _footL.FollowKinematic(pose.AnkleL, pose.ToeL);
                _footR.FollowKinematic(pose.AnkleR, pose.ToeR);
            }

            _time += dt;

            if (logEachSecond)
            {
                _logTimer += dt;
                if (_logTimer >= 1f)
                {
                    _logTimer = 0f;
                    Debug.Log($"[HoloTile] replay t={_time:F1}s pelvis=({pose.Pelvis.x:F3},{pose.Pelvis.z:F3}) {centerCmd}");
                }
            }
        }

        void StepPhysics()
        {
            int substeps = HolotileConfig.PhysicsSubsteps;
            float subDt = HolotileConfig.ControlDt / substeps;
            for (int i = 0; i < substeps; i++)
            {
                _footL.ApplyRegionalBelt(_floor);
                _footR.ApplyRegionalBelt(_floor);
                Physics.Simulate(subDt);
            }
        }

        static TileCommand AverageStanceCommand(TileCommand a, TileCommand b)
        {
            Vector3 avg = (a.SurfaceVelocity + b.SurfaceVelocity) * 0.5f;
            DiskKinematics.CommandFromVelocity(avg, a.AzimuthRad, HolotileConfig.SpinMax,
                out float az, out float spin);
            return new TileCommand(az, spin);
        }

        SkeletonPose CenterPose(SkeletonPose pose)
        {
            if (!centerPelvis)
                return pose;

            float dx = pose.Pelvis.x - _pelvisStartX;
            Vector3 shift = new Vector3(-dx, 0f, 0f);
            return new SkeletonPose
            {
                Pelvis = pose.Pelvis + shift,
                Chest = pose.Chest + shift,
                Head = pose.Head + shift,
                HipL = pose.HipL + shift,
                HipR = pose.HipR + shift,
                KneeL = pose.KneeL + shift,
                KneeR = pose.KneeR + shift,
                AnkleL = pose.AnkleL + shift,
                AnkleR = pose.AnkleR + shift,
                ToeL = pose.ToeL + shift,
                ToeR = pose.ToeR + shift,
            };
        }

        Vector2 EstimatePelvisVelocity()
        {
            float dt = HolotileConfig.ControlDt;
            WalkFrame a = _replay.Sample(_time, loopReplay, pingPongLoop);
            WalkFrame b = _replay.Sample(_time + dt, loopReplay, pingPongLoop);
            float scale = tuning.walkSpeed / Mathf.Max(HolotileConfig.WalkSpeed, 0.01f);
            float vx = (b.Pose.Pelvis.x - a.Pose.Pelvis.x) / dt;
            return new Vector2(vx, 0f) * scale;
        }

        void SetupCamera()
        {
            var cam = Camera.main;
            if (cam == null) return;
            cam.transform.position = HolotileConfig.DemoCameraPosition;
            cam.transform.rotation = Quaternion.Euler(HolotileConfig.DemoCameraEuler);
            cam.fieldOfView = HolotileConfig.DemoCameraFov;
        }
    }
}
