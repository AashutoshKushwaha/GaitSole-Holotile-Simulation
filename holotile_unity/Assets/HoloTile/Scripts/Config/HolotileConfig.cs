using UnityEngine;

namespace HoloTile
{
    /// <summary>
    /// Single source of truth for the HoloTile mechanism.
    /// Values mirror holotile_sim/holotile_config.py and US20180217662A1.
    ///
    /// Coordinate convention (Unity):
    ///   Horizontal plane = XZ,  up = +Y.
    /// MuJoCo holotile_sim uses XY horizontal + Z up; map (mx, my, mz) -> (mx, mz, my).
    /// </summary>
    public static class HolotileConfig
    {
        // --- Patent reference -------------------------------------------------
        // [§0014] Tilt angle 5-60 deg (prototype 5-15 deg common; we use 35 deg
        //         hemisphere variant per holotile_sim).
        // [FIG 3]  Tile ~1 ft square; we use 0.30 m.
        // [FIG 11-12] Disk assembly: swashplate azimuth + spin about tilted axis.

        public const string PatentId = "US20180217662A1";

        // Control rate (Phase C controller; unused in Phase D kinematics).
        public const float ControlHz = 100f;
        public static float ControlDt => 1f / ControlHz;

        // --- Tile geometry [FIG 3, §0068] -----------------------------------
        /// <summary>Square tile edge length (m). Patent: ~1 ft.</summary>
        public const float TileSize = 0.30f;

        /// <summary>Gap between adjacent tiles (m). Patent: &lt;= 0.25 in.</summary>
        public const float TileGap = 0.004f;

        /// <summary>Disks along each tile axis -> DisksPerTile^2 per active tile.</summary>
        public const int DisksPerTile = 5;

        // --- Disk geometry [FIG 11, §0092] ------------------------------------
        /// <summary>
        /// Fixed tilt angle theta (deg). Defines which rim arc is the raised portion.
        /// Patent range 5-60 deg.
        /// </summary>
        public const float TiltDeg = 35f;

        public static float TiltRad => TiltDeg * Mathf.Deg2Rad;

        /// <summary>Visual coin half-thickness (m).</summary>
        public const float DiskHalfThickness = 0.006f;

        /// <summary>Disk body centre height above tile base (m).</summary>
        public const float DiskCenterY = 0.030f;

        public static float DiskPitch => TileSize / DisksPerTile;

        /// <summary>Contact sphere / disk radius (m).</summary>
        public static float DiskRadius => 0.46f * DiskPitch;

        /// <summary>Minimum rim-marker diameter (m) so it is visible in Game view.</summary>
        public const float RimMarkerMinDiameter = 0.014f;

        public static float RimMarkerDiameter =>
            Mathf.Max(DiskRadius * 0.9f, RimMarkerMinDiameter);

        // --- FIG 19-22 hemisphere recess visual --------------------------------
        /// <summary>Fraction of sphere radius sitting below the plate top (recess depth).</summary>
        public const float HemisphereRecessFraction = 0.72f;

        /// <summary>Socket hole diameter as multiple of disk diameter.</summary>
        public const float SocketOpeningScale = 1.15f;

        public static float PlateTopLocalY => DiskRadius;

        /// <summary>Lift rim marker slightly above contact plane for visibility.</summary>
        public const float RimMarkerLift = 0.002f;

        /// <summary>
        /// Support plane Y: top of structural plate / walking surface [FIG 19, 1966].
        /// SUPPORT_Y = DISK_CENTER_Y + DISK_RADIUS
        /// </summary>
        public static float SupportY => DiskCenterY + DiskRadius;

        /// <summary>Hemisphere centre Y from tile origin (= SupportY - R).</summary>
        public static float HemisphereCenterY => SupportY - DiskRadius;

        /// <summary>
        /// Horizontal surface speed per unit spin rate (m/s per rad/s):
        /// DRAG_PER_OMEGA = R * sin(tilt).  [§0089], holotile_sim derivation.
        /// </summary>
        public static float DragPerOmega => DiskRadius * Mathf.Sin(TiltRad);

        // --- Structural pad (Phase C; visual in Phase D) ----------------------
        public const float SupportPadHalfThickness = 0.02f;

        // --- Default floor grid -----------------------------------------------
        public const int DefaultTilesX = 1;
        public const int DefaultTilesY = 1;
        public const int FullDemoTilesX = 8;
        public const int FullDemoTilesY = 8;

        // --- Actuator limits [§0091] (Phase C) --------------------------------
        public const float SpinMax = 150f;
        public static float AziSlewRadPerSec => 720f * Mathf.Deg2Rad;
        public const float SpinSlewPerSec = 800f;

        // --- Foot puck (Phase C test object) ----------------------------------
        public static readonly Vector3 PuckHalfExtents = new Vector3(0.060f, 0.015f, 0.040f);
        public const float PuckMass = 2.0f;
        public const float PuckStartClear = 0.02f;

        // --- Belt model (Phase C only) ----------------------------------------
        public const float BeltMu = 1.0f;
        public const float BeltK = 3000f;
        public const float WalkerBeltMu = 1.5f;
        public const float WalkerBeltK = 5000f;

        // --- Walker / M4 centering demo ---------------------------------------
        public const float HipHalfWidth = 0.09f;
        public const float StepDuration = 0.55f;
        public const float SwingLiftHeight = 0.07f;
        public const float WalkSpeed = 0.35f;
        public const float TurnStartTime = 3.0f;
        public const float TurnDuration = 1.0f;
        public const float FloorControllerKp = 4.0f;
        public const int WalkerDemoTiles = 7;
        public const int WalkerSettleSteps = 300;
        public const int PhysicsSubsteps = 6;
        public const float FootMinSeparation = 0.14f;
        public static float MaxSurfaceSpeed => SpinMax * DragPerOmega;

        // --- OpenSim 4-region foot (Phase M6) ---------------------------------
        public static readonly string[] FootRegionNames = { "Heel", "Midfoot", "Forefoot", "Toe" };
        /// <summary>Local sole offsets (x forward, z lateral) from ankle, metres.</summary>
        public static readonly Vector3[] RegionOffsetRight = {
            new Vector3(-0.04f, 0f, 0f),
            new Vector3(-0.01f, 0f, 0f),
            new Vector3(0.05f, 0f, 0f),
            new Vector3(0.10f, 0f, 0f),
        };
        public const float RegionPatchMassFraction = 0.22f;
        public const float RegionBeltK = 4500f;
        public const float RegionBeltMu = 1.4f;
        public const float ReplayPelvisScale = 1.0f;
        /// <summary>Visual shoe sole sits this far above SupportY (m).</summary>
        public const float FootSoleClearance = 0.004f;
        /// <summary>Fallback ankle-bone to sole distance when toe bones are unavailable (m).</summary>
        public const float FootBoneToSoleOffset = 0.045f;
        /// <summary>Spring guiding dynamic replay feet toward OpenSim ankle targets.</summary>
        public const float ReplayFootGuideK = 12000f;
        public const float ReplayFootGuideD = 500f;
        public const float StanceGrfThreshold = 35f;
        public const float StanceGrfThresholdLow = 12f;
        public const float StanceGrfThresholdHigh = 38f;
        /// <summary>Crossfade window when looping non-ping-pong replays (s).</summary>
        public const float LoopCrossfadeSec = 0.06f;

        // --- Demo defaults ----------------------------------------------------
        public const float DemoSpinRadPerSec = 30f;
        public const float DemoAzimuthHz = 0.1f;

        /// <summary>Default Main Camera for HumanWalker / floor demos (5×5 tile).</summary>
        public static readonly Vector3 DemoCameraPosition = new Vector3(2.05f, 1.54f, -2.02f);
        public static readonly Vector3 DemoCameraEuler = new Vector3(15f, -50f, 0f);
        public const float DemoCameraFov = 48f;

        // --- Visual colours [FIG 19-22] ---------------------------------------
        public static readonly Color PlateColor = new Color(0.55f, 0.58f, 0.62f);
        public static readonly Color SocketColor = new Color(0.12f, 0.13f, 0.16f);
        public static readonly Color HemisphereColor = new Color(0.72f, 0.76f, 0.82f);
        public static readonly Color TileColor = PlateColor;
        public static readonly Color DiskColor = HemisphereColor;
        public static readonly Color RimColor = new Color(0.98f, 0.82f, 0.20f);
        public static readonly Color TiltAxisColor = new Color(1f, 0.4f, 0.1f);
        public static readonly Color VelocityColor = new Color(0.2f, 1f, 0.3f);
        public static readonly Color FootColor = new Color(0.85f, 0.30f, 0.25f);
        public static readonly Color PelvisColor = new Color(0.25f, 0.55f, 0.95f);
        public static readonly Color CenterMarkerColor = new Color(0.1f, 0.1f, 0.1f);
    }
}
