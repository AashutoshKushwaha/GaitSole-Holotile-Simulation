using UnityEngine;
using HoloTile;
using HoloTile.Math;
using HoloTile.PhysicsSim;

namespace HoloTile.Control
{
    /// <summary>
    /// Two-foot stance/swing gait coupled to the floor controller.
    /// Mirrors holotile_sim/run_control.py Walker (M4, est_mode=true).
    /// </summary>
    public class WalkerSim
    {
        readonly FloorController _controller;
        WalkerTuning _tuning = WalkerTuning.Default;

        string _stanceSide = "r";
        string _swingSide = "l";
        float _stepTimer;
        Vector2 _swingFromRel;

        readonly System.Collections.Generic.Dictionary<string, Vector2> _footRel =
            new System.Collections.Generic.Dictionary<string, Vector2>();

        public bool ControllerEnabled { get; set; } = true;
        public float Time { get; private set; }
        public Vector2 PelvisXz { get; private set; }
        public Vector2 LastCommandedVelocity { get; private set; }
        public float MaxDrift { get; private set; }

        public WalkerTuning Tuning => _tuning;

        public WalkerSim(float controlDt, WalkerTuning tuning = null)
        {
            _tuning = tuning ?? WalkerTuning.Default;
            _controller = new FloorController(controlDt, _tuning.controllerKp, _tuning.spinMaxRadPerSec);
            _footRel["l"] = LateralOffset("l");
            _footRel["r"] = LateralOffset("r");
            _swingFromRel = _footRel["l"];
        }

        public static Vector2 LateralOffset(string side)
        {
            float z = side == "l" ? HolotileConfig.HipHalfWidth : -HolotileConfig.HipHalfWidth;
            return new Vector2(0f, z);
        }

        public void ApplyTuning(WalkerTuning tuning)
        {
            _tuning = tuning;
            _controller.ApplyTuning(tuning);
        }

        /// <summary>Pre-physics: update gait model and compute tile command.</summary>
        public TileCommand PrePhysicsTick(FootPuck footLeft, FootPuck footRight, float dt)
        {
            Vector2 vCmd = IntendedVelocity.CommandedPath(Time, speed: _tuning.walkSpeed);
            LastCommandedVelocity = vCmd;

            _footRel[_stanceSide] -= vCmd * dt;

            float phase = Mathf.Min(1f, _stepTimer / HolotileConfig.StepDuration);
            Vector2 plantRel = LateralOffset(_swingSide) + 0.5f * vCmd * HolotileConfig.StepDuration;
            _footRel[_swingSide] = Vector2.Lerp(_swingFromRel, plantRel, phase);

            TileCommand cmd;
            if (ControllerEnabled)
            {
                FootPuck stance = StanceFoot(footLeft, footRight);
                cmd = _controller.CommandStance(stance.FootXz, _footRel[_stanceSide], -vCmd,
                    out _);
            }
            else
            {
                cmd = _controller.Idle();
            }

            _pendingPhase = phase;
            return cmd;
        }

        float _pendingPhase;

        /// <summary>Post-physics: derive pelvis, place swing foot, advance gait clock.</summary>
        public void PostPhysicsTick(FootPuck footLeft, FootPuck footRight, float dt)
        {
            UpdatePelvis(footLeft, footRight);
            PlaceSwingFoot(footLeft, footRight, _pendingPhase);

            Time += dt;
            _stepTimer += dt;

            float drift = PelvisXz.magnitude;
            if (drift > MaxDrift)
                MaxDrift = drift;

            if (_stepTimer >= HolotileConfig.StepDuration)
                CompleteStep(footLeft, footRight);
        }

        void CompleteStep(FootPuck footLeft, FootPuck footRight)
        {
            UpdatePelvis(footLeft, footRight);

            string oldSwing = _swingSide;
            _stanceSide = oldSwing;
            _swingSide = oldSwing == "l" ? "r" : "l";
            _stepTimer = 0f;

            FootPuck newStance = StanceFoot(footLeft, footRight);
            _footRel[_stanceSide] = newStance.FootXz - PelvisXz;
            _swingFromRel = _footRel[_swingSide];

            newStance.SetDynamic(newStance.transform.position);
            PlaceSwingFoot(footLeft, footRight, 0f);
        }

        FootPuck StanceFoot(FootPuck footLeft, FootPuck footRight) =>
            _stanceSide == "l" ? footLeft : footRight;

        FootPuck SwingFoot(FootPuck footLeft, FootPuck footRight) =>
            _swingSide == "l" ? footLeft : footRight;

        public void ResetDriftStats() => MaxDrift = 0f;

        /// <summary>Call once after feet settle — swing foot kinematic, stance dynamic.</summary>
        public void BeginWalking(FootPuck footLeft, FootPuck footRight)
        {
            _footRel["l"] = LateralOffset("l");
            _footRel["r"] = LateralOffset("r");
            _swingFromRel = _footRel[_swingSide];

            footLeft.SetDynamic(new Vector3(0f, FootPuck.RestFootCenterY, HolotileConfig.HipHalfWidth));
            footRight.SetDynamic(new Vector3(0f, FootPuck.RestFootCenterY, -HolotileConfig.HipHalfWidth));
            PelvisXz = Vector2.zero;
            ResetDriftStats();
            PlaceSwingFoot(footLeft, footRight, 0f);
        }

        void UpdatePelvis(FootPuck footLeft, FootPuck footRight)
        {
            FootPuck stance = StanceFoot(footLeft, footRight);
            PelvisXz = stance.FootXz - _footRel[_stanceSide];
        }

        void PlaceSwingFoot(FootPuck footLeft, FootPuck footRight, float phase)
        {
            FootPuck stance = StanceFoot(footLeft, footRight);
            FootPuck swing = SwingFoot(footLeft, footRight);
            Vector2 swingWorld = PelvisXz + _footRel[_swingSide];
            swingWorld = EnforceFootSeparation(stance.FootXz, swingWorld, _swingSide);

            float footY = FootPuck.RestFootCenterY
                + HolotileConfig.SwingLiftHeight * Mathf.Sin(Mathf.PI * phase);
            swing.SetKinematic(new Vector3(swingWorld.x, footY, swingWorld.y));
        }

        static Vector2 EnforceFootSeparation(Vector2 stanceXz, Vector2 swingXz, string swingSide)
        {
            Vector2 delta = swingXz - stanceXz;
            float minSep = HolotileConfig.FootMinSeparation;
            float sep = delta.magnitude;
            if (sep >= minSep)
                return swingXz;

            if (sep > 1e-5f)
                return stanceXz + delta / sep * minSep;

            float lateral = swingSide == "l" ? minSep : -minSep;
            return stanceXz + new Vector2(0f, lateral);
        }
    }
}
