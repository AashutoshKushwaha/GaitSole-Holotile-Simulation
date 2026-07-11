using System.Collections.Generic;
using UnityEngine;
using HoloTile;
using HoloTile.PhysicsSim;

namespace HoloTile.OpenSim
{
    /// <summary>
    /// 2D OpenSim sagittal FK mapped to Unity XZ floor (x forward, y up, z lateral).
    /// Matches motion_predictor/animate_skeleton.py + lateral hip offset.
    /// </summary>
    public static class OpenSimWalkFK
    {
        public static SkeletonPose Solve(Dictionary<string, float> row, float heightM)
        {
            float thigh = 0.245f * heightM;
            float shank = 0.246f * heightM;
            float foot = 0.152f * heightM;
            float trunk = 0.30f * heightM;
            float hipHalf = HolotileConfig.HipHalfWidth;

            float px = row.TryGetValue("pelvis_tx", out float tx) ? tx : 0f;
            float py = row.TryGetValue("pelvis_ty", out float ty) ? ty : 0f;
            float tilt = row.TryGetValue("pelvis_tilt", out float pt) ? pt : 0f;

            px *= HolotileConfig.ReplayPelvisScale;
            py *= HolotileConfig.ReplayPelvisScale;

            var pelvis = new Vector3(px, py, 0f);
            var chest = pelvis + new Vector3(Mathf.Sin(tilt) * trunk, Mathf.Cos(tilt) * trunk, 0f);
            var head = chest + new Vector3(0f, 0.12f, 0f);

            SolveLeg(row, "l", pelvis, hipHalf, thigh, shank, foot,
                out Vector3 hipL, out Vector3 kneeL, out Vector3 ankleL, out Vector3 toeL);
            SolveLeg(row, "r", pelvis, -hipHalf, thigh, shank, foot,
                out Vector3 hipR, out Vector3 kneeR, out Vector3 ankleR, out Vector3 toeR);

            return new SkeletonPose
            {
                Pelvis = pelvis,
                Chest = chest,
                Head = head,
                HipL = hipL,
                HipR = hipR,
                KneeL = kneeL,
                KneeR = kneeR,
                AnkleL = ankleL,
                AnkleR = ankleR,
                ToeL = toeL,
                ToeR = toeR,
            };
        }

        static void SolveLeg(Dictionary<string, float> row, string side, Vector3 pelvis, float lateral,
            float thigh, float shank, float footLen,
            out Vector3 hip, out Vector3 knee, out Vector3 ankle, out Vector3 toe)
        {
            float hf = row.TryGetValue($"hip_flexion_{side}", out float h) ? h : 0f;
            float kn = row.TryGetValue($"knee_angle_{side}", out float k) ? k : 0f;
            float an = row.TryGetValue($"ankle_angle_{side}", out float a) ? a : 0f;

            hip = pelvis + new Vector3(0f, 0f, lateral);
            float th = hf;
            float sh = hf + kn;
            float fa = sh + an;

            knee = hip + new Vector3(Mathf.Sin(th) * thigh, Mathf.Cos(th) * thigh, 0f);
            ankle = knee + new Vector3(Mathf.Sin(sh) * shank, -Mathf.Cos(sh) * shank, 0f);
            toe = ankle + new Vector3(Mathf.Cos(fa) * footLen, Mathf.Sin(fa) * footLen, 0f);
        }

        public static Vector3 RegionWorldPoint(Vector3 ankle, Vector3 toe, int regionIndex, float lateralZ)
        {
            float t = regionIndex switch
            {
                0 => 0.08f,
                1 => 0.35f,
                2 => 0.62f,
                _ => 0.88f,
            };
            var p = Vector3.Lerp(ankle, toe, t);
            p.z = lateralZ;
            p.y = HolotileConfig.SupportY + HolotileConfig.PuckHalfExtents.y;
            return p;
        }

        public static SkeletonPose AlignToFloor(SkeletonPose pose)
        {
            float footY = Mathf.Min(pose.AnkleL.y, pose.AnkleR.y);
            float dy = FootPuck.RestFootCenterY - footY;
            var shift = new Vector3(0f, dy, 0f);
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
    }
}
