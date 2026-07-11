using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace HoloTile.OpenSim
{
    public struct ReplayClock
    {
        /// <summary>Time within one forward STO segment [0, duration].</summary>
        public float SegmentTime;
        public bool Reverse;
        /// <summary>Phase within full cycle (ping-pong length = 2×duration when enabled).</summary>
        public float CycleTime;
        public float CycleDuration;
    }

    [Serializable]
    public class WalkReplayJson
    {
        public string source;
        public int frameCount;
        public float duration;
        public float height_m;
        public float[] time;
        public float[] pelvis_tilt;
        public float[] pelvis_tx;
        public float[] pelvis_ty;
        public float[] hip_flexion_l;
        public float[] hip_flexion_r;
        public float[] knee_angle_l;
        public float[] knee_angle_r;
        public float[] ankle_angle_l;
        public float[] ankle_angle_r;
        public float[] lumbar;
        public float[] grf_heel_r_fx;
        public float[] grf_heel_r_fy;
        public float[] grf_heel_r_fz;
        public float[] grf_heel_l_fx;
        public float[] grf_heel_l_fy;
        public float[] grf_heel_l_fz;
        public float[] grf_midfoot_r_fx;
        public float[] grf_midfoot_r_fy;
        public float[] grf_midfoot_r_fz;
        public float[] grf_midfoot_l_fx;
        public float[] grf_midfoot_l_fy;
        public float[] grf_midfoot_l_fz;
        public float[] grf_forefoot_r_fx;
        public float[] grf_forefoot_r_fy;
        public float[] grf_forefoot_r_fz;
        public float[] grf_forefoot_l_fx;
        public float[] grf_forefoot_l_fy;
        public float[] grf_forefoot_l_fz;
        public float[] grf_toe_r_fx;
        public float[] grf_toe_r_fy;
        public float[] grf_toe_r_fz;
        public float[] grf_toe_l_fx;
        public float[] grf_toe_l_fy;
        public float[] grf_toe_l_fz;
    }

    public struct RegionGrf
    {
        public Vector3 Force; // OpenSim 2D: Fx forward, Fy vertical
    }

    public struct WalkFrame
    {
        public float Time;
        public float PelvisTilt;
        public float PelvisTx;
        public float PelvisTy;
        public float HipFlexL;
        public float HipFlexR;
        public float KneeL;
        public float KneeR;
        public float AnkleL;
        public float AnkleR;
        public SkeletonPose Pose;
        public RegionGrf[,] Regions; // [side 0=r,1=l][region 0..3]
    }

    public struct SkeletonPose
    {
        public Vector3 Pelvis;
        public Vector3 Chest;
        public Vector3 HipL;
        public Vector3 HipR;
        public Vector3 KneeL;
        public Vector3 KneeR;
        public Vector3 AnkleL;
        public Vector3 AnkleR;
        public Vector3 ToeL;
        public Vector3 ToeR;
        public Vector3 Head;
    }

    public class OpenSimReplayData
    {
        public float Duration { get; private set; }
        public int FrameCount { get; private set; }
        public string Source { get; private set; }
        public float HeightM { get; private set; } = 1.7f;

        readonly List<WalkFrame> _frames = new List<WalkFrame>();

        public IReadOnlyList<WalkFrame> Frames => _frames;

        /// <summary>Full walk cycle length (2× segment when using ping-pong).</summary>
        public float FullCycleDuration(float replayTimeScale, bool pingPong) =>
            Duration * (pingPong ? 2f : 1f) / Mathf.Max(replayTimeScale, 0.01f);

        public static ReplayClock MapTime(float timeSec, float duration, bool loop, bool pingPong)
        {
            if (duration < 1e-6f)
                return new ReplayClock { SegmentTime = 0f, CycleDuration = duration };

            if (!loop)
            {
                float clamped = Mathf.Clamp(timeSec, 0f, duration);
                return new ReplayClock
                {
                    SegmentTime = clamped,
                    Reverse = false,
                    CycleTime = clamped,
                    CycleDuration = duration,
                };
            }

            if (pingPong)
            {
                float cycle = duration * 2f;
                float phase = Mathf.Repeat(timeSec, cycle);
                bool reverse = phase > duration;
                float seg = reverse ? cycle - phase : phase;
                return new ReplayClock
                {
                    SegmentTime = seg,
                    Reverse = reverse,
                    CycleTime = phase,
                    CycleDuration = cycle,
                };
            }

            float t = Mathf.Repeat(timeSec, duration);
            return new ReplayClock
            {
                SegmentTime = t,
                Reverse = false,
                CycleTime = t,
                CycleDuration = duration,
            };
        }

        public static OpenSimReplayData LoadFromStreamingAssets(string fileName = "OpenSim/walk_replay.json")
        {
            string path = Path.Combine(Application.streamingAssetsPath, fileName);
            if (!File.Exists(path))
                throw new FileNotFoundException($"OpenSim replay not found: {path}. Run HoloTile > Export OpenSim Walk Replay.");

            string json = File.ReadAllText(path);
            var raw = JsonUtility.FromJson<WalkReplayJson>(json);
            return Build(raw);
        }

        static OpenSimReplayData Build(WalkReplayJson raw)
        {
            float height = raw.height_m > 0f ? raw.height_m : 1.70f;
            var data = new OpenSimReplayData
            {
                Duration = raw.duration,
                FrameCount = raw.frameCount,
                Source = raw.source,
                HeightM = height,
            };
            for (int i = 0; i < raw.frameCount; i++)
            {
                var coords = new Dictionary<string, float>
                {
                    ["pelvis_tilt"] = raw.pelvis_tilt[i],
                    ["pelvis_tx"] = raw.pelvis_tx[i],
                    ["pelvis_ty"] = raw.pelvis_ty[i],
                    ["hip_flexion_l"] = raw.hip_flexion_l[i],
                    ["hip_flexion_r"] = raw.hip_flexion_r[i],
                    ["knee_angle_l"] = raw.knee_angle_l[i],
                    ["knee_angle_r"] = raw.knee_angle_r[i],
                    ["ankle_angle_l"] = raw.ankle_angle_l[i],
                    ["ankle_angle_r"] = raw.ankle_angle_r[i],
                };

                var frame = new WalkFrame
                {
                    Time = raw.time[i],
                    PelvisTilt = raw.pelvis_tilt[i],
                    PelvisTx = raw.pelvis_tx[i],
                    PelvisTy = raw.pelvis_ty[i],
                    HipFlexL = raw.hip_flexion_l[i],
                    HipFlexR = raw.hip_flexion_r[i],
                    KneeL = raw.knee_angle_l[i],
                    KneeR = raw.knee_angle_r[i],
                    AnkleL = raw.ankle_angle_l[i],
                    AnkleR = raw.ankle_angle_r[i],
                    Pose = OpenSimWalkFK.AlignToFloor(OpenSimWalkFK.Solve(coords, height)),
                    Regions = new RegionGrf[2, 4],
                };

                FillRegion(ref frame, 0, 0, raw.grf_heel_r_fx[i], raw.grf_heel_r_fy[i], raw.grf_heel_r_fz[i]);
                FillRegion(ref frame, 0, 1, raw.grf_midfoot_r_fx[i], raw.grf_midfoot_r_fy[i], raw.grf_midfoot_r_fz[i]);
                FillRegion(ref frame, 0, 2, raw.grf_forefoot_r_fx[i], raw.grf_forefoot_r_fy[i], raw.grf_forefoot_r_fz[i]);
                FillRegion(ref frame, 0, 3, raw.grf_toe_r_fx[i], raw.grf_toe_r_fy[i], raw.grf_toe_r_fz[i]);
                FillRegion(ref frame, 1, 0, raw.grf_heel_l_fx[i], raw.grf_heel_l_fy[i], raw.grf_heel_l_fz[i]);
                FillRegion(ref frame, 1, 1, raw.grf_midfoot_l_fx[i], raw.grf_midfoot_l_fy[i], raw.grf_midfoot_l_fz[i]);
                FillRegion(ref frame, 1, 2, raw.grf_forefoot_l_fx[i], raw.grf_forefoot_l_fy[i], raw.grf_forefoot_l_fz[i]);
                FillRegion(ref frame, 1, 3, raw.grf_toe_l_fx[i], raw.grf_toe_l_fy[i], raw.grf_toe_l_fz[i]);

                data._frames.Add(frame);
            }

            return data;
        }

        static void FillRegion(ref WalkFrame frame, int side, int region, float fx, float fy, float fz)
        {
            frame.Regions[side, region] = new RegionGrf
            {
                Force = new Vector3(fx, fy, fz),
            };
        }

        public WalkFrame Sample(float timeSec, bool loop = true, bool pingPong = true)
        {
            if (_frames.Count == 0)
                return default;

            var clock = MapTime(timeSec, Duration, loop, pingPong);
            return SampleSegment(clock.SegmentTime, loop && !pingPong);
        }

        WalkFrame SampleSegment(float segmentTime, bool crossfadeLoop)
        {
            if (_frames.Count == 1)
                return _frames[0];

            float t = Mathf.Clamp(segmentTime, 0f, Duration);
            float blend = HolotileConfig.LoopCrossfadeSec;

            if (crossfadeLoop && blend > 1e-5f && Duration > blend * 2f)
            {
                if (t < blend)
                {
                    WalkFrame fromEnd = _frames[_frames.Count - 1];
                    WalkFrame toCurrent = SampleSegmentLinear(t);
                    float u = t / blend;
                    return LerpFrame(fromEnd, toCurrent, u);
                }

                if (t > Duration - blend)
                {
                    WalkFrame fromCurrent = SampleSegmentLinear(t);
                    WalkFrame toStart = _frames[0];
                    float u = (t - (Duration - blend)) / blend;
                    return LerpFrame(fromCurrent, toStart, u);
                }
            }

            return SampleSegmentLinear(t);
        }

        WalkFrame SampleSegmentLinear(float t)
        {
            for (int i = 0; i < _frames.Count - 1; i++)
            {
                if (t >= _frames[i].Time && t <= _frames[i + 1].Time)
                {
                    float u = Mathf.InverseLerp(_frames[i].Time, _frames[i + 1].Time, t);
                    return LerpFrame(_frames[i], _frames[i + 1], u);
                }
            }

            return _frames[_frames.Count - 1];
        }

        public WalkFrame SamplePair(float timeA, float timeB, bool loop, bool pingPong, out float segmentDt)
        {
            var ca = MapTime(timeA, Duration, loop, pingPong);
            var cb = MapTime(timeB, Duration, loop, pingPong);
            segmentDt = Mathf.Max(Mathf.Abs(cb.SegmentTime - ca.SegmentTime), 1e-4f);
            if (ca.Reverse != cb.Reverse)
                segmentDt = Mathf.Max(segmentDt, HolotileConfig.ControlDt * 0.5f);

            return SamplePairSegments(ca.SegmentTime, cb.SegmentTime, loop && !pingPong);
        }

        WalkFrame SamplePairSegments(float segA, float segB, bool crossfadeLoop)
        {
            WalkFrame a = SampleSegment(segA, crossfadeLoop);
            WalkFrame b = SampleSegment(segB, crossfadeLoop);
            return LerpFrame(a, b, 0.5f);
        }

        static WalkFrame LerpFrame(WalkFrame a, WalkFrame b, float u)
        {
            var pose = new SkeletonPose
            {
                Pelvis = Vector3.Lerp(a.Pose.Pelvis, b.Pose.Pelvis, u),
                Chest = Vector3.Lerp(a.Pose.Chest, b.Pose.Chest, u),
                HipL = Vector3.Lerp(a.Pose.HipL, b.Pose.HipL, u),
                HipR = Vector3.Lerp(a.Pose.HipR, b.Pose.HipR, u),
                KneeL = Vector3.Lerp(a.Pose.KneeL, b.Pose.KneeL, u),
                KneeR = Vector3.Lerp(a.Pose.KneeR, b.Pose.KneeR, u),
                AnkleL = Vector3.Lerp(a.Pose.AnkleL, b.Pose.AnkleL, u),
                AnkleR = Vector3.Lerp(a.Pose.AnkleR, b.Pose.AnkleR, u),
                ToeL = Vector3.Lerp(a.Pose.ToeL, b.Pose.ToeL, u),
                ToeR = Vector3.Lerp(a.Pose.ToeR, b.Pose.ToeR, u),
                Head = Vector3.Lerp(a.Pose.Head, b.Pose.Head, u),
            };

            var regions = new RegionGrf[2, 4];
            for (int s = 0; s < 2; s++)
            {
                for (int r = 0; r < 4; r++)
                {
                    regions[s, r] = new RegionGrf
                    {
                        Force = Vector3.Lerp(a.Regions[s, r].Force, b.Regions[s, r].Force, u),
                    };
                }
            }

            return new WalkFrame
            {
                Time = Mathf.Lerp(a.Time, b.Time, u),
                PelvisTx = Mathf.Lerp(a.PelvisTx, b.PelvisTx, u),
                PelvisTy = Mathf.Lerp(a.PelvisTy, b.PelvisTy, u),
                Pose = pose,
                Regions = regions,
            };
        }
    }
}
