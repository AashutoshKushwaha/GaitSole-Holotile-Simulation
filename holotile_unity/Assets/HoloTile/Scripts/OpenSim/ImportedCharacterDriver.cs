using UnityEngine;
using UnityEngine.Animations;
using UnityEngine.Playables;

namespace HoloTile.OpenSim
{
    /// <summary>
    /// Positions, scales, and scrubs a Mixamo / FBX character on the HoloTile floor
    /// so it stays centered on the replay pelvis and in sync with walk_replay.json.
    /// </summary>
    public class ImportedCharacterDriver : MonoBehaviour
    {
        [Header("Alignment")]
        [Tooltip("Mixamo models face +Z; OpenSim walk is +X. Default 90° faces the tile forward axis.")]
        [SerializeField] float facingYawDeg = 90f;

        [Tooltip("Auto-scale mesh height to OpenSim subject height from replay JSON.")]
        [SerializeField] bool autoScaleToHeight = true;

        [Tooltip("Disable root motion on Animator so replay pelvis drives placement.")]
        [SerializeField] bool disableRootMotion = true;

        [Header("Animation")]
        [Tooltip("Scrub embedded walk clip to match replay loop time.")]
        [SerializeField] bool syncClipToReplay = true;

        [SerializeField] AnimationClip walkClipOverride;

        [Tooltip("Clip loops per full gait cycle. 1 = one walk clip per ping-pong replay cycle.")]
        [SerializeField] float animationTimeScale = 1f;

        [Tooltip("Shift clip phase (0..1) to align Mixamo stride with OpenSim GRF timing.")]
        [Range(0f, 1f)]
        [SerializeField] float clipPhaseOffset = 0f;

        [Tooltip("Extra lift so shoe soles sit on SupportY, not ankle bones.")]
        [SerializeField] float soleClearance = HolotileConfig.FootSoleClearance;

        Transform _modelRoot;
        Animator _animator;
        AnimationClip _walkClip;
        Transform _leftFoot;
        Transform _rightFoot;
        Transform _leftToes;
        Transform _rightToes;
        PlayableGraph _graph;
        AnimationClipPlayable _clipPlayable;
        float _targetHeightM = 1.7f;
        float _appliedScale = 1f;
        bool _playableReady;
        bool _ready;

        public bool IsReady => _ready;
        public float AppliedScale => _appliedScale;
        public AnimationClip WalkClip => _walkClip;

        void OnDestroy() => DestroyPlayable();

        public void Setup(GameObject modelInstance, float targetHeightM, AnimationClip walkClip = null)
        {
            DestroyPlayable();
            _ready = false;
            _targetHeightM = targetHeightM;
            _modelRoot = modelInstance.transform;
            _modelRoot.SetParent(transform, false);
            _modelRoot.localRotation = Quaternion.identity;
            _modelRoot.localPosition = Vector3.zero;

            StripPhysics(modelInstance);

            _animator = modelInstance.GetComponentInChildren<Animator>();
            if (_animator != null)
            {
                if (disableRootMotion)
                    _animator.applyRootMotion = false;
                _animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;
            }

            if (autoScaleToHeight)
                ApplyHeightScale();

            _walkClip = walkClipOverride != null ? walkClipOverride
                : walkClip != null ? walkClip
                : ResolveWalkClipFromPrefab(modelInstance);

            if (_walkClip == null)
                Debug.LogWarning("[HoloTile] No walk AnimationClip found on FBX. Character will stay in bind pose. " +
                                 "Assign Walk Clip Override on ImportedCharacterDriver, or re-import FBX as Humanoid with animation.");

            CacheFootBones();
            EnsurePlayable();

            if (_walkClip != null && syncClipToReplay)
            {
                transform.rotation = Quaternion.Euler(0f, facingYawDeg, 0f);
                transform.position = Vector3.zero;
                SampleClip(0f);
            }

            _ready = true;
            LogSetupDiagnostics();
        }

        public void ApplyPose(SkeletonPose pose, float replayTimeSec, float replayDurationSec, bool loop, bool pingPong = true)
        {
            if (!_ready || _modelRoot == null)
                return;

            ApplyRootAndAnimation(pose.Pelvis, replayTimeSec, replayDurationSec, loop, pingPong);
            AlignFeetToFloor();
        }

        /// <summary>Anchor character to physics foot rigidbodies (belt-coupled replay).</summary>
        public void ApplyPoseFromPhysicsFeet(SkeletonPose pose, Vector3 leftAnkle, Vector3 rightAnkle,
            float replayTimeSec, float replayDurationSec, bool loop, bool pingPong = true)
        {
            if (!_ready || _modelRoot == null)
                return;

            var anchor = new Vector3(
                (leftAnkle.x + rightAnkle.x) * 0.5f,
                0f,
                (leftAnkle.z + rightAnkle.z) * 0.5f);
            ApplyRootAndAnimation(anchor, replayTimeSec, replayDurationSec, loop, pingPong);
            AlignFeetToWorld(new Vector3(leftAnkle.x, 0f, leftAnkle.z), new Vector3(rightAnkle.x, 0f, rightAnkle.z));
            AlignFeetToFloor();
        }

        void ApplyRootAndAnimation(Vector3 rootXz, float replayTimeSec, float replayDurationSec, bool loop, bool pingPong)
        {
            transform.rotation = Quaternion.Euler(0f, facingYawDeg, 0f);
            transform.position = new Vector3(rootXz.x, 0f, rootXz.z);

            float clipTime = MapReplayToClipTime(replayTimeSec, replayDurationSec, loop, pingPong);
            if (syncClipToReplay && _walkClip != null)
                SampleClip(clipTime);
        }

        void AlignFeetToWorld(Vector3 leftTargetXz, Vector3 rightTargetXz)
        {
            if (_leftFoot == null || _rightFoot == null)
                return;

            Vector3 modelMid = (_leftFoot.position + _rightFoot.position) * 0.5f;
            Vector3 targetMid = new Vector3(
                (leftTargetXz.x + rightTargetXz.x) * 0.5f,
                modelMid.y,
                (leftTargetXz.z + rightTargetXz.z) * 0.5f);
            transform.position += targetMid - modelMid;
        }

        /// <summary>Find the walk clip embedded in a Mixamo FBX prefab asset.</summary>
        public static AnimationClip ResolveWalkClipFromPrefab(GameObject prefabOrInstance)
        {
            if (prefabOrInstance == null)
                return null;

#if UNITY_EDITOR
            var source = UnityEditor.PrefabUtility.GetCorrespondingObjectFromSource(prefabOrInstance);
            if (source == null)
                source = prefabOrInstance;

            string path = UnityEditor.AssetDatabase.GetAssetPath(source);
            if (!string.IsNullOrEmpty(path))
            {
                foreach (var asset in UnityEditor.AssetDatabase.LoadAllAssetsAtPath(path))
                {
                    if (asset is AnimationClip clip && !clip.name.StartsWith("__preview"))
                        return clip;
                }
            }
#endif
            return null;
        }

        void EnsurePlayable()
        {
            DestroyPlayable();

            if (_walkClip == null || _animator == null)
                return;

            _graph = PlayableGraph.Create("HoloTileReplayScrub");
            _graph.SetTimeUpdateMode(DirectorUpdateMode.Manual);
            _clipPlayable = AnimationClipPlayable.Create(_graph, _walkClip);
            _clipPlayable.SetApplyFootIK(false);

            var output = AnimationPlayableOutput.Create(_graph, "Animation", _animator);
            output.SetSourcePlayable(_clipPlayable);
            _graph.Play();
            _playableReady = true;
        }

        void DestroyPlayable()
        {
            if (_graph.IsValid())
                _graph.Destroy();
            _playableReady = false;
        }

        void AlignFeetToFloor()
        {
            float targetSoleY = HolotileConfig.SupportY + soleClearance;
            float footY = MeasureFootBottomY();
            transform.position += new Vector3(0f, targetSoleY - footY, 0f);
        }

        float MeasureFootBottomY()
        {
            if (_leftToes != null && _rightToes != null)
                return Mathf.Min(_leftToes.position.y, _rightToes.position.y);

            if (_leftFoot != null && _rightFoot != null)
                return Mathf.Min(_leftFoot.position.y, _rightFoot.position.y) - HolotileConfig.FootBoneToSoleOffset;

            var renderers = _modelRoot.GetComponentsInChildren<Renderer>();
            if (renderers.Length == 0)
                return transform.position.y;

            var bounds = renderers[0].bounds;
            for (int i = 1; i < renderers.Length; i++)
                bounds.Encapsulate(renderers[i].bounds);

            return bounds.min.y;
        }

        static void StripPhysics(GameObject modelInstance)
        {
            foreach (var rb in modelInstance.GetComponentsInChildren<Rigidbody>(true))
                Destroy(rb);

            foreach (var cc in modelInstance.GetComponentsInChildren<CharacterController>(true))
                Destroy(cc);

            foreach (var col in modelInstance.GetComponentsInChildren<Collider>(true))
                col.enabled = false;
        }

        float MapReplayToClipTime(float replayTimeSec, float replayDurationSec, bool loop, bool pingPong)
        {
            if (_walkClip == null || replayDurationSec < 1e-6f)
                return 0f;

            // One full ping-pong replay cycle = one forward clip loop (no reverse playback).
            var clock = OpenSimReplayData.MapTime(replayTimeSec, replayDurationSec, loop, pingPong);
            float phase = clock.CycleDuration > 1e-6f ? clock.CycleTime / clock.CycleDuration : 0f;
            phase = Mathf.Repeat(phase * animationTimeScale + clipPhaseOffset, 1f);
            return phase * _walkClip.length;
        }

        /// <summary>World-space foot bone targets so pucks track the visible legs exactly.</summary>
        public bool TryGetFootTargets(out Vector3 ankleL, out Vector3 toeL, out Vector3 ankleR, out Vector3 toeR)
        {
            ankleL = toeL = ankleR = toeR = Vector3.zero;
            if (!_ready || _leftFoot == null || _rightFoot == null)
                return false;

            ankleL = _leftFoot.position;
            ankleR = _rightFoot.position;
            toeL = _leftToes != null ? _leftToes.position : ankleL + transform.forward * 0.12f;
            toeR = _rightToes != null ? _rightToes.position : ankleR + transform.forward * 0.12f;
            return true;
        }

        void SampleClip(float timeSec)
        {
            if (_walkClip == null)
                return;

            if (_playableReady && _clipPlayable.IsValid())
            {
                _clipPlayable.SetTime(timeSec);
                _graph.Evaluate(0f);
                return;
            }

            // Generic / Legacy fallback only.
            _walkClip.SampleAnimation(_modelRoot.gameObject, timeSec);
        }

        void ApplyHeightScale()
        {
            var renderers = _modelRoot.GetComponentsInChildren<Renderer>();
            if (renderers.Length == 0)
                return;

            var bounds = renderers[0].bounds;
            for (int i = 1; i < renderers.Length; i++)
                bounds.Encapsulate(renderers[i].bounds);

            float height = bounds.size.y;
            if (height < 0.05f)
                return;

            _appliedScale = _targetHeightM / height;
            _modelRoot.localScale = Vector3.one * _appliedScale;
        }

        void CacheFootBones()
        {
            _leftFoot = null;
            _rightFoot = null;
            _leftToes = null;
            _rightToes = null;

            if (_animator == null || !_animator.isHuman)
                return;

            _leftFoot = _animator.GetBoneTransform(HumanBodyBones.LeftFoot);
            _rightFoot = _animator.GetBoneTransform(HumanBodyBones.RightFoot);
            _leftToes = _animator.GetBoneTransform(HumanBodyBones.LeftToes);
            _rightToes = _animator.GetBoneTransform(HumanBodyBones.RightToes);
        }

        void LogSetupDiagnostics()
        {
            var renderers = _modelRoot.GetComponentsInChildren<Renderer>();
            Bounds bounds = default;
            if (renderers.Length > 0)
            {
                bounds = renderers[0].bounds;
                for (int i = 1; i < renderers.Length; i++)
                    bounds.Encapsulate(renderers[i].bounds);
            }

            string clipName = _walkClip != null ? $"{_walkClip.name} ({_walkClip.length:F2}s)" : "(none)";
            string mode = _playableReady ? "PlayableGraph scrub" : "SampleAnimation fallback";
            string feet = _leftFoot != null && _rightFoot != null ? "LeftFoot/RightFoot" : "renderer bounds";
            Debug.Log($"[HoloTile] ImportedCharacterDriver: scale={_appliedScale:F3}, height={bounds.size.y:F2}m, " +
                      $"clip={clipName}, drive={mode}, humanoid={(_animator != null && _animator.isHuman)}, " +
                      $"feet={feet}, yaw={facingYawDeg}°.");
        }
    }
}
