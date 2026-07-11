using System.Linq;
using UnityEditor;
using UnityEngine;
using HoloTile;
using HoloTile.OpenSim;

namespace HoloTile.Editor
{
    /// <summary>
    /// Inspector / menu checks for Mixamo FBX scale, clip, and floor alignment.
    /// </summary>
    public static class CharacterImportValidator
    {
        const string DefaultFbxPath = "Assets/HoloTile/Characters/Walking.fbx";

        [MenuItem("HoloTile/Validate Character FBX")]
        public static void ValidateFromMenu()
        {
            var fbx = AssetDatabase.LoadAssetAtPath<GameObject>(DefaultFbxPath);
            if (fbx == null)
            {
                EditorUtility.DisplayDialog("HoloTile",
                    $"No FBX at {DefaultFbxPath}.\nDrag your Mixamo FBX into Assets/HoloTile/Characters/.",
                    "OK");
                return;
            }

            ValidateAsset(fbx, DefaultFbxPath);
        }

        public static void ValidateAsset(GameObject fbxAsset, string assetPath)
        {
            var importer = AssetImporter.GetAtPath(assetPath) as ModelImporter;
            var clips = AssetDatabase.LoadAllAssetsAtPath(assetPath).OfType<AnimationClip>()
                .Where(c => !c.name.StartsWith("__preview")).ToArray();

            var temp = Object.Instantiate(fbxAsset);
            temp.transform.position = Vector3.zero;
            temp.transform.rotation = Quaternion.identity;

            var renderers = temp.GetComponentsInChildren<Renderer>();
            Bounds bounds = default;
            if (renderers.Length > 0)
            {
                bounds = renderers[0].bounds;
                foreach (var r in renderers)
                    bounds.Encapsulate(r.bounds);
            }

            var animator = temp.GetComponentInChildren<Animator>();
            bool humanoid = importer != null && importer.animationType == ModelImporterAnimationType.Human;
            Transform hips = humanoid && animator != null && animator.isHuman
                ? animator.GetBoneTransform(HumanBodyBones.Hips)
                : null;

            Object.DestroyImmediate(temp);

            float tileHalf = 2.5f * (HolotileConfig.TileSize + HolotileConfig.TileGap);
            string report =
                $"FBX: {assetPath}\n" +
                $"Import type: {(importer != null ? importer.animationType.ToString() : "?")}\n" +
                $"Bounds height: {bounds.size.y:F2} m (target ~1.7 m — auto-scaled at runtime)\n" +
                $"Bounds centre: {bounds.center}\n" +
                $"SupportY (floor): {HolotileConfig.SupportY:F3} m\n" +
                $"Walk clips: {clips.Length} ({string.Join(", ", clips.Select(c => $"{c.name} {c.length:F2}s"))})\n" +
                $"Humanoid hips: {(hips != null ? "OK" : "missing — set Rig = Humanoid on Model tab")}\n" +
                $"5×5 tile half-extent: ±{tileHalf:F2} m on X/Z\n\n" +
                "Animation is scrubbed via PlayableGraph (Humanoid-safe).\n" +
                "If frozen: confirm a walk clip is listed above and Rig = Humanoid.";

            Debug.Log($"[HoloTile] Character validation\n{report}");
            EditorUtility.DisplayDialog("HoloTile — Character FBX", report, "OK");
        }
    }
}
