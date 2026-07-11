#if UNITY_EDITOR
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace HoloTile.Editor
{
    /// <summary>
    /// One-click URP setup for projects opened without the URP template.
    /// Menu: HoloTile → Create And Assign URP Pipeline
    /// </summary>
    public static class HolotileRenderPipelineSetup
    {
        const string SettingsDir = "Assets/HoloTile/Settings";
        const string RendererPath = SettingsDir + "/HolotileForwardRenderer.asset";
        const string UrpAssetPath = SettingsDir + "/HolotileURP.asset";

        [MenuItem("HoloTile/Create And Assign URP Pipeline")]
        public static void CreateAndAssign()
        {
            EnsureFolder(SettingsDir);

            var renderer = AssetDatabase.LoadAssetAtPath<UniversalRendererData>(RendererPath);
            if (renderer == null)
            {
                renderer = ScriptableObject.CreateInstance<UniversalRendererData>();
                renderer.name = "HolotileForwardRenderer";
                AssetDatabase.CreateAsset(renderer, RendererPath);
            }

            var urp = AssetDatabase.LoadAssetAtPath<UniversalRenderPipelineAsset>(UrpAssetPath);
            if (urp == null)
            {
                urp = ScriptableObject.CreateInstance<UniversalRenderPipelineAsset>();
                urp.name = "HolotileURP";
                AssetDatabase.CreateAsset(urp, UrpAssetPath);
            }

            // Wire renderer list via SerializedObject (public API varies by URP version).
            var so = new SerializedObject(urp);
            var list = so.FindProperty("m_RendererDataList");
            if (list != null)
            {
                list.ClearArray();
                list.InsertArrayElementAtIndex(0);
                list.GetArrayElementAtIndex(0).objectReferenceValue = renderer;
                so.ApplyModifiedPropertiesWithoutUndo();
            }

            EditorUtility.SetDirty(urp);
            EditorUtility.SetDirty(renderer);
            AssetDatabase.SaveAssets();

            GraphicsSettings.defaultRenderPipeline = urp;
            QualitySettings.renderPipeline = urp;

            HoloTile.Mechanism.HolotileMaterials.InvalidateCache();

            Debug.Log("[HoloTile] URP pipeline assigned. Graphics Settings now use HolotileURP.asset. " +
                      "Press Play again — materials should render with URP Unlit/Lit.");
        }

        [MenuItem("HoloTile/Log Active Render Pipeline")]
        public static void LogPipeline()
        {
            var rp = GraphicsSettings.currentRenderPipeline;
            if (rp == null)
                Debug.Log("[HoloTile] Active pipeline: Built-in (no SRP asset assigned). " +
                          "Use HoloTile → Create And Assign URP Pipeline if you want URP.");
            else
                Debug.Log($"[HoloTile] Active pipeline: {rp.name} ({rp.GetType().Name})");
        }

        static void EnsureFolder(string path)
        {
            if (AssetDatabase.IsValidFolder(path))
                return;
            var parent = Path.GetDirectoryName(path)?.Replace('\\', '/');
            var leaf = Path.GetFileName(path);
            if (!string.IsNullOrEmpty(parent) && !AssetDatabase.IsValidFolder(parent))
                EnsureFolder(parent);
            AssetDatabase.CreateFolder(parent, leaf);
        }
    }
}
#endif
