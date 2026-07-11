using UnityEngine;
using UnityEngine.Rendering;
using HoloTile;

namespace HoloTile.Mechanism
{
    public static class HolotileMaterials
    {
        static Material _plate;
        static Material _socket;
        static Material _hemisphere;
        static Material _rim;
        static Material _foot;
        static Shader _cachedShader;
        static bool _loggedShader;

        public static Material Plate => _plate ??= Create(HolotileConfig.PlateColor);
        public static Material Socket => _socket ??= Create(HolotileConfig.SocketColor);
        public static Material Hemisphere => _hemisphere ??= Create(HolotileConfig.HemisphereColor);
        public static Material Rim => _rim ??= CreateEmissive(HolotileConfig.RimColor, 2.8f);
        public static Material Foot => _foot ??= Create(HolotileConfig.FootColor);

        public static Material CreateRuntime(Color color) => Create(color);

        // Legacy names
        public static Material Tile => Plate;
        public static Material Disk => Hemisphere;

        public static void InvalidateCache()
        {
            _plate = _socket = _hemisphere = _rim = _foot = null;
            _cachedShader = null;
            _loggedShader = false;
        }

        static Shader ResolveShader()
        {
            if (_cachedShader != null)
                return _cachedShader;

            if (GraphicsSettings.currentRenderPipeline != null)
            {
                _cachedShader = Shader.Find("Universal Render Pipeline/Unlit")
                    ?? Shader.Find("Universal Render Pipeline/Lit");
            }
            else
            {
                _cachedShader = Shader.Find("Standard")
                    ?? Shader.Find("Legacy Shaders/Diffuse");
            }

            if (_cachedShader == null)
                _cachedShader = Shader.Find("Hidden/InternalErrorShader");

            if (!_loggedShader)
            {
                _loggedShader = true;
                var pipeline = GraphicsSettings.currentRenderPipeline != null
                    ? GraphicsSettings.currentRenderPipeline.name
                    : "Built-in";
                Debug.Log($"[HoloTile] Materials using shader '{_cachedShader.name}' (pipeline: {pipeline})");
            }

            return _cachedShader;
        }

        static Material Create(Color color)
        {
            var mat = new Material(ResolveShader());
            ApplyColor(mat, color);
            return mat;
        }

        static Material CreateEmissive(Color color, float intensity)
        {
            var mat = Create(color);
            var hdr = color * intensity;
            if (mat.HasProperty("_EmissionColor"))
            {
                mat.EnableKeyword("_EMISSION");
                mat.SetColor("_EmissionColor", hdr);
                mat.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;
            }
            return mat;
        }

        static void ApplyColor(Material mat, Color color)
        {
            if (mat.HasProperty("_BaseColor"))
                mat.SetColor("_BaseColor", color);
            if (mat.HasProperty("_Color"))
                mat.SetColor("_Color", color);
            else
                mat.color = color;
        }
    }
}
