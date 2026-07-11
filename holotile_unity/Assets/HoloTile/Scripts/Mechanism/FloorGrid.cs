using System.Collections.Generic;
using UnityEngine;
using HoloTile;
using HoloTile.Math;
using HoloTile.PhysicsSim;

namespace HoloTile.Mechanism
{
    /// <summary>
    /// [§0046] Modular floor — grid of active tiles side-by-side with small gaps.
    ///
    /// Physics function:
    ///   Maps world position (x, z) -> active tile index for per-tile commands.
    ///   Each tile operates independently (multi-user patent FIG 9 — future).
    /// </summary>
    public class FloorGrid : MonoBehaviour
    {
        readonly Dictionary<Vector2Int, ActiveTile> _tiles = new Dictionary<Vector2Int, ActiveTile>();

        public float Pitch { get; private set; }
        public float OriginX { get; private set; }
        public float OriginZ { get; private set; }
        public int TilesX { get; private set; }
        public int TilesY { get; private set; }
        public IReadOnlyDictionary<Vector2Int, ActiveTile> Tiles => _tiles;

        public void Build(int tilesX, int tilesY, bool physicsSupport = false)
        {
            TilesX = tilesX;
            TilesY = tilesY;
            Pitch = HolotileConfig.TileSize + HolotileConfig.TileGap;
            OriginX = -(tilesX - 1) * 0.5f * Pitch;
            OriginZ = -(tilesY - 1) * 0.5f * Pitch;
            gameObject.name = $"FloorGrid_{tilesX}x{tilesY}";

            for (int tx = 0; tx < tilesX; tx++)
            {
                for (int tz = 0; tz < tilesY; tz++)
                {
                    var index = new Vector2Int(tx, tz);
                    float cx = OriginX + tx * Pitch;
                    float cz = OriginZ + tz * Pitch;
                    var go = new GameObject();
                    var tile = go.AddComponent<ActiveTile>();
                    tile.Build(index, new Vector3(cx, 0f, cz), transform, physicsSupport);
                    _tiles[index] = tile;
                }
            }

            if (physicsSupport)
                BuildContinuousBacking(tilesX, tilesY);
        }

        void BuildContinuousBacking(int tilesX, int tilesY)
        {
            // One frictionless slab — closes tile gaps so feet cannot fall through.
            float spanX = tilesX * HolotileConfig.TileSize + (tilesX - 1) * HolotileConfig.TileGap;
            float spanZ = tilesY * HolotileConfig.TileSize + (tilesY - 1) * HolotileConfig.TileGap;
            float padH = HolotileConfig.SupportPadHalfThickness;

            var backGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
            backGo.name = "PhysicsBacking";
            backGo.transform.SetParent(transform, false);
            backGo.transform.localScale = new Vector3(spanX, padH * 2f, spanZ);
            backGo.transform.localPosition = new Vector3(0f, HolotileConfig.SupportY - padH, 0f);

            var col = backGo.GetComponent<Collider>();
            if (col != null)
                col.material = HolotilePhysicsMaterials.Frictionless;

            var rend = backGo.GetComponent<Renderer>();
            if (rend != null)
                rend.enabled = false;
        }

        public ActiveTile TileAtWorld(Vector3 worldPos)
        {
            int tx = Mathf.Clamp(Mathf.RoundToInt((worldPos.x - OriginX) / Pitch), 0, TilesX - 1);
            int tz = Mathf.Clamp(Mathf.RoundToInt((worldPos.z - OriginZ) / Pitch), 0, TilesY - 1);
            return _tiles[new Vector2Int(tx, tz)];
        }

        public void SetAllTiles(TileCommand command)
        {
            foreach (var tile in _tiles.Values)
                tile.SetCommand(command);
        }

        public void SetAllTiles(float azimuthRad, float spinRadPerSec)
        {
            SetAllTiles(new TileCommand(azimuthRad, spinRadPerSec));
        }

        public void SetTile(Vector2Int index, TileCommand command)
        {
            if (_tiles.TryGetValue(index, out var tile))
                tile.SetCommand(command);
        }

        public void SetTileAtWorld(Vector3 worldPos, TileCommand command)
        {
            var tile = TileAtWorld(worldPos);
            if (tile != null)
                SetTile(tile.Index, command);
        }

        public void SetAllTilesIdle(float prevAzimuthRad = 0f)
        {
            SetAllTiles(new TileCommand(prevAzimuthRad, 0f));
        }

        public void Tick(float dt)
        {
            foreach (var tile in _tiles.Values)
                tile.Tick(dt);
        }
    }
}
