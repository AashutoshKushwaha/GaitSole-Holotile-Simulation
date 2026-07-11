using UnityEngine;
using HoloTile;
using HoloTile.PhysicsSim;

namespace HoloTile.Mechanism
{
    /// <summary>
    /// [FIG 3, FIG 19 item 1966] Structural plate — flush walking surface.
    ///
    /// Upper surface at SupportY. Hemispheres (FIG 21-22) sit in recesses below this
    /// plane; when tilted, only a raised rim arc (2204) breaks through the plate.
    /// </summary>
    public class StructuralPad : MonoBehaviour
    {
        [SerializeField] Transform _plateTransform;

        public float TopY => HolotileConfig.SupportY;

        public void Build(Transform parent, bool physicsSupport = false)
        {
            var plateGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
            plateGo.name = "StructuralPlate";
            _plateTransform = plateGo.transform;
            _plateTransform.SetParent(parent, false);

            float padH = HolotileConfig.SupportPadHalfThickness;
            _plateTransform.localScale = new Vector3(
                HolotileConfig.TileSize,
                padH * 2f,
                HolotileConfig.TileSize);
            _plateTransform.localPosition = new Vector3(0f, HolotileConfig.SupportY - padH, 0f);

            var col = plateGo.GetComponent<Collider>();
            if (physicsSupport && col != null)
                col.material = HolotilePhysicsMaterials.Frictionless;
            else if (col != null)
                Destroy(col);

            var rend = plateGo.GetComponent<Renderer>();
            if (rend != null)
                rend.sharedMaterial = HolotileMaterials.Plate;
        }
    }
}
