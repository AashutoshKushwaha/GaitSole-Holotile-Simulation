using UnityEngine;
using HoloTile;
using HoloTile.Math;

namespace HoloTile.Mechanism
{
    /// <summary>
    /// [FIG 11-12] Swashplate drive — disk orienting mechanism.
    ///
    /// Physics function:
    ///   Rotates about vertical (Y) by azimuth alpha to set where the raised rim
    ///   segment sits, defining the horizontal push direction before spin.
    ///
    /// Patent: swashplate rotatable about vertical axis, independent of spin shaft.
    /// </summary>
    public class SwashplateDrive : MonoBehaviour
    {
        [SerializeField] Transform _azimuthPivot;

        public Transform AzimuthPivot => _azimuthPivot;

        public void Build(Transform parent)
        {
            var pivotGo = new GameObject("AzimuthPivot");
            _azimuthPivot = pivotGo.transform;
            _azimuthPivot.SetParent(parent, false);
            _azimuthPivot.localPosition = Vector3.zero;
            _azimuthPivot.localRotation = Quaternion.identity;
        }

        /// <summary>Apply azimuth alpha (rad) — orients all raised portions.</summary>
        public void SetAzimuth(float azimuthRad)
        {
            if (_azimuthPivot == null) return;
            _azimuthPivot.localRotation = DiskKinematics.AzimuthRotation(azimuthRad);
        }
    }
}
