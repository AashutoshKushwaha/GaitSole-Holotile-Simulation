using UnityEngine;
using HoloTile;
using HoloTile.Math;

namespace HoloTile.Mechanism
{
    /// <summary>
    /// [FIG 19-22] Recessed hemisphere disk visual.
    ///
    /// FIG 21 (idle): hemisphere 2014 wholly below structural plate 1966.
    /// FIG 22 (active): motor linkage tilts shaft; raised surface 2204 protrudes
    /// flush through the plate — the shoe contact arc.
    ///
    /// Visual recipe:
    ///   - Dark socket ring on the plate (hole in 1966)
    ///   - Light hemisphere sphere, centre at drive point, mostly below plate
    ///   - Yellow rim cap at kinematic contact (2204 / 1104)
    /// </summary>
    public class ContactDiskVisual : MonoBehaviour
    {
        Transform _hemisphere;
        Transform _rimMarker;
        Transform _diskCenter;

        public void Build(Transform spinPivot)
        {
            _diskCenter = spinPivot;
            float r = HolotileConfig.DiskRadius;
            float plateY = HolotileConfig.PlateTopLocalY;

            BuildSocketRing(plateY, r);
            BuildHemisphere(spinPivot, r);
            BuildRimMarker(r);
        }

        void BuildSocketRing(float plateY, float r)
        {
            // Dark recess opening flush with plate top [FIG 19 socket in 1966].
            var socketGo = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            socketGo.name = "SocketOpening";
            Object.Destroy(socketGo.GetComponent<Collider>());
            var t = socketGo.transform;
            t.SetParent(transform, false);
            t.localPosition = new Vector3(0f, plateY + 0.0004f, 0f);
            float d = r * 2f * HolotileConfig.SocketOpeningScale;
            t.localScale = new Vector3(d, 0.0012f, d);
            var rend = socketGo.GetComponent<Renderer>();
            if (rend != null)
                rend.sharedMaterial = HolotileMaterials.Socket;

            // Inner lip — slightly smaller, same colour as plate (edge of hole).
            var lipGo = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            lipGo.name = "SocketLip";
            Object.Destroy(lipGo.GetComponent<Collider>());
            var lip = lipGo.transform;
            lip.SetParent(transform, false);
            lip.localPosition = new Vector3(0f, plateY + 0.0008f, 0f);
            float lipD = r * 2f * 1.02f;
            lip.localScale = new Vector3(lipD, 0.0006f, lipD);
            var lipRend = lipGo.GetComponent<Renderer>();
            if (lipRend != null)
                lipRend.sharedMaterial = HolotileMaterials.Plate;
        }

        void BuildHemisphere(Transform spinPivot, float r)
        {
            // [2014] Hemispherical contact disk on drive shaft — sphere centred on pivot.
            // Top of sphere at plate level when upright; bulk hidden below plate by plate mesh.
            var hemiGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            hemiGo.name = "Hemisphere";
            Object.Destroy(hemiGo.GetComponent<Collider>());
            _hemisphere = hemiGo.transform;
            _hemisphere.SetParent(spinPivot, false);
            _hemisphere.localPosition = Vector3.zero;
            _hemisphere.localScale = Vector3.one * (r * 2f);
            var rend = hemiGo.GetComponent<Renderer>();
            if (rend != null)
                rend.sharedMaterial = HolotileMaterials.Hemisphere;
        }

        void BuildRimMarker(float r)
        {
            // [2204] Raised contact arc — bright cap on the protruding rim segment.
            var rimGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            rimGo.name = "RaisedRimSegment";
            Object.Destroy(rimGo.GetComponent<Collider>());
            _rimMarker = rimGo.transform;
            _rimMarker.SetParent(transform, false);
            float d = HolotileConfig.RimMarkerDiameter;
            _rimMarker.localScale = new Vector3(d * 1.2f, d * 0.55f, d * 1.2f);
            var rend = rimGo.GetComponent<Renderer>();
            if (rend != null)
                rend.sharedMaterial = HolotileMaterials.Rim;
        }

        /// <summary>Place raised rim segment at patent contact point 2204.</summary>
        public void SyncRimMarker(float azimuthRad, float spinPhaseRad)
        {
            if (_rimMarker == null || _diskCenter == null)
                return;

            Vector3 center = _diskCenter.position;
            Vector3 rim = DiskKinematics.RimContactPosition(center, azimuthRad, spinPhaseRad);
            rim.y += HolotileConfig.RimMarkerLift;
            _rimMarker.position = rim;

            // Orient cap to sit on the bump (flat side down).
            Vector3 n = (rim - center).normalized;
            if (n.sqrMagnitude > 1e-6f)
                _rimMarker.rotation = Quaternion.FromToRotation(Vector3.up, n);
        }

        public Vector3 RimWorldPosition(float azimuthRad, float spinPhaseRad)
        {
            if (_diskCenter == null)
                return transform.position;
            return DiskKinematics.RimContactPosition(_diskCenter.position, azimuthRad, spinPhaseRad);
        }
    }
}
