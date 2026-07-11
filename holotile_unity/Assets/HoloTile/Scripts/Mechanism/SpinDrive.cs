using UnityEngine;
using HoloTile;
using HoloTile.Math;

namespace HoloTile.Mechanism
{
    /// <summary>
    /// [FIG 11-12] Spin drive — disk rotation mechanism.
    ///
    /// Physics function:
    ///   Rotates the contact disk about the tilted spin axis at rate omega (rad/s).
    ///   Spin phase phi = integral omega dt; direction of rotation sets travel sign.
    ///
    /// Implemented as accumulated spin phase on a pivot under the fixed tilt joint.
    /// </summary>
    public class SpinDrive : MonoBehaviour
    {
        [SerializeField] Transform _tiltPivot;
        [SerializeField] Transform _spinPivot;

        float _spinPhaseRad;
        float _azimuthRad;

        public Transform SpinPivot => _spinPivot;
        public float SpinPhaseRad => _spinPhaseRad;

        public void Build(Transform azimuthPivot)
        {
            var tiltGo = new GameObject("TiltPivot");
            _tiltPivot = tiltGo.transform;
            _tiltPivot.SetParent(azimuthPivot, false);
            _tiltPivot.localPosition = Vector3.zero;
            _tiltPivot.localRotation = DiskKinematics.TiltRotation();

            var spinGo = new GameObject("SpinPivot");
            _spinPivot = spinGo.transform;
            _spinPivot.SetParent(_tiltPivot, false);
            _spinPivot.localPosition = Vector3.zero;
            _spinPivot.localRotation = Quaternion.identity;
        }

        public void SetAzimuthReference(float azimuthRad)
        {
            _azimuthRad = azimuthRad;
        }

        public void SetSpinRate(float spinRadPerSec, float dt)
        {
            _spinPhaseRad += spinRadPerSec * dt;
            ApplySpinTransform();
        }

        public void ApplySpinTransform()
        {
            if (_spinPivot == null || _tiltPivot == null) return;
            Vector3 axisWorld = DiskKinematics.SpinAxisWorld(_azimuthRad);
            Vector3 axisLocal = _tiltPivot.InverseTransformDirection(axisWorld).normalized;
            _spinPivot.localRotation = Quaternion.AngleAxis(_spinPhaseRad * Mathf.Rad2Deg, axisLocal);
        }
    }
}
