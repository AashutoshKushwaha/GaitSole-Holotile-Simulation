using UnityEngine;
using HoloTile;
using HoloTile.Math;

namespace HoloTile.Mechanism
{
    /// <summary>
    /// [FIG 19-22] One disk assembly — recessed hemisphere + motor tilt/spin.
    ///
    ///   DiskMount (hemisphere centre height)
    ///     └── SwashplateDrive   [AzimuthPivot]  — motor linkage azimuth [2122/2160]
    ///           └── SpinDrive     [Tilt + Spin] — shaft tilt + rotation [2012]
    ///                 └── ContactDiskVisual     — socket + hemisphere + raised rim 2204
    /// </summary>
    public class DiskAssembly : MonoBehaviour
    {
        SwashplateDrive _swashplate;
        SpinDrive _spin;
        ContactDiskVisual _contact;

        TileCommand _command = TileCommand.Zero;
        float _spinPhaseRad;

        public TileCommand Command => _command;
        public float SpinPhaseRad => _spinPhaseRad;
        public Vector3 SurfaceVelocity => _command.SurfaceVelocity;

        public void Build(Transform parent, Vector3 localPosition)
        {
            transform.SetParent(parent, false);
            transform.localPosition = localPosition;
            transform.localRotation = Quaternion.identity;

            _swashplate = gameObject.AddComponent<SwashplateDrive>();
            _swashplate.Build(transform);

            _spin = gameObject.AddComponent<SpinDrive>();
            _spin.Build(_swashplate.AzimuthPivot);

            _contact = gameObject.AddComponent<ContactDiskVisual>();
            _contact.Build(_spin.SpinPivot);
        }

        /// <summary>[Claim 1 step 1] Orient disk — set azimuth alpha.</summary>
        public void Orient(float azimuthRad)
        {
            _command.AzimuthRad = azimuthRad;
            _swashplate.SetAzimuth(azimuthRad);
            _spin.SetAzimuthReference(azimuthRad);
        }

        /// <summary>[Claim 1 step 2] Set spin rate omega (rad/s).</summary>
        public void SetSpinRate(float spinRadPerSec)
        {
            _command.SpinRadPerSec = spinRadPerSec;
            _command.ClampSpin();
        }

        public void SetCommand(TileCommand cmd)
        {
            cmd.ClampSpin();
            _command = cmd;
            Orient(cmd.AzimuthRad);
            SetSpinRate(cmd.SpinRadPerSec);
            _contact?.SyncRimMarker(_command.AzimuthRad, _spinPhaseRad);
        }

        /// <summary>Advance one frame: integrate spin phase, update visuals.</summary>
        public void Tick(float dt)
        {
            _spin.SetAzimuthReference(_command.AzimuthRad);
            _spin.SetSpinRate(_command.SpinRadPerSec, dt);
            _spinPhaseRad = _spin.SpinPhaseRad;
            if (_contact != null)
                _contact.SyncRimMarker(_command.AzimuthRad, _spinPhaseRad);
        }

        public Vector3 RimWorldPosition()
        {
            return _contact != null
                ? _contact.RimWorldPosition(_command.AzimuthRad, _spinPhaseRad)
                : transform.position;
        }
    }
}
