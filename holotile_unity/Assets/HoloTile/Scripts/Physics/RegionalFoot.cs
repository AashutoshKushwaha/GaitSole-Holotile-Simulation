using UnityEngine;
using HoloTile;
using HoloTile.Mechanism;

namespace HoloTile.PhysicsSim
{
    /// <summary>
    /// One foot rigidbody with 4 plantar belt patches (heel/mid/fore/toe).
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class RegionalFoot : MonoBehaviour
    {
        public string Side { get; private set; }
        public int SideIndex => Side == "l" ? 1 : 0;

        Rigidbody _rb;
        BoxCollider _collider;
        Transform[] _patches = new Transform[4];
        Vector3[] _patchVelocities = new Vector3[4];
        bool _isStance;

        public Rigidbody Body => _rb;
        public Vector3 AnkleWorld => transform.position;
        public bool IsStance => _isStance;

        public void Build(string side, Vector3 ankleWorld, bool hideCollider = true)
        {
            Side = side;
            _rb = GetComponent<Rigidbody>();
            if (_rb == null)
                _rb = gameObject.AddComponent<Rigidbody>();

            _rb.mass = HolotileConfig.PuckMass;
            _rb.interpolation = RigidbodyInterpolation.Interpolate;
            _rb.constraints = RigidbodyConstraints.FreezeRotation;
            _rb.collisionDetectionMode = CollisionDetectionMode.Continuous;

            _collider = GetComponent<BoxCollider>();
            if (_collider == null)
                _collider = gameObject.AddComponent<BoxCollider>();
            _collider.size = HolotileConfig.PuckHalfExtents * 2f;
            _collider.material = HolotilePhysicsMaterials.Frictionless;
            _collider.enabled = !hideCollider;

            transform.position = ankleWorld;
            gameObject.name = $"RegionalFoot_{side}";

            for (int i = 0; i < 4; i++)
            {
                var go = new GameObject(HolotileConfig.FootRegionNames[i]);
                go.transform.SetParent(transform, false);
                go.transform.localPosition = HolotileConfig.RegionOffsetRight[i];
                _patches[i] = go.transform;
            }
        }

        public void EnablePhysics()
        {
            if (_rb == null) return;
            _rb.isKinematic = false;
            if (_collider != null)
                _collider.enabled = true;
            _rb.linearVelocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }

        public void SetPatchVelocities(Vector3[] velocities)
        {
            if (velocities == null || velocities.Length < 4) return;
            for (int i = 0; i < 4; i++)
                _patchVelocities[i] = velocities[i];
        }

        public void FollowKinematic(Vector3 ankle, Vector3 toe)
        {
            if (_rb == null) return;

            if (!_rb.isKinematic)
            {
                _rb.linearVelocity = Vector3.zero;
                _rb.angularVelocity = Vector3.zero;
                _rb.isKinematic = true;
            }

            if (_collider != null)
                _collider.enabled = false;
            transform.position = ankle;
            AlignFoot(toe);
        }

        /// <summary>Dynamic foot: belt on stance + spring toward OpenSim target (no hard mode pops).</summary>
        public void DrivePhysicsStep(Vector3 targetAnkle, Vector3 targetToe, float normalForceSum)
        {
            if (_rb == null)
                return;

            UpdateStanceState(normalForceSum);

            if (_rb.isKinematic)
                EnablePhysics();

            SnapVerticalToSupport();
            float springScale = _isStance ? 1f : 2.2f;
            ApplyReplaySpring(targetAnkle, springScale);
            AlignFoot(targetToe);
        }

        void UpdateStanceState(float load)
        {
            if (load >= HolotileConfig.StanceGrfThresholdHigh)
                _isStance = true;
            else if (load <= HolotileConfig.StanceGrfThresholdLow)
                _isStance = false;
        }

        void SnapVerticalToSupport()
        {
            float restY = FootPuck.RestFootCenterY;
            if (transform.position.y < restY - 0.015f)
            {
                var p = transform.position;
                p.y = restY;
                transform.position = p;
                _rb.position = p;
                var v = _rb.linearVelocity;
                v.y = 0f;
                _rb.linearVelocity = v;
            }
        }

        void ApplyReplaySpring(Vector3 targetAnkle, float scale)
        {
            Vector3 error = targetAnkle - transform.position;
            Vector3 vel = _rb.linearVelocity;
            Vector3 force = error * (HolotileConfig.ReplayFootGuideK * scale)
                - vel * (HolotileConfig.ReplayFootGuideD * (0.6f + 0.4f * scale));
            force.y *= 2.5f;
            _rb.AddForce(force, ForceMode.Force);
        }

        public void ApplyRegionalBelt(FloorGrid grid)
        {
            if (_rb == null || _rb.isKinematic || grid == null || !_isStance)
                return;

            float frac = HolotileConfig.RegionPatchMassFraction;
            float totalNormal = _rb.mass * Mathf.Abs(Physics.gravity.y);

            for (int i = 0; i < 4; i++)
            {
                Vector3 world = _patches[i].position;
                ActiveTile tile = grid.TileAtWorld(world);
                Vector3 vTile = tile != null ? tile.SurfaceVelocity : Vector3.zero;
                Vector3 vCmd = _patchVelocities[i].sqrMagnitude > 1e-6f ? _patchVelocities[i] : vTile;
                Vector3 vSurf = vCmd.sqrMagnitude > 1e-6f ? vCmd : vTile;

                bool grounded = world.y <= FootPuck.RestFootCenterY + 0.03f;
                float n = grounded ? totalNormal * frac : 0f;
                RegionalBeltDrive.ApplyAtPoint(_rb, world, vSurf, n);
            }
        }

        void AlignFoot(Vector3 toe)
        {
            Vector3 fwd = toe - transform.position;
            fwd.y = 0f;
            if (fwd.sqrMagnitude > 1e-6f)
                transform.rotation = Quaternion.LookRotation(fwd.normalized, Vector3.up);
        }
    }
}
