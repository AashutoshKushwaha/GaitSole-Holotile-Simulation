using UnityEngine;
using HoloTile;
using HoloTile.Mechanism;

namespace HoloTile.PhysicsSim
{
    /// <summary>
    /// Foot puck rigidbody — patent "shoe" on the floor (Phase C test object).
    /// Not used by Phase D MechanismDemo.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class FootPuck : MonoBehaviour
    {
        public string Side { get; private set; }
        public bool IsGrounded { get; private set; }

        Rigidbody _rb;
        BoxCollider _collider;
        Vector3 _externalForce;

        public Rigidbody Body => _rb;
        public bool IsSwingFoot => _rb != null && _rb.isKinematic;

        public Vector2 FootXz => new Vector2(transform.position.x, transform.position.z);

        public void Initialize(string side, Vector3 position)
        {
            Side = side;
            _rb = GetComponent<Rigidbody>();
            if (_rb == null)
                _rb = gameObject.AddComponent<Rigidbody>();

            _rb.mass = HolotileConfig.PuckMass;
            _rb.interpolation = RigidbodyInterpolation.Interpolate;
            _rb.constraints = RigidbodyConstraints.FreezeRotation;

            _collider = GetComponent<BoxCollider>();
            if (_collider == null)
                _collider = gameObject.AddComponent<BoxCollider>();
            _collider.size = HolotileConfig.PuckHalfExtents * 2f;
            _collider.material = HolotilePhysicsMaterials.Frictionless;

            EnsureVisual();

            transform.position = position;
            gameObject.name = $"Foot_{side}";
        }

        void EnsureVisual()
        {
            if (transform.Find("Visual") != null)
                return;

            var visualGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
            visualGo.name = "Visual";
            Destroy(visualGo.GetComponent<Collider>());
            visualGo.transform.SetParent(transform, false);
            visualGo.transform.localScale = HolotileConfig.PuckHalfExtents * 2f;
            var rend = visualGo.GetComponent<Renderer>();
            if (rend != null)
                rend.sharedMaterial = HolotileMaterials.Foot;
        }

        public void SetExternalForce(Vector3 forceXZ)
        {
            _externalForce = new Vector3(forceXZ.x, 0f, forceXZ.z);
        }

        public void SetKinematic(Vector3 position)
        {
            if (!_rb.isKinematic)
            {
                _rb.linearVelocity = Vector3.zero;
                _rb.angularVelocity = Vector3.zero;
            }

            _rb.isKinematic = true;
            if (_collider != null)
                _collider.enabled = false;
            transform.position = position;
        }

        public void SetDynamic()
        {
            if (_collider != null)
                _collider.enabled = true;
            _rb.isKinematic = false;
            _rb.linearVelocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }

        public void SetDynamic(Vector3 position)
        {
            transform.position = position;
            if (_collider != null)
                _collider.enabled = true;
            _rb.isKinematic = false;
            _rb.position = position;
            _rb.linearVelocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }

        public void ApplyBeltDrive(FloorGrid grid, bool walkerBelt = false)
        {
            if (_rb.isKinematic || grid == null) return;

            IsGrounded = IsPuckGrounded();
            ActiveTile tile = grid.TileAtWorld(transform.position);
            Vector3 vSurf = tile != null ? tile.SurfaceVelocity : Vector3.zero;
            float n = BeltDrive.EstimateNormalForce(_rb, IsGrounded);
            if (walkerBelt)
            {
                BeltDrive.Apply(_rb, vSurf, n, _externalForce,
                    HolotileConfig.WalkerBeltK, HolotileConfig.WalkerBeltMu);
            }
            else
            {
                BeltDrive.Apply(_rb, vSurf, n, _externalForce);
            }
        }

        bool IsPuckGrounded()
        {
            float halfH = HolotileConfig.PuckHalfExtents.y;
            var origin = transform.position - Vector3.up * (halfH - 0.002f);
            bool rayHit = Physics.Raycast(origin, Vector3.down, 0.02f);
            bool nearSupport = transform.position.y <= RestFootCenterY + 0.025f;
            return rayHit || nearSupport;
        }

        public static float RestFootCenterY =>
            HolotileConfig.SupportY + HolotileConfig.PuckHalfExtents.y;
    }
}
