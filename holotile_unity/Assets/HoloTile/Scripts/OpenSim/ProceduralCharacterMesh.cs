using UnityEngine;
using HoloTile;
using HoloTile.Mechanism;

namespace HoloTile.OpenSim
{
    /// <summary>
    /// Procedural clothed mannequin skinned to OpenSim FK joint transforms.
    /// Drop a custom FBX under Assets/HoloTile/Characters/ and assign on HumanWalkerDemo to replace.
    /// </summary>
    public class ProceduralCharacterMesh : MonoBehaviour
    {
        static readonly Color Shirt = new Color(0.13f, 0.42f, 0.20f);
        static readonly Color Pants = new Color(0.12f, 0.15f, 0.36f);
        static readonly Color Skin = new Color(0.80f, 0.62f, 0.48f);
        static readonly Color Shoe = new Color(0.09f, 0.09f, 0.11f);

        Transform _pelvis;
        Transform _chest;
        Transform _head;
        Transform _hipL;
        Transform _hipR;
        Transform _kneeL;
        Transform _kneeR;
        Transform _ankleL;
        Transform _ankleR;
        Transform _toeL;
        Transform _toeR;

        public Transform Pelvis => _pelvis;

        public void Build()
        {
            _pelvis = CreateBone("Pelvis", null);
            _chest = CreateBone("Chest", _pelvis);
            _head = CreateBone("Head", _chest);

            _hipL = CreateBone("HipL", _pelvis);
            _hipR = CreateBone("HipR", _pelvis);
            _kneeL = CreateBone("KneeL", _hipL);
            _kneeR = CreateBone("KneeR", _hipR);
            _ankleL = CreateBone("AnkleL", _kneeL);
            _ankleR = CreateBone("AnkleR", _kneeR);
            _toeL = CreateBone("ToeL", _ankleL);
            _toeR = CreateBone("ToeR", _ankleR);

            AddCapsule(_pelvis, _chest, 0.11f, Shirt, "Torso");
            AddCapsule(_chest, _head, 0.09f, Skin, "HeadMesh");
            AddCapsule(_hipL, _kneeL, 0.06f, Pants, "ThighL");
            AddCapsule(_hipR, _kneeR, 0.06f, Pants, "ThighR");
            AddCapsule(_kneeL, _ankleL, 0.05f, Pants, "ShankL");
            AddCapsule(_kneeR, _ankleR, 0.05f, Pants, "ShankR");
            AddCapsule(_ankleL, _toeL, 0.045f, Shoe, "FootL");
            AddCapsule(_ankleR, _toeR, 0.045f, Shoe, "FootR");
        }

        public void ApplyPose(SkeletonPose pose)
        {
            if (_pelvis == null) return;

            _pelvis.position = pose.Pelvis;
            _chest.position = pose.Chest;
            _head.position = pose.Head;
            _hipL.position = pose.HipL;
            _hipR.position = pose.HipR;
            _kneeL.position = pose.KneeL;
            _kneeR.position = pose.KneeR;
            _ankleL.position = pose.AnkleL;
            _ankleR.position = pose.AnkleR;
            _toeL.position = pose.ToeL;
            _toeR.position = pose.ToeR;
        }

        Transform CreateBone(string name, Transform parent)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent != null ? parent : transform, false);
            return go.transform;
        }

        void AddCapsule(Transform a, Transform b, float radius, Color color, string meshName)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = meshName;
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(transform, true);
            UpdateCapsule(go.transform, a.position, b.position, radius);
            var rend = go.GetComponent<Renderer>();
            if (rend != null)
                rend.sharedMaterial = HolotileMaterials.CreateRuntime(color);
        }

        static void UpdateCapsule(Transform cap, Vector3 a, Vector3 b, float radius)
        {
            Vector3 mid = (a + b) * 0.5f;
            Vector3 dir = b - a;
            float len = dir.magnitude;
            cap.position = mid;
            if (len > 1e-5f)
                cap.rotation = Quaternion.FromToRotation(Vector3.up, dir);
            cap.localScale = new Vector3(radius * 2f, len * 0.5f, radius * 2f);
        }

        void LateUpdate()
        {
            // Keep limb meshes aligned after pose update.
            foreach (Transform child in transform)
            {
                if (child.name == "Torso")
                    UpdateCapsule(child, _pelvis.position, _chest.position, 0.11f);
                else if (child.name == "HeadMesh")
                    UpdateCapsule(child, _chest.position, _head.position, 0.09f);
                else if (child.name == "ThighL")
                    UpdateCapsule(child, _hipL.position, _kneeL.position, 0.06f);
                else if (child.name == "ThighR")
                    UpdateCapsule(child, _hipR.position, _kneeR.position, 0.06f);
                else if (child.name == "ShankL")
                    UpdateCapsule(child, _kneeL.position, _ankleL.position, 0.05f);
                else if (child.name == "ShankR")
                    UpdateCapsule(child, _kneeR.position, _ankleR.position, 0.05f);
                else if (child.name == "FootL")
                    UpdateCapsule(child, _ankleL.position, _toeL.position, 0.045f);
                else if (child.name == "FootR")
                    UpdateCapsule(child, _ankleR.position, _toeR.position, 0.045f);
            }
        }
    }
}
