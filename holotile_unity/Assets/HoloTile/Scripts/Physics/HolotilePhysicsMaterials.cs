using UnityEngine;

namespace HoloTile.PhysicsSim
{
    /// <summary>Runtime PhysicMaterials for Phase C belt-drive simulation.</summary>
    public static class HolotilePhysicsMaterials
    {
        static PhysicsMaterial _frictionless;
        static PhysicsMaterial _foot;

        public static PhysicsMaterial Frictionless
        {
            get
            {
                if (_frictionless == null)
                {
                    _frictionless = new PhysicsMaterial("HolotileFrictionless")
                    {
                        dynamicFriction = 0f,
                        staticFriction = 0f,
                        frictionCombine = PhysicsMaterialCombine.Minimum,
                        bounciness = 0f
                    };
                }
                return _frictionless;
            }
        }

        public static PhysicsMaterial Foot
        {
            get
            {
                if (_foot == null)
                {
                    _foot = new PhysicsMaterial("HolotileFoot")
                    {
                        dynamicFriction = 0.4f,
                        staticFriction = 0.4f,
                        frictionCombine = PhysicsMaterialCombine.Average,
                        bounciness = 0f
                    };
                }
                return _foot;
            }
        }
    }
}
