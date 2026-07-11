using UnityEngine;
using HoloTile;

namespace HoloTile.Control
{
    /// <summary>
    /// Intended travel velocity v_cmd(t) — what the floor must cancel to keep the
    /// person centered. Mirrors holotile_sim/intended_velocity.py (M4: commanded).
    /// </summary>
    public static class IntendedVelocity
    {
        /// <summary>
        /// Default scripted path: walk +X, then a sharp turn to +Z (Unity XZ).
        /// MuJoCo +y maps to Unity +z.
        /// </summary>
        public static Vector2 CommandedPath(float timeSec,
            float turnStart = HolotileConfig.TurnStartTime,
            float turnDuration = HolotileConfig.TurnDuration,
            float speed = HolotileConfig.WalkSpeed)
        {
            float ang;
            if (timeSec < turnStart)
                ang = 0f;
            else if (timeSec < turnStart + turnDuration)
                ang = (Mathf.PI * 0.5f) * (timeSec - turnStart) / turnDuration;
            else
                ang = Mathf.PI * 0.5f;

            return speed * new Vector2(Mathf.Cos(ang), Mathf.Sin(ang));
        }
    }
}
