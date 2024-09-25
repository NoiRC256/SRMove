using UnityEngine;

namespace CCLab.SRMove.Util
{
    public static partial class MathUtil
    {
        /// <summary>
        /// Interpolates between a and b by t, where t can be multiplied by delta time.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static float ExLerp(float a, float b, float t)
        {
            return Mathf.Lerp(b, a, Mathf.Exp(-t));
        }

        /// <summary>
        /// Interpolates between a and b by t, where t can be multiplied by delta time.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static Vector2 ExLerp(Vector2 a, Vector2 b, float t)
        {
            return Vector2.Lerp(b, a, Mathf.Exp(-t));
        }

        /// <summary>
        /// Interpolates between a and b by t, where t can be multiplied by delta time.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static Vector3 ExLerp(Vector3 a, Vector3 b, float t)
        {
            return Vector3.Lerp(b, a, Mathf.Exp(-t));
        }
    }
}
