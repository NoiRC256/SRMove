using UnityEngine;

namespace NekoLab.SRMove.NekoMath
{
    public static partial class MathUtil
    {

        /// <summary>
        /// Returns an angle clamped between the specified minimum and maximum values.
        /// </summary>
        /// <param name="angle"></param>
        /// <param name="min"></param>
        /// <param name="max"></param>
        /// <returns></returns>
        public static float ClampAngle(float angle, float min, float max)
        {
            return Mathf.Clamp(NormalizeAngle(angle), min, max);
        }

        /// <summary>
        /// Returns an angle normalized to between -180 and 180.
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        public static float NormalizeAngle(float angle)
        {
            angle %= 360;
            angle = angle > 180 ? angle - 360 : angle < -180 ? angle + 360 : angle;
            return angle;
        }

        /// <summary>
        /// Returns angles normalized to between -180 and 180.
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        public static Vector2 NormalizeAngle(Vector2 angle)
        {
            angle.x = NormalizeAngle(angle.x);
            angle.y = NormalizeAngle(angle.y);
            return angle;
        }

        /// <summary>
        /// Returns angles normalized to between -180 and 180.
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        public static Vector3 NormalizeAngle(Vector3 angle)
        {
            angle.x = NormalizeAngle(angle.x);
            angle.y = NormalizeAngle(angle.y);
            angle.z = NormalizeAngle(angle.z);
            return angle;
        }
    }
}