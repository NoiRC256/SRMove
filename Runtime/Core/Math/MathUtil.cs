using UnityEngine;

namespace CC.SRMove.Util
{
    public static partial class MathUtil
    {
        /// <summary>
        /// Returns an angle normalized to between 0 and 360.
        /// </summary>
        /// <param name="eulerAngles"></param>
        /// <returns></returns>
        public static float NormalizeAngle360(float eulerAngles)
        {
            float result = eulerAngles - Mathf.CeilToInt(eulerAngles / 360f) * 360f;
            if (result < 0)
            {
                result += 360f;
            }
            return result;
        }
    }
}