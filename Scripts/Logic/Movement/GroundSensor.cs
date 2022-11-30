using UnityEngine;

namespace NekoNeko
{
    public class GroundSensor
    {
        public float GroundThresholdDistance { get; set; }
        public LayerMask LayerMask { get; set; }
        public bool UseRealGroundNormal { get; set; }

        /// <summary>
        /// Probe ground from origin downwards for a specified range.
        /// </summary>
        /// <param name="groundInfo"></param>
        /// <param name="range"></param>
        /// <param name="thickness"></param>
        /// <param name="origin"></param>
        /// <returns></returns>
        public bool ProbeGround(out GroundInfo groundInfo, float range, float thickness, Vector3 origin)
        {
            return ProbeGround(out groundInfo, range, thickness, origin, LayerMask);
        }

        /// <summary>
        /// Probe ground from origin downwards for a specified range.
        /// </summary>
        /// <param name="groundInfo"></param>
        /// <param name="range"></param>
        /// <param name="thickness"></param>
        /// <param name="origin"></param>
        /// <param name="layerMask"></param>
        /// <returns></returns>
        public bool ProbeGround(out GroundInfo groundInfo, float range, float thickness, Vector3 origin, LayerMask layerMask)
        {
            groundInfo = GroundInfo.Empty;
            bool isGroundInRange = false;

            RaycastHit hitInfo;
            bool hasHit = false;
            if (thickness <= 0f)
            {
                hasHit = Physics.Raycast(new Ray(origin, Vector3.down), out hitInfo, maxDistance: range);
            }
            else
            {
                hasHit = Physics.SphereCast(new Ray(origin, Vector3.down), thickness / 2f, out hitInfo, maxDistance: range, layerMask: layerMask);
            }

#if UNITY_EDITOR
            Vector3 groundThresholdPoint = origin - new Vector3(0f, GroundThresholdDistance, 0f);
            Debug.DrawLine(origin, groundThresholdPoint, Color.grey);
#endif

            if (hasHit)
            {
                groundInfo.Distance = origin.y - hitInfo.point.y;
                if (isWithinGround(groundInfo.Distance))
                {
                    isGroundInRange = true;
                    groundInfo.Normal = hitInfo.normal;
                    groundInfo.Point = hitInfo.point;
                    groundInfo.Collider = hitInfo.collider;

                    if (UseRealGroundNormal && thickness > 0f)
                    {
                        Vector3 tmpOrigin = hitInfo.point + new Vector3(0f, 0.01f, 0f);
                        RaycastHit realNormalHitInfo;
                        if (hitInfo.collider.Raycast(new Ray(tmpOrigin, Vector3.down), out realNormalHitInfo, maxDistance: 0.1f))
                        {
                            groundInfo.Normal = realNormalHitInfo.normal;
                        }
                    }
                }
            }

            groundInfo.IsOnGround = isGroundInRange;
            return isGroundInRange;
        }

        /// <summary>
        /// Returns true if the provided ground distance is within ground distance threshold.
        /// </summary>
        /// <param name="groundDistance"></param>
        /// <returns></returns>
        public bool isWithinGround(float groundDistance)
        {
            return groundDistance <= GroundThresholdDistance;
        }
    }
}