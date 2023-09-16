using UnityEngine;
using NekoLib.SRMove.NekoPhysics;

namespace NekoLib.SRMove
{
    public static class GroundSensor
    {
        /// <summary>
        /// Probe ground from origin downwards for a specified range.
        /// </summary>
        /// <param name="groundInfo"></param>
        /// <param name="distance"></param>
        /// <param name="thickness"></param>
        /// <param name="origin"></param>
        /// <param name="layerMask"></param>
        /// <returns></returns>
        public static bool ProbeGround(out GroundInfo groundInfo, float thresholdDistance, float distance, float thickness, Vector3 origin, LayerMask layerMask,
            bool useRealGroundNormal = false)
        {
            groundInfo = GroundInfo.Empty;
            bool isGroundInRange = false;

            bool hasHit = false;
            RaycastHit hitInfo;
            if (thickness <= 0f) hasHit = Physics.Raycast(origin, Vector3.down, out hitInfo,
                    maxDistance: distance, layerMask: layerMask);
            else hasHit = Physics.SphereCast(origin, thickness / 2f, Vector3.down, out hitInfo,
                    maxDistance: distance, layerMask: layerMask);

            if (hasHit)
            {
                groundInfo.Distance = origin.y - hitInfo.point.y;
                if (groundInfo.Distance <= thresholdDistance)
                {
                    isGroundInRange = true;
                    groundInfo.Normal = hitInfo.normal;
                    groundInfo.Point = hitInfo.point;
                    groundInfo.Collider = hitInfo.collider;

                    if (useRealGroundNormal && thickness > 0f)
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
    }
}