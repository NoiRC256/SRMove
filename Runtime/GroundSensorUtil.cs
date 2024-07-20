using UnityEngine;

namespace NekoLib.SRMove
{
    public static class GroundSensorUtil
    {
        /// <summary>
        /// Probe for ground downwards from origin.
        /// </summary>
        /// <param name="info"></param>
        /// <param name="origin"></param>
        /// <param name="distance"></param>
        /// <param name="thickness"></param>
        /// <param name="layerMask"></param>
        /// <param name="groundDistanceThreshold"></param>
        /// <param name="findRealNormal"></param>
        /// <returns></returns>
        public static bool Probe(out GroundProbeInfo info,
            Vector3 origin, float distance, float thickness, LayerMask layerMask,
            float groundDistanceThreshold, float minGroundAngleDot,
            bool findRealNormal = false, bool debug = false)
        {
            info = GroundProbeInfo.Empty;
            bool hit = false;
            RaycastHit hitInfo;
            if (thickness <= 0f) hit = Physics.Raycast(origin, Vector3.down, out hitInfo,
                    maxDistance: distance, layerMask: layerMask);
            else hit = Physics.SphereCast(origin, thickness / 2f, Vector3.down, out hitInfo,
                    maxDistance: distance, layerMask: layerMask);

            if (hit)
            {
                info.Distance = origin.y - hitInfo.point.y;
                info.Normal = hitInfo.normal;
                info.Point = hitInfo.point;
                info.Collider = hitInfo.collider;
                info.IsOnGround = (info.Distance <= groundDistanceThreshold) && (info.Normal.y >= minGroundAngleDot);

                // Ground normal from off-centre spherecast may not be real normal.
                // Fire another raycast towards ground point to find real normal.
                if (findRealNormal && info.IsOnGround && thickness > 0f)
                {
                    Vector3 tmpOrigin = hitInfo.point;
                    tmpOrigin.y += 0.01f;
                    if (hitInfo.collider.Raycast(new Ray(tmpOrigin, Vector3.down),
                        out RaycastHit realNormalHitInfo, maxDistance: 0.1f))
                    {
                        info.Normal = realNormalHitInfo.normal;
                    }
                }
            }

#if UNITY_EDITOR
            if (debug)
            {
                Vector3 end = origin + new Vector3(0f, -distance, 0f);
                Debug.DrawLine(origin, end, Color.grey);
            }
#endif

            return info.IsOnGround;
        }

        /// <summary>
        /// Approximate the ground slope normal by sampling ground points ahead and behind.
        /// </summary>
        /// <param name="groundProbeInfo"></param>
        /// <param name="origin"></param>
        /// <param name="groundProbeDistance"></param>
        /// <param name="layerMask"></param>
        /// <param name="forward"></param>
        /// <param name="range"></param>
        /// <param name="iters"></param>
        /// <returns></returns>
        public static Vector3 ApproximateSlope(in GroundProbeInfo groundProbeInfo, Vector3 origin,
            float groundProbeDistance, LayerMask layerMask,
            Vector3 forward, float range, int iters = 1, bool debug = false)
        {
            Vector3 slopeNormal = groundProbeInfo.Normal;
            float rangeStep = range / (float)iters;

            // Find front proxy ground point.
            Vector3 frontGroundPoint = groundProbeInfo.Point;
            bool frontProxyHit = SampleGroundPoint(ref frontGroundPoint,
                origin, groundProbeDistance, layerMask, forward, rangeStep, iters, debug);

            // Find back proxy ground point.
            Vector3 backGroundPoint = groundProbeInfo.Point;
            bool backProxyHit = SampleGroundPoint(ref backGroundPoint,
                origin, groundProbeDistance, layerMask, -forward, rangeStep, iters, debug);

            // Calculate slope normal from 2 proxy ground points.
            if (frontProxyHit || backProxyHit)
            {
                Vector3 slopeSegment = frontGroundPoint - backGroundPoint;
                slopeNormal = Vector3.Cross(slopeSegment, Vector3.Cross(Vector3.up, slopeSegment)).normalized;
#if UNITY_EDITOR
                if (debug)
                {
                    Debug.DrawLine(frontGroundPoint, backGroundPoint, Color.yellow);
                }
#endif
            }

            return slopeNormal;
        }

        /// <summary>
        /// Finds the farthest ground point in the specified direction from origin.
        /// <para>For each iteration, moves a step towards the specified direction,
        /// creates a proxy origin, then probes downwards for ground from that proxy origin.</para>
        /// <para>If the probe is succesful, update the ground point and continue;
        /// Otherwise, directly return false.</para>
        /// </summary>
        /// <param name="groundPoint">Output ground point. Should be initialized with a fallback value.</param>
        /// <param name="origin"></param>
        /// <param name="groundProbeDistance">Distance to probe downward for ground.</param>
        /// <param name="layerMask">Ground layer mask.</param>
        /// <param name="direction">Direction to move towards for each iteration.</param>
        /// <param name="step">Distance to look ahead in each iteration.</param>
        /// <param name="iters">Number of iterations to run for.</param>
        /// <returns></returns>
        private static bool SampleGroundPoint(ref Vector3 groundPoint,
            Vector3 origin, float groundProbeDistance, LayerMask layerMask,
            Vector3 direction, float step, int iters, bool debug = false)
        {
            for (int i = 0; i < iters; i++)
            {
                Vector3 proxyOrigin = origin + (step * ((float)i + 1f) * direction);
                bool hit = Physics.Raycast(proxyOrigin, Vector3.down, out RaycastHit hitInfo,
                    maxDistance: groundProbeDistance, layerMask: layerMask);
#if UNITY_EDITOR
                if (debug)
                {
                    Vector3 endHit = proxyOrigin + new Vector3(0f, -hitInfo.distance, 0f);
                    Vector3 endTotal = proxyOrigin + new Vector3(0f, -groundProbeDistance, 0f);
                    Debug.DrawLine(proxyOrigin, endHit, Color.green);
                    Debug.DrawLine(endHit, endTotal, Color.grey);
                }
#endif
                if (hit)
                {
                    groundPoint = hitInfo.point;
                }
                else return false;
            }
            return true;
        }
    }
}