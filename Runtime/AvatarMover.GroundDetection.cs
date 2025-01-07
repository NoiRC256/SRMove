using UnityEngine;

namespace NoiRC.SRMove
{
    public partial class AvatarMover : MonoBehaviour
    {
        /// <summary>
        /// Probe for ground downwards from origin.
        /// </summary>
        /// <param name="findRealNormal"></param>
        /// <param name="debug"></param>
        /// <returns></returns>
        public GroundInfo Probe(bool findRealNormal = false, bool debug = false)
        {
            Vector3 origin = GroundProbeOrigin;
            Vector3 up = _up;
            float distance = GroundProbeDistance;
            float thickness = _groundProbeThickness;
            GroundInfo groundInfo = GroundInfo.Empty;
            bool hit = false;
            RaycastHit hitInfo;
            if (thickness <= 0f) hit = Physics.Raycast(origin, -up, out hitInfo,
                    maxDistance: distance, layerMask: _groundLayerMask);
            else hit = Physics.SphereCast(origin, thickness / 2f, -up, out hitInfo,
                    maxDistance: distance, layerMask: _groundLayerMask);

            if (hit)
            {
                groundInfo.Distance = Vector3.Distance(hitInfo.point, origin);
                groundInfo.Point = hitInfo.point;
                groundInfo.Normal = hitInfo.normal;
                groundInfo.IsOnGround = (groundInfo.Distance <= GroundDistanceThreshold) && (groundInfo.Normal.y > 0);
                if (groundInfo.IsOnGround)
                {
                    groundInfo.Collider = hitInfo.collider;
                }

                // Ground normal from off-centre spherecast may not be real normal.
                // Fire another raycast towards ground point to find real normal.
                if (findRealNormal && groundInfo.IsOnGround && thickness > 0f)
                {
                    Vector3 tmpOrigin = hitInfo.point + 0.01f * -up;
                    if (hitInfo.collider.Raycast(new Ray(tmpOrigin, -up),
                        out RaycastHit realNormalHitInfo, maxDistance: 0.1f))
                    {
                        groundInfo.Normal = realNormalHitInfo.normal;
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

            return groundInfo;
        }

        /// <summary>
        /// Approximate the ground slope normal by sampling ground points ahead and behind.
        /// </summary>
        /// <param name="groundInfo"></param>
        /// <param name="slopePoint"></param>
        /// <param name="forward"></param>
        /// <param name="range"></param>
        /// <param name="iters"></param>
        /// <param name="debug"></param>
        /// <returns></returns>
        public Vector3 ApproximateSlope(in GroundInfo groundInfo, out Vector3 slopePoint,
            Vector3 forward, float range, int iters = 1, bool debug = false)
        {
            Vector3 origin = GroundProbeOrigin;
            Vector3 up = _up;
            float maxGroundDistance = GroundDistanceThreshold + 1f;
            float maxHeightDiff = _stepUpHeight;
            Vector3 slopeNormal = groundInfo.Normal;
            slopePoint = groundInfo.Point;
            float rangeStep = range / (float)iters;

            // Find front proxy ground point.
            Vector3 frontGroundPoint = groundInfo.Point;
            bool frontProxyHit = SampleFarthestGroundPoint(ref frontGroundPoint,
                maxGroundDistance, maxHeightDiff,
                forward, rangeStep, iters, debug);

            // Find back proxy ground point.
            Vector3 backGroundPoint = groundInfo.Point;
            bool backProxyHit = SampleFarthestGroundPoint(ref backGroundPoint,
                maxGroundDistance, maxHeightDiff,
                -forward, rangeStep, iters, debug);

            // Calculate slope normal from 2 proxy ground points.
            if (frontProxyHit || backProxyHit)
            {
                Vector3 slopeSegment = frontGroundPoint - backGroundPoint;
                slopeNormal = Vector3.Cross(slopeSegment, Vector3.Cross(up, slopeSegment)).normalized;
#if UNITY_EDITOR
                if (debug)
                {
                    Debug.DrawLine(frontGroundPoint, backGroundPoint, Color.yellow);
                }
#endif
                if (frontProxyHit && backProxyHit)
                {
                    Vector3 groundProbeSegment = (maxGroundDistance + 100f) * -up;
                    bool hasIntersection = GetIntersection(origin, groundProbeSegment, backGroundPoint, slopeSegment, out slopePoint);
                }
            }

            return slopeNormal;
        }

        public static bool GetIntersection(Vector3 p1, Vector3 v1, Vector3 p2, Vector3 v2, out Vector3 intersection)
        {
            intersection = Vector3.zero;

            Vector3 cross_v1v2 = Vector3.Cross(v1, v2);
            float denominator = cross_v1v2.sqrMagnitude;

            // If denominator is close to zero, lines are parallel or coincident
            if (denominator < Mathf.Epsilon)
            {
                return false;
            }

            Vector3 p2_p1 = p2 - p1;
            float t = Vector3.Dot(Vector3.Cross(p2_p1, v2), cross_v1v2) / denominator;

            intersection = p1 + t * v1;
            return true;
        }

        /// <summary>
        /// Finds the farthest ground point in the specified direction from origin.
        /// <para>For each iteration, moves a step towards the specified direction,
        /// creates a proxy origin, then probes downwards for ground from that proxy origin.</para>
        /// <para>If the probe is succesful, update the ground point and continue;
        /// Otherwise, directly return false.</para>
        /// </summary>
        /// <param name="groundPoint">Output ground point. Should be initialized with a fallback value.</param>
        /// <param name="groundProbeDistance">Distance to probe downward for ground.</param>
        /// <param name="maxHeightDiff"></param>
        /// <param name="direction">Direction to move towards for each iteration.</param>
        /// <param name="step">Distance to look ahead in each iteration.</param>
        /// <param name="iters">Number of iterations to run for.</param>
        /// <param name="debug"></param>
        /// <returns></returns>
        private bool SampleFarthestGroundPoint(ref Vector3 groundPoint,
            float groundProbeDistance, float maxHeightDiff,
            Vector3 direction, float step, int iters, bool debug = false)
        {
            Vector3 origin = GroundProbeOrigin;
            Vector3 up = _up;
            Vector3 prevGroundPoint = groundPoint;
            bool proxyHit = false;
            for (int i = 0; i < iters; i++)
            {
                Vector3 proxyOrigin = origin + (step * ((float)i + 1f) * direction);
                bool hit = Physics.Raycast(proxyOrigin, -up, out RaycastHit hitInfo,
                    maxDistance: groundProbeDistance, layerMask: _groundLayerMask);
#if UNITY_EDITOR
                if (debug)
                {
                    Vector3 endHit = proxyOrigin + hitInfo.distance * -up;
                    Vector3 endTotal = proxyOrigin + groundProbeDistance * -up;
                    Debug.DrawLine(proxyOrigin, endHit, Color.green);
                    Debug.DrawLine(endHit, endTotal, Color.grey);
                }
#endif
                if (hit)
                {
                    float heightDiff = Vector3.Distance(hitInfo.point, prevGroundPoint);
                    proxyHit = true;
                    if (heightDiff > maxHeightDiff)
                    {
                        return proxyHit;
                    }
                    groundPoint = hitInfo.point;
                    prevGroundPoint = hitInfo.point;
                }
                else return proxyHit;
            }
            return proxyHit;
        }
    }
}