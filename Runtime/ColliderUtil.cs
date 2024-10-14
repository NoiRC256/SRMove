using UnityEngine;

namespace CC.SRMove
{
    public class ColliderUtil
    {
        public static void SetHeight(CapsuleCollider collider, float height, float stepHeight, Vector3 offset = default)
        {
            if (stepHeight > height) stepHeight = height;
            Vector3 center = offset + new Vector3(0f, height / 2f, 0f);
            center.y += stepHeight / 2f;
            collider.center = center;
            collider.height = height - stepHeight;
            LimitRadius(collider);
        }

        public static void SetThickness(CapsuleCollider collider, float thickness)
        {
            float radius = thickness / 2f;
            collider.radius = radius;
            LimitRadius(collider);
        }

        /// <summary>
        /// Restrict collider max thickness to collider height.
        /// </summary>
        private static void LimitRadius(CapsuleCollider collider)
        {
            if (collider.radius * 2f > collider.height) collider.radius = collider.height / 2f;
        }
    }
}