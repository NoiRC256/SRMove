using UnityEngine;

namespace NoiRC.SRMove
{
    public struct GroundInfo
    {
        public static GroundInfo Empty => new GroundInfo(false, 0f, Vector3.zero, Vector3.up, null);
        public bool IsOnGround;
        public float Distance;
        public Vector3 Point;
        public Vector3 Normal;
        public Collider Collider;

        public GroundInfo(bool isOnGround, float distance, Vector3 point, Vector3 normal, Collider collider)
        {
            IsOnGround = isOnGround;
            Distance = distance;
            Point = point;
            Normal = normal;
            Collider = collider;
        }

        public void Copy(GroundInfo other)
        {
            IsOnGround = other.IsOnGround;
            Distance = other.Distance;
            Point = other.Point;
            Normal = other.Normal;
            Collider = other.Collider;
        }
    }
}