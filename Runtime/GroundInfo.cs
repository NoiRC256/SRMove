using UnityEngine;

namespace NekoLib.SRMove
{
    public struct GroundInfo
    {
        public static GroundInfo Empty => new GroundInfo(false, 0f, Vector3.zero, Vector3.up);
        public bool IsOnGround;
        public float Distance;
        public Vector3 Point;
        public Vector3 Normal;

        public GroundInfo(bool isOnGround, float distance, Vector3 point, Vector3 normal)
        {
            IsOnGround = isOnGround;
            Distance = distance;
            Normal = normal;
            Point = point;
        }
    }
}