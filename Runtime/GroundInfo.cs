using UnityEngine;

namespace NekoLab.SRMove
{
    public struct GroundInfo
    {
        public static GroundInfo Empty = new GroundInfo(false, Vector3.up, Vector3.zero, 0f, null);
        public bool IsOnGround { get; set; }
        public Vector3 Normal { get; set; }
        public Vector3 Point { get; set; }
        public float Distance { get; set; }
        public Collider Collider { get; set; }


        public GroundInfo(bool isOnGround, Vector3 normal, Vector3 point, float distance, Collider collider)
        {
            IsOnGround = isOnGround;
            Normal = normal;
            Point = point;
            Distance = distance;
            Collider = collider;
        }
    }
}