using UnityEngine;

namespace NekoLib.SRMove
{
    public struct GroundProbeInfo
    {
        public static GroundProbeInfo Empty = new GroundProbeInfo(false, 0f, Vector3.up, Vector3.zero, null);
        public bool IsOnGround { get; set; }
        public float Distance { get; set; }
        public Vector3 Normal { get; set; }
        public Vector3 Point { get; set; }
        public Collider Collider { get; set; }


        public GroundProbeInfo(bool isOnGround, float distance, Vector3 normal, Vector3 point, Collider collider)
        {
            IsOnGround = isOnGround;
            Distance = distance;
            Normal = normal;
            Point = point;
            Collider = collider;
        }
    }
}