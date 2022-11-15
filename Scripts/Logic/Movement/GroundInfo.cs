using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.Universal.Internal;

namespace NekoNeko
{
    public struct GroundInfo
    {
        public static GroundInfo Empty = new GroundInfo(false, Vector3.up, Vector3.zero, 0f);
        public bool IsOnGround { get; set; }
        public Vector3 Normal { get; set; }
        public Vector3 Point { get; set; }
        public float Distance { get; set; }


        public GroundInfo(bool isOnGround, Vector3 normal, Vector3 point, float distance)
        {
            IsOnGround = isOnGround;
            Normal = normal;
            Point = point;
            Distance = distance;
        }
    }
}