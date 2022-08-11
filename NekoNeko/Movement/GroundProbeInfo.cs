using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace NekoNeko.Movement
{
    public struct GroundProbeInfo
    {
        public bool IsGrounded { get; }
        public float Distance { get; }

        public static GroundProbeInfo Empty { get { return new GroundProbeInfo(false, -1); } }

        public GroundProbeInfo(bool isGrounded, float distance)
        {
            this.IsGrounded = isGrounded;
            this.Distance = distance;
        }
    }
}