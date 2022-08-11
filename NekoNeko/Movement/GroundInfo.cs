using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace NekoNeko.Movement
{
    public struct GroundInfo
    {
        public bool IsGrounded { get; }
        public float Distance { get; }
        public Vector3 Point { get; }
        public Vector3 Normal { get; }
        public Rigidbody Rigidbody { get; }

        public static GroundInfo Empty { get { return new GroundInfo(false, -1, Vector3.zero, Vector3.up, null); } }

        public GroundInfo(bool isGrounded, float distance, Vector3 point, Vector3 normal, Rigidbody rigidbody)
        {
            this.IsGrounded = isGrounded;
            this.Distance = distance;
            this.Point = point;
            this.Normal = normal;
            this.Rigidbody = rigidbody;
        }

        public GroundInfo(bool isGrounded, float distance, RaycastHit hit)
            : this(isGrounded, distance, hit.point, hit.normal, hit.rigidbody) { }
        public GroundInfo(GroundProbeInfo groundProbeInfo, Vector3 point, Vector3 normal, Rigidbody rigidbody)
            : this(groundProbeInfo.IsGrounded, groundProbeInfo.Distance, point, normal, rigidbody) { }
        public GroundInfo(GroundProbeInfo groundProbeInfo, RaycastHit hit)
            : this(groundProbeInfo.IsGrounded, groundProbeInfo.Distance, hit.point, hit.normal, hit.rigidbody) { }

    }
}