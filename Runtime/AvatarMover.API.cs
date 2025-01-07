using System;
using UnityEngine;

namespace NoiRC.SRMove
{
    public partial class AvatarMover : MonoBehaviour
    {
        public void Move(Vector3 velocity)
        {
            _velocityInput = velocity;
        }

        public void LeaveGround()
        {
            if (_isTouchingCeiling) return;
            _shouldLeaveGround = true;
        }

        public void EndLeaveGround()
        {
            _shouldLeaveGround = false;
            _isOnGroundChangedThisFrame = true;
            _velocityGravity = Vector3.zero;
        }

        [Obsolete]
        public void AddLeaveGroundForce(Vector3 force)
        {
            _velocityLeaveGround += force;
        }

        [Obsolete]
        public void SetConstForce(Vector3 force)
        {
            _velocityConstForce = force;
        }
    }
}