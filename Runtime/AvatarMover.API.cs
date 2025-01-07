using System;
using UnityEngine;

namespace NoiRC.SRMove
{
    public partial class AvatarMover : MonoBehaviour
    {
        /// <summary>
        /// Set the intended movement velocity for one physics frame.
        /// </summary>
        /// <param name="velocity"></param>
        public void Move(Vector3 velocity)
        {
            _velocityInput = velocity;
        }

        /// <summary>
        /// Pause ground-related functionalities.
        /// </summary>
        public void LeaveGround()
        {
            if (_isTouchingCeiling) return;
            _shouldLeaveGround = true;
        }

        /// <summary>
        /// Resume ground-related functionalities.
        /// </summary>
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