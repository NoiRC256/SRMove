using System;
using UnityEngine;

namespace Nap.Movement
{
    public interface ICharacterMover
    {
        bool IsOnGround { get; set; }
        bool IsOnFlatGround { get; set; }
        Vector3 GroundNormal { get; set; }
        Vector3 GroundPoint { get; set; }

        #region Events
        event Action GainedGroundContact;
        event Action LostGroundContact;
        #endregion

        void SetInputVelocity(float inputSpeed, Vector3 inputDirection);
        void SetExtraVelocity(Vector3 vel);
        void DirectMove(Vector3 velocity, bool restrictToGround = false, bool ignoreConnectedGround = false);

        void AddImpulse(Impulse impulse);
        void RemoveImpulse(Impulse impulse);
    }
}
