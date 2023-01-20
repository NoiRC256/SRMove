using System;
using UnityEngine;

namespace NekoLib.Movement
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

        void InputMove(float inputSpeed, Vector3 inputDirection);
        void SetExtraVelocity(Vector3 vel);
        public void SetOverrideVelocity(Vector3 velocity,
            bool clearActiveVelocity = false, bool ignoreConnectedGround = false);
        void AddImpulse(Impulse impulse);
        void RemoveImpulse(Impulse impulse);
    }
}
