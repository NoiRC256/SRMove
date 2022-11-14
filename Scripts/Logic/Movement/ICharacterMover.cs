using System;
using UnityEngine;

namespace NekoNeko
{
    public interface ICharacterMover
    {
        bool IsOnGround { get; set; }
        bool IsOnFlatGround { get; set; }
        Vector3 GroundNormal { get; set; }
        float GroundHeight { get; set; }

        #region Events
        event Action GainedGroundContact;
        event Action LostGroundContact;
        #endregion

        void InputMove(float inputSpeed, Vector3 inputDirection);
        void DirectMove(Vector3 velocity, bool restrictToGround = false, bool ignoreConnectedGround = false);

        void AddImpulse(Impulse impulse);
        void RemoveImpulse(Impulse impulse);
    }
}
