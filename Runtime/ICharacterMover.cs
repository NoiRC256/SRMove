using System;
using UnityEngine;

namespace NekoLab.SRMove
{
    public interface ICharacterMover
    {
        bool IsOnGround { get; }
        bool IsOnFlatGround { get; }
        Vector3 GroundNormal { get; }
        Vector3 GroundPoint { get; }
        float Speed { get; }
        Vector3 Direction { get; }

        #region Events

        event Action GainedGroundContact;
        event Action LostGroundContact;

        #endregion

        void Move(float inputSpeed, Vector3 inputDirection);
        void SetActiveVelocity(Vector3 velocity);
        void ClearActiveVelocity();
        void MoveDeltaPosition(Vector3 deltaPosition,
            bool alignToGround = true, bool restrictToGround = false);
    }
}
