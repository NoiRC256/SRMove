using System;
using UnityEngine;

namespace NekoNeko
{
    public abstract class BaseCharacterMover : MonoBehaviour
    {
        public virtual bool IsOnGround { get; set; }
        public virtual bool IsOnFlatGround { get; set; }
        public virtual Vector3 GroundNormal { get; set; }
        public virtual float GroundHeight { get; set; }

        #region Events
        public event Action GainedGroundContact = delegate { };
        public event Action LostGroundContact = delegate { };
        #endregion

        public abstract void ActiveMove(float inputSpeed, Vector3 inputDirection);
        public abstract void DirectMove(Vector3 velocity, bool restrictToGround = false, bool ignoreConnectedGround = false);

        public abstract void AddImpulse(Impulse impulse);
        public abstract void RemoveImpulse(Impulse impulse);

        #region Event Methods
        protected void InvokeGainedGroundContact()
        {
            GainedGroundContact.Invoke();
        }

        protected void InvokeLostGroundContact()
        {
            LostGroundContact.Invoke();
        }
        #endregion
    }
}
