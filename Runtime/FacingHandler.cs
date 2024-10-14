using CC.SRMove.Util;
using System;
using UnityEngine;
using UnityEngine.UIElements.Experimental;

namespace CC.SRMove
{
    [Serializable]
    public class FacingHandler
    {
        private Transform _transform;
        private float _facing = 0f;
        private float _facingGoal = 0f;
        private float _facingDiff = 0f;
        private float _facingVel = 0f;
        public bool IsRotationStarted { get; private set; } = false;

        public FacingHandler(Transform transform)
        {
            _transform = transform;
        }

        #region API

        /// <summary>
        /// Apply the current calculated facing value to the transform.
        /// </summary>
        public void Apply()
        {
            Apply(_facing);
        }

        /// <summary>
        /// Immediately rotate to face the provided direction.
        /// <para>If interrupt is true, stops any ongoing rotations and updates the facing goal.</para>
        /// </summary>
        /// <param name="lookDirection"></param>
        /// <param name="interrupt"></param>
        public void Rotate(Vector3 lookDirection, bool interrupt = true)
        {
            float facing = DirectionToFacing(lookDirection);
            Apply(facing);
            _facing = facing;
            if (interrupt)
            {
                _facingGoal = facing;
                IsRotationStarted = false;
            }
        }

        /// <summary>
        /// Allow rotation over time.
        /// <para>You should call <see cref="UpdateRotation(float, float)"/> every frame to smoothly rotate towards the rotation goal.</para>
        /// </summary>
        public void StartRotation()
        {
            _facing = MathUtil.NormalizeAngle360(_transform.eulerAngles.y);
            IsRotationStarted = true;
        }

        /// <summary>
        /// Set the rotation goal to smoothly rotate towards.
        /// </summary>
        /// <param name="lookDirection"></param>
        public void UpdateRotationGoal(Vector3 lookDirection)
        {
            if (!IsRotationStarted) return;
            _facingGoal = DirectionToFacing(lookDirection);
        }

        /// <summary>
        /// Updates smooth rotation towards the current rotation goal.
        /// </summary>
        /// <param name="duration"></param>
        /// <param name="deltaTime"></param>
        public void UpdateRotation(float duration)
        {
            if (!IsRotationStarted) return;
            _facing = Mathf.SmoothDampAngle(_facing, _facingGoal, ref _facingVel, duration);
            _facingDiff = Mathf.DeltaAngle(_facingGoal, _facing);
            _facing = MathUtil.NormalizeAngle360(_facing);
            Apply(_facing);
            if (Mathf.Abs(_facing - _facingGoal) <= 0.01f)
            {
                CompleteRotation();
            }
        }

        public void StopRotation()
        {
            _facingGoal = _facing;
            IsRotationStarted = false;
        }

        public void CompleteRotation()
        {
            _facing = _facingGoal;
            IsRotationStarted = false;
            Apply();
        }

        #endregion

        private void Apply(float facing)
        {
            _transform.rotation = Quaternion.Euler(_transform.rotation.x, facing, _transform.rotation.z);
            _transform.localRotation = Quaternion.Euler(0f, _transform.localEulerAngles.y, 0f);
        }

        private float DirectionToFacing(Vector3 direction)
        {
            return Quaternion.LookRotation(direction, _transform.up).eulerAngles.y;
        }
    }
}