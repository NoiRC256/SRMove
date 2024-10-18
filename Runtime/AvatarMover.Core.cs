using UnityEngine;

namespace CC.SRMove
{
    public partial class AvatarMover : MonoBehaviour
    {
        private bool CheckDirectCollision(Collision collision, out GroundInfo groundInfo, out bool isTouchingCeiling, out bool isTouchingWall)
        {
            groundInfo = new GroundInfo();
            groundInfo.IsOnGround = false;
            isTouchingCeiling = false;
            isTouchingWall = false;
            if (!LayerMaskContains(_groundLayerMask, collision.gameObject.layer))
            {
                return false;
            }
            //if (collision.contactCount == 0) continue;
            ContactPoint contact = collision.GetContact(0);
            if (contact.normal.y > 0.01f)
            {
                groundInfo.Distance = GroundDistanceDesired;
                groundInfo.Point = contact.point;
                groundInfo.Normal = contact.normal;
                groundInfo.IsOnGround = true;
                groundInfo.Collider = collision.collider;

            }
            else if (contact.normal.y < -0.25f)
            {
                isTouchingCeiling = true;
            }
            else
            {
                isTouchingWall = true;
            }
            return true;
        }

        private void UpdateCollisionCheck()
        {
            GroundInfo groundInfoNew = Probe(findRealNormal: _groundProbeFindRealNormal, debug: _debugGroundDetection);
            if (!groundInfoNew.IsOnGround && _collisionGroundInfo.IsOnGround)
            {
                //groundInfoNew.Copy(_collisionGroundInfo);
            }

            // If ground distance changed drastically, stop step smoothing for a while.
            if (Mathf.Abs(groundInfoNew.Distance - GroundInfo.Distance) > 0.1f * (_stepUpHeight + _stepDownHeight))
            {
                _stepSmoothDelayCounter = _stepSmoothDelay;
            }

            if (IsLeavingGround)
            {
                if (_isTouchingCeiling) EndLeaveGround();
                else groundInfoNew.IsOnGround = false;
            }

            GroundInfo = groundInfoNew;
            IsTouchingCeiling = _collisionIsTouchingCeiling;
        }

        private void UpdateMovement(float deltaTime)
        {
            if (_stepSmoothDelayCounter > 0f) _stepSmoothDelayCounter -= deltaTime;
            if (_velocityInput.magnitude > 0f) _lastNonZeroDirection = _velocityInput.normalized;

            if (IsOnGround)
            {
                _velocityGroundRb = _groundRb == null ? Vector3.zero : _groundRb.linearVelocity;

                // Approximate the slope to move along.
                if (_velocityInput != Vector3.zero)
                {
                    _slopeNormal = ApproximateSlope(in _groundInfo, out _slopePoint,
                        forward: _lastNonZeroDirection, range: _slopeApproxRange, iters: 5,
                        debug: _debugSlopeApproximation);
                }

                // Update hover velocity.
                _velocityHover = UpdateStepHoverVelocity(GroundInfo.Distance, Time.deltaTime);
                _velocityGravity = Vector3.zero;
            }
            else
            {
                _slopeNormal = _up;
                _velocityGravity += _gravityAccel * Time.deltaTime;
                _velocityGravity = Vector3.ClampMagnitude(_velocityGravity, _gravitySpeedMax);
            }

            // Assemble the velocity to apply.
            Vector3 velocityGravity = IsLeavingGround ? Vector3.zero : _velocityGravity;
            Vector3 velocityMove = _alignVelocityToSlope ? AlignVelocityToPlane(_velocityInput, _slopeNormal) : _velocityInput;
            Vector3 velocityToApply = _velocityGroundRb + velocityGravity +
                _velocityHover + velocityMove +
                _velocityLeaveGround + _velocityConstForce;
            _velocityLeaveGround = Vector3.MoveTowards(_velocityLeaveGround, Vector3.zero, _rigidbody.mass * deltaTime);

            ApplyVelocity(velocityToApply);
        }

        /// <summary>
        /// Calculate the adjustment hover velocity required to maintain desired ground distance (step height).
        /// </summary>
        ///   
        private Vector3 UpdateStepHoverVelocity(float groundDistance, float deltaTime,
            float groundDistanceOffset = 0f, bool smoothing = true)
        {
            Vector3 vel = Vector3.zero;
            float hoverHeightPatch = GroundDistanceDesired + groundDistanceOffset - groundDistance;
            _stepHeightHoverPatch = hoverHeightPatch;
            if (_isOnGroundChangedThisFrame || !smoothing)
            {
                vel = _up * (hoverHeightPatch / deltaTime);
            }
            else
            {
                if (_stepSmoothDelayCounter >= 0f)
                {
                    return Vector3.zero;
                }
                float stepSmooth = hoverHeightPatch > 0f ? _stepUpSmooth : _stepDownSmooth;
                if (_velocityInput != Vector3.zero)
                {
                    stepSmooth = stepSmooth * _stepSmoothMovingMultipler;
                }
                float directionSign = Mathf.Sign(hoverHeightPatch);
                float hoverHeightDelta = Mathf.Abs(hoverHeightPatch) / (stepSmooth * deltaTime);
                vel = _up * directionSign * hoverHeightDelta;
            }
            return vel;
        }

        private void ApplyVelocity(Vector3 velocity)
        {
            _rigidbody.linearVelocity = velocity;
        }

        private void UpdateCleanup()
        {
            _velocityHover = Vector3.zero;
            _velocityInput = Vector3.zero;
            _isOnGroundChangedThisFrame = false;
            _hasDirectCollision = false;
            _collisionGroundInfo.IsOnGround = false;
            _collisionIsTouchingCeiling = false;
            _collisionIsTouchingWall = false;
        }

        #region Events

        private void HandleIsOnGroundChange(bool isOnGround)
        {
            _isOnGroundChangedThisFrame = true;
            OnIsOnGroundChanged.Invoke(isOnGround);
            _velocityGravity = Vector3.zero;
        }

        private void HandleIsTouchingCeilingChange(bool isTouchingCeiling)
        {
            OnIsTouchingCeilingChanged.Invoke(isTouchingCeiling);
        }

        private void HandleUpDirectionChange(Vector3 up)
        {
            OnUpDirectionChanged.Invoke(up);
        }

        private void HandleGroundParentChange(ParentableGround ground)
        {
            if (ground == null) OnGroundParentChanged.Invoke(false);
            else OnGroundParentChanged.Invoke(true);
        }

        #endregion
    }
}