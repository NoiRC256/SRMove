using UnityEngine;

namespace NekoLab.SRMove.NekoPhysics
{
    public static partial class PhysicsUtil
    {
        public static Vector3 CalculatePhysicalVelocity(Vector3 currentVelocity, Vector3 targetVelocity,
               float deltaTime,
               float accel = VelocityProfile.kMaxAccel,
               float decel = VelocityProfile.kMaxDecel, float brakingDecel = VelocityProfile.kMaxDecel,
               float friction = VelocityProfile.kMaxFriction, float brakingFriction = VelocityProfile.kMaxFriction)
        {
            float currentSpeedSqr = currentVelocity.sqrMagnitude;
            float targetSpeedSqr = targetVelocity.sqrMagnitude;
            float targetSpeed = targetVelocity.magnitude;
            Vector3 currentDirection = currentVelocity.normalized;
            Vector3 targetDirection = targetVelocity.normalized;

            if (currentSpeedSqr > targetSpeedSqr * 1.01f)
            {
                if (targetSpeedSqr > 0f && targetDirection != Vector3.zero)
                {
                    currentVelocity = ApplyDeceleration(currentVelocity, currentDirection, targetSpeed, decel, deltaTime);
                }
                else currentVelocity = ApplyVelocityBraking(currentVelocity, currentDirection, brakingFriction, brakingDecel, deltaTime);
            }
            else
            {
                float currentSpeed = currentVelocity.magnitude;
                currentVelocity = ApplyFriction(currentVelocity, currentSpeed, targetDirection, friction, deltaTime);
                ;
                if (currentSpeed < targetSpeed)
                {
                    currentVelocity += deltaTime * accel * targetDirection;
                    currentVelocity = Vector3.ClampMagnitude(currentVelocity, targetSpeed);
                }
            }
            return currentVelocity;
        }

        /// <summary>
        /// Decelerate velocity to zero.
        /// </summary>
        /// <param name="currentVelocity"></param>
        /// <param name="friction"></param>
        /// <param name="decel"></param>
        /// <param name="deltaTime"></param>
        /// <returns></returns>
        private static Vector3 ApplyVelocityBraking(Vector3 currentVelocity, Vector3 currentDirection,
            float friction, float decel, float deltaTime)
        {
            bool isZeroFriction = friction == 0f;
            bool isZeroDecel = decel == 0f;
            if (isZeroFriction && isZeroDecel) return currentVelocity;

            // Calculate braking deceleration.
            Vector3 oldVel = currentVelocity;
            Vector3 decelVec = isZeroDecel ? Vector3.zero : -decel * currentDirection;

            // Apply friction and deceleration.
            currentVelocity += (-friction * currentVelocity + decelVec) * deltaTime;

            // Stop before we start to go backwards.
            if (Vector3.Dot(currentVelocity, oldVel) <= 0f) return Vector3.zero;

            // Snap to zero.
            if (currentVelocity.sqrMagnitude <= 0.1f) return Vector3.zero;

            return currentVelocity;
        }

        private static Vector3 ApplyDeceleration(Vector3 currentVelocity, Vector3 currentDirection,
            float targetSpeed, float decel, float deltaTime)
        {
            bool isZeroDecel = decel == 0f;
            if (isZeroDecel) return currentVelocity;

            currentVelocity += deltaTime * -decel * currentDirection;
            currentVelocity = Vector3.ClampMagnitude(currentVelocity, targetSpeed);
            return currentVelocity;
        }

        private static Vector3 ApplyFriction(Vector3 currentVelocity, float currentSpeed,
            Vector3 desiredDirection, float friction, float deltaTime)
        {
            currentVelocity -= (currentVelocity - desiredDirection * currentSpeed) * Mathf.Min(friction * deltaTime, 1f);
            return currentVelocity;
        }
    }
}