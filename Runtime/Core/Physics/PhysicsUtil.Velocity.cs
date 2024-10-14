using UnityEngine;

namespace CC.SRMove.Util
{
    public static partial class PhysicsUtil
    {
        public static Vector3 UpdatePhysicalVelocity(Vector3 vel, Vector3 velGoal,
               float deltaTime,
               float accel = VelocityProfile.kMaxAccel,
               float decel = VelocityProfile.kMaxDecel, float decelBraking = VelocityProfile.kMaxDecel,
               float friction = VelocityProfile.kMaxFriction, float frictionBraking = VelocityProfile.kMaxFriction)
        {
            float speedSqr = vel.sqrMagnitude;
            float speedGoalSqr = velGoal.sqrMagnitude;
            float speedGoal = velGoal.magnitude;
            Vector3 dir = vel.normalized;
            Vector3 dirGoal = velGoal.normalized;

            if (speedSqr > speedGoalSqr * 1.01f)
            {
                if (speedGoalSqr > 0f && dirGoal != Vector3.zero)
                {
                    vel = ApplyDecel(vel, dir, speedGoal, decel, deltaTime);
                }
                else vel = ApplyBraking(vel, dir, frictionBraking, decelBraking, deltaTime);
            }
            else
            {
                float speed = vel.magnitude;
                vel = ApplyFriction(vel, speed, dirGoal, friction, deltaTime);
                ;
                if (speed < speedGoal)
                {
                    vel += deltaTime * accel * dirGoal;
                    vel = Vector3.ClampMagnitude(vel, speedGoal);
                }
            }
            return vel;
        }

        /// <summary>
        /// Decelerate velocity to zero.
        /// </summary>
        /// <param name="vel"></param>
        /// <param name="friction"></param>
        /// <param name="decel"></param>
        /// <param name="deltaTime"></param>
        /// <returns></returns>
        private static Vector3 ApplyBraking(Vector3 vel, Vector3 dir,
            float friction, float decel, float deltaTime)
        {
            bool isZeroFriction = friction == 0f;
            bool isZeroDecel = decel == 0f;
            if (isZeroFriction && isZeroDecel) return vel;

            // Calculate braking deceleration.
            Vector3 oldVel = vel;
            Vector3 decelVec = isZeroDecel ? Vector3.zero : -decel * dir;

            // Apply friction and deceleration.
            vel += (-friction * vel + decelVec) * deltaTime;

            // Stop before we start to go backwards.
            if (Vector3.Dot(vel, oldVel) <= 0f) return Vector3.zero;

            // Snap to zero.
            if (vel.sqrMagnitude <= 0.1f) return Vector3.zero;

            return vel;
        }

        private static Vector3 ApplyDecel(Vector3 vel, Vector3 dir,
            float speedGoal, float decel, float deltaTime)
        {
            bool isZeroDecel = decel == 0f;
            if (isZeroDecel) return vel;

            vel += deltaTime * -decel * dir;
            vel = Vector3.ClampMagnitude(vel, speedGoal);
            return vel;
        }

        private static Vector3 ApplyFriction(Vector3 vel, float speed,
            Vector3 dirGoal, float friction, float deltaTime)
        {
            vel -= (vel - dirGoal * speed) * Mathf.Min(friction * deltaTime, 1f);
            return vel;
        }
    }
}