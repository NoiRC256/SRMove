using System;
using System.Collections.Generic;
using UnityEngine;

namespace NekoNeko
{
    public class CharacterMover : MonoBehaviour
    {
        public const float kMaxSpeedChange = 50f;

        public enum VelocityPhysicsMode
        {
            None,
            Simple,
            Advanced,
        }
        #region Exposed Fields
        [Header("Collider Dimensions")]
        [SerializeField][Min(0f)] private float _height = 2f;
        [SerializeField][Min(0f)] private float _thickness = 1f;
        [SerializeField] private Vector3 _colliderOffset = Vector3.zero;

        [Header("Velocity Physics")]
        [Tooltip("Defines how we handle acceleration, deceleration, friction for active velocity.\n" +
            "None -- Bypass velocity physics calculations.\n" + "Simple -- Use Speed Change.\n" + "Advanced -- Use Velocity Config.")]
        [SerializeField] private VelocityPhysicsMode _velocityMode = VelocityPhysicsMode.Simple;
        [SerializeField][Range(0f, kMaxSpeedChange)] private float _speedChange = 30f;
        [SerializeField] private VelocityConfig _velocityConfig = new VelocityConfig();

        [Header("Ground Detection")]
        [SerializeField][Min(0f)] private float _maxGroundRange = 5f;
        [SerializeField][Range(0f, 90f)] private float _groundAngleLimit = 60f;
        #endregion

        #region Properties
        public Rigidbody Rigidbody { get; private set; }
        public Collider Collider { get; private set; }
        public Vector3 ColliderCenter { get; private set; }

        /// <summary>
        /// Capsule collider height.
        /// </summary>
        public float Height { get => _height; set { _height = value; UpdateColliderHeight(); } }

        /// <summary>
        /// Capsule collider thickness is equal to 2 times its radius.
        /// </summary>
        public float Thickness { get => _thickness; set { _thickness = value; UpdateColliderRadius(); } }
        public Vector3 Velocity { get => Rigidbody.velocity; }
        public VelocityConfig VelocityConfig { get => _velocityConfig; set => _velocityConfig = value; }

        public float GroundProbeRange { get => _maxGroundRange; set => _maxGroundRange = value; }
        public float GroundAngleLimit {
            get => _groundAngleLimit;
            set {
                _groundAngleLimit = value;
                _minGroundAngleDot = Mathf.Cos(value * Mathf.Deg2Rad);
            }
        }
        public bool IsOnGround { get; set; }
        public Vector3 GroundNormal { get => _groundNormal; set => _groundNormal = value; }
        #endregion

        #region Fields

        // Convenience cache for collider.
        private CapsuleCollider _capsuleCollider;
        private float _capsuleHalfHeight;
        // Points with ground normal dot larger than this value is considered as ground.
        private float _minGroundAngleDot;

        private List<Collision> _collisions = new List<Collision>();
        private List<Impulse> _impulses = new List<Impulse>();

        private float _inputSpeed;
        private Vector3 _inputDirection;
        // Active velocity, driven by input and affected by velocity physics.
        private Vector3 _activeVel = Vector3.zero;
        // Previous nonzero active vel direction.
        private Vector3 _nonZeroActiveDirection = Vector3.zero;

        // Direct velocity, overrides active velocity and bypasses all velocity physics.
        private Vector3 _directVel = Vector3.zero;
        private bool _hasDirectVel = false;

        // Base velocity contribution from connected body.
        private Vector3 _connectedVel;

        private int _groundCollisionCount = 0;
        private Vector3 _groundNormal = Vector3.up;

        #endregion

        #region Events
        public event Action LostGroundContact;
        #endregion

        #region MonoBehaviour

        private void OnValidate()
        {
            AddComponents();
            UpdateColliderDimensions();
            _minGroundAngleDot = Mathf.Cos(_groundAngleLimit * Mathf.Deg2Rad);
        }

        private void Awake()
        {
            OnValidate();
        }

        private void FixedUpdate()
        {
            UpdateMovement();
            IsOnGround = false;
            _groundNormal = Vector3.up;
        }

        private void OnCollisionEnter(Collision collision)
        {
            _collisions.Add(collision);
            CheckGround(collision);
        }

        private void OnCollisionStay(Collision collision)
        {
            CheckGround(collision);

#if UNITY_EDITOR
            for (int i = 0; i < collision.contactCount; i++)
            {
                ContactPoint contact = collision.GetContact(i);
                Vector3 normal = contact.normal;
                Debug.DrawLine(contact.point, contact.point + contact.normal * 0.1f, Color.blue);
            }
#endif
        }

        private void OnCollisionExit(Collision collision)
        {
            _collisions.Remove(collision);
        }

        private void CheckGround(Collision collision)
        {
            bool isCollisionGround = EvaluateGroundCollision(collision, out Vector3 groundNormal);
            if (isCollisionGround)
            {
                IsOnGround = true;
                _groundNormal += groundNormal;
                _groundNormal.Normalize();
                _groundNormal = groundNormal;
            }
        }

        private bool EvaluateGroundCollision(Collision collision, out Vector3 groundNormal)
        {
            bool isCollisionGround = false;
            int groundContactCount = 0;
            groundNormal = Vector3.up;
            for (int i = 0; i < collision.contactCount; i++)
            {
                Vector3 normal = collision.GetContact(i).normal;
                bool isContactGround = normal.y > _minGroundAngleDot;
                isCollisionGround |= isContactGround;
                if (isCollisionGround)
                {
                    groundContactCount++;
                    groundNormal = normal;
                }
            }
            groundNormal.Normalize();
            return isCollisionGround;
        }

        #endregion

        #region Movement Processing Methods
        private void UpdateMovement()
        {
            float deltaTime = Time.deltaTime;

            // Refresh active velocity.
            // Add any velocity from input, apply velocity physics to existing active velocity.
            bool noActiveVel = (_inputSpeed == 0f || _inputDirection == Vector3.zero) && _activeVel == Vector3.zero;
            if (!noActiveVel)
            {
                switch (_velocityMode)
                {
                    case VelocityPhysicsMode.None:
                        _activeVel = _inputSpeed * _inputDirection;
                        break;
                    case VelocityPhysicsMode.Simple:
                        _activeVel = CalculatePhysicsVelocitySimple(_activeVel, _inputSpeed * _inputDirection, _speedChange);
                        break;
                    case VelocityPhysicsMode.Advanced:
                        _activeVel = CalculatePhysicsVelocity(_activeVel, _inputSpeed, _inputDirection, deltaTime: Time.deltaTime,
                            _velocityConfig.Accel, _velocityConfig.Decel,
                            _velocityConfig.Friction, _velocityConfig.BrakingFriction);
                        break;
                }
            }

            // Apply impulses.
            Vector3 impulseVel = Vector3.zero;
            for (int i = _impulses.Count - 1; i >= 0; i--)
            {
                Impulse impulse = _impulses[i];
                Vector3 thisImpulseVel = impulse.Evaluate(deltaTime);

                // If flagged as use max speed,
                // limit this impulse speed so the final result not exceed its max speed when combined with the active velocity.
                if (impulse.SpeedModeFlag == Impulse.SpeedMode.Max)
                {
                    thisImpulseVel = Vector3.ClampMagnitude(thisImpulseVel, impulse.MaxSpeed - _activeVel.magnitude);
                }

                // If flagged as align to ground.
                if (impulse.AlignToGroundFlag == true)
                {
                    thisImpulseVel = AlignVelocityToNormal(thisImpulseVel, _groundNormal);
                }

                impulseVel += thisImpulseVel;
                if (!impulse.IsActive) _impulses.RemoveAt(i);
            }


            Vector3 _adjustedActiveVel = AlignVelocityToNormal(_activeVel, _groundNormal);

#if UNITY_EDITOR
            // Ground normal.
            Debug.DrawLine(transform.position, transform.position + _groundNormal, Color.white);
            // Desired velocity line.
            Debug.DrawLine(transform.position, transform.position + _adjustedActiveVel, Color.yellow);
#endif


            Vector3 _finalVel = _hasDirectVel ? _directVel 
                : _adjustedActiveVel + _connectedVel + impulseVel;
            Move(_finalVel);

            // Clean up.
            if (_activeVel != Vector3.zero) _nonZeroActiveDirection = _activeVel.normalized;
            _inputSpeed = 0f;
            _inputDirection = Vector3.zero;
            _hasDirectVel = false;
        }
        #endregion

        #region Movement Methods
        // Internal method for moving.
        private void Move(Vector3 velocity)
        {
            Rigidbody.velocity = velocity;
        }

        /// <summary>
        /// Move by setting active movement velocity using the provided input speed and direction.
        /// Active movement velocity is subject to velocity physics calculations.
        /// </summary>
        /// <param name="inputSpeed"></param>
        /// <param name="inputDirection"></param>
        public void ActiveMove(float inputSpeed, Vector3 inputDirection)
        {
            _inputSpeed = inputSpeed;
            _inputDirection = inputDirection;
        }

        /// <summary>
        /// Move by setting direct velocity, bypassing active velocity and velocity physics calculations.
        /// </summary>
        /// <param name="velocity"></param>
        /// <param name="restrictToGround">If true, only move if the movement will not cause us to lose ground contact.</param>
        /// <param name="ignoreConnectedGround">If true, ignore base velocity from connected body.</param>
        public void DirectMove(Vector3 velocity, bool restrictToGround = false, bool ignoreConnectedGround = false)
        {
            if (ignoreConnectedGround)
            {
                _directVel = velocity;
            }
            else
            {
                _directVel = velocity + _connectedVel;
            }
            _hasDirectVel = true;
        }
        #endregion

        #region Impulse Methods
        public void AddImpulse(Impulse impulse)
        {
            // If impulse already exists, restart it. Otherwise add impulse.
            // We expect to deal with only 1~10 impulses max, so this should work fine. 
            if (_impulses.Contains(impulse))
            {
                impulse.Start();
            }
            else
            {
                _impulses.Add(impulse);
            }
        }

        public void RemoveImpulse(Impulse impulse)
        {
            _impulses.Remove(impulse);
        }
        #endregion

        #region Velocity Physics Methods
        private Vector3 CalculatePhysicsVelocitySimple(Vector3 currentVel, Vector3 desiredVel, float accel = kMaxSpeedChange)
        {
            Vector3 finalVel = currentVel;
            float speedChange = accel * Time.deltaTime;

            finalVel = Vector3.MoveTowards(finalVel, desiredVel, speedChange);


#if UNITY_EDITOR
            // Desired velocity line.
            Debug.DrawLine(transform.position, transform.position + desiredVel, Color.gray);
            // Final velocity line.
            Debug.DrawLine(transform.position, transform.position + finalVel, Color.black);
#endif

            return finalVel;
        }

        private Vector3 CalculatePhysicsVelocity(Vector3 currentVel, float desiredSpeed, Vector3 desiredDirection, float deltaTime,
            float accel = VelocityConfig.kMaxAccel, float decel = VelocityConfig.kMaxDecel,
            float friction = VelocityConfig.kMaxFriction, float brakingFriction = VelocityConfig.kMaxFriction)
        {
            float currentSpeed = currentVel.magnitude;
            Vector3 currentDirection = currentVel.normalized;
            Vector3 desiredVel = desiredSpeed * desiredDirection;

            bool isZeroAccel = accel == 0f || desiredDirection == Vector3.zero;
            bool isSpeedExceeded = currentSpeed > desiredSpeed;

            Vector3 accelVec = accel * desiredDirection;

            // Braking deceleration.
            if (isZeroAccel || isSpeedExceeded)
            {
                Vector3 oldVel = currentVel;
                currentVel = ApplyVelocityBraking(currentVel, brakingFriction, decel, deltaTime);

                //if(isSpeedExceeded && currentVel.sqrMagnitude < desiredSpeed * desiredSpeed 
                //    && Vector3.Dot(accelVec, oldVel) > 0f)
                //{
                //    currentVel = desiredSpeed * oldVel.normalized;
                //}
            }
            // Friction.
            else
            {
                currentVel -= Mathf.Min(friction * deltaTime, 1f) * (currentVel - currentSpeed * desiredDirection);
            }

            // Acceleration.
            if (!isZeroAccel)
            {
                currentVel += accelVec * deltaTime;
                currentVel = Vector3.ClampMagnitude(currentVel, desiredSpeed);
            }

#if UNITY_EDITOR
            // Desired velocity line.
            Debug.DrawLine(transform.position, transform.position + desiredVel, Color.gray);
            // Final velocity line.
            Debug.DrawLine(transform.position, transform.position + currentVel, Color.black);
#endif

            return currentVel;
        }

        /// <summary>
        /// Decelerate velocity to zero.
        /// </summary>
        /// <param name="currentVel"></param>
        /// <param name="friction"></param>
        /// <param name="decel"></param>
        /// <param name="deltaTime"></param>
        /// <returns></returns>
        private static Vector3 ApplyVelocityBraking(Vector3 currentVel, float friction, float decel, float deltaTime)
        {
            bool isZeroFriction = friction == 0f;
            bool isZeroDecel = decel == 0f;
            if (isZeroFriction && isZeroDecel) return currentVel;

            // Apply braking deceleration.
            Vector3 oldVel = currentVel;
            Vector3 revserseAccel = isZeroDecel ? Vector3.zero : -decel * currentVel.normalized;
            currentVel += (-friction * currentVel + revserseAccel) * deltaTime;

            // Stop before we start to go backwards.
            if (Vector3.Dot(currentVel, oldVel) <= 0f) return Vector3.zero;

            // Snap to zero.
            float sqrSpeed = currentVel.sqrMagnitude;
            if (sqrSpeed <= 0.001f || !isZeroDecel && sqrSpeed <= 0.01f) return Vector3.zero;

            return currentVel;
        }
        #endregion

        #region Velocity Manipulation Methods
        private Vector3 AlignVelocityToNormal(Vector3 velocity, Vector3 normal)
        {
            float speed = velocity.magnitude;
            Vector3 alignedDirection = Quaternion.FromToRotation(Vector3.up, normal) * velocity.normalized;
            return speed * alignedDirection.normalized;
        }
        #endregion

        #region Initialization Methods
        private void AddComponents()
        {
            TryGetComponent(out Rigidbody rb);
            Rigidbody = rb ? rb : gameObject.AddComponent<Rigidbody>();
            rb.useGravity = false;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.freezeRotation = true;

            TryGetComponent(out CapsuleCollider capsuleCollider);
            Collider = capsuleCollider ? capsuleCollider : gameObject.AddComponent<CapsuleCollider>();
            _capsuleCollider = Collider as CapsuleCollider;
        }
        #endregion

        #region Collider Dimensions Methods
        private void UpdateColliderDimensions()
        {
            UpdateColliderHeight();
            UpdateColliderRadius();
        }

        private void UpdateColliderHeight()
        {
            _capsuleCollider.height = _height;
            _capsuleHalfHeight = _capsuleCollider.height / 2f;
            Vector3 center = _colliderOffset + new Vector3(0f, _capsuleHalfHeight, 0f);
            _capsuleCollider.center = center;
            LimitRadius();

        }

        public void UpdateColliderRadius()
        {
            float radius = _thickness / 2f;
            _capsuleCollider.radius = radius;
            LimitRadius();
        }

        /// <summary>
        /// Restrict collider minimum thickness to collider height.
        /// </summary>
        private void LimitRadius()
        {
            if (_capsuleCollider.radius * 2f > _capsuleCollider.height) _capsuleCollider.radius = _capsuleCollider.height / 2f;
        }
        #endregion
    }
}