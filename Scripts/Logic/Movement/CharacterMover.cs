using Autodesk.Fbx;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using Unity.PlasticSCM.Editor.WebApi;
using Unity.VisualScripting.FullSerializer;
using UnityEditor.Build.Pipeline.Tasks;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Assertions.Must;

namespace NekoNeko
{
    public class CharacterMover : MonoBehaviour
    {
        public const float kMaxSpeedChange = 50f;

        public enum VelocityMode
        {
            Simple,
            Advanced,
        }
        #region Exposed Fields
        [Header("Collider Dimensions")]
        [SerializeField][Min(0f)] private float _height = 1f;
        [SerializeField][Min(0f)] private float _thickness = 0.6f;
        [SerializeField] private Vector3 _colliderOffset = Vector3.zero;

        [Header("Velocity Physics")]
        [SerializeField] private bool _useVelocityPhysics = true;
        [SerializeField] private VelocityMode _velocityMode = VelocityMode.Simple;
        [SerializeField][Range(0f, kMaxSpeedChange)] private float _speedChange = 45f;
        [SerializeField] private VelocityConfig _velocityConfig = new VelocityConfig();

        [Header("Ground Detection")]
        [SerializeField][Min(0f)] private float _maxGroundRange = 5f;
        [SerializeField][Range(0f, 90f)] private float _maxGroundAngle = 70f;
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

        public float InputSpeed { get; set; }
        public Vector3 InputDirection { get; set; }

        public Vector3 Velocity { get => Rigidbody.velocity; }
        public VelocityConfig VelocityConfig { get => _velocityConfig; set => _velocityConfig = value; }
        #endregion

        #region Fields
        // Convenience cache for collider.
        private CapsuleCollider _capsuleCollider;
        private float _capsuleHalfHeight;

        // Current input velocity.
        private Vector3 _inputVel;
        // Active velocity is driven by input and affected by velocity physics.
        private Vector3 _activeVel = Vector3.zero;
        // Previous nonzero active vel direction.
        private Vector3 _nonZeroActiveDirection = Vector3.zero;

        // Passive velocity.
        private Vector3 _passiveVel = Vector3.zero;
        // Current velocity contribution from connected body.
        private Vector3 _connectedVel;
        #endregion

        #region MonoBehaviour

        private void OnValidate()
        {
            AddComponents();
            UpdateColliderDimensions();
        }

        private void Awake()
        {
            UpdateColliderDimensions();
        }

        private void FixedUpdate()
        {
            UpdateMovement();
        }

        #endregion

        private void UpdateMovement()
        {
            if (!_useVelocityPhysics)
            {
                _activeVel = InputSpeed * InputDirection;
            }
            else
            {
                switch (_velocityMode)
                {
                    case VelocityMode.Simple:
                        _activeVel = CalculatePhysicsVelocitySimple(_activeVel, InputSpeed * InputDirection, _speedChange);
                        break;
                    case VelocityMode.Advanced:
                        _activeVel = CalculatePhysicsVelocity(_activeVel, InputSpeed, InputDirection, deltaTime: Time.deltaTime,
                            _velocityConfig.Accel, _velocityConfig.Decel,
                            _velocityConfig.Friction, _velocityConfig.BrakingFriction);
                        break;
                }
            }

            Vector3 _finalVel = _activeVel + _passiveVel;

            Move(_finalVel);

            // Clean up.
            InputSpeed = 0f;
            InputDirection = Vector3.zero;
            if (_activeVel != Vector3.zero) _nonZeroActiveDirection = _activeVel.normalized;
        }

        public void Move(Vector3 velocity)
        {
            Rigidbody.velocity = velocity;
        }

        #region Velocity Physics Methods
        private Vector3 CalculatePhysicsVelocitySimple(Vector3 currentVel, Vector3 desiredVel, float accel = kMaxSpeedChange)
        {
            Vector3 finalVel = currentVel;
            float speedChange = accel * Time.deltaTime;

            finalVel = Vector3.MoveTowards(finalVel, desiredVel, speedChange);


#if UNITY_EDITOR
            // Desired velocity line.
            Debug.DrawLine(transform.position, transform.position + desiredVel, Color.yellow);
            // Final velocity line.
            Debug.DrawLine(transform.position, transform.position + finalVel, Color.green);
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
            Debug.DrawLine(transform.position, transform.position + desiredVel, Color.yellow);
            // Final velocity line.
            Debug.DrawLine(transform.position, transform.position + currentVel, Color.green);
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