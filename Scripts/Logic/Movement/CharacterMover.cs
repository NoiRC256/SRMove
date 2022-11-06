using System;
using System.Collections.Generic;
using UnityEngine;

namespace NekoNeko
{
    public class CharacterMover : BaseCharacterMover
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
        [SerializeField] private LayerMask _groundLayer;
        [Tooltip("Snap to ground when grounded. Useful for cases like moving down stairs.")]
        [SerializeField] private bool _shouldSnapToGround = true;
        [Tooltip("Surfaces with normal below this angle is considered as ground.")]
        [SerializeField][Range(0f, 90f)] private float _groundAngleLimit = 90f;
        [Tooltip("Surfaces with normal below this angle is considered as flat ground.")]
        [SerializeField][Range(0f, 90f)] private float _flatGroundAngleLimit = 60f;
        [SerializeField][Min(0f)] private float _groundProbeRange = 10f;
        [SerializeField][Min(0f)] private float _groundThresholdRange = 0f;
        [SerializeField][Min(0f)] private float _stepHeight = 0.4f;
        [SerializeField][Min(0f)] private float _stepSearchOvershoot = 0.01f;
        #endregion

        #region Properties
        public Rigidbody Rigidbody { get; private set; }
        public Collider Collider { get; private set; }
        public Vector3 ColliderCenter { get => _capsuleCollider.bounds.center; }

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

        public bool ShouldSnapToGround { get => _shouldSnapToGround; set => _shouldSnapToGround = value; }
        public float GroundProbeRange { get => _groundProbeRange; set => _groundProbeRange = value; }
        public float GroundAngleLimit {
            get => _groundAngleLimit;
            set {
                _groundAngleLimit = value;
                _minGroundAngleDot = Mathf.Cos(value * Mathf.Deg2Rad);
            }
        }
        public float FlatGroundAngleLimit {
            get => _flatGroundAngleLimit;
            set {
                _flatGroundAngleLimit = value;
                _minFlatGroundAngleDot = Mathf.Cos(value * Mathf.Deg2Rad);
            }
        }
        #endregion

        #region Fields

        // Convenience cache for collider.
        private CapsuleCollider _capsuleCollider;
        private float _capsuleHalfHeight;
        // Points with ground normal dot larger than this value is considered as ground.
        private float _minGroundAngleDot;
        // Points with ground normal dot larger than this value is considered as flat ground.
        private float _minFlatGroundAngleDot;

        private List<Collision> _collisions = new List<Collision>();
        private List<Impulse> _impulses = new List<Impulse>();

        // Input velocity that drives active velocity.
        private float _inputSpeed;
        private Vector3 _inputDirection;
        // Active velocity, driven by input and affected by velocity physics.
        private Vector3 _activeVel = Vector3.zero;
        // Previous nonzero active vel direction.
        private Vector3 _nonZeroActiveDirection = Vector3.zero;

        // Passive velocity, overrides active velocity and bypasses all velocity physics.
        private Vector3 _directVel = Vector3.zero;
        private bool _hasDirectVel = false;

        // Base velocity contribution from connected body.
        private Vector3 _connectedVel;

        // Last frame rigidbody velocity.
        private Vector3 _lastVel;

        private int _groundCollisionCount = 0;
        private Vector3 _groundNormal = Vector3.up;
        private float _groundHeight = 0f;

        // Ground contact state.
        private bool _wasOnGround = false;

        #endregion

        #region MonoBehaviour

        private void OnValidate()
        {
            AddComponents();
            UpdateColliderDimensions();
            _minFlatGroundAngleDot = Mathf.Cos(_flatGroundAngleLimit * Mathf.Deg2Rad);
        }

        private void Awake()
        {
            OnValidate();
        }

        private void FixedUpdate()
        {
#if UNITY_EDITOR
            DrawContactNormals();
#endif
            // Init frame.
            IsOnGround = false;
            _groundNormal = Vector3.up;
            _groundHeight = 0f;

            // Update ground.
            IsOnGround = FindGround(out _groundNormal, out _groundHeight, _collisions);
            if (!_wasOnGround && IsOnGround) InvokeGainedGroundContact();
            else if (_wasOnGround && !IsOnGround) InvokeLostGroundContact();
            _wasOnGround = IsOnGround;

            // Update movement.
            UpdateMovement();

            // Clean up frame.
            _collisions.Clear();
            _lastVel = Rigidbody.velocity;
        }

        private void OnCollisionEnter(Collision collision)
        {
            _collisions.Add(collision);
        }

        private void OnCollisionStay(Collision collision)
        {
            _collisions.Add(collision);
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
            Debug.DrawLine(transform.position, transform.position + _groundNormal, Color.blue);
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
        public override void ActiveMove(float inputSpeed, Vector3 inputDirection)
        {
            _inputSpeed = inputSpeed;
            _inputDirection = inputDirection;
        }

        /// <summary>
        /// Move by setting direct velocity, bypassing active velocity and velocity physics calculations.
        /// <para>Typically used to apply animation root motion.</para>
        /// </summary>
        /// <param name="velocity"></param>
        /// <param name="restrictToGround">If true, only move if the movement will not cause us to lose ground contact.</param>
        /// <param name="ignoreConnectedGround">If true, ignore base velocity from connected body.</param>
        public override void DirectMove(Vector3 velocity, bool restrictToGround = false, bool ignoreConnectedGround = false)
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

        #region Impulse Methods
        public override void AddImpulse(Impulse impulse)
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

        public override void RemoveImpulse(Impulse impulse)
        {
            _impulses.Remove(impulse);
        }
        #endregion

        #region Ground Detection Methods
        /// <summary>
        /// Find ground by iterating through every collision contact point and checking the contact normal.
        /// Outputs average ground normal.
        /// </summary>
        /// <param name="groundNormal"></param>
        /// <param name="collisions"></param>
        /// <returns></returns>
        private bool FindGround(out Vector3 groundNormal, out float groundHeight, List<Collision> collisions)
        {
            bool found = false;
            int groundCollisionCount = 0;
            Vector3 accGroundNormal = Vector3.zero;
            groundHeight = 0f;
            for (int i = 0; i < collisions.Count; i++)
            {
                Collision collision = collisions[i];

                // Calculate average ground normal and height for contacts in this collision.
                bool isGroundCollision = false;
                int groundContactCount = 0;
                Vector3 accGroundCollisionNormal = Vector3.zero; // Average ground contact normal.
                for (int j = 0; j < collision.contactCount; j++)
                {
                    ContactPoint contact = collision.GetContact(j);
                    if (contact.normal.y > _minGroundAngleDot + 0.001f)
                    {
                        found = true;
                        isGroundCollision = true;
                        accGroundCollisionNormal = contact.normal;
                        groundContactCount++;
                        // Update average ground height.
                        groundHeight = groundHeight * (groundContactCount - 1) / groundContactCount
                            + (contact.point.y / groundContactCount);
                    }
                }
                // This is a ground collision if it contains ground contacts.
                if (isGroundCollision)
                {
                    accGroundNormal += (accGroundCollisionNormal / (float)groundContactCount);
                    groundCollisionCount++;
                }
            }
            // Average ground normal from all ground collisions.
            groundNormal = found ? accGroundNormal / (float)groundCollisionCount : Vector3.up;

            // Ground probing and snapping.
            Vector3 probedGroundNormal = Vector3.zero;
            float probedGroundHeight = 0f;
            bool foundProbedGround = false;
            foundProbedGround = ProbeGround(out probedGroundNormal, out probedGroundHeight, _groundProbeRange);

            return found;
        }

        /// <summary>
        /// Probe ground from origin downwards for a specified range.
        /// </summary>
        /// <param name="groundNormal"></param>
        /// <param name="groundHeight"></param>
        /// <param name="origin"></param>
        /// <param name="range"></param>
        /// <param name="groundThresholdRange"></param>
        /// <returns></returns>
        private bool ProbeGround(out Vector3 groundNormal, out float groundHeight, float range)
        {
            bool isGroundInRange = false;
            groundNormal = Vector3.zero;
            groundHeight = 0f;

            RaycastHit hitInfo;
            Vector3 origin = ColliderCenter;
            bool hasHit = Physics.Raycast(new Ray(origin, Vector3.down), out hitInfo, maxDistance: range);
            if (hasHit)
            {
                float groundRange = Mathf.Max(_stepHeight, _groundThresholdRange);
                if (origin.y - hitInfo.point.y <= groundRange)
                {
                    isGroundInRange = true;
                    groundNormal = hitInfo.normal;
                    groundHeight = hitInfo.point.y;
#if UNITY_EDITOR
                    Debug.DrawLine(origin, hitInfo.point, Color.green);
#endif
                }
            }
            return isGroundInRange;
        }
        #endregion

        #region Step Traversal Methods
        private bool FindStep(out Vector3 stepUpOffset, List<Collision> collisions, float groundHeight)
        {
            stepUpOffset = default(Vector3);
            for (int i = 0; i < collisions.Count; i++)
            {
                Collision collision = collisions[i];
                for (int j = 0; j < collision.contacts.Length; j++)
                {
                    bool test = ResolveStepUp(out stepUpOffset, collision.GetContact(j), groundHeight);
                    if (test) return test;
                }
            }
            return false;
        }

        private bool ResolveStepUp(out Vector3 stepUpOffset, ContactPoint stepTestContact, float groundHeight)
        {
            stepUpOffset = default(Vector3);
            Collider stepCollider = stepTestContact.otherCollider;

            // Contact normal must be a ground normal.
            if (Mathf.Abs(stepTestContact.normal.y) <= _minGroundAngleDot)
            {
                return false;
            }

            if (!(stepTestContact.point.y - groundHeight < _stepHeight))
            {
                return false;
            }

            // Check step space in front of us.
            RaycastHit hitInfo;
            float stepTestHeight = groundHeight + _stepHeight + 0.01f;
            Vector3 stepTestInvDir = new Vector3(-stepTestContact.normal.x, 0f, -stepTestContact.normal.z);
            Vector3 origin = new Vector3(stepTestContact.point.x, stepTestHeight, stepTestContact.point.z);
            Vector3 direction = Vector3.down;

            if (!(stepCollider.Raycast(new Ray(origin, direction), out hitInfo, _stepHeight)))
            {
                return false;
            }

#if UNITY_EDITOR
            Debug.DrawLine(origin, direction * 10f, Color.red);
#endif

            // Calculate points.
            Vector3 stepUpPoint = new Vector3(stepTestContact.point.x, hitInfo.point.y + 0.0001f, stepTestContact.point.z)
                + (stepTestInvDir * _stepSearchOvershoot);
            Vector3 stepUpPointOffset = stepUpPoint - new Vector3(stepTestContact.point.x, groundHeight, stepTestContact.point.z);

            stepUpOffset = stepUpPointOffset;
            return true;
        }
        #endregion

        #region Velocity Manipulation Methods
        /// <summary>
        /// Align velocity to a plane defined by the specified plane normal.
        /// </summary>
        /// <param name="velocity"></param>
        /// <param name="normal"></param>
        /// <returns></returns>
        private Vector3 AlignVelocityToNormal(Vector3 velocity, Vector3 normal)
        {
            float speed = velocity.magnitude;
            Vector3 alignedDirection = Quaternion.FromToRotation(Vector3.up, normal) * (velocity / speed);
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

#if UNITY_EDITOR
        private void DrawContactNormals()
        {
            for (int i = 0; i < _collisions.Count; i++)
            {
                Collision collision = _collisions[i];
                for (int j = 0; j < collision.contactCount; j++)
                {
                    ContactPoint contact = collision.GetContact(j);
                    Color color = Color.grey;
                    if (contact.normal.y > _minFlatGroundAngleDot)
                    {
                        color = Color.white;
                    }
                    Debug.DrawLine(contact.point, contact.point + contact.normal * 0.1f, color);
                }
            }
        }
#endif
    }
}