using System;
using UnityEngine;
using NekoSystems.PhysicsLib;

namespace NekoSystems.Movement
{
    /// <summary>
    /// Character movement core.
    /// Provides methods for handling character movement with rigidbody and capsule collider.
    /// </summary>
    [RequireComponent(typeof(CapsuleCollider), typeof(Rigidbody))]
    public class CharacterMover : MonoBehaviour
    {
        #region Exposed Fields

        [SerializeField] private bool isDebugMode = false;

        [Header("Capsule Collider Options")]
        [Tooltip("Total height of the character collider.")]
        [SerializeField] [Min(0)] protected float height = 2f;
        [Tooltip("Diameter of the character collider.")]
        [SerializeField] [Min(0)] protected float thickness = 1f;
        [Tooltip("Offset of the character collider.")]
        [SerializeField] protected Vector3 offset = Vector3.zero;

        [Header("Stairs and Slopes")]
        [Tooltip("Max height of a step the character can traverse over while grounded.")]
        [SerializeField] [Min(0f)] protected float stepHeight = 0f;
        [Tooltip("Maximum ground slope angle the character can traverse over while grounded.")]
        [SerializeField] [Range(1f, 89f)] protected float maxSlopeAngle = 60f;
        [Tooltip("Extra offset to probe ground again to check whether the ground is really a steep slope or just a step normal followed by not steep ground.")]
        [SerializeField] [Min(0f)] protected float stepProbeOffset = 0.01f;

        [Header("Climbing")]
        [Tooltip("Minimum climbing angle allowed.")]
        [SerializeField] [Range(1f, 89f)] protected float minClimbAngle = 50f;
        [Tooltip("Maximum climbing angle allowed.")]
        [SerializeField] [Range(1f, 179f)] protected float maxClimbAngle = 100f;

        [Header("Grond Probing Options")]
        [SerializeField] protected LayerMask groundMask = -1;
        [Tooltip("How far below desired ground distance to probe for ground." +
            "Larger values allow more reliable ground detection at higher speeds when moving down slopes.")]
        [SerializeField] [Min(0)] protected float groundProbeDistance = 1.5f;
        [Tooltip("Factor to multiply by character collider's radius when calculating ground probe radius.")]
        [SerializeField] [Min(0.1f)] protected float groundCheckRadiusFactor = 0.5f;
        [Tooltip("Factor to multiply by desired ground distance to account for floating point errors.")]
        [SerializeField] [Min(0)] protected float groundCheckToleranceFactor = 0.01f;
        [Tooltip("Minimum extra ground threshold distance for snapping to ground. The greater value between step height and this value will be used.")]
        [SerializeField] [Min(0f)] protected float minExtraGroundThreshold = 0.1f;

        [Header("Ground Collider Options")]
        [Tooltip("Maximum angle for a direct collision contact to be considered as ground contact.")]
        [SerializeField] [Range(1f, 89f)] protected float maxGroundContactAngle = 89f;
        #endregion

        #region Properties
        public Rigidbody Rigidbody { get; private set; }
        public Collider Collider { get { return _capsuleCollider; } }
        public Vector3 UpAxis { get { return _upAxis; } }
        public Rigidbody ConnectedBody { get; private set; }
        public Rigidbody PreviousConnectedBody { get; private set; }
        public Vector3 ConnectionVelocity { get { return _connectionVelocity; } }

        public float Height { get { return height; } set { height = value; UpdateColliderDimensions(); } }
        public float Thickness { get { return thickness; } set { thickness = value; UpdateColliderDimensions(); } }
        public Vector3 ColliderOffset { get { return offset; } set { offset = value; UpdateColliderDimensions(); } }
        public float StepHeight { get { return stepHeight; } set { stepHeight = value; UpdateColliderDimensions(); } }

        /// <summary>
        /// Slope limit in the form of minimum allowed dot product between up vector and ground normal.
        /// </summary>
        public float SlopeLimitMinDot { get { return _maxSlopeAngleMinDot; } }
        public float StepProbeOffset { get { return stepProbeOffset; } }

        public Vector3 ColliderCenter { get { return _capsuleCollider.bounds.center; } }
        public Vector3 Velocity { get { return Rigidbody.velocity; } }
        public Vector3 Direction { get { return Rigidbody.velocity.normalized; } }

        /// <summary>
        /// True if ground check suggests we're grounded.
        /// </summary>
        public bool IsGrounded { get { return _isGrounded; } }

        /// <summary>
        /// True if ground probing suggests we're grounded but ground normal is too steep.
        /// </summary>
        public bool IsOnSteepGround { get { return _isOnSteepGround; } }

        /// <summary>
        /// True if collider itself is touching ground that is not too steep.
        /// This typically happens when the capsule collider's bottom sphere is touching ground.
        /// </summary>
        public bool IsCollidingGround { get { return _isCollidingGround; } }

        /// <summary>
        /// Desired ground distance when grounded.
        /// </summary>
        public float DesiredGroundDistance { get { return _desiredGroundDistance; } }

        /// <summary>
        /// Ground probing will indicate grounded if probed ground distance is within this threshold.
        /// </summary>
        public float GroundThresholdDistance { get { return _groundThresholdDistance; } }

        /// <summary>
        /// If true, extend desired ground distance by step height to snap to ground when going downstairs.
        /// </summary>
        public bool UseExtraGroundThresholdDistance { get { return _useExtraGroundThresholdDistance; } set { _useExtraGroundThresholdDistance = value; } }

        /// <summary>
        /// Ground distance probed, not necessarily when grounded.
        /// </summary>
        public float GroundDistance { get { return _groundDistance; } }

        /// <summary>
        /// Ground point probed, not necessarily when grounded.
        /// </summary>
        public Vector3 GroundPoint { get { return _groundPoint; } }

        /// <summary>
        /// Last ground surface normal detected by ground probing when grounded.
        /// This is the actual surface normal, not an inverse hit normal from spherecast.
        /// </summary>
        public Vector3 GroundSurfaceNormal { get { return _groundSurfaceNormal; } }

        /// <summary>
        /// Ground dot representing ground slope angle.
        /// </summary>
        public float UpDotGround { get { return _upDotGround; } }

        /// <summary>
        /// Contact normal detected by the collider itself when touching ground that is not too steep.
        /// </summary>
        public Vector3 GroundCollisionNormal { get { return _groundCollisionNormal; } }

        public Vector3 GroundAdjustmentVelocity { get { return _groundAdjustmentVelocity; } }

        #endregion

        #region Internal Fields

        protected CapsuleCollider _capsuleCollider;
        protected float _capsuleHalfHeight;
        protected float _desiredGroundDistance;
        protected float _groundCheckLength;
        protected float _groundCheckRadius;
        protected Vector3 _upAxis = Vector3.up;

        protected bool _hasCollision = false;
        // True if either the collider is touching ground or ground probing indicates grounded.
        protected bool _isGrounded = false;
        // True if collider itself is touching ground.
        protected bool _isCollidingGround = false;
        // True if the ground slope is too steep.
        protected bool _isOnSteepGround = false;

        protected bool _useExtraGroundThresholdDistance;
        protected float _groundThresholdDistance;
        protected float _extraGroundThresholdDistance;

        // Angle dot limits.
        protected float _groundContactMinDot;
        protected float _maxSlopeAngleMinDot;
        protected float _maxClimbAngleMinDot;
        protected float _minClimbAngleMaxDot;

        // Cache.
        protected float _groundDistance;
        protected Vector3 _groundPoint;
        protected Vector3 _groundSurfaceNormal;
        protected Vector3 _groundAdjustmentVelocity;
        protected float _upDotGround;

        // Direct collision cache.
        protected Collision _collision;
        protected int _collisionCount;
        protected Collision _groundCollision;
        protected int _groundCollisionCount;
        protected Vector3 _groundCollisionNormal;

        // Connected body cache.
        private Vector3 _connectionVelocity;
        private Vector3 _connectedBodyWorldPosition;

        #endregion

        #region MonoBehaviour

        private void OnValidate()
        {
            SetupRigidbody();
            SetupCollider();
            UpdateColliderDimensions();
            minClimbAngle = Mathf.Clamp(minClimbAngle, 0f, maxClimbAngle);

            // Calculate and cache dot from angles.
            _groundContactMinDot = Mathf.Cos(maxGroundContactAngle * Mathf.Deg2Rad);
            _maxSlopeAngleMinDot = Mathf.Cos(maxSlopeAngle * Mathf.Deg2Rad);
            _minClimbAngleMaxDot = Mathf.Cos(minClimbAngle * Mathf.Deg2Rad);
            _maxClimbAngleMinDot = Mathf.Cos(maxClimbAngle * Mathf.Deg2Rad);
        }

        private void Awake()
        {
            OnValidate();
        }

        private void OnCollisionEnter(Collision collision)
        {
            _collisionCount += 1;
        }

        private void OnCollisionStay(Collision collision)
        {
            _collision = collision;
        }

        private void OnCollisionExit(Collision collision)
        {
            _collisionCount -= 1;
        }

        #endregion

        #region Methods

        /// <summary>
        /// Move the character by setting rigidbody velocity.
        /// </summary>
        /// <param name="velocity"></param>
        public void Move(Vector3 velocity)
        {
            //Rigidbody.velocity = velocity + _groundAdjustmentVelocity;
            Rigidbody.AddForce(velocity + _groundAdjustmentVelocity - Rigidbody.velocity, ForceMode.VelocityChange);
        }

        public void PreventGroundPenetration(Vector3 velocity)
        {
            // If not directly contacting ground (i.e. using ground probing to maintain step height),
            // update adjustment velocity to prevent ground penetration if displacement is downwards (toward ground).
            if (!_isCollidingGround && velocity.y < 0f)
            {
                RaycastHit hit;
                bool willBeGrounded;
                float nextGroundDistance;
                PredictGround(velocity * Time.fixedDeltaTime, out hit, out willBeGrounded, out nextGroundDistance);
                if (willBeGrounded)
                {
                    UpdateGroundAdjustmentVelocity(nextGroundDistance);
                }
            }
        }

        /// <summary>
        /// Evaluate direct collision contacts.
        /// Check whether the collision is a ground contact.
        /// </summary>
        /// <param name="collision"></param>
        private void EvaluateCollision(Collision collision)
        {
            for (int i = 0; i < collision.contactCount; i++)
            {
                Vector3 normal = collision.GetContact(i).normal;
                if (normal.y > _groundContactMinDot)
                {
                    _isCollidingGround = true;
                    _groundCollisionNormal += normal;
                    _groundCollision = collision;
                    _groundCollisionCount += 1;
                }
            }
        }

        /// <summary>
        /// Perform ground check at collider center, update ground contact info.
        /// </summary>
        public void CheckGround()
        {
            CheckGround(Vector3.zero);
        }

        /// <summary>
        /// Perform ground check at a position displaced from collider center.
        /// Update ground contact info.
        /// </summary>
        /// <param name="displacement"></param>
        public void CheckGround(Vector3 displacement)
        {
            // Reset cache.
            _groundAdjustmentVelocity = Vector3.zero;
            _isCollidingGround = false;
            _groundCollisionCount = 0;
            _groundCollision = null;
            _groundCollisionNormal = Vector3.zero;
            PreviousConnectedBody = ConnectedBody;
            ConnectedBody = null;

            // Evaluate and update any direct collisions.
            // Some of them may be ground collisions.
            if (_collision != null)
            {
                EvaluateCollision(_collision);
            }

            // If has direct ground collision, use it.
            if (_isCollidingGround)
            {
                _groundAdjustmentVelocity = Vector3.zero;
                UpdateGroundCollisionInfo();
            }
            // If no direct ground collisions, probe for ground and try to maintain step height.
            else
            {
                RaycastHit hit;
                bool isGrounded;
                float groundDistance;
                InternalProbeGround(ColliderCenter + displacement, out hit, out isGrounded, out groundDistance);

                UpdateGroundProbeInfo(hit, isGrounded, _groundDistance);
                // Update ground adjustment velocity to maintain step height.
                if (isGrounded)
                {
                    UpdateGroundAdjustmentVelocity(groundDistance);
                }
            }
        }

        #region Auxillary Ground Probing Methods

        /// <summary>
        /// Probe for ground at a position displaced from colldier center using spherecast.
        /// Does not update grounded state and info. Instead, outpus such info for external handling.
        /// Typically used for lookahead checks.
        /// </summary>
        /// <param name="displacement"></param>
        /// <param name="hit"></param>
        /// <param name="isGrounded"></param>
        /// <param name="groundDistance"></param>
        public void PredictGround(Vector3 displacement, out RaycastHit hit, out bool isGrounded, out float groundDistance)
        {
            InternalProbeGround(ColliderCenter + displacement, out hit, out isGrounded, out groundDistance);
        }

        /// <summary>
        /// Same as <seealso cref="PredictGround(Vector3, out RaycastHit, out bool, out float)"/>, 
        /// except the base origin is at last GroundPoint's lateral position and ColliderCenter's height, rather than ColliderCenter.
        /// </summary>
        /// <param name="displacement"></param>
        /// <param name="hit"></param>
        /// <param name="isGrounded"></param>
        /// <param name="groundDistance"></param>
        public void PredictGroundFromGroundPoint(Vector3 displacement, out RaycastHit hit, out bool isGrounded, out float groundDistance)
        {
            Vector3 origin = ColliderCenter;
            InternalProbeGround(origin + new Vector3(_groundPoint.x - origin.x, 0f, _groundPoint.z - origin.z) + displacement,
                out hit, out isGrounded, out groundDistance);
        }

        /// <summary>
        /// Probe for ground at a position displaced from colldier center, using raycast rather than spherecast.
        /// Does not update grounded state and info. Instead, outputs such info for external handling.
        /// Typically used for lookahead checks.
        /// </summary>
        /// <param name="displacement"></param>
        /// <param name="hit"></param>
        /// <param name="isGrounded"></param>
        /// <param name="groundDistance"></param>
        public void PredictGroundRaycast(Vector3 displacement, out RaycastHit hit, out bool isGrounded, out float groundDistance)
        {
            InternalProbeGroundRaycast(ColliderCenter + displacement, out hit, out isGrounded, out groundDistance);
        }

        /// <summary>
        /// Same as <seealso cref="PredictGroundRaycast(Vector3, out RaycastHit, out bool, out float)"/>, 
        /// except the base origin is at last GroundPoint's lateral position and ColliderCenter's height, rather than ColliderCenter.
        /// </summary>
        /// <param name="displacement"></param>
        /// <param name="hit"></param>
        /// <param name="isGrounded"></param>
        /// <param name="groundDistance"></param>
        public void PredictGroundFromGroundpPointRaycast(Vector3 displacement, out RaycastHit hit, out bool isGrounded, out float groundDistance)
        {
            Vector3 origin = ColliderCenter;
            InternalProbeGroundRaycast(origin + new Vector3(_groundPoint.x - origin.x, 0f, _groundPoint.z - origin.z) + displacement,
                out hit, out isGrounded, out groundDistance);
        }

        #endregion

        // Update ground info from ground probing. Cache relevant ground info.
        private void UpdateGroundProbeInfo(RaycastHit hit, bool isGrounded, float groundDistance)
        {
            // Cache is grounded and probed ground point.
            _isGrounded = isGrounded;
            _groundPoint = hit.point;
            _groundDistance = groundDistance;

            // If is grounded, cache ground contact info.
            if (isGrounded)
            {
                _groundSurfaceNormal = hit.GetActualSurfaceNormal(-_upAxis);
                _upDotGround = Vector3.Dot(_upAxis, _groundSurfaceNormal);
                _isOnSteepGround = _upDotGround < SlopeLimitMinDot && _upDotGround > 0f;
                ConnectedBody = hit.rigidbody;
            }
        }

        // Update ground info from direct collision. Cache relevant ground info.
        private void UpdateGroundCollisionInfo()
        {
            // This should only be called when there is direct ground collision.
            // Cache ground contact info.
            _isGrounded = true;
            _groundDistance = 0f;
            _groundSurfaceNormal = _groundCollisionNormal.normalized;
            _upDotGround = Vector3.Dot(_upAxis, _groundSurfaceNormal);
            _isOnSteepGround = _upDotGround < _maxSlopeAngleMinDot;
            ConnectedBody = _groundCollision.rigidbody;
        }

        // Update ground adjustment velocity for maintaining step height (i.e. desired ground distance).
        private void UpdateGroundAdjustmentVelocity(float groundDistance)
        {
            // Update ground adjustemnt velocity needed to maintain floating capsule.
            float requiredDelta = (_capsuleHalfHeight + stepHeight) - groundDistance;
            _groundAdjustmentVelocity = _upAxis * (requiredDelta / Time.deltaTime);
        }

        // Probe ground by spherecast and output relevant info.
        // This is the main realization for ground probing.
        protected void InternalProbeGround(Vector3 origin, out RaycastHit hit, out bool isGrounded, out float groundDistance)
        {
            bool hasHit = UnityEngine.Physics.SphereCast(origin, _groundCheckRadius, -transform.up, out hit, _groundCheckLength,
                groundMask);

            if (!hasHit)
            {
                isGrounded = false;
                // Ground distance is -1 when no ground found via probing.
                groundDistance = -1f;
                return;
            }

            groundDistance = _upAxis == Vector3.up ? origin.y - hit.point.y : Vector3.Distance(origin, hit.point);

            // Whether to extend ground threshold distance by step height to snap to ground when going downstaris.
            float groundThresholdDistance = _desiredGroundDistance;
            if (_useExtraGroundThresholdDistance)
            {
                groundThresholdDistance += _extraGroundThresholdDistance;
            }
            // Cache.
            _groundThresholdDistance = groundDistance;

            // If ground distance is within ground threshold distance, the character is grounded.
            if (groundDistance <= groundThresholdDistance)
            {
                isGrounded = true;
            }
            else
            {
                isGrounded = false;
            }

#if UNITY_EDITOR
            if (isDebugMode)
            {
                Debug.DrawLine(origin, origin + -transform.up * _groundCheckLength, Color.grey);
                Debug.DrawLine(origin, origin + -transform.up * groundThresholdDistance, Color.blue);
            }
#endif
        }

        // Probe ground by raycast and output relevant info.
        protected void InternalProbeGroundRaycast(Vector3 origin, out RaycastHit hit, out bool isGrounded, out float groundDistance)
        {
            bool hasHit = UnityEngine.Physics.Raycast(origin, -transform.up, out hit, _groundCheckLength, groundMask);

            if (!hasHit)
            {
                isGrounded = false;
                // Ground distance is -1 when no ground found via probing.
                groundDistance = -1f;
                return;
            }

            groundDistance = _upAxis == Vector3.up ? origin.y - hit.point.y : Vector3.Distance(origin, hit.point);

            // Whether to extend ground threshold distance by step height to snap to ground when going downstaris.
            float groundThresholdDistance = _desiredGroundDistance;
            if (_useExtraGroundThresholdDistance)
            {
                groundThresholdDistance += _extraGroundThresholdDistance;
            }
            // Cache.
            _groundThresholdDistance = groundDistance;

            // If ground distance is within ground threshold distance, the character is grounded.
            if (groundDistance <= groundThresholdDistance)
            {
                isGrounded = true;
            }
            else
            {
                isGrounded = false;
            }

#if UNITY_EDITOR
            if (isDebugMode)
            {
                Debug.DrawLine(origin, origin + -transform.up * _groundCheckLength, Color.grey);
                Debug.DrawLine(origin, origin + -transform.up * groundThresholdDistance, Color.blue);
            }
#endif
        }

        /// <summary>
        /// Update connected body velocity.
        /// </summary>
        /// <param name="connectedBody"></param>
        public void UpdateConnectedBody()
        {
            _connectionVelocity = Vector3.zero;

            // Only calculate connection velocity if the connected body is kinematic or heavier.
            if (ConnectedBody && (ConnectedBody.isKinematic || ConnectedBody.mass >= Rigidbody.mass))
            {
                if (ConnectedBody == PreviousConnectedBody)
                {
                    Vector3 connectedBodyMovement = ConnectedBody.position - _connectedBodyWorldPosition;
                    _connectionVelocity = connectedBodyMovement / Time.deltaTime;
                }
                _connectedBodyWorldPosition = ConnectedBody.position;
            }
        }

        protected void UpdateColliderDimensions()
        {
            // Collider base dimensions.
            _capsuleCollider.radius = thickness / 2f;
            Vector3 center = offset * height;

            // Step height.
            if (stepHeight > height)
            {
                stepHeight = height;
            }
            // Move collider up by step height and reduce collider height to keep top fixed.
            center.y += stepHeight / 2f;
            _capsuleCollider.height = height - stepHeight; // (This also moves center up by half step height).

            // Cache and adjust.
            _capsuleCollider.center = center;
            _capsuleHalfHeight = _capsuleCollider.height / 2f;
            if (_capsuleCollider.radius > _capsuleHalfHeight)
            {
                _capsuleCollider.radius = _capsuleHalfHeight;
            }

            UpdateGroundCheckDimensions();
        }

        protected void UpdateGroundCheckDimensions()
        {
            // Update desired ground distance.
            _desiredGroundDistance = (_capsuleHalfHeight + stepHeight) * (1 + groundCheckToleranceFactor);
            _extraGroundThresholdDistance = Mathf.Max(stepHeight, minExtraGroundThreshold);
            // Update total length for ground checking.
            _groundCheckLength = _desiredGroundDistance + groundProbeDistance;
            // Update radius for ground checking.
            _groundCheckRadius = _capsuleCollider.radius * groundCheckRadiusFactor;
        }

        /// <summary>
        /// Returns true if the specified ground dot indicates that the ground is too steep.
        /// </summary>
        /// <param name="upDotGround"></param>
        /// <returns></returns>
        public bool IsGroundSteep(float upDotGround)
        {
            return upDotGround < _maxSlopeAngleMinDot;
        }

        #region Setup Methods

        protected void SetupRigidbody()
        {
            if (!Rigidbody)
            {
                TryGetComponent<Rigidbody>(out Rigidbody rigidbody);
                Rigidbody = rigidbody;
                rigidbody.useGravity = false;
            }
        }

        protected void SetupCollider()
        {
            if (!_capsuleCollider)
            {
                TryGetComponent<CapsuleCollider>(out CapsuleCollider capsuleCollider);
                _capsuleCollider = capsuleCollider;
            }
        }

        #endregion

        #endregion
    }
}