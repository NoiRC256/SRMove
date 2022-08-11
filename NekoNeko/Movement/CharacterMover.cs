using System;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using NekoNeko.PhysicsLib;

namespace NekoNeko.Movement
{
    /// <summary>
    /// Character movement core.
    /// Provides methods for handling character movement with rigidbody and capsule collider.
    /// </summary>
    [RequireComponent(typeof(CapsuleCollider), typeof(Rigidbody))]
    public class CharacterMover : MonoBehaviour
    {
        #region Exposed Fields

        [Header("Capsule Collider Options")]
        [Tooltip("Total height of the mover collider.")]
        [SerializeField] [Min(0)] protected float height = 2f;
        [Tooltip("Diameter of the mover collider.")]
        [SerializeField] [Min(0)] protected float thickness = 0.8f;
        [Tooltip("Offset of the mover collider.")]
        [SerializeField] protected Vector3 offset = Vector3.zero;

        [Header("Stair Traversal")]
        [Tooltip("If true, sets step height as a ratio of mover height.")]
        [SerializeField] protected bool useStepHeightRatio = true;
        [Tooltip("Step height as ratio of mover height.")]
        [SerializeField] [Range(0, 1)] protected float stepHeightRatio = 0.3f;
        [Tooltip("Maximum height of a step the mover can traverse over while grounded.")]
        [SerializeField] [Min(0f)] protected float stepHeight = 0f;
        [Tooltip("Damping of snapping to ground.")]
        [SerializeField] [Range(0, 10)] protected float stepSmooth = 6f;

        [Header("Slope Traversal")]
        [Tooltip("Maximum ground slope angle the mover can traverse over while grounded.")]
        [SerializeField] [Range(1f, 80f)] protected float maxSlopeAngle = 60f;

        [Header("Slope and Stair Options")]
        [Tooltip("If true, when on steep ground, probes to check if ground is actually the tip of a stair.")]
        [SerializeField] protected bool useStairProbing = true;
        [Tooltip("Probing offset for checking whether steep ground is actually the tip of a stair.")]
        [SerializeField] [Min(0.03f)] protected float stairProbeOffset = 0.03f;
        [SerializeField] protected bool useRealGroundNormal = false;

        [Header("Ground Probing Options")]
        [Tooltip("Automatically enable extra ground threshold distance when grounded. " +
            "Snapping to ground allows us to safely step down stairs and slopes while remaining grounded.")]
        [SerializeField] protected bool autoSnapToGround = true;
        [SerializeField] protected LayerMask groundMask = -1;
        [Tooltip("How far below desired ground distance to probe for ground." +
            "Larger values allow more reliable ground detection at higher speeds when moving down slopes.")]
        [SerializeField] [Min(0)] protected float groundProbeDistance = 1.5f;
        [Tooltip("Factor to multiply by mover collider's radius when calculating ground probe radius.")]
        [SerializeField] [Min(0.1f)] protected float groundCheckRadiusFactor = 0.2f;
        [Tooltip("Factor to multiply by desired ground distance to account for floating point errors.")]
        [SerializeField] [Min(0)] protected float groundCheckToleranceFactor = 0.01f;
        [Tooltip("Minimum extra ground threshold distance for snapping to ground. The greater value between step height and this value will be used.")]
        [SerializeField] [Min(0f)] protected float minExtraGroundThreshold = 0.1f;

        [Header("Ground Collision Options (Not Implemented)")]
        [Tooltip("Maximum angle for a collision contact to be considered as ground contact.")]
        [SerializeField] [Range(1f, 89f)] protected float maxGroundContactAngle = 89f;

        [Header("Climbing (Not Implemented)")]
        [Tooltip("Minimum climbing angle allowed.")]
        [SerializeField] [Range(1f, 89f)] protected float minClimbAngle = 50f;
        [Tooltip("Maximum climbing angle allowed.")]
        [SerializeField] [Range(1f, 179f)] protected float maxClimbAngle = 100f;

        [Header("Debug")]
        [SerializeField] private bool isDebugMode = false;
        [Tooltip("Blue: Not grounded. Yellow: Grounded via probe. Green: Grounded via collision. ")]
        [SerializeField] private bool showGizmos = false;

        #endregion

        #region Properties
        public Rigidbody Rigidbody { get; private set; }
        public Collider Collider { get { return _capsuleCollider; } }
        public Vector3 UpAxis { get { return _upAxis; } }
        public Rigidbody ConnectedBody { get; private set; }
        public Rigidbody PrevConnectedBody { get; private set; }
        public Vector3 ConnectionVelocity { get { return _connectionVelocity; } }

        public float Height { get { return height; } set { height = value; UpdateColliderDimensions(); } }
        public float Thickness { get { return thickness; } set { thickness = value; UpdateColliderDimensions(); } }
        public Vector3 ColliderOffset { get { return offset; } set { offset = value; UpdateColliderDimensions(); } }
        public float StepHeight { get { return stepHeight; } set { stepHeight = value; UpdateColliderDimensions(); } }

        /// <summary>
        /// Slope limit in the form of minimum allowed dot product between up vector and ground normal.
        /// </summary>
        public float SlopeLimitMinDot { get { return _maxSlopeAngleMinDot; } }
        public float StepProbeOffset { get { return stairProbeOffset; } }

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
        /// Desired ground distance when grounded.
        /// </summary>
        public float DesiredGroundDistance { get { return _desiredGroundDistance; } }

        /// <summary>
        /// Ground probing will indicate grounded if probed ground distance is within this threshold.
        /// </summary>
        public float GroundThresholdDistance { get { return _groundThresholdDistance; } }

        /// <summary>
        /// If true, extend desired ground distance by step height to snap to ground when going downstairs.
        /// Updates ground threshold distance.
        /// </summary>
        public bool UseExtraGroundThresholdDistance {
            get {
                return _useExtraGroundThresholdDistance;
            }
            set {
                _useExtraGroundThresholdDistance = value;
                _groundThresholdDistance = value ?
                _desiredGroundDistance + _extraGroundThresholdDistance :
                _desiredGroundDistance;
            }
        }

        public bool UseRealGroundNormal { get { return useRealGroundNormal; } set { useRealGroundNormal = value; } }

        /// <summary>
        /// Ground distance probed, not necessarily when grounded.
        /// </summary>
        public float GroundDistance { get { return _groundDistance; } }

        /// <summary>
        /// Ground point probed, not necessarily when grounded.
        /// </summary>
        public Vector3 GroundPoint { get { return _groundPoint; } }

        /// <summary>
        /// Last ground surface normal detected by ground check when grounded.
        /// This is the actual surface normal, not an inverse hit normal from spherecast.
        /// </summary>
        public Vector3 GroundSurfaceNormal { get { return _groundSurfaceNormal; } }

        /// <summary>
        /// Ground dot representing ground slope angle.
        /// </summary>
        public float UpDotGround { get { return _upDotGround; } }

        public Vector3 GroundAdjustmentVelocity { get { return _groundAdjustmentVelocity; } }

        #endregion

        #region Internal Fields

        protected CapsuleCollider _capsuleCollider;
        protected float _capsuleHalfHeight;
        protected float _desiredGroundDistance;
        protected float _groundCheckLength;
        protected float _groundCheckRadius;
        protected Vector3 _upAxis = Vector3.up;

        protected bool _hasGroundContactStateChanged;
        protected bool _wasGrounded = false;
        // True if either the collider is touching ground or ground probing indicates grounded.
        protected bool _isGrounded = false;
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

        // Connected body cache.
        private Vector3 _connectionVelocity;
        private Vector3 _prevConnectedBodyPosition;

        #endregion

        #region MonoBehaviour

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (!showGizmos)
            {
                return;
            }

            if (_isGrounded)
            {
                Gizmos.color = Color.yellow;
            }
            else
            {
                Gizmos.color = Color.blue;
            }
            Gizmos.DrawWireSphere(ColliderCenter, _groundCheckRadius);
            Vector3 endPoint = ColliderCenter + -transform.up * _groundCheckLength;
            Gizmos.DrawWireSphere(endPoint, _groundCheckRadius);
            Gizmos.DrawSphere(_groundPoint, 0.1f);
        }
#endif

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

        private void OnEnable()
        {
            Physics.ContactModifyEvent += FilterContactPoints;
            Physics.ContactModifyEventCCD += FilterContactPoints;
        }

        private void OnDisable()
        {
            Physics.ContactModifyEvent -= FilterContactPoints;
            Physics.ContactModifyEventCCD -= FilterContactPoints;
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

        /// <summary>
        /// Perform ground check at collider center, update ground contact info.
        /// </summary>
        public void CheckGround()
        {
            CheckGround(Vector3.zero);
        }

        /// <summary>
        /// Perform ground check and update ground contact information.
        /// </summary>
        /// <param name="displacement"></param>
        public void CheckGround(Vector3 displacement)
        {
            // Reset cache.
            ResetGroundCache();
            _groundAdjustmentVelocity = Vector3.zero;

            GroundProbeInfo groundProbeInfo;
            RaycastHit hit;
            EvaluateGroundProbe(ColliderCenter + displacement, out groundProbeInfo, out hit);

            ApplyGroundProbeInfo(groundProbeInfo, hit);
            // Update ground adjustment velocity to maintain step height.
            if (groundProbeInfo.IsGrounded)
            {
                _groundAdjustmentVelocity = CalculateGroundAdjustmentVelocity(groundProbeInfo.Distance);
            }
        }

        /// <summary>
        /// Update adjustment velocity to prevent floating at a lower height than desired.
        /// </summary>
        /// <param name="velocity"></param>
        /// <param name="useRaycast"></param>
        public void PreventGroundPenetration(Vector3 velocity, bool useRaycast = false)
        {
            // If not directly contacting ground (i.e. using ground probing to maintain step height),
            // update adjustment velocity to prevent ground penetration if displacement is downwards (toward ground).
            if (velocity.y < 0f)
            {
                GroundInfo groundInfo;
                PredictGround(displacement: velocity * Time.fixedDeltaTime, out groundInfo, useRaycast);
                if (groundInfo.IsGrounded)
                {
                    UpdateGroundAdjustmentVelocity(groundInfo.Distance);
                }
            }
        }

        /// <summary>
        /// Verify whether steep ground is really a slope or the tip of a stair.
        /// Returns true if ground is a slope.
        /// </summary>
        /// <param name="groundPoint"></param>
        /// <returns></returns>
        private bool VerifySteepSlope(Vector3 groundPoint)
        {
            Vector2 direction = new Vector3(groundPoint.x - ColliderCenter.x, 0f, groundPoint.z - ColliderCenter.z).normalized;
            Vector3 origin = groundPoint;
            origin.y += _desiredGroundDistance;
            GroundInfo groundInfo;
            PredictGround(displacement: stairProbeOffset * direction, origin: origin, out groundInfo, useRaycast: true);
            if (!groundInfo.IsGrounded)
            {
                return false;
            }

            // Calculate 2D up dot ground.
            float heightDiff = groundInfo.Point.y - groundPoint.y;
            float upDotGroundLine = Vector2.Dot(Vector2.up, new Vector2(stairProbeOffset, heightDiff).normalized);
            float upDotGround = 1f - Mathf.Abs(upDotGroundLine);

#if UNITY_EDITOR
            if (isDebugMode)
            {
                Debug.DrawLine(groundPoint, groundInfo.Point, Color.red);
            }
#endif

            if (upDotGround < _maxSlopeAngleMinDot)
            {
                // Is on steep slope.
                return true;
            }

            return false;
        }


        #region Ground Check Implementation Methods

        // Probe ground by spherecast.
        protected void EvaluateGroundProbe(Vector3 origin, out GroundProbeInfo groundProbeInfo, out RaycastHit hit)
        {
            bool hasHit = Physics.SphereCast(origin, _groundCheckRadius, -_upAxis, out hit,
                _groundCheckLength, groundMask);
            if (!hasHit)
            {
                groundProbeInfo = GroundProbeInfo.Empty;
                return;
            }

            // Found ground. Check if we're on ground.
            float groundDistance = origin.y - hit.point.y;
            bool isGrounded = IsOnGround(groundDistance);
            groundProbeInfo = new GroundProbeInfo(isGrounded, groundDistance);

#if UNITY_EDITOR
            if (isDebugMode)
            {
                Debug.DrawLine(origin, origin + -_upAxis * _groundCheckLength, Color.grey);
                Color color = Color.blue;
                if (isGrounded)
                {
                    color = Color.yellow;
                }
                Debug.DrawLine(origin, origin + -_upAxis * _groundThresholdDistance, color);
            }
#endif
        }

        protected void EvaluateGroundProbeRay(Vector3 origin, out GroundProbeInfo groundProbeInfo, out RaycastHit hit)
        {
            bool hasHit = Physics.Raycast(origin, -_upAxis, out hit, _groundCheckLength, groundMask);
            if (!hasHit)
            {
                groundProbeInfo = GroundProbeInfo.Empty;
                return;
            }

            // Found ground. Check if we're on ground.
            float groundDistance = origin.y - hit.point.y;
            bool isGrounded = IsOnGround(groundDistance);
            groundProbeInfo = new GroundProbeInfo(isGrounded, groundDistance);

#if UNITY_EDITOR
            if (isDebugMode)
            {
                Debug.DrawLine(origin, origin + -_upAxis * _groundCheckLength, Color.grey);
                Color color = Color.blue;
                if (isGrounded)
                {
                    color = Color.yellow;
                }
                Debug.DrawLine(origin, origin + -_upAxis * _groundThresholdDistance, color);
            }
#endif
        }

        #endregion

        #region Auxillary Ground Probing Methods

        /// <summary>
        /// Probe for ground at a position displaced from collider center.
        /// Does not update grounded state and info. Instead, outputs ground info for external handling.
        /// Typically used for lookahead checks.
        /// </summary>
        /// <param name="displacement"></param>
        /// <param name="groundInfo"></param>
        /// <param name="useRaycast"></param>
        public void PredictGround(Vector3 displacement, out GroundInfo groundInfo, bool useRaycast = false)
        {
            PredictGround(displacement, ColliderCenter, out groundInfo, useRaycast);
        }

        /// <summary>
        /// Probe for ground at a position displaced from lateral origin.
        /// Does not update grounded state and info. Instead, outputs ground info for external handling.
        /// Typically used for lookahead checks.
        /// </summary>
        /// <param name="displacement"></param>
        /// <param name="origin"></param>
        /// <param name="groundInfo"></param>
        /// <param name="useRaycast"></param>
        public void PredictGround(Vector3 displacement, Vector3 origin, out GroundInfo groundInfo, bool useRaycast = false)
        {
            GroundProbeInfo groundProbeInfo;
            RaycastHit hit;
            if (!useRaycast)
            {
                EvaluateGroundProbe(origin + displacement, out groundProbeInfo, out hit);
            }
            else
            {
                EvaluateGroundProbeRay(origin + displacement, out groundProbeInfo, out hit);
            }
            groundInfo = new GroundInfo(groundProbeInfo, hit);
        }

        #endregion

        #region Ground Info Cache Methods

        // Clear some ground info cache variables.
        private void ResetGroundCache()
        {
            PrevConnectedBody = ConnectedBody;
            ConnectedBody = null;
        }

        // Cache ground info from ground probing.
        private void ApplyGroundProbeInfo(GroundProbeInfo groundProbeInfo, RaycastHit hit)
        {
            // Cache ground contact state and ground info.
            _isGrounded = groundProbeInfo.IsGrounded;
            _groundPoint = hit.point;
            _groundDistance = groundProbeInfo.Distance;

            // If is grounded, cache ground contact info.
            if (groundProbeInfo.IsGrounded)
            {
                _groundSurfaceNormal = useRealGroundNormal ? hit.GetSurfaceNormal(-_upAxis) : hit.normal;
                SetConnectedBody(hit.rigidbody);

                _upDotGround = Vector3.Dot(_upAxis, _groundSurfaceNormal);
                _isOnSteepGround = _upDotGround < SlopeLimitMinDot && _upDotGround > 0f;

                // Verify whether we are on steep slope or the tip of a stair.
                if (_isOnSteepGround && useStairProbing)
                {
                    _isOnSteepGround = VerifySteepSlope(hit.point);
                }
            }

            UpdateGroundContactState(_isGrounded);
        }

        // Update if ground contact state has changed.
        private void UpdateGroundContactState(bool isGrounded)
        {
            _hasGroundContactStateChanged = false;

            // Ground contact changed.
            if (isGrounded != _wasGrounded)
            {
                _hasGroundContactStateChanged = true;
                _wasGrounded = isGrounded;

                if (autoSnapToGround)
                {
                    if (isGrounded)
                    {
                        UseExtraGroundThresholdDistance = true;
                    }
                    else
                    {
                        UseExtraGroundThresholdDistance = false;
                    }
                }
            }
        }

        #endregion

        #region Physics Cache Methods

        // Calculate ground adjustment velocity for maintaining step height (i.e. desired ground distance).
        private Vector3 CalculateGroundAdjustmentVelocity(float groundDistance)
        {
            // Update ground adjustemnt velocity needed to maintain floating capsule.
            float requiredDelta = (_capsuleHalfHeight + stepHeight) - groundDistance;

            // No smoothing if ground contact state has just changed.
            if (_hasGroundContactStateChanged)
            {
                return _upAxis * (requiredDelta / Time.deltaTime);
            }
            else
            {
                return _upAxis * (requiredDelta / (Time.deltaTime * stepSmooth));
            }
        }

        /// <summary>
        /// Update the ground adjustment velocity used to maintain step height.
        /// </summary>
        /// <param name="groundDistance"></param>
        public void UpdateGroundAdjustmentVelocity(float groundDistance)
        {
            _groundAdjustmentVelocity = CalculateGroundAdjustmentVelocity(groundDistance);
        }

        public void SetConnectedBody(Rigidbody rigidbody)
        {
            PrevConnectedBody = ConnectedBody;
            ConnectedBody = rigidbody;
        }

        /// <summary>
        /// Update connected body velocity.
        /// </summary>
        /// <param name="connectedBody"></param>
        public void UpdateConnectedBodyVelocity()
        {
            _connectionVelocity = Vector3.zero;

            // Only calculate connection velocity if the connected body is kinematic.
            if (ConnectedBody != null && (ConnectedBody.isKinematic))
            {
                //Vector3 connectedBodyMovement = ConnectedBody.position - _prevConnectedBodyPosition;
                //_connectionVelocity = connectedBodyMovement / Time.deltaTime;
                _connectionVelocity = ConnectedBody.velocity;
                _prevConnectedBodyPosition = ConnectedBody.position;
            }
        }

        #endregion

        private void FilterContactPoints(PhysicsScene scene, NativeArray<ModifiableContactPair> pairs)
        {
            for (int i = 0; i < pairs.Length; i++)
            {
                ModifiableContactPair pair = pairs[i];
                for (int j = 0; j < pair.contactCount; j++)
                {
                    if (pair.GetNormal(j).y >= _maxSlopeAngleMinDot)
                    {
                        pair.IgnoreContact(j);
                    }
                }
            }
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

            UseExtraGroundThresholdDistance = false;
        }

        protected void UpdateColliderDimensions()
        {
            // Collider base dimensions.
            _capsuleCollider.radius = thickness / 2f;
            Vector3 center = offset * height;

            // Step height.
            if (useStepHeightRatio)
            {
                stepHeight = height * stepHeightRatio;
            }
            else
            {
                if (stepHeight > height)
                {
                    stepHeight = height;
                }
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

        /// <summary>
        /// Returns true if the provided ground distance is within ground threshold distance.
        /// </summary>
        /// <param name="groundDistance"></param>
        /// <returns></returns>
        public bool IsOnGround(float groundDistance)
        {
            if (groundDistance <= _groundThresholdDistance)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Returns true if the provided ground dot indicates that the ground is too steep.
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
            _capsuleCollider.hasModifiableContacts = true;
        }

        #endregion

        #endregion
    }
}