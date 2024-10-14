using System;
using System.Collections.Generic;
using UnityEngine;

namespace CC.SRMove
{
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(CapsuleCollider))]
    public class AvatarMover : MonoBehaviour
    {
        #region Variables

        #region Inspector Fields

        [Header("Collider")]
        [SerializeField][Min(0f)] private float _height = 2f;
        [SerializeField][Min(0f)] private float _thickness = 1f;
        [SerializeField] private Vector3 _colliderOffset = Vector3.zero;

        [Header("Physics")]
        [SerializeField] private bool _enableGravity = false;
        [SerializeField] private Vector3 _gravityAccel = Vector3.down * 9.8f;
        [SerializeField] private float _gravitySpeedMax = 20f;

        [Header("Ground Detection")]
        [Tooltip("Defines ground layers.")]
        [SerializeField] private LayerMask _groundLayerMask = 1 << 0;
        [SerializeField][Min(0f)] private float _groundProbeExtraDistance = 10f;
        [SerializeField][Min(0f)] private float _groundProbeThickness = 0.1f;
        [Tooltip("If true, while on ground, if ground probe thickness is greater than 0," +
            "fires a secondary raycast towards ground point to obtain actual surface normal.")]
        [SerializeField] private bool _groundProbeFindRealNormal = false;

        [Header("Step")]
        [SerializeField][Min(0f)] private float _stepSmoothDelay = 0.05f;
        [SerializeField][Min(0f)] private float _stepUpHeight = 0.3f;
        [SerializeField][Min(0f)] private float _stepDownHeight = 0.3f;
        [SerializeField][Min(1f)] private float _stepUpSmooth = 3f;
        [SerializeField][Min(1f)] private float _stepDownSmooth = 3f;
        [Tooltip("Step up and step down smoothing multipler while moving.")]
        [SerializeField][Min(0f)] private float _stepSmoothMovingMultipler = 1f;
        [Tooltip("Range in front of and behind for ground slope approximation.")]
        [SerializeField][Min(0f)] private float _slopeApproxRange = 1f;
        [SerializeField][Min(0)] private int _slopeApproxIters = 4;

        [Header("Debug")]
        [SerializeField] private bool _debugGroundDetection = false;
        [SerializeField] private bool _debugSlopeApproximation = false;

        #endregion

        #region Properties

        public GroundInfo GroundInfo {
            get => _groundInfo;
            private set {
                _groundInfo = value;
                IsOnGround = value.IsOnGround;
                GroundCollider = value.IsOnGround ? value.Collider : null;
            }
        }
        /// <summary>
        /// Whether the mover is on ground this physics frame.
        /// <para>True if ground probe has detected ground or capsule collider is touching ground.</para>
        /// </summary>
        public bool IsOnGround {
            get => _isOnGround;
            private set {
                if (value != _isOnGround)
                {
                    HandleIsOnGroundChange(value);
                }
                _isOnGround = value;
            }
        }
        public bool IsTouchingCeiling {
            get => _isTouchingCeiling;
            private set {
                if (value == true && value != _isTouchingCeiling)
                {
                    HandleIsTouchingCeilingChange(value);
                }
                _isTouchingCeiling = value;
            }
        }
        public Collider GroundCollider {
            get => _groundCollider;
            private set {
                if (value == null) UnparentFromGround();
                else
                {
                    _groundInfo.Collider.TryGetComponent<Rigidbody>(out _groundRb);
                    ParentToGround(value);
                }
                _groundCollider = value;
            }
        }
        public Vector3 Up {
            get => _up;
            set {
                if (value != _up)
                {
                    HandleUpDirectionChange(value);
                }
                _up = value;
            }
        }
        private ParentableGround GroundParent {
            get => _groundParent;
            set {
                if (value != _groundParent)
                {
                    HandleGroundParentChange(value);
                }
                _groundParent = value;
            }
        }
        private float ColliderHalfHeight => _collider.height / 2f;
        /// <summary>
        /// Desired ground distance from the capsule collider center.
        /// </summary>
        private float GroundDistanceDesired => ColliderHalfHeight + _stepUpHeight;
        /// <summary>
        /// Ground probe hits witin this distance from the capsule collider center is considered to be ground.
        /// </summary>
        private float GroundDistanceThreshold {
            get {
                float value = GroundDistanceDesired;
                if (IsOnGround) value += _stepDownHeight;
                return value * 1.01f;
            }
        }
        /// <summary>
        /// Total distance to probe downwards for ground from the capsule collider center.
        /// </summary>
        private float GroundProbeDistance => GroundDistanceThreshold + _groundProbeExtraDistance;
        /// <summary>
        /// Capsule collider center.
        /// </summary>
        private Vector3 GroundProbeOrigin => _collider.transform.position + GroundDistanceDesired * _up;

        #endregion

        #region Fields

        #region Cache Fields

        [SerializeField][HideInInspector] private Rigidbody _rigidbody;
        [SerializeField][HideInInspector] private CapsuleCollider _collider;

        private CollisionStore _collisionStore = new CollisionStore();
        private List<Collision> _collisions = new List<Collision>();
        private GroundInfo _groundInfo = GroundInfo.Empty;
        private bool _isOnGround;
        private bool _isTouchingCeiling = false;
        private Collider _groundCollider = null;
        private Rigidbody _groundRb = null;
        private ParentableGround _groundParent = null;
        private Vector3 _slopePoint = Vector3.zero;
        private Vector3 _slopeNormal = Vector3.up;
        private Vector3 _velocityGroundRb = Vector3.zero;
        private Vector3 _velocityGravity = Vector3.zero;
        private Vector3 _velocityHover = Vector3.zero;
        private Vector3 _velocityInput = Vector3.zero;

        private Vector3 _lastNonZeroDirection = Vector3.forward;
        private float _stepHeightHoverPatch = 0f;
        private float _stepSmoothDelayCounter = 0f;

        #endregion

        #region State Fields

        [SerializeField] private Vector3 _up = Vector3.up;
        private bool IsParentedToGround = false;
        private bool _isOnGroundChangedThisFrame;
        private bool _shouldLeaveGround = false;
        private float _leaveGroundTimeElapsed = 0f;
        private bool _collisionIsTouchingCeiling = false;
        private bool _hasDirectCollision = false;

        #endregion

        #endregion

        #region Events

        public event Action<bool> OnIsOnGroundChanged = delegate { };
        public event Action<bool> OnGroundParentChanged = delegate { };
        public event Action<Vector3> OnUpDirectionChanged = delegate { };
        public event Action<bool> OnIsTouchingCeilingChanged = delegate { };

        #endregion

        #endregion

        #region API

        public void Move(Vector3 velocity)
        {
            _velocityInput = velocity;
        }

        public void LeaveGround()
        {
            if (_isTouchingCeiling) return;
            _leaveGroundTimeElapsed = 0f;
            _shouldLeaveGround = true;
        }

        public void EndLeaveGround()
        {
            _shouldLeaveGround = false;
            _isOnGroundChangedThisFrame = true;
        }

        #endregion

        #region MonoBehaviour

        private void OnCollisionStay(Collision collision)
        {
            _hasDirectCollision = true;
            CheckDirectCollision(collision, out bool isOnGround, out bool isTouchingCeiling, out bool itTouchingWall);
            _collisionIsTouchingCeiling = isTouchingCeiling;
        }

        private void OnValidate()
        {
            InitComponents();
            InitColliderDimensions();
        }

        private void Awake()
        {
            OnValidate();
            _lastNonZeroDirection = _collider.transform.forward;
        }

        private void FixedUpdate()
        {
            UpdateCollisionCheck();
            _hasDirectCollision = false;
            UpdateMovement(Time.deltaTime);
            UpdateCleanup();
        }

        private void Update()
        {
            _up = transform.up;
            Debug.DrawLine(GroundProbeOrigin, GroundProbeOrigin + GroundProbeDistance * -_up, Color.green);
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(_groundInfo.Point, 0.25f);
        }

        #endregion

        #region Init

        private void InitComponents()
        {
            // Rigidbody.
            TryGetComponent(out _rigidbody);
            _rigidbody.useGravity = false;
            _rigidbody.interpolation = RigidbodyInterpolation.Interpolate;
            _rigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            _rigidbody.freezeRotation = true;

            // Mover collider.
            TryGetComponent(out _collider);
        }

        private void InitColliderDimensions()
        {
            ColliderUtil.SetHeight(_collider, _height, _stepUpHeight);
            ColliderUtil.SetThickness(_collider, _thickness);
        }

        #endregion

        #region Update

        private void UpdateCollisionCheck()
        {
            GroundInfo groundInfoNew = GroundInfo.Empty;
            groundInfoNew = GroundSensorUtil.Probe(GroundProbeOrigin, _up, GroundProbeDistance, _groundProbeThickness,
                _groundLayerMask, GroundDistanceThreshold,
                _groundProbeFindRealNormal, _debugGroundDetection);

            // If ground distance changed drastically, stop step smoothing for a while.
            if (Mathf.Abs(groundInfoNew.Distance - GroundInfo.Distance) > (0.1f * (_stepUpHeight + _stepDownHeight)))
            {
                _stepSmoothDelayCounter = _stepSmoothDelay;
            }

            if (_shouldLeaveGround)
            {
                if (_isTouchingCeiling) EndLeaveGround();
                else groundInfoNew.IsOnGround = false;
            }

            GroundInfo = groundInfoNew;
            if (_hasDirectCollision)
            {
                IsTouchingCeiling = _collisionIsTouchingCeiling;
            }
            else
            {
                IsTouchingCeiling = false;
            }
        }

        private void UpdateMovement(float deltaTime)
        {
            if (_stepSmoothDelayCounter > 0f) _stepSmoothDelayCounter -= deltaTime;
            if (_velocityInput.magnitude > 0f) _lastNonZeroDirection = _velocityInput.normalized;

            if (IsOnGround)
            {
                if (_groundRb != null)
                {
                    _velocityGroundRb = _groundRb.linearVelocity;
                }
                else
                {
                    _velocityGroundRb = Vector3.zero;
                }

                // Approximate the slope to move along.
                if (_velocityInput != Vector3.zero)
                {
                    _slopeNormal = GroundSensorUtil.ApproximateSlope(in _groundInfo, out _slopePoint,
                        GroundProbeOrigin, _up, GroundDistanceThreshold + 1f, _stepUpHeight, _groundLayerMask,
                        _lastNonZeroDirection, _slopeApproxRange, 5, _debugSlopeApproximation);
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
            Vector3 velocityMove = AlignVelocityToPlane(_velocityInput, _slopeNormal);
            Vector3 velocityToApply = _velocityGroundRb + _velocityGravity + _velocityHover + velocityMove;

            ApplyVelocity(velocityToApply);
        }

        private void UpdateCleanup()
        {
            _collisionStore.Clear();
            _velocityHover = Vector3.zero;
            _velocityInput = Vector3.zero;
            _isOnGroundChangedThisFrame = false;
        }

        #endregion

        private void ApplyVelocity(Vector3 velocity)
        {
            _rigidbody.linearVelocity = velocity;
        }

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

        private void ParentToGround(Collider collider)
        {
            if (collider.transform.TryGetComponent(out ParentableGround groundParent))
            {
                transform.SetParent(groundParent.transform, worldPositionStays: true);
                _up = transform.up;
                IsParentedToGround = true;
                GroundParent = groundParent;
            }
            else
            {
                UnparentFromGround();
            }
        }

        private void UnparentFromGround()
        {
            Vector3 forward = Vector3.ProjectOnPlane(transform.forward, Vector3.up);
            transform.SetParent(null, worldPositionStays: true);
            transform.up = Vector3.up;
            transform.forward = forward;
            _up = Vector3.up;
            IsParentedToGround = false;
            GroundParent = null;
        }

        #region Helpers

        #region Velocity Calculation Helpers

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
                float hoverHeightDelta = Sigmoid(Mathf.Abs(hoverHeightPatch)) / (stepSmooth * deltaTime);
                vel = _up * directionSign * hoverHeightDelta;
            }
            return vel;
        }

        public static float Sigmoid(float value)
        {
            return value;
        }

        /// <summary>
        /// Align velocity to a plane defined by the specified plane normal.
        /// </summary>
        /// <param name="velocity"></param>
        /// <param name="normal"></param>
        /// <returns></returns>
        private Vector3 AlignVelocityToPlane(Vector3 velocity, Vector3 normal)
        {
            float speed = velocity.magnitude;
            Vector3 alignedDirection = Quaternion.FromToRotation(_up, normal) * (velocity / speed);
            return speed * alignedDirection.normalized;
        }

        #endregion

        #region Direct Collision Check Helpers

        private bool CheckDirectCollision(Collision collision, out bool isOnGround, out bool isTouchingCeiling, out bool isTouchingWall)
        {
            isOnGround = false;
            isTouchingCeiling = false;
            isTouchingWall = false;
            Debug.Log("Check collision: " + collision.collider.gameObject.name);
            if (!LayerMaskContains(_groundLayerMask, collision.gameObject.layer))
            {
                return false;
            }
            //if (collision.contactCount == 0) continue;
            ContactPoint contact = collision.GetContact(0);
            if (contact.normal.y > 0.01f)
            {
                isOnGround = true;
                Debug.Log("Is touching ground");
            }
            else if (contact.normal.y < -0.25f)
            {
                isTouchingCeiling = true;
                Debug.Log("Is touching ceiling");
            }
            else
            {
                isTouchingWall = true;
                Debug.Log("Is touching wall");
            }
            return true;
        }
        #endregion

        private bool LayerMaskContains(LayerMask layerMask, int layer)
        {
            return layerMask == (layerMask | (1 << layer));
        }

        #endregion

    }
}