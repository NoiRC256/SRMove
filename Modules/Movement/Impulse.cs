using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

namespace Nap.Movement
{
    /// <summary>
    /// Velocity impulse that evaluates a speed curve over a specified duration.
    /// </summary>
    public class Impulse
    {
        /// <summary>
        /// Impulse speed handling flag.
        /// </summary>
        public enum SpeedMode
        {
            /// <summary>
            /// Directly add impulse speed.
            /// </summary>
            Add,
            /// <summary>
            /// Use max between impulse speed and existing speed.
            /// </summary>
            Max,
        }

        public bool IsComplete { get; set; }
        public bool IsActive { get; set; }
        public float MaxSpeed { get => _config.MaxSpeedRatio; set => _config.MaxSpeedRatio = value; }
        public Vector3 Velocity { get => _config.Velocity; set => _config.Velocity = value; }
        /// <summary>
        /// If true, will not automatically deactivate on completion. Evaluation counter will be fixed at t = 1.0.
        /// </summary>
        public bool HoldOnComplete { get => _config.HoldOnComplete; set => _config.HoldOnComplete = value; }
        /// <summary>
        /// Ground alignment flag. If true, suggests to align velocity to ground.
        /// </summary>
        public bool AlignToGroundFlag { get => _config.AlignToGround; set => _config.AlignToGround = value; }
        /// <summary>
        /// Speed mode flag. Suggests how to handle speed from this impulse.
        /// </summary>
        public SpeedMode SpeedModeFlag { get => _config.SpeedMode; set => _config.SpeedMode = value; }
        /// <summary>
        /// Leave ground flag. If true, suggests to disable ground snapping.
        /// </summary>
        public bool LeaveGroundFlag { get => _config.LeaveGround; set => _config.LeaveGround = value; }

        private ImpulseConfig _config;
        private float _counter;

        public Impulse(ImpulseConfig config)
        {
            _config = config;
            _counter = 0f;
            IsComplete = false;
            IsActive = false;
        }

        public Vector3 Evaluate(float deltaTime)
        {
            if (!IsActive) return Vector3.zero;
            float speedRatio = _config.MaxSpeedRatio * _config.Curve.Evaluate(_counter / _config.Duration);
            _counter += Time.deltaTime;
            if (_counter > _config.Duration)
            {
                _counter = _config.Duration;
                IsComplete = true;
                if (!_config.HoldOnComplete)
                {
                    Deactivate();
                }
            }
            return speedRatio * _config.Velocity;
        }

        /// <summary>
        /// Start evaluation counter from the beginning.
        /// </summary>
        public void Start()
        {
            _counter = 0f;
            IsComplete = false;
            IsActive = true;
        }

        /// <summary>
        /// Deactivate impulse.
        /// </summary>
        public void Deactivate()
        {
            IsActive = false;
        }
    }

    [System.Serializable]
    public class ImpulseConfig
    {
        [SerializeField] [Min(0f)] private float _maxSpeedRatio = 1f;
        [SerializeField] private Vector3 _velocity = Vector3.up;
        [SerializeField] [Min(0f)] private float _duration = 0.5f;
        [SerializeField] private AnimationCurve _curve = AnimationCurve.Linear(0f, 1f, 1f, 0f);
        [SerializeField] private bool _holdOnComplete = true;
        [Header("Flags")]
        [SerializeField] private bool _alignToGround = false;
        [SerializeField] private Impulse.SpeedMode _speedMode = Impulse.SpeedMode.Add;
        [SerializeField] private bool _leaveGround = true;

        public float MaxSpeedRatio { get => _maxSpeedRatio; set => _maxSpeedRatio = value; }
        public Vector3 Velocity { get => _velocity; set => _velocity = value; }
        public float Duration { get => _duration; set => _duration = value; }
        public AnimationCurve Curve { get => _curve; set => _curve = value; }
        public bool HoldOnComplete { get => _holdOnComplete; set => _holdOnComplete = value; }
        public bool AlignToGround { get => _alignToGround; set => _alignToGround = value; }
        public Impulse.SpeedMode SpeedMode { get => _speedMode; set => _speedMode = value; }
        public bool LeaveGround { get => _leaveGround; set => _leaveGround = value; }
    }
}