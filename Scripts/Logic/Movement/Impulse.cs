using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

namespace NekoNeko
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
        public float MaxSpeed { get => _config.MaxSpeed; set => _config.MaxSpeed = value; }
        public Vector3 Direction { get => _config.Direction; set => _config.Direction = value; }
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
            float speed = _config.MaxSpeed * _config.Curve.Evaluate(_counter / _config.Duration);
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
            return speed * _config.Direction;
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
        [SerializeField] [Min(0f)] private float _maxSpeed = 3f;
        [SerializeField] private Vector3 _direction = Vector3.forward;
        [SerializeField] [Min(0f)] private float _duration = 0.5f;
        [SerializeField] private AnimationCurve _curve = AnimationCurve.Linear(0f, 1f, 1f, 0f);
        [SerializeField] private bool _holdOnComplete = true;
        [Header("Flags")]
        [SerializeField] private bool _alignToGround = false;
        [SerializeField] private Impulse.SpeedMode _speedMode = Impulse.SpeedMode.Add;

        public float MaxSpeed { get => _maxSpeed; set => _maxSpeed = value; }
        public Vector3 Direction { get => _direction; set => _direction = value; }
        public float Duration { get => _duration; set => _duration = value; }
        public AnimationCurve Curve { get => _curve; set => _curve = value; }
        public bool HoldOnComplete { get => _holdOnComplete; set => _holdOnComplete = value; }
        public bool AlignToGround { get => _alignToGround; set => _alignToGround = value; }
        public Impulse.SpeedMode SpeedMode { get => _speedMode; set => _speedMode = value; }
    }
}