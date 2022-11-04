using UnityEngine;

namespace NekoNeko
{
    [System.Serializable]
    public class VelocityConfig
    {
        public const float kMaxAccel = 50f;
        public const float kMaxDecel = 50f;
        public const float kMaxFriction = 50f;

        private const float kDefaultAccel = 40f;
        private const float kDefaultDecel = 20f;
        private const float kDefaultFriction = 30f;
        private const float kDefaultBrakingFriction = 1f;

        [SerializeField][Range(0f, kMaxAccel)] private float _accel = kDefaultAccel;
        [SerializeField][Range(0f, kMaxDecel)] private float _decel = kDefaultDecel;
        [SerializeField][Range(0f, kMaxFriction)] private float _friction = kDefaultFriction;
        [SerializeField][Range(0f, kMaxFriction)] private float _brakingFriction = kDefaultBrakingFriction;

        public VelocityConfig(float accel = kDefaultAccel, float decel = kDefaultDecel, 
            float friction = kDefaultFriction, float brakingFriction = kDefaultBrakingFriction)
        {
            Accel = accel;
            Decel = decel;
            Friction = friction;
            BrakingFriction = brakingFriction;
        }

        public float Accel { get => _accel; set => _accel = value <= 0f ? kDefaultAccel : value; }
        public float Decel { get => _decel; set => _decel = value <= 0f ? kDefaultDecel : value; }
        public float Friction { get => _friction; set => _friction = value <= 0f ? kDefaultFriction : value; }
        public float BrakingFriction { get => _brakingFriction; set => _brakingFriction = value <= 0f ? kDefaultBrakingFriction : value; }
    }
}