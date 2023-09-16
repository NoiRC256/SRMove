using UnityEngine;

namespace NekoLib.SRMove.NekoPhysics
{

    [System.Serializable]
    public class VelocityProfile
    {
        public const float kMaxAccel = 50f;
        public const float kMaxDecel = 50f;
        public const float kMaxFriction = 50f;

        private const float kDefaultAccel = 30f;
        private const float kDefaultDecel = 30f;
        private const float kDefaultBrakingDecel = 30f;
        private const float kDefaultFriction = 20f;
        private const float kDefaultBrakingFriction = 0f;

        [SerializeField][Range(0f, kMaxAccel)] private float _accel = kDefaultAccel;
        [SerializeField][Range(0f, kMaxDecel)] private float _decel = kDefaultDecel;
        [SerializeField][Range(0f, kMaxDecel)] private float _brakingDecel = kDefaultBrakingDecel;
        [SerializeField][Range(0f, kMaxFriction)] private float _friction = kDefaultFriction;
        [SerializeField][Range(0f, kMaxFriction)] private float _brakingFriction = kDefaultBrakingFriction;

        public VelocityProfile(float accel = kDefaultAccel,
            float decel = kDefaultDecel, float brakingDecel = kDefaultBrakingDecel,
            float friction = kDefaultFriction, float brakingFriction = kDefaultBrakingFriction)
        {
            Accel = accel;
            Decel = decel;
            BrakingDecel = brakingDecel;
            Friction = friction;
            BrakingFriction = brakingFriction;
        }

        public float Accel {
            get => _accel;
            set => _accel = value <= 0f ? kDefaultAccel : value;
        }
        public float Decel {
            get => _decel;
            set => _decel = value <= 0f ? kDefaultDecel : value;
        }
        public float BrakingDecel {
            get => _brakingDecel;
            set => _brakingDecel = value <= 0f ? kDefaultBrakingDecel : value;
        }
        public float Friction {
            get => _friction;
            set => _friction = value <= 0f ? kDefaultFriction : value;
        }
        public float BrakingFriction {
            get => _brakingFriction;
            set => _brakingFriction = value <= 0f ? kDefaultBrakingFriction : value;
        }
    }
}