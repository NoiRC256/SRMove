using UnityEngine;
using BehaviorDesigner.Runtime;
using BehaviorDesigner.Runtime.Tasks;

namespace NekoLib.Movement
{
    [TaskCategory("NekoMovement")]
    [TaskName("Input Move")]
    public class InputMove : Action
    {
        public CharacterMover Mover;
        public SharedFloat Speed = 0f;
        public SharedVector3 Direction = Vector3.forward;

        public override TaskStatus OnUpdate()
        {
            if(Speed.Value > 0f && Direction.Value != Vector3.zero)
            {
                return TaskStatus.Running;
            }
            return TaskStatus.Success;
        }

        public override void OnFixedUpdate()
        {
            Mover.InputMove(Speed.Value, Direction.Value);
        }
    }
}