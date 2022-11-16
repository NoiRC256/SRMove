# Overview #
NekoNekoMovement is a rigidbody physics character movement solution, designed as an alternative to Unity's Character Controller.

### Key Features ###
:heavy_check_mark: Intuitive collider adjustment  
:heavy_check_mark: Ground detection  
:heavy_check_mark: Stair traversal with smoothing  
:heavy_check_mark: Slope traversal with ground snapping  
:heavy_check_mark: Tweakable acceleration, deceleration, friction  
:heavy_check_mark: Inherit moving platform velocity  
:heavy_check_mark: Versatile impulse system  

### Environment ###

Developed in Unity 2022.2.0b12

### Usage ###

- Add `CharacterMover` component to a gameobject. A rigidbody and capsule collider will be automatically added and configured.
- In your own movement controller, import `NekoNeko` namespace, and reference the `CharacterMover` instance.
- Now you can use `CharacterMover` to implement your movement logic.

A simple example:
```csharp
using NekoNeko.Movement
public class MyMovementController : MonoBehaviour
{
    [SerializeField] private MyInputSource _inputSource;
    [SerializeField] private CharacterMover _characterMover;
    [SerializeField] private Transform _cameraTr;
    [SerializeField] private float _speed = 3f;

    private void Update()
    {
        float inputSpeed = _speed;
        Vector3 inputDirection = DirectionFromInput(_inputSource.GetMovementInput());
        _characterMover.InputMove(inputSpeed, inputDirection);
    }

    private Vector3 DirectionFromInput(Vector2 input)
    {
        Vector3 direction = input.x * Vector3.ProjectOnPlane(_cameraTr.right, Vector3.up);
        direction += input.y * Vector3.ProjectOnPlane(_cameraTr.forward, Vector3.up);
        return direction.normalized;
    }
}
```