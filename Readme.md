# Overview
NekoMovement is an experimental rigidbody physics character movement solution, designed as an alternative to Unity's Character Controller. It provides the following key features.

:heavy_check_mark: **Intuitive collider adjustment** - provides the ability to tweak movement collider height with fixed bottom / tweak floating step height with fixed top.

:heavy_check_mark: **Ground detection** - updates ground contact state each physics frame. Detects ground with configurable parameters, and uses spherecast to support ledge perching. 

:heavy_check_mark: **Slope traversal with ground snapping** - provides the ability to snap to ground when moving down slopes. Supports slope approximation to obtain stable slope information on complex terrain.

:heavy_check_mark: **Smooth stair traversal** - don't rely on IK for smoothly stepping up and down. Gives reliable smoothing behaviour when combined with the slope approximation feature.    

:heavy_check_mark: **Velocity physics** - provides the ability to tweak acceleration, deceleration, friction of active velocity.

:heavy_check_mark: **Inherit ground velocity** - correctly handles velocity when on moving platforms.

:heavy_check_mark: **Versatile impulse system** - provides a controllable way to simulate physical impulses.


### Environment

Developed and tested in Unity 2022.2.0b12

# Usage

- Add a `CharacterMover` component to a gameobject. A rigidbody and capsule collider will be automatically added and configured.
- In your own movement controller, import `NekoNeko` namespace, and reference the `CharacterMover` instance.
- Now you can implement your movement state logic. Let `CharacterMover` handle the rest ~

A simple example:
```csharp
// Example third-person movement controller.
public class MyMovementController : MonoBehaviour
{
    [SerializeField] private MyInputSource _inputSource;
    [SerializeField] private NekoNeko.CharacterMover _characterMover;
    [SerializeField] private Transform _cameraTr;
    [SerializeField] private float _speed = 3f;

    private void Update()
    {
        float inputSpeed = _speed;
        Vector3 inputDirection = DirectionFromInput(_inputSource.GetMovementInput());
        _characterMover.InputMove(inputSpeed, inputDirection);
    }

    // Convert 2D input vector to 3D direction relative to camera transform.
    private Vector3 DirectionFromInput(Vector2 input)
    {
        Vector3 direction = input.x * Vector3.ProjectOnPlane(_cameraTr.right, Vector3.up);
        direction += input.y * Vector3.ProjectOnPlane(_cameraTr.forward, Vector3.up);
        return direction.normalized;
    }
}
```