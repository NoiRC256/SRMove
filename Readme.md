# Overview
NekoMovement is an experimental rigidbody physics character movement solution, designed as an alternative to Unity's Character Controller. It provides the following features.


:heavy_check_mark: **Intuitive collider adjustment** - provides the ability to tweak movement collider height with fixed bottom / tweak floating step height with fixed top.

:heavy_check_mark: **Ground detection** - updates ground contact state each physics frame. Performs ground detection with configurable parameters, using spherecast to support ledge perching.

:heavy_check_mark: **Slope traversal with ground snapping** - provides the option to snap to ground when moving down slopes. Supports slope approximation, which provides interpolated slope information on complex terrain.

:heavy_check_mark: **Smooth stair traversal** - smoothly move up and down on steps and stairs. Gives reliable smoothing behaviour when combined with slope approximation. 

:heavy_check_mark: **Velocity physics** - provides the capability to teak the acceleration, deceleration, friction of planar movement triggered from input.

:heavy_check_mark: **Inherit ground velocity** - correctly handles velocity when on moving platforms.

:heavy_check_mark: **Impulses** - provides a controllable way to simulate physical impulses.


### Environment

Developed and tested in Unity 2022.2.0b12

# Usage

- Add a `CharacterMover` component to a gameobject. A rigidbody and capsule collider will be automatically added and configured.
- In your own movement controller, import `Nap.Movement` namespace, and reference the `CharacterMover` instance.
- Now you can implement your movement state logic. Let `CharacterMover` handle the rest ~

A simple example:
```csharp
// Example third-person movement controller.
public class MyMovementController : MonoBehaviour
{
    [SerializeField] private MyInputSource _inputSource;
    [SerializeField] private Nap.Movement.CharacterMover _characterMover;
    [SerializeField] private Transform _cameraTr;
    [SerializeField] private float _speed = 3f;

    private void Update()
    {
        if(!_inputSource.HasMovementInput())
        {
            _characterMover.SetInputVelocity(0f, Vector3.zero);
            return;
        }
        Vector3 inputDirection = DirectionFromInput(_inputSource.GetMovementInput());
        _characterMover.SetInputVelocity(_speed, inputDirection);
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