# Overview
NekoMovement is an experimental rigidbody physics character movement solution, designed as an alternative to Unity's Character Controller. It provides the following features:


:heavy_check_mark: **Intuitive collider adjustment** - provides the ability to tweak movement collider height with fixed bottom / tweak floating step height with fixed top.

:heavy_check_mark: **Ground detection** - updates ground contact state each physics frame. Performs ground detection with configurable parameters, using spherecast to support ledge perching.

:heavy_check_mark: **Slope traversal with ground snapping** - provides the option to snap to ground when moving down slopes. Supports slope approximation, which provides interpolated slope information on complex terrain.

:heavy_check_mark: **Smooth stair traversal** - smoothly move up and down on steps and stairs. Gives reliable smoothing behaviour when combined with slope approximation. 

:heavy_check_mark: **Velocity physics** - provides the capability to teak the acceleration, deceleration, friction of planar movement. There are independent velocity channels to separate different types of velocity.

:heavy_check_mark: **Inherit ground velocity** - correctly handles velocity when on moving platforms.

:heavy_check_mark: **Impulses** - provides a controllable way to simulate physical impulses using animation curves.


### Environment

Developed and tested in Unity 2022.2.0b12

# Quick Start

- Add a `CharacterMover` component to a gameobject. A rigidbody and capsule collider will be automatically added and configured.
- In your own movement controller, import `NekoLib.Movement` namespace, reference the `CharacterMover` instance.
- Now you can implement your movement statemachine. Let `CharacterMover` handle the rest.

A simple example:
```csharp
// Example third-person movement controller.
public class MyMovementController : MonoBehaviour
{
    [SerializeField] private MyInputSource _inputSource; // Example input source.
    [SerializeField] private NekoLib.Movement.CharacterMover _characterMover;
    [SerializeField] private Transform _cameraTr;
    [SerializeField] private float _speed = 3f;

    private void Update()
    {
        if(!_inputSource.HasMovementInput())
        {
            // Idle.
            _characterMover.InputMove(0f, Vector3.zero);
            return;
        }
        // Moving.
        Vector3 inputDirection = DirectionFromInput(_inputSource.GetMovementInput());
        _characterMover.InputMove(_speed, inputDirection);
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

# Usage

### Movement Methods
`CharacterMover` provides several methods for initiating movement.
#### 1. Active Velocity
This is the easiest way to get moving. 
Sets the `activeVelocity` of the mover based on the provided speed and direction `activeVelocity` will be part of the final rigidbody velocity at the end of each fixed update.
```csharp
InputMove(float speed, Vector3 direction)
```
`activeVelocity` **persists across fixed updates**, and undergoes built-in velocity physics processing based on the velocity mode. 
Three different velocity modes are provided: None, Simple Advanced. The current mode can be set in the `_velocityMode` inspector field.
- None - No velocity physics. `activeVelocity` changes instantaneously. 
- Simple - `activeVelocity` smoothly changes based on the `_speedChange` inspector field.
- Advanced - `activeVelocity` smoothly changes based on acceleration, deceleration, braking deceleration, friction, and braking friction set in the `_velocityConfig` inspector field. 


#### 2. Passive Velocity
`passiveVelocity` is a background velocity that **persists across fixed updates**.
```csharp
SetPassiveVelocity(Vector3 velocity);
```

#### 3. Extra Velocity
`extraVelocity` will be **cleared at the end of each fixed update**.
```csharp
SetExtraVelocity(Vector3 velocity);
```

#### 4. Override Velocity
Velocity that overrides `activeVelocity`, `passiveVelocity`, `extraVelocity` when present. Will be **cleared at the end of each fixed update**.
```csharp
SetOverrideVelocity(Vector3 velocity, bool clearActiveVelocity = false, bool ignoreConnectedGround = false)
```

#### 5. Move Position
Directly move to a position based on the provided delta position. Usually used to apply root motion.

Calls `Rigidbody.MovePosition` after calculating the position to move to based on the provided parameters and the current state of the mover.
```csharp
MoveDeltaPosition(Vector3 deltaPosition, bool alignToGround = true, bool restrictToGround = false)
```
- `alignToGround` - If grounded, will move to a target position along the current ground slope plane.
- `restrictToGround` - If grounded, will only move if the movement will not cause the mover to leave ground.

