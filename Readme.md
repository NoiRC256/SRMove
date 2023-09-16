
# Overview
[![license](https://img.shields.io/badge/license-MIT-green.svg?style=flat&cacheSeconds=2592000)](https://github.com/NoiRC256/SRMove/blob/main/LICENSE)
[![WebGL Demo](https://img.shields.io/badge/demo-WebGL-orange.svg?style=flat&logo=google-chrome&logoColor=white&cacheSeconds=2592000)](https://noirccc.net/blog/predictive-damping-demo)

This is a rigidbody character movement solution that uses efficient slope approximation to move smoothly on stairs and obstacles.

[WebGL Demo](https://noirccc.net/blog/predictive-damping-demo)


Key Features:


:heavy_check_mark: **Intuitive collider adjustment** - Configure movement collider height while keeping bottom fixed. Configure floating step height while keeping top fixed.

:heavy_check_mark: **Ground detection** - Ground contact information is provided each physics frame. Performs ground detection with configurable parameters, using spherecast to support ledge perching.

:heavy_check_mark: **Slope traversal with ground snapping** - Snap to ground surface while moving. Correctly handles velocity on angled surfaces.

:heavy_check_mark: **Smooth stair traversal** - Smoothly move up and down on steps and stairs. Produces reliable smoothing behaviour when combined with slope approximation. 

:heavy_check_mark: **Velocity physics** - Configurable velocity modes with different acceleration / friction.

:heavy_check_mark: **Supports moving surfaces** - Correctly handles velocity on moving platforms.

### Environment

Developed and tested in Unity 2022.2.0b12

# Quick Start

`CharacterMover` can be used like Unity's `CharacterContoller`.

- Add a `CharacterMover` component to a gameobject.

- In your own movement controller, reference a `CharacterMover` instance using `NekoLib.Movement`.

- Implement your control logic. Let `CharacterMover` handle the rest.

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

