
# SRMove
[![license](https://img.shields.io/badge/license-MIT-green.svg?style=flat&cacheSeconds=2592000)](https://github.com/NoiRC256/SRMove/blob/main/LICENSE)
[![WebGL Demo](https://img.shields.io/badge/demo-WebGL-orange.svg?style=flat&logo=google-chrome&logoColor=white&cacheSeconds=2592000)](https://noirccc.net/blog/predictive-damping-demo)

SRMove is a rigidbody character movement solution that uses efficient slope approximation to move smoothly on stairs and obstacles.


This project aims to recreate the low-level character movement logic used in Genshin Impact and Honkai: Star Rail. 

<img src="https://github.com/NoiRC256/SRMove/assets/33998067/33ac7870-36e6-4c35-9390-4bbd4c5b6832" alt="SRMove-Side-By-Side-Example" width="320">

In Genshin Impact, even though character IK is disabled while the character is moving, the character is still able to travel smoothly on stairs that have unmodified mesh colliders. This may partially be the result of some slope approximation system, which allows character movement logic to consider *stairs* as *angled ground* for the purpose of aligning movement velocity with ground surface.

[WebGL Demo](https://noirccc.net/blog/predictive-damping-demo)


## Key Features

:heavy_check_mark: **Ground detection** - Ground contact information is provided each physics frame. Performs ground detection with configurable parameters, using spherecast to support ledge perching

:heavy_check_mark: **Slope traversal with ground snapping** - Snap to ground surface while moving. Correctly handles velocity on angled surfaces

:heavy_check_mark: **Smooth stair traversal** - Smoothly move up and down on steps and stairs. Produces reliable smoothing behaviour when combined with slope approximation

:heavy_check_mark: **Velocity physics** - Optional built-in accleration and deceleration.

:heavy_check_mark: **Supports moving surfaces** - Correctly handles velocity on moving platforms

:heavy_check_mark: **Intuitive collider adjustment** - Configure collider height or step height while keeping collider bottom or top fixed

### Environment

Developed and tested in Unity 2022.2.0b12

# Quick Start

`CharacterMover` is an equivalent of Unity's `CharacterContoller`.

- Add a `CharacterMover` component to the gameobject you want to move.

- In your own movement controller, add a using directive for `NekoLab.SRMove`.

- Implement your control logic. Reference a `CharacterMover` instance to handle the movement.

A simple example:
```csharp
using UnityEngine;
using NekoLab.SRMove;

// Example third-person movement controller.
public class MyMovementController : MonoBehaviour
{
    [SerializeField] private MyInputSource _inputSource; // Example input source.
    [SerializeField] private CharacterMover _characterMover;
    [SerializeField] private Transform _cameraTr; // Reference transform used to determine movement direction.
    [SerializeField] private float _speed = 3f;

    private void FixedUpdate()
    {
        if (!_inputSource.HasMoveInput())
        {
            _characterMover.Move(Vector3.zero);
            return;
        }

        // Calculate the direction you want to move along.
        Vector3 moveDirection = GetMoveDirection(_input.GetMoveInput(), _cameraTr);

        // Move.
        _characterMover.Move(_speed * moveDirection);
    }

    // Convert 2D input vector to 3D worldspace direction relative to the reference transform.
    private Vector3 GetMoveDirection(Vector2 input, Transform directionReference)
    {
         Vector3 direction = (input.x * Vector3.ProjectOnPlane(directionReference.right, Vector3.up)).normalized;
         direction += (input.y * Vector3.ProjectOnPlane(directionReference.forward, Vector3.up)).normalized;
         return direction.normalized;
    }
}
```

# Usage

### Movement Methods
`CharacterMover` provides several methods for movement.
#### 1. Move

This is the easiest way to get moving. Call `Move` every fixed update to set the intended movement velocity.

```csharp
Move(Vector3 velocity);
Move(float speed, Vector3 direction);
```

#### 2. Active Velocity
`activeVelocity` is an internal velocity field that CharacterMover uses to set Rigidbody velocity at the end of each fixed update. When the Velocity Mode inspector field is set to a mode that uses velocity physics (i.e. acceleration, deceleration, friction), `activeVelocity` will persist across fixed updates and gradually change based on the configured velocity physics logic.

To ignore any applicable velocity physics and directly set CharacerMover velocity, you can set `activeVelocity` by:
```csharp
SetActiveVelocity(Vector3 velocity);
```

You can clear the `activeVelocity` by:
```csharp
ClearActiveVelocity();
```


#### 3. Delta Position

Directly sets the intended position change for the current fixed update.

This can be used to drive CharacterMover by animation root motion.
```csharp
MoveDeltaPosition(Vector3 deltaPosition, bool alignToGround, bool restrictToGround)
```

### Velocity Physics Modes
Use the Velocity Mode inspector field on a CharacerMover to configure velocity physics behaviour.
- Raw: Instant acceleration and deceleration
- Simple: Fixed acceleration and deceleration based on the Speed Change Rate inspector field



