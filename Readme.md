
# SRMove
[![license](https://img.shields.io/badge/license-MIT-green.svg?style=flat&cacheSeconds=2592000)](https://github.com/NoiRC256/SRMove/blob/main/LICENSE)
[![WebGL Demo](https://img.shields.io/badge/demo-WebGL-orange.svg?style=flat&logo=google-chrome&logoColor=white&cacheSeconds=2592000)](https://noirccc.net/blog/srmove-v2-demo)

A low-level rigidbody character movement solution that uses slope approximation to move smoothly across stairs and obstacles.

<img src="https://github.com/NoiRC256/SRMove/assets/33998067/33ac7870-36e6-4c35-9390-4bbd4c5b6832" alt="SRMove-Side-By-Side-Example" width="320">

*Smooth stair traversal without IK or mesh modification.*

[v2.0 WebGL Demo](https://noirccc.net/blog/srmove-v2-demo)

## Key Features

:heavy_check_mark: **Reliable slope traversal** - Snap to angled ground surface while moving

:heavy_check_mark: **Smooth stair traversal** - Smoothly move up / down stairs and across obstacles

:heavy_check_mark: **Supports moving surfaces** - Correctly handle velocity on moving platforms

:heavy_check_mark: **Intuitive collider adjustment** - Configure collider height or step height while keeping collider bottom or top fixed

### Environment

Developed and tested in Unity 6000.0.32f1

# Quick Start

`NoiRC.SRMove.AvatarMover` is equivalent to Unity's `CharacterContoller`.

1. Add an `AvatarMover` component to your character gameobject.

2. In your own movement controller, implement your control logic.
    - Reference the `AvatarMover` component to handle actual movement.

Simple example:

```csharp
using UnityEngine;
using NoiRC.SRMove;

// Example third-person movement controller.
public class MyMovementController : MonoBehaviour
{
    // Example input source that provides a Vector2 movement input every fame.
    [SerializeField] private MyInputSource _inputSource;

    [SerializeField] private AvatarMover _avatarMover;
    [SerializeField] private Transform _cam; // Reference transform used to determine movement direction.
    [SerializeField] private float _moveSpeed = 3f;

    private void FixedUpdate()
    {
        // Calculate the intended movement speed.
        float speed = _inputSource.HasInput() ? _moveSpeed : 0f;

        // Calculate the intended movement direction.
        Vector3 direction = GetMoveDirection(_input.GetMoveInput(), _cam);

        // Move.
        _avatarMover.Move(speed * direction);    
    }

    // Convert 2D input vector into 3D worldspace direction relative to the third-person camera.
    private Vector3 GetMoveDirection(Vector2 input, Transform cam)
    {
         Vector3 direction = (input.x * Vector3.ProjectOnPlane(cam.right, Vector3.up)).normalized;
         direction += (input.y * Vector3.ProjectOnPlane(cam.forward, Vector3.up)).normalized;
         return direction.normalized;
    }
}
```

# Usage

### Movement

```csharp
Move(Vector3 velocity)
```
Set the intended movement velocity for one physics frame.

### Jumping

```csharp
LeaveGround()
```
When intending to start a jump, call `LeaveGround()` to pause ground-related functionalities.


```csharp
EndLeaveGround()
```
When intending to end a jump, call `EndLeaveGround()` to resume ground-related functionalities.

### Moving Surfaces

By default, a character with an `AvatarMover` component inherits the velocity (but not rotation) of any moving ground collider it's on.

Add a `ParentableGround` component to a ground gameobject to enable **ground parenting** when a character is on it.

- Ground parenting: The character is parented to the ground gameobject it's currently on, inheriting both position and rotation.



### Status Properties

```csharp
bool IsOnGround; // Whether the mover is currently on ground.
bool IsTouchingCeiling; // Whether the top of the mover's collider is touching a collider in the ground layer.
Collider GroundCollider; // The collider of the ground the mover is on.
bool IsParentedToGround; // Whether the mover is currently parented to the ground object.
Vector3 Up; // Up direction of the mover.
```
