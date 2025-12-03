
# SRMove
[![license](https://img.shields.io/badge/license-MIT-green.svg?style=flat&cacheSeconds=2592000)](https://github.com/NoiRC256/SRMove/blob/main/LICENSE)
[![WebGL Demo](https://img.shields.io/badge/demo-WebGL-orange.svg?style=flat&logo=google-chrome&logoColor=white&cacheSeconds=2592000)](https://noirccc.net/blog/srmove-v2-demo)

A low-level rigidbody character movement solution that uses slope approximation to move smoothly across stairs and obstacles.

<img src="https://github.com/NoiRC256/SRMove/assets/33998067/33ac7870-36e6-4c35-9390-4bbd4c5b6832" alt="SRMove-Side-By-Side-Example" width="320">

*Smooth stair traversal without IK or mesh modification.*

[V2.0 WebGL Demo](https://noirccc.net/blog/srmove-v2-demo)

[Demo project](https://github.com/NoiRC256/SRMoveDev)

## Key Features

:heavy_check_mark: **Reliable slope traversal** - Snap to angled ground surface while moving

:heavy_check_mark: **Smooth stair traversal** - Smoothly move up / down stairs and across obstacles

:heavy_check_mark: **Supports moving surfaces** - Correctly handle velocity on moving platforms

:heavy_check_mark: **Intuitive collider adjustment** - Configure collider height or step height while keeping collider bottom or top fixed

### Environment

Developed and tested in Unity 6000.0.32f1

# Quick Start

### Installation

Install unitypackage from [Releases](https://github.com/NoiRC256/SRMove/releases)
<!-- Install via git URL: https://github.com/NoiRC256/SRMove.git

![srmove-install-git-package-manager](https://github.com/user-attachments/assets/ea5ae557-12a7-4c28-985d-bb77f60efe44) -->

### Usage Example

`NoiRC.SRMove.AvatarMover` is equivalent to Unity's `CharacterContoller`.

1. Add an `AvatarMover` component to your character gameobject.

2. In your own movement controller, implement your control logic.
    - Reference the `AvatarMover` component to handle actual movement.

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

`AvatarMover` expects you to drive jumps explicitly. The mover only manages grounding behaviour and velocity application — you decide the jump velocity profile and timing. 

Use the provided methods to pause/resume grounding behaviour:
- `LeaveGround()`
- `EndLeaveGround()`
   
#### Example Jump Logic

##### Concept

A jump can be broken down into:
1. **Start** - character leaves ground - Jump starts here
1. **Rising** - character moves upward
2. **Apex** - character reaches highest point, rising stops - Jump stops here

"Jump duration" = time from the start `(1)` to the apex `(3)`. You could use this duration with elapsed time to drive your jump velocity (e.g. a simple lerp or stylized curve).

##### Example

[SimpleAvatarController](https://github.com/NoiRC256/SRMoveDev/blob/main/Assets/Scripts/Avatar/SimpleAvatarController.cs)

Keep calls to the mover exactly as shown below:

1. Whens starting a jump, call:
    ```csharp
    LeaveGround()
    ```
    - Tells the mover you're intentionally leaving the ground.
    - The mover will stop trying to stick to ground / do slope detection while you're airborne.
  
2. During a jump, every fixed update, produce a `jumpVelocity` yourself and apply it using something like:
    ```csharp
    Move(moveVelocity * airControlFactor + jumpVelocity)
    ```   
    - Note: Do this in `FixedUpdate` so timing is stable.
    - `moveVelocity` - horizontal input / ground-relative velocity you want while in air.
    - `airControlFactor` - between 0 and 1, reduces horizontal control while airborne.
    - `jumpVelocity` - should be a Vector3 (typically only with a positive Y component).

3. When the jump is supposed to end (reached jump duration), call:
    ```csharp
    EndLeaveGround()
    ```
    - Tells the mover you are no longer intentionally leaving ground.
    - After this call the mover will resume normal gravity/grounding behaviour (falling / landing detection).

### Moving Surfaces

By default, a character using `AvatarMover` **inherits the ground’s velocity** (but **not** rotation) whenever it stands on a moving collider.

To make the character follow the moving collider's **position *and* rotation**, add a `ParentableGround` component to that moving collider gameobject.
- **Ground Parenting**: When the character stands on a gameobject with a `ParentableGround` component, it becomes a child of that gameobject and fully inherits its movement and rotation.


### Status Properties

```csharp
bool IsOnGround; // Whether the mover is currently on ground.
bool IsTouchingCeiling; // Whether the top of the mover's collider is touching a collider in the ground layer.
Collider GroundCollider; // The collider of the ground the mover is on.
bool IsParentedToGround; // Whether the mover is currently parented to the ground object.
Vector3 Up; // Up direction of the mover.
```
