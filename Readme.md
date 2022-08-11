# Overview #
NekoNekoMovement is a rigidbody physics character movement solution, designed as an alternative to Unity's Character Controller.

This is useful if you want to handle velocity physics yourself in a rigidbody physics-based approach, while having basic functionalities handled for you in a lower-level MonoBehaviour.


### Key Features ###
:heavy_check_mark: Ground detection  
:heavy_check_mark: Stair traversal with smoothing  
:heavy_check_mark: Slope traversal  
:heavy_check_mark: Intuitive capsule collider adjustment  
:heavy_check_mark: Inherit moving platform velocity  

### Environment ###

Tested in Unity 2021.3.6f1 LTS

Requires Unity 2021.2 and above


### Usage ###

- Add `CharacterMover` component to a gameobject. A rigidbody and capsule collider will be automatically added.
- In your own movement controller, import `NekoSystems.Movement`, and reference the `CharacterMover`.
- Now you can use methods from `CharacterMover` to implement your movement logic.

For examples, see the [Examples Section](#examples)

```csharp
using NekoNeko.Movement
...
private void FixedUpdate()
{
    CharacterMover.CheckGround();
    ...
    Vector3 velocity = ...
    ...
    CharacterMover.Move(velocity);
}
```

### WIP ###
- Influence from moving platform rotation


# Implementation Details #

This movement solution uses three components: a rigidbody, a capsule collider, and `CharacterMover`.

Capsule collider handles non-ground collisions. Its dimensions can be adjusted by `CharacterMover`'s inspector fields in an intuitive way.

`CharacterMover` handles ground collisions, and provides core movement features.

Ground detection and collision is completely handled by the `CharacterMover` MonoBehaviour. [Contact modification](https://docs.unity3d.com/2021.3/Documentation/ScriptReference/Physics.ContactModifyEvent.html) is used to ensure the capsule collider never collides with traversable ground, even at steep angles.

## Stair Traversal ##
NekoNekoMovement uses a floating capsule approach to handle stair traversal. The character's capsule collider constantly floats at a distance from ground to maintain step height.

This approach allows minimal hassle for stair detection. `CharacterMover` uses only one spherecast for ground detection, and one short raycast for stair detection when necessary.

In a `CheckGround()` call, `CharacterMover` calculates and caches a ground adjustment velocity based on the detected ground distance. This ground adjustment velocity is used to maintain desired distance from ground in the next `Move(velocity)` call.

## Ground Probing ##
NekoNekoMovement uses spherecast for ground probing. Using raycast provides slightly better performance, but might leave us vulnerable to misdetections due to small gaps on the ground. Spherecast ground probing provides more consistent results, and allows us to perch off ledges.

In each `CheckGround()` call, we use ground probing to find ground. Ground probing performs a spherecast from collider center downwards for a fixed distance, and refreshes ground information. If the hit distance is within ground threshold distance, then we are on ground. Ground threshold distance is derived from a combination of capsule halfheight, configured step height, and a small error threshold. 

## Snapping To Ground ##
The character is able to stick to ground when traversing across small obstacles, stepping down stairs, or going down slopes. This can be achieved by a combination of methods:

1. By default, `UseExtraGroundThresholdDistance` is automatically set to true while grounded. The extended ground probing distrance allows us to safely step down stairs while remaining grounded. You can handle this manually by setting the inspector field `autoSnapToGround` to false.

2. For more reliable ground snapping, you can also align your movement velocity to ground surface. 

For example:

```csharp
private FixedUpdate()
{
    // Lateral movement direction from input.
    Vector3 desiredDirection = ...
    ...
    if(CharacterMover.IsGrounded)
    {
        // Rotate direction to align with ground slope.
        Quaternion deltaAlign = Quaternion.FromToRotation(Vector3.up, CharacterMover.GroundSurfaceNormal);
        Vector3 groundAlignedDirection = (deltaAlign * desiredDirection).normalized;
        velocity = _speed * groundAlignedDirection;
    }
    ...

}
```

This also normalizes your ground speed on slopes.

## Preventing Ground Penetration ##

When landing at a high speed, the character may sometimes appear to slightly penetrate the ground for one frame. This is an artiface of the floating capsule approach. Since the ground adjustment velocity used for floating is often only computed at the beginning of a physics frame, but applied to rigidbody velocity after some new movement velocity has been calculated, it actually lags behind one physics frame. 

(This may also be related to [Physics.autoSyncTransforms defaulting to false from Unity 2018.3 onwards](https://unity.com/how-to/best-practices-performance-optimization-unity#transforms-performance-scattered-vs-batched-physics-query), although not tested).

To mitigate this artifact, you can choose to apply `PreventGroundPenetration(velocity, useRaycast)` before `Move(velocity)`. This preemptively updates ground adjustment velocity, at the cost of one extra spherecast or raycast.


## Distinguishing Stairs from Steep Slopes ##

\* *Steep slope* refers to ground with a slope angle that exceeds `maxSlopeAngle`.

Slope angle is determined by the dot product of the up vector and ground normal.

The original normal obtained by spherecast's `hit.normal` is not the actual surface normal of the hit point, but a direction vector relative to the centre of the sphere. This means when probing ground on the tip of a stair, the hit normal might be slanted, which causes the ground to be recognized as a slope. 

This can be problematic when we have a small slope angle limit. The tip of a stair may be erroneously considered as steep slope, which might prevent the character from moving up the stair depending on the movement implementation. To mitigate this issue, we have two solutions:

### Method 1. Verify Steep Slope ###

This is done by default, with `useStairProbing` set to true.
When ground probing detects we're on steep slope, `VerifySteepSlope` is called to perform a predictive raycast at a small offset from the ground point, away from collider center. The resulting hit point and the original ground point forms an angle. If this angle is still too steep, then we're on steep ground; Otherwise this is considered as the tip of a stair.

### Method 2. Calculate Real Ground Normal ###

Enable `useRealGroundNormal` to let ground probing calculate the real ground normal of the ground hit. A very short raycast is performed in the same direction against the ground collider to obtain the real ground normal.

This is not recommended, as it may produce incorrect results for scaled gameobjects.


# Examples #
NekoNekoMovement is designed as an alternative to Unity's CharacterController.

`CharacterMover` provides core features to support character movement. You can implement your own movement controllers on top of this.

```csharp
using NekoNeko.Movement
```

Usage example:

```csharp
private void FixedUpdate()
{
    Vector3 velocity = _desiredSpeed * _desiredDirection;
    Vector3 extraVelocity = Vector3.zero;

    CharacterMover.CheckGround();

    if(CharacterMover.IsGrounded)
    {
        // Update moving platform velocity cache.
        CharacterMover.UpdateConnectedBodyVelocity();
    } 
    else
    {   
        // Update and apply gravity.
        _currentFallSpeed = Mathf.Min(_currentFallSpeed + Mathf.Abs(gravity) * Time.deltaTime, _maxFallSpeed);
        extraVelocity.y += -_currentFallSpeed;

        // Update adjustment velocity to prevent slight ground penetration when landing.
        CharacterMover.PreventGroundPenetration(velocity, useRaycast: true);
    }

    // Apply cached moving platform velocity.
    extraVelocity += CharacterMover.ConnectionVelocity;

    velocity += extraVelocity;

    CharacterMover.Move(velocity);
}
```

If you want to align movement velocity to ground surface, see the example in [Snapping To Ground](#snapping-to-ground)
