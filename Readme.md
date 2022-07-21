# Overview #
NekoNekoMovement is a rigidbody physics character movement solution.

`CharacterMover` provides methods for moving a character with a rigidbody and a capsule collider. It is designed as a rigidbody physics alternative to Unity's `CharacterController`.

This is useful if you want to handle acceleration, deceleration, friction, gravity yourself in a rigidbody physics-based movement approach, but also have functionalities like ground detection, stair and slope traversal, ground snapping, etc. handled for you in a lower-level MonoBehaviour. You also retain the ability to interact with the physics world with rigidbody physics.

### Key Features ###
- Ground detection
- Stair traversal
- Slope traversal
- Inherit moving platform velocity
- Intuitive capsule collider adjustment
- Rigidbody physics

### Environment ###

This was tested on Unity 2021.3.6f1, but technically works with any modern Unity version.

### Usage ###

- Add `CharacterMover` component to a gameobject with rigidbody and capsule collider, ideally with frictionless physics material.
- In your own movement controller, import `NekoSystems.Movement`.
- You can call methods from `CharacterMover` to implement your movement logic.

For examples, see [Examples](#examples)

# Implementation Details #

## Stair Traversal ##
NekoNekoMovement uses a floating capsule approach to handle stair traversal. The character's capsule collider constantly floats at a distance from ground to maintain a step height.

In a `CheckGround()` call, `CharacterMover` calculates and caches a ground adjustment velocity based on the detected ground distance. This ground adjustment velocity is used to maintain desired distance from ground in the next `Move(velocity)` call.

Note that since the ground adjustment velocity is often only computed at the beginning of a physics frame but applied to rigidbody velocity after some new movement velocity has been calculated, it actually lags behind one physics frame. When landing at a high speed, this may cause ground penetration for one frame. To mitigate this artifact, it is recommended to apply `PreventGroundPenetration(velocity)` before `Move(velocity)` to preemptively update ground adjustment velocity.

```csharp
private FixedUpdate()
{
    Vector3 velocity = ...
    ...
    CharacterMover.PreventGroundPenetration(velocity);
    CharacterMover.Move(velocity);
}
```

## Snap To Ground ##
The character is able to stick to ground when traversing across small obstacles, stepping down stairs, or going down slopes.
This is achieved by a combination of functionalities.

- Ground threshold distance can be extended by step height while grounded. This allows us to safely step down stairs while remaining grounded.
- Ground detection is achieved by both ground probing and direct collision. This ensures reliable ground detection on steep slopes, where ground threshold distance might be too short or spherecast too thin to reach ground but the collider itself is touching ground on one side.

For more reliable ground snapping, you can also align your velocity to ground surface before moving. For example:

```csharp
private FixedUpdate()
{
    // Lateral movement direction from input, on Vector3.up plane.
    Vector3 desiredDirection = ...
    ...
    if(CharacterMover.IsGrounded)
    {
        // Rotate direction to align with ground slope.
        Vector3 groundNormal = CharacterMover.GroundSurfaceNormal;
        Quaternion deltaAlign = Quaternion.FromToRotation(Vector3.up, CharacterMover.GroundSurfaceNormal);
        Vector3 groundAlignedDirection = (deltaAlign * desiredDirection).normalized;
        velocity = _speed * groundAlignedDirection;
    }
    ...

}
```

## Ground Probing and Real Ground Surface Normal Detection ##
NekoNeko Character Movement uses spherecast for ground probing. This is because using raycast might leave us vulnerable to misdetections due to small cracks and crevices on the ground. Spherecast ground probing provides more consistent results, and allows us to perch off ledges. 

In each `CheckGround()` call, we first check whether there are any existing direct collisions with ground. If there are, use those collisions to update ground information. Otherwise, we use ground probing to find ground. Ground probing performs a spherecast downwards for a fixed distance. If the hit distance is within ground threshold distance, then it is a ground. Ground threshold distance is derived from a combination of capsule halfheight, configured step height, and a small error threshold. 

It is also worth noting that ground probing obtains the real surface normal of the ground hit. Originally, the normal obtained by spherecast's `hit.normal` is not the actual surface normal of the hit point, but an inverse hit normal relative to the centre of the sphere. This means when probing ground on the tip of a stair, the hit normal might be slanted, which causes the ground to be recognized as a slope. This can be problematic when we have a small slope angle limit, as the tip of a stair may be erroneously considered as non-traversable slope. To mitigate this issue, we have two solutions:

1. Calculate real ground surface normal from spherecast hit. This is already done by default.
This is achieved by performing another very short raycast in the same direction against the hit collider, and using that hit normal.

2. When ground probing detects non-traversable slope, perform another ground probing by raycast, still from the collider center's height, but at a small offset from the detected ground point, towards movement direction. If this second ground probing returns a slope angle that is within slope limit, then this can be considered as a traversable stair.

Method 1 is always done by default in `CharacterMover`'s internal ground probing. This is usually sufficient.

If you need more customizable results, Method 2 can be implemented in your own movement controller. `CharacterMover` provides exposed variables and auxillary methods to support this.

Method 2 example:

``` csharp
private FixedUpdate()
{
    ...
    if(CharacterMover.IsOnSteepGround)
    {
        // Check whether the ground is a steep slope or the ground is just the tip of a stair.
        if(!FinsStairSurface(direction))
        {
            // Ground is actually a steep slope.
        }
    }
    ...
}

// Returns true if ground can be considered as a traversable stair, false if ground is a steep slope. 
public bool FindStairSurface(Vector3 direction)
{
    // Probe ground a bit forward from the last ground point.
    RaycastHit hit;
    bool willBeGrounded;
    float nextGroundDistance;
    CharacterMover.PredictGroundFromGroundpPointRaycast(CharacterMover.StepProbeOffset * direction, out hit, out willBeGrounded, out nextGroundDistance);

    if(!willBeGrounded)
    {
        return true;
    }
    float nextUpDotGround = Vector3.Dot(_upAxis, hit.normal);
    return !CharacterMover.IsGroundSteep(nextUpDotGround);
}
```

# Examples #

`CharacterMover` provides core features to support character movement. You can implement your own movement controllers on top of this.

Usage example:

```csharp
private void FixedUpdate()
{
    Vector3 velocity = _desiredSpeed * _desiredDirection;
    Vector3 extraVelocity = Vector3.zero;

    CharacterMover.CheckGround();

    if(CharacterMover.IsGrounded)
    {
        // Use extended ground threshold to snap to ground.
        CharacterMover.UseExtraGroundThresholdDistance = true;

        // Update moving platform's velocity, if any.
        CharacterMover.UpdateConnectedBody();
    } 
    else
    {   
        // Don't use extended ground threshold.
        CharacterMover.UseExtraGroundThresholdDistance = false;

        // Update and apply gravity.
        _currentFallSpeed = Mathf.Min(_currentFallSpeed + Mathf.Abs(gravity) * Time.deltaTime, _maxFallSpeed);
        extraVelocity.y += -_currentFallSpeed;
    }

    // Apply cached moving platform velocity.
    extraVelocity += CharacterMover.ConnectionVelocity;

    velocity += extraVelocity;

    CharacterMover.PreventGroundPenetration(velocity);
    CharacterMover.Move(velocity);
}
```


Currently, upon landing, `CharacterMover.ConnectionVelocity` may lag behind one frame due to how it's calculated. For optimal smoothness, you could implement logic that prevents applying connection velocity for one frame if we just landed in that frame.

(The same logic can also be used to fire ground contact changed events).

```csharp
private void FixedUpdate()
{
    ...
    CheckGround();

    if(CharacterMover.IsGrounded)
    {
        ...
        // Update moving platform's velocity, if any.
        CharacterMover.UpdateConnectedBody();
        if(!_groundContactChanged)
        {
            _lastConnectionVelocity = CharacterMover.ConnectionVelocity;
        }
    } 
    ...
    // Apply cached moving platform velocity.
    extraVelocity += _lastConnectionVelocity;
    ...
}

private void CheckGround()
{
    CharacterMover.CheckGround();
    if(CharacterMover.IsGrounded != _wasGrounded)
    {
        // Grounded state has changed.
        _groundContactChanged;
        // Some event.
        GroundContactChanged?.Invoke();
        _wasGrounded = CharacterMover.IsGrounded;
    }
}

```