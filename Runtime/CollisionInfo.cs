using UnityEngine;

public struct CollisionInfo
{
    public static CollisionInfo Empty => new CollisionInfo(false, false, false, Vector3.zero, Vector3.up);
    public bool IsTouchingGround;
    public bool IsTouchingWall;
    public bool IsTouchingCeiling;
    public Vector3 GroundPoint;
    public Vector3 GroundNormal;

    public CollisionInfo(bool isTouchingGround, bool isTouchingWall, bool isTouchingCeiling, Vector3 groundPoint, Vector3 groundNormal)
    {
        IsTouchingGround = isTouchingGround;
        IsTouchingWall = isTouchingWall;
        IsTouchingCeiling = isTouchingCeiling;
        GroundPoint = groundPoint;
        GroundNormal = groundNormal;
    }
}
