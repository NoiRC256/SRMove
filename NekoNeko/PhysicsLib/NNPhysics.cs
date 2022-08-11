using UnityEngine;

namespace NekoNeko.PhysicsLib
{
    /// <summary>
    /// Physics library extensions.
    /// </summary>
    public static class NNPhysics
    {
        /// <summary>
        /// Calculate the actual surface normal instead of inverse hit normal for CapsuleCast and SphereCast hit.
        /// </summary>
        /// <param name="hit">original hit</param>
        /// <param name="dir">original direction of the raycast</param>
        /// <returns>correct normal</returns>
        /// <remarks>https://forum.unity.com/threads/spherecast-capsulecast-raycasthit-normal-is-not-the-surface-normal-as-the-documentation-states.275369/</remarks>
        public static Vector3 GetSurfaceNormal(this RaycastHit hit, Vector3 dir)
        {
            if (hit.collider == null)
            {
                return default;
            }

            if (hit.collider is MeshCollider)
            {
                var collider = hit.collider as MeshCollider;
                var mesh = collider.sharedMesh;
                var tris = mesh.triangles;
                var verts = mesh.vertices;

                var v0 = verts[tris[hit.triangleIndex * 3]];
                var v1 = verts[tris[hit.triangleIndex * 3 + 1]];
                var v2 = verts[tris[hit.triangleIndex * 3 + 2]];

                var n = Vector3.Cross(v1 - v0, v2 - v1).normalized;

                return hit.transform.TransformDirection(n);
            }
            else
            {
                RaycastHit result;
                hit.collider.Raycast(new Ray(hit.point - dir * 0.01f, dir), out result, 0.011f);
                return result.normal;
            }
        }
    }
}