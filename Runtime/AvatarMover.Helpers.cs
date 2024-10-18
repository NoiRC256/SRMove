using UnityEngine;

namespace CC.SRMove
{
    public partial class AvatarMover : MonoBehaviour
    {
        #region Ground Parent

        private void ParentToGround(Collider collider)
        {
            if (collider.transform.TryGetComponent(out ParentableGround groundParent))
            {
                transform.SetParent(groundParent.transform, worldPositionStays: true);
                _up = transform.up;
                IsParentedToGround = true;
                GroundParent = groundParent;
            }
            else
            {
                UnparentFromGround();
            }
        }

        private void UnparentFromGround()
        {
            Vector3 forward = Vector3.ProjectOnPlane(transform.forward, Vector3.up);
            transform.SetParent(null, worldPositionStays: true);
            transform.up = Vector3.up;
            transform.forward = forward;
            _up = Vector3.up;
            IsParentedToGround = false;
            GroundParent = null;
        }

        #endregion

        #region Utility Functions

        /// <summary>
        /// Align velocity to a plane defined by the specified plane normal.
        /// </summary>
        /// <param name="velocity"></param>
        /// <param name="normal"></param>
        /// <returns></returns>
        private Vector3 AlignVelocityToPlane(Vector3 velocity, Vector3 normal, float ratio = 1f)
        {
            float speed = velocity.magnitude;
            Vector3 direction = velocity / speed;
            Vector3 alignedDirection = Quaternion.FromToRotation(_up, normal) * direction;
            alignedDirection = Vector3.Lerp(direction, alignedDirection, ratio);
            return speed * alignedDirection.normalized;
        }

        private bool LayerMaskContains(LayerMask layerMask, int layer)
        {
            return layerMask == (layerMask | (1 << layer));
        }

        #endregion

    }
}