using System.Collections.Generic;
using UnityEngine;

namespace CCLab.SRMove
{
    public class CollisionStore
    {
        private List<Collision> _collisions = new List<Collision>();

        private bool _isTouchingGround = false;
        private bool _isTouchingCeiling = false;
        private Vector3 _groundNormal;
        private float _groundHeight;

        public void Add(Collision collision)
        {
           _collisions.Add(collision);
        }

        public void Remove(Collision collision)
        {
           _collisions.Remove(collision);
        }

        public void OnUpdate(float minGroundAngleDot)
        {
            _isTouchingGround = false;
            _isTouchingCeiling = false;
            _groundNormal = Vector3.up;
            _groundHeight = 0f;

            int groundCollisionCount = 0;
            Vector3 accGroundCollisionNormal = Vector3.zero;
            for (int collisionId = 0; collisionId < _collisions.Count; collisionId++)
            {
                Collision collision = _collisions[collisionId];
                bool isGround = false;
                int groundContactCount = 0;
                Vector3 accGroundContactNormal = Vector3.zero;
                for (int contactId = 0; contactId < collision.contactCount; contactId++)
                {
                    ContactPoint contact = collision.GetContact(contactId);
 
                    // If is ground contact.
                    if (contact.normal.y > minGroundAngleDot + 0.001f)
                    {
                        isGround = true;
                        _isTouchingGround = true;
                        accGroundContactNormal = contact.normal;
                        groundContactCount++;
                        _groundHeight = _groundHeight * (groundContactCount - 1) / groundContactCount
                            + (contact.point.y / groundContactCount);
                    }
                    // If is ceiling contact.
                    else if (contact.normal.y < -0.001f)
                    {
                        _isTouchingCeiling = true;
                    }
                }
                if(isGround)
                {
                    accGroundCollisionNormal += (accGroundContactNormal / (float)groundContactCount);
                    groundCollisionCount++;
                }
            }

            if (_isTouchingGround) _groundNormal = accGroundCollisionNormal / (float)groundCollisionCount;
        }

        public void Clear()
        {
            _collisions.Clear();
        }
    }
}