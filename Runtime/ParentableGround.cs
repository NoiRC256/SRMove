using UnityEngine;

namespace CC.SRMove
{
    public class ParentableGround : MonoBehaviour
    {
        public enum UpVectorMode
        {
            /// <summary>
            /// Always use world up as up vector.
            /// </summary>
            World,
            /// <summary>
            /// Maintain the up vector when initially parent to this ground. 
            /// </summary>
            Initial,
            /// <summary>
            /// Use the up vector of this ground.
            /// </summary>
            Parent,
            /// <summary>
            /// Use ground normal as up vector.
            /// </summary>
            Normal,
        }
        [field: SerializeField] public UpVectorMode Mode { get; private set; }
    }
}