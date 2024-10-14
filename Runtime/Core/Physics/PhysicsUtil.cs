using UnityEngine;

namespace CC.SRMove.Util
{
    public static partial class PhysicsUtil
    {
        public static bool LayerMaskContains(LayerMask layerMask, int layer)
        {
            return layerMask == (layerMask | 1 << layer);
        }
    }
}