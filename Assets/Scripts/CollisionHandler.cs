using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    public class CollisionHandler : MonoBehaviour
    {
        public void Setup(CapsuleCollider collision)
        {
            _collision = collision;
        }

        public int Sweep (Vector3 position, Vector3 direction, float distance, RaycastHit[] results, out RaycastHit closestHit, int layerMask = -1, System.Func<RaycastHit, bool> filter = null)
        {
            Vector3 size = _collision.bounds.size;
            return Sweep(position, Quaternion.identity, size.y, Mathf.Max(size.x, size.z) * 0.5f, direction, distance, results, out closestHit, layerMask, filter);
        }
        public int Sweep (Vector3 position, Quaternion rotation, float height, float radius, Vector3 direction, float distance, RaycastHit[] results, out RaycastHit closestHit, int layerMask = -1, System.Func<RaycastHit, bool> filter = null)
        {
            int ret = 0;
            closestHit = results[0];
            float closestDist = float.PositiveInfinity;

            Vector3 center = position + rotation * _collision.center;
            Vector3 anchor = rotation * ((0.5f * height - radius) * Vector3.up);
            int count = Physics.CapsuleCastNonAlloc(center + anchor, center - anchor,
                radius, direction, results, distance, layerMask, QueryTriggerInteraction.Ignore);

            for (int cur = 0; cur < count; ++cur)
            {
                ref RaycastHit hit = ref results[cur];

                if (filter != null && !filter(hit))
                {
                    continue;
                }
                // Recalculate overlaped collider's hitInfo because it has reversed normal
                if (hit.distance <= 0.0f)
                {
                    UPhysUtility.GetPosAndRot(hit.collider, out Vector3 hitPos, out Quaternion hitRot);

                    if (Physics.ComputePenetration(
                        _collision, position, rotation,
                        hit.collider, hitPos, hitRot,
                        out Vector3 dir, out float _))
                    {
                        hit.normal = dir;
                    }
                }
                // Ignore reversed normal 
                if (Vector3.Dot(direction, hit.normal) >= 0.0f)
                {
                    continue;
                }

                // Update to closer hitInfo
                if (hit.distance < closestDist)
                {
                    closestHit = hit;
                    closestDist = hit.distance;
                }
                results[ret++] = hit;
            }
            return ret;
        }

        public int Overlap (Vector3 position, Collider[] results, int layerMask = -1, System.Func<Collider, bool> filter = null)
        {
            Vector3 size = _collision.bounds.size;
            return Overlap(position, Quaternion.identity, size.y, Mathf.Max(size.x, size.z) * 0.5f, results, layerMask, filter);
        }

        public int Overlap (Vector3 position, Quaternion rotation, float height, float radius, Collider[] results, int layerMask = -1, System.Func<Collider, bool> filter = null)
        {
            Vector3 center = position + rotation * _collision.center;
            Vector3 anchor = rotation * ((0.5f * height - radius) * Vector3.up);
            int count = Physics.OverlapCapsuleNonAlloc(center + anchor, center - anchor,
                radius, results, layerMask, QueryTriggerInteraction.Ignore);

            int ret = 0;
            for (int cur = 0; cur < count; ++cur)
            {
                if (filter != null && !filter(results[cur]))
                {
                    continue;
                }
                results[ret++] = results[cur];
            }
            return ret;
        }

        private CapsuleCollider _collision;
    }
}