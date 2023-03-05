using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    public static class UPhysUtility 
    {
        public static void GetPosAndRot(Collider collider, out Vector3 pos, out Quaternion rot)
        {
            Transform colTransform = collider.transform;
            pos = colTransform.position;
            rot = colTransform.rotation;

            // Get changed transform if UPhys Object
            //Rigidbody overlapRigidbody = collider.attachedRigidbody;
            //if (overlapRigidbody != null)
            //{
            //    if (overlapRigidbody.isKinematic)
            //    {
            //        pos = overlapRigidbody.position;
            //        if (overlapRigidbody.gameObject != collider.gameObject)
            //        {
            //            pos += overlapRigidbody.rotation * collider.transform.localPosition;
            //        }
            //        rot = overlapRigidbody.rotation;
            //    }
            //    else
            //    {
            //        pos = overlapRigidbody.position;
            //        rot = overlapRigidbody.rotation;
            //    }
            //}
            //else
            //{
            //    Transform overlapTransform = collider.transform;
            //    pos = overlapTransform.position;
            //    rot = overlapTransform.rotation;
            //}
        }
    }
}