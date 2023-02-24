using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    public class UBounceZone : MonoBehaviour
    {
        public Vector3 BounceForce { get { return m_bounceForce; } }

        [SerializeField] private Vector3 m_bounceForce;
    }
}