using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    public class UBoostZone : MonoBehaviour
    {
        public float BoostSpeed { get { return m_boostSpeed; } }

        public void Boost(UCharacterController character)
        {
            
        }

        [SerializeField] private float m_boostSpeed = 15.0f;
    }
}