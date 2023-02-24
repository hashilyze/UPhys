using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    public class UWarpZone : MonoBehaviour
    {
        public bool IsOutWarped
        {
            get { return m_isOutWarped; }
            set { m_isOutWarped = value; }
        }

        public void Warp(UCharacterController character)
        {
            if(!m_isLocked && !m_isOutWarped && m_outZone != null)
            {
                character.Teleport(m_outZone.transform.position);
                m_outZone.IsOutWarped = true;
            }
            m_isOutWarped = false;
        }


        [ReadOnly] [SerializeField] private bool m_isOutWarped;
        [SerializeField] private bool m_isLocked;
        [SerializeField] private UWarpZone m_outZone;
    }
}
