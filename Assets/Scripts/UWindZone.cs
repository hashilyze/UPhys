using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    public class UWindZone : MonoBehaviour
    {
        public Vector3 GetWindVelocity (Vector3 position, Vector3 velocity)
        {
            float dist = Mathf.Abs(Vector3.Dot(m_sourcePosition + transform.position - position, m_windDirection));
            if(dist > m_edgeLength)
            {
                float characterSpeed = Vector3.Dot(velocity, m_windDirection);
                if(characterSpeed < 0.0f)
                {
                    return -characterSpeed * m_windDirection;
                }
            }


             return m_windDirection * Mathf.Lerp(m_sourcePower, m_edgePower, dist / m_edgeLength);
        }

        [SerializeField] private Vector3 m_sourcePosition;
        [SerializeField] private Vector3 m_windDirection;
        [SerializeField] private float m_sourcePower;
        [SerializeField] private float m_edgePower;
        [SerializeField] private float m_weakness;
        [SerializeField] private float m_edgeLength;
        
    }
}