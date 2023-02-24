using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    [SerializeField] private Transform m_target;
    [SerializeField] private float m_springArmDist = 10.0f;
    [SerializeField] private float m_springArmDeg = 30.0f;

    private void LateUpdate ()
    {
        Vector3 cameraPos
            = m_target.position
            + Vector3.back * m_springArmDist * Mathf.Cos(m_springArmDeg * Mathf.Deg2Rad)
            + Vector3.up * m_springArmDist * Mathf.Sin(m_springArmDeg * Mathf.Deg2Rad);

        transform.position = cameraPos;
    }
}
