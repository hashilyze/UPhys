using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace UPhys
{
    [System.Serializable]
    public struct InputSet
    {
        public Vector2 moveAxis;
    }

    public class UInputManager : MonoBehaviour
    {
        #region Singleton
        public static UInputManager GetInstance ()
        {
            if (s_instance == null)
            {
                s_instance = FindObjectOfType<UInputManager>();
                if (s_instance != null)
                {
                    DontDestroyOnLoad(s_instance.gameObject);
                }
            }
            return s_instance;
        }

        private static UInputManager s_instance = null;
        #endregion

        public static void RegisterPlayer (UCharacterController player)
        {
            if (!m_players.Contains(player)) m_players.Add(player);
        }
        public static void UnregisterPlayer (UCharacterController player)
        {
            m_players.Remove(player);
        }


        public void OnMove(InputAction.CallbackContext ctx)
        {
            m_inputSet.moveAxis = ctx.ReadValue<Vector2>();
        }
        public void OnJump(InputAction.CallbackContext ctx)
        {
            switch (ctx.phase)
            {
            case InputActionPhase.Started:
                for (int cur = 0, cnt = m_players.Count; cur < cnt; ++cur)
                {
                    UCharacterController player = m_players[cur];
                    player.DoJump();
                }
                break;
            case InputActionPhase.Canceled:
                for (int cur = 0, cnt = m_players.Count; cur < cnt; ++cur)
                {
                    UCharacterController player = m_players[cur];
                    player.StopJump();
                }
                break;
            }
            //ctx.ReadValueAsButton();
        }

        private static readonly List<UCharacterController> m_players = new List<UCharacterController>();
        public InputSet m_inputSet;


        private void Awake ()
        {
            if (s_instance == null)
            {
                s_instance = this;
                s_instance.gameObject.hideFlags = HideFlags.NotEditable;
                s_instance.hideFlags = HideFlags.NotEditable;

                DontDestroyOnLoad(s_instance.gameObject);
            }
            else
            {
                if (s_instance != this)
                {
                    Destroy(this.gameObject);
                }
            }
        }

        private void Update ()
        {
            //m_inputSet.moveAxis = Vector2.left;
            for (int cur = 0, cnt = m_players.Count; cur < cnt; ++cur)
            {
                UCharacterController player = m_players[cur];
                player.SetPlayerInput(m_inputSet);
            }
        }
    }
}