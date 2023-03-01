using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using Utility;

namespace UPhys
{
    [System.Serializable]
    public struct InputSet
    {
        public Vector2 moveAxis;
    }

    public class InputManager : SingletonComponent<InputManager>
    {
        #region Singleton
        protected override void InitializeInstance () 
        {
            this.hideFlags = HideFlags.NotEditable;
            this.gameObject.hideFlags = HideFlags.NotEditable;
        }
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
        }

        private static readonly List<UCharacterController> m_players = new List<UCharacterController>();
        public InputSet m_inputSet;

        private void Update ()
        {
            for (int cur = 0, cnt = m_players.Count; cur < cnt; ++cur)
            {
                UCharacterController player = m_players[cur];
                player.SetPlayerInput(m_inputSet);
            }
        }
    }
}