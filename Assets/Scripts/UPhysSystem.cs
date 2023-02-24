using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    public class UPhysSystem : MonoBehaviour
    {
        #region Singleton 
        public static UPhysSystem GetInstance ()
        {
            // If have not instance, generate to ensure exsitence of instance
            if (s_instance == null)
            {
                GameObject instanceObject = new GameObject(InstanceName);
                s_instance = instanceObject.AddComponent<UPhysSystem>();

                instanceObject.hideFlags = HideFlags.NotEditable;
                s_instance.hideFlags = HideFlags.NotEditable;

                DontDestroyOnLoad(instanceObject);
            }

            return s_instance;
        }

        private const string InstanceName = "UPhysSystem";
        private static UPhysSystem s_instance;
        #endregion

        // Collect CCT needed simulate
        public static void RegisterCharacterController (UCharacterController character)
        {
            if (!m_characters.Contains(character)) m_characters.Add(character);
        }
        public static void UnregisterCharacterController (UCharacterController character)
        {
            m_characters.Remove(character);
        }
        // Collect ACT needed simulate
        public static void RegisterPlatformController (UPlatformController platform)
        {
            if (!m_platforms.Contains(platform)) m_platforms.Add(platform);
        }
        public static void UnregisterPlatformController (UPlatformController platform)
        {
            m_platforms.Remove(platform);
        }

        private static readonly List<UCharacterController> m_characters = new List<UCharacterController>();
        private static readonly List<UPlatformController> m_platforms = new List<UPlatformController>();


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
        /// <summary>
        /// Entities based on UPhys simulated by UPhysSystem not itself
        /// </summary>
        private void FixedUpdate ()
        {
            // Simulate
            {
                for (int cur = 0, cnt = m_platforms.Count; cur < cnt; ++cur)
                {
                    UPlatformController platform = m_platforms[cur];
                    platform.Simulate(Time.fixedDeltaTime);
                }

                for (int cur = 0, cnt = m_characters.Count; cur < cnt; ++cur)
                {
                    UCharacterController character = m_characters[cur];
                    character.Simulate(Time.fixedDeltaTime);
                }
            }

            // Simulate Commit
            {
                for (int cur = 0, cnt = m_characters.Count; cur < cnt; ++cur)
                {
                    UCharacterController character = m_characters[cur];
                    character.SimulateCommit();
                }
                for (int cur = 0, cnt = m_platforms.Count; cur < cnt; ++cur)
                {
                    UPlatformController platform = m_platforms[cur];
                    platform.SimulateCommit();
                }
            }
        }

    }
}