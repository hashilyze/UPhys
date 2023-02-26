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
                GameObject instanceObject = new GameObject(kInstanceName);
                InitializeSingleton(instanceObject.AddComponent<UPhysSystem>());
            }

            return s_instance;
        }

        private const string kInstanceName = "UPhysSystem";
        private static UPhysSystem s_instance;


        private static void InitializeSingleton (UPhysSystem instance)
        {
            s_instance = instance;
            GameObject instanceObject = s_instance.gameObject;

            instanceObject.hideFlags = HideFlags.NotEditable;
            s_instance.hideFlags = HideFlags.NotEditable;

            if(instance.m_settings == null)
            {
                // Use default settings
                instance.m_settings = ScriptableObject.CreateInstance<UPhysSettings>();
            }

            DontDestroyOnLoad(instanceObject);
        }
        #endregion

        // Collect CCT needed simulate
        public static void RegisterCharacterMovement (UCharacterMovement movement)
        {
            if (!m_movements.Contains(movement)) m_movements.Add(movement);
        }
        public static void UnregisterCharacterMovement (UCharacterMovement movement)
        {
            m_movements.Remove(movement);
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
        public static UPhysSettings Settings => s_instance.m_settings;

        private static readonly List<UCharacterMovement> m_movements = new List<UCharacterMovement>();
        private static readonly List<UPlatformController> m_platforms = new List<UPlatformController>();
        [SerializeField] private UPhysSettings m_settings;

        private void Awake ()
        {
            if (s_instance == null)
            {
                InitializeSingleton(this);
            }
            else
            {
                if (s_instance != this)
                {
                    Destroy(this.gameObject);
                }
            }
        }

        private void FixedUpdate ()
        {   
            // Update Entities based on UPhys
            // Simulate
            {
                for (int cur = 0, count = m_platforms.Count; cur < count; ++cur)
                {
                    UPlatformController platform = m_platforms[cur];
                    platform.Simulate(Time.fixedDeltaTime);
                }
                for (int cur = 0, count = m_movements.Count; cur < count; ++cur)
                {
                    UCharacterMovement movement = m_movements[cur];
                    movement.Simulate(Time.fixedDeltaTime);
                }
            }
            // Simulate Commit
            {
                for (int cur = 0, count = m_platforms.Count; cur < count; ++cur)
                {
                    UPlatformController platform = m_platforms[cur];
                    platform.SimulateCommit();
                }
                for (int cur = 0, count = m_movements.Count; cur < count; ++cur)
                {
                    UCharacterMovement movement = m_movements[cur];
                    movement.SimulateCommit();
                }
            }
        }

    }
}