using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utility;

namespace UPhys
{
    public class UPhysSystem : SingletonComponent<UPhysSystem>
    {
        #region Singleton 
        private const string kInstanceName = "UPhysSystem";

        protected override void InitializeInstance ()
        {
            GameObject instanceObject = this.gameObject;
            instanceObject.name = kInstanceName;

            instanceObject.hideFlags = HideFlags.NotEditable;
            this.hideFlags = HideFlags.NotEditable;
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

        private static readonly List<UCharacterMovement> m_movements = new List<UCharacterMovement>();
        private static readonly List<UPlatformController> m_platforms = new List<UPlatformController>();

        private void FixedUpdate ()
        {
            float deltaTime = Time.fixedDeltaTime;
            if(deltaTime < 0.0f)
            {
                return;
            }

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