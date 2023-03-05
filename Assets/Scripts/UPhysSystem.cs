using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utility;

namespace UPhys
{
    public class UPhysSystem : SingletonComponent<UPhysSystem>
    {
        private const string kInstanceName = "UPhysSystem";
        private static readonly HashSet<UCharacterMovement> m_characters = new HashSet<UCharacterMovement>();
        private static readonly HashSet<UPlatformController> m_platforms = new HashSet<UPlatformController>();

        // Collect character needed simulate
        public static void RegisterCharacterMovement (UCharacterMovement movement)
        {
            m_characters.Add(movement);
        }
        public static void UnregisterCharacterMovement (UCharacterMovement movement)
        {
            m_characters.Remove(movement);
        }
        // Collect platform needed simulate
        public static void RegisterPlatformController (UPlatformController platform)
        {
            m_platforms.Add(platform);
        }
        public static void UnregisterPlatformController (UPlatformController platform)
        {
            m_platforms.Remove(platform);
        }

        private void FixedUpdate ()
        {
            float deltaTime = Time.fixedDeltaTime;
            if(!CanSimulate(deltaTime))
            {
                return;
            }

            // Update Entities based on UPhys
            // Simulate
            {
                foreach(UPlatformController platform in m_platforms)
                {
                    platform.Simulate(deltaTime);
                }
                foreach(UCharacterMovement character in m_characters)
                {
                    character.Simulate(deltaTime);
                }
            }
            // Simulate Commit
            {
                foreach (UPlatformController platform in m_platforms)
                {
                    platform.SimulateCommit();
                }
                foreach (UCharacterMovement character in m_characters)
                {
                    character.SimulateCommit();
                }
            }
        }
        private bool CanSimulate(float deltaTime)
        {
            return deltaTime > 0.0f;
        }

        // Singleton Override
        protected override void InitializeInstance ()
        {
            GameObject instanceObject = this.gameObject;
            instanceObject.name = kInstanceName;

            instanceObject.hideFlags = HideFlags.NotEditable;
            this.hideFlags = HideFlags.NotEditable;
        }
    }
}