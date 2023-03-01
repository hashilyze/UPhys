using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Utility
{
    public abstract class SingletonComponent<T> : MonoBehaviour where T : SingletonComponent<T>
    {
        private static T m_instance;
        private static bool m_isShutDown = false;
        private static readonly object m_lock = new object();

        public static T Instance
        {
            get
            {
                if (m_isShutDown)
                {
                    return null;
                }

                EnsureInstance(m_instance);

                return m_instance;
            }
        }

        private static void EnsureInstance (T instance)
        {
            lock (m_lock)
            {
                if (m_instance == null)
                {
                    // Set given instance to singleton
                    m_instance = instance;

                    if (m_instance == null)
                    {
                        // Search Instance
                        m_instance = FindObjectOfType<T>();
                    }

                    if (m_instance == null)
                    {
                        // Create Instance
                        GameObject singletonObject = new GameObject();
                        m_instance = singletonObject.AddComponent<T>();
                    }

                    m_instance.InitializeInstance();
                    DontDestroyOnLoad(m_instance.gameObject);
                }
                else
                {
                    // Ensure only one instance in scene
                    if (m_instance != instance)
                    {
                        Destroy(instance);
                    }
                }
            }
        }


        protected virtual void Awake ()
        {
            EnsureInstance((T)this);
        }

        private void OnApplicationQuit ()
        {
            m_isShutDown = true;
        }
        private void OnDestroy ()
        {
            m_isShutDown = true;
        }


        protected abstract void InitializeInstance ();
    }
}