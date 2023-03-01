using UnityEngine;

namespace Utility
{
    public class SingletonScriptableObject<T> : ScriptableObject where T : SingletonScriptableObject<T>
    {
        private static T _instance;
        public static T Instance
        {
            get
            {
                if(_instance == null)
                {
                    T[] assets = Resources.LoadAll<T>("");
                    if(assets == null || assets.Length < 1)
                    {
                        Debug.LogError("Could not find any scriptable object instance");
                    } else if (assets.Length > 1)
                    {
                        Debug.LogWarning("There are mutilple intances");
                    }
                    _instance = assets[0];
                }
                return _instance;
            }
        }
    }
}