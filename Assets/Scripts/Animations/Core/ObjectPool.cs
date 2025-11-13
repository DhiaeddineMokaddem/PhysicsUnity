using UnityEngine;
using System.Collections.Generic;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Generic object pool for reducing allocations and improving performance
    /// Useful for particle systems, projectiles, and temporary physics objects
    /// </summary>
    /// <typeparam name="T">Type of object to pool (must be a Component)</typeparam>
    public class ObjectPool<T> where T : Component
    {
        #region Private Fields
        private readonly T prefab;
        private readonly Transform parent;
        private readonly Queue<T> availableObjects = new Queue<T>();
        private readonly List<T> allObjects = new List<T>();
        private readonly int initialSize;
        private readonly bool expandable;
        #endregion

        #region Constructor
        /// <summary>
        /// Creates a new object pool
        /// </summary>
        /// <param name="prefab">Prefab to instantiate</param>
        /// <param name="initialSize">Initial pool size</param>
        /// <param name="parent">Parent transform for pooled objects</param>
        /// <param name="expandable">Can the pool grow beyond initial size?</param>
        public ObjectPool(T prefab, int initialSize = 10, Transform parent = null, bool expandable = true)
        {
            this.prefab = prefab;
            this.initialSize = initialSize;
            this.parent = parent;
            this.expandable = expandable;

            // Pre-populate the pool
            for (int i = 0; i < initialSize; i++)
            {
                CreateNewObject();
            }
        }
        #endregion

        #region Public Methods
        /// <summary>
        /// Get an object from the pool
        /// </summary>
        public T Get()
        {
            T obj;
            
            if (availableObjects.Count > 0)
            {
                obj = availableObjects.Dequeue();
            }
            else if (expandable)
            {
                obj = CreateNewObject();
            }
            else
            {
                // Pool is exhausted and not expandable, reuse oldest
                Debug.LogWarning($"ObjectPool<{typeof(T).Name}> exhausted. Consider increasing pool size.");
                return null;
            }

            obj.gameObject.SetActive(true);
            return obj;
        }

        /// <summary>
        /// Return an object to the pool
        /// </summary>
        public void Return(T obj)
        {
            if (obj == null) return;

            obj.gameObject.SetActive(false);
            
            if (!availableObjects.Contains(obj))
            {
                availableObjects.Enqueue(obj);
            }
        }

        /// <summary>
        /// Return all active objects to the pool
        /// </summary>
        public void ReturnAll()
        {
            foreach (var obj in allObjects)
            {
                if (obj != null && obj.gameObject.activeSelf)
                {
                    Return(obj);
                }
            }
        }

        /// <summary>
        /// Get the total size of the pool (active + inactive)
        /// </summary>
        public int TotalSize => allObjects.Count;

        /// <summary>
        /// Get the number of available objects
        /// </summary>
        public int AvailableCount => availableObjects.Count;

        /// <summary>
        /// Get the number of active objects
        /// </summary>
        public int ActiveCount => TotalSize - AvailableCount;

        /// <summary>
        /// Clear and destroy all pooled objects
        /// </summary>
        public void Clear()
        {
            foreach (var obj in allObjects)
            {
                if (obj != null)
                {
                    Object.Destroy(obj.gameObject);
                }
            }
            
            availableObjects.Clear();
            allObjects.Clear();
        }
        #endregion

        #region Private Methods
        private T CreateNewObject()
        {
            T obj = Object.Instantiate(prefab, parent);
            obj.gameObject.SetActive(false);
            allObjects.Add(obj);
            availableObjects.Enqueue(obj);
            return obj;
        }
        #endregion
    }

    /// <summary>
    /// Static manager for managing multiple object pools
    /// </summary>
    public static class ObjectPoolManager
    {
        private static Dictionary<string, object> pools = new Dictionary<string, object>();

        /// <summary>
        /// Get or create a pool for a specific prefab
        /// </summary>
        public static ObjectPool<T> GetPool<T>(T prefab, int initialSize = 10, Transform parent = null) where T : Component
        {
            string key = prefab.GetType().Name + "_" + prefab.GetInstanceID();
            
            if (!pools.ContainsKey(key))
            {
                pools[key] = new ObjectPool<T>(prefab, initialSize, parent);
            }

            return pools[key] as ObjectPool<T>;
        }

        /// <summary>
        /// Clear all pools
        /// </summary>
        public static void ClearAllPools()
        {
            foreach (var pool in pools.Values)
            {
                // Clear each pool properly
                var poolType = pool.GetType();
                var clearMethod = poolType.GetMethod("Clear");
                clearMethod?.Invoke(pool, null);
            }
            pools.Clear();
        }
    }
}
