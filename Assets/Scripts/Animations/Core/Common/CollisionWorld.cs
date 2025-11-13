
using System.Collections.Generic;
using PhysicsSimulation.Indiv_Work.Aziz;
using UnityEngine;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Lightweight shared registry for collision participants and a single CollisionDetector.
    /// Pure math friendly: no Unity physics; just references and lists.
    /// </summary>
    [DefaultExecutionOrder(-200)]
    public class CollisionWorld : MonoBehaviour
    {
        public static CollisionWorld Instance { get; private set; }

        [Tooltip("Optional explicit CollisionDetector. If not set, will search in scene on Awake.")]
        public CollisionDetector collisionDetector;

        private readonly List<RigidBody3D> _bodies = new List<RigidBody3D>();
        public IReadOnlyList<RigidBody3D> Bodies => _bodies;

        void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(this);
                return;
            }
            Instance = this;

            if (collisionDetector == null)
                collisionDetector = FindObjectOfType<CollisionDetector>();

            RefreshBodies();
        }

        /// <summary>
        /// Rebuild the registry from scene objects.
        /// </summary>
        public void RefreshBodies()
        {
            _bodies.Clear();
            var found = FindObjectsOfType<RigidBody3D>();
            if (found != null && found.Length > 0)
                _bodies.AddRange(found);
        }

        public void Register(RigidBody3D body)
        {
            if (body != null && !_bodies.Contains(body))
                _bodies.Add(body);
        }

        public void Unregister(RigidBody3D body)
        {
            if (body != null)
                _bodies.Remove(body);
        }

        public bool TryGetDetector(out CollisionDetector detector)
        {
            detector = collisionDetector;
            if (detector == null)
            {
                detector = FindObjectOfType<CollisionDetector>();
                collisionDetector = detector;
            }
            return detector != null;
        }
    }
}

