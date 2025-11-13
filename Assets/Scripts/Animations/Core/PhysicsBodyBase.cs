using UnityEngine;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Base class for all physics bodies in the system
    /// Provides common functionality and interface for different physics implementations
    /// </summary>
    public abstract class PhysicsBodyBase : MonoBehaviour
    {
        #region Common Properties
        [Header("Physics Properties")]
        [SerializeField] protected float mass = 1.0f;
        [SerializeField] protected bool useGravity = true;
        [SerializeField] protected bool isKinematic = false;

        public float Mass { get => mass; set => mass = Mathf.Max(PhysicsConstants.EPSILON, value); }
        public bool UseGravity { get => useGravity; set => useGravity = value; }
        public bool IsKinematic { get => isKinematic; set => isKinematic = value; }
        
        public abstract Vector3 Velocity { get; set; }
        public abstract Vector3 Position { get; set; }
        public abstract Quaternion Rotation { get; set; }
        #endregion

        #region Abstract Methods
        /// <summary>
        /// Apply a force to the body
        /// </summary>
        public abstract void AddForce(Vector3 force);

        /// <summary>
        /// Apply an impulse to the body
        /// </summary>
        public abstract void AddImpulse(Vector3 impulse);

        /// <summary>
        /// Integrate physics for one timestep
        /// </summary>
        public abstract void IntegratePhysics(float deltaTime);
        #endregion

        #region Common Utility Methods
        /// <summary>
        /// Get the inverse mass (0 if kinematic or infinite mass)
        /// </summary>
        public float GetInverseMass()
        {
            return isKinematic ? 0f : 1f / mass;
        }

        /// <summary>
        /// Check if this body can move
        /// </summary>
        public bool CanMove()
        {
            return !isKinematic && mass > PhysicsConstants.EPSILON;
        }

        /// <summary>
        /// Apply gravity force if enabled
        /// </summary>
        protected void ApplyGravityForce()
        {
            if (useGravity && !isKinematic)
            {
                AddForce(mass * PhysicsConstants.GRAVITY_VECTOR);
            }
        }
        #endregion
    }
}

