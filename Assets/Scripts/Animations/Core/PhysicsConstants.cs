using UnityEngine;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Centralized physics constants used throughout the project
    /// Ensures consistency and makes tuning easier
    /// </summary>
    public static class PhysicsConstants
    {
        #region Gravity Constants
        /// <summary>Standard gravity acceleration (m/s²)</summary>
        public const float GRAVITY = 9.81f;
        
        /// <summary>Gravity vector pointing downward</summary>
        public static readonly Vector3 GRAVITY_VECTOR = new Vector3(0, -GRAVITY, 0);
        #endregion

        #region Collision Tolerances
        /// <summary>Minimum penetration depth to consider for collision resolution</summary>
        public const float COLLISION_TOLERANCE = 0.01f;
        
        /// <summary>Minimum distance to consider two objects separated</summary>
        public const float SEPARATION_THRESHOLD = 0.001f;
        
        /// <summary>Small epsilon for floating point comparisons</summary>
        public const float EPSILON = 1e-8f;
        
        /// <summary>Very small epsilon for near-zero checks</summary>
        public const float EPSILON_SMALL = 0.0001f;
        #endregion

        #region Default Material Properties
        /// <summary>Default coefficient of restitution (bounciness)</summary>
        public const float DEFAULT_RESTITUTION = 0.5f;
        
        /// <summary>Default friction coefficient</summary>
        public const float DEFAULT_FRICTION = 0.3f;
        
        /// <summary>Default linear damping (air resistance)</summary>
        public const float DEFAULT_LINEAR_DAMPING = 0.001f;
        
        /// <summary>Default angular damping (rotational resistance)</summary>
        public const float DEFAULT_ANGULAR_DAMPING = 0.01f;
        #endregion

        #region Simulation Settings
        /// <summary>Default physics time step</summary>
        public const float DEFAULT_TIME_STEP = 0.02f;
        
        /// <summary>Default number of solver iterations for constraint resolution</summary>
        public const int DEFAULT_SOLVER_ITERATIONS = 4;
        
        /// <summary>Default number of substeps per frame</summary>
        public const int DEFAULT_SUBSTEPS = 2;
        
        /// <summary>Separation factor to resolve penetration slightly more than needed</summary>
        public const float SEPARATION_FACTOR = 1.02f;
        #endregion

        #region Velocity Thresholds
        /// <summary>Minimum velocity magnitude before applying collision response</summary>
        public const float MIN_COLLISION_VELOCITY = 0.01f;
        
        /// <summary>Sleep threshold - objects below this velocity can be put to sleep</summary>
        public const float SLEEP_VELOCITY_THRESHOLD = 0.1f;
        #endregion
    }
}

