// Import Unity's Vector3 for gravity vector constant
using UnityEngine;

// Namespace for core physics simulation utilities
namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Centralized physics constants used throughout the project
    /// Ensures consistency and makes tuning easier
    /// </summary>
    public static class PhysicsConstants
    {
        #region Gravity Constants
        /// <summary>Standard gravity acceleration on Earth in meters per second squared (m/s²)</summary>
        // This is the acceleration due to gravity at Earth's surface
        public const float GRAVITY = 9.81f;
        
        /// <summary>Gravity vector pointing downward along the Y-axis (world space)</summary>
        // Negative Y direction represents downward in Unity's coordinate system
        public static readonly Vector3 GRAVITY_VECTOR = new Vector3(0, -GRAVITY, 0);
        #endregion

        #region Collision Tolerances
        /// <summary>Minimum penetration depth in meters to consider for collision resolution</summary>
        // Objects penetrating less than this are considered touching but not colliding
        public const float COLLISION_TOLERANCE = 0.01f;
        
        /// <summary>Minimum distance in meters to consider two objects separated</summary>
        // Used to determine if objects are just touching or actually separated
        public const float SEPARATION_THRESHOLD = 0.001f;
        
        /// <summary>Small epsilon value for floating point comparisons to avoid division by zero</summary>
        // Use this when checking if a value is effectively zero (within numerical precision)
        public const float EPSILON = 1e-8f;
        
        /// <summary>Very small epsilon for near-zero checks with less strict tolerance</summary>
        // Use this for cases where exact zero is not required but close to zero is sufficient
        public const float EPSILON_SMALL = 0.0001f;
        #endregion

        #region Default Material Properties
        /// <summary>Default coefficient of restitution - controls bounciness (0 = no bounce, 1 = perfectly elastic)</summary>
        // 0.5 means object bounces back to 50% of its original velocity
        public const float DEFAULT_RESTITUTION = 0.5f;
        
        /// <summary>Default friction coefficient - controls sliding resistance (0 = frictionless, higher = more friction)</summary>
        // Typical values: ice ~0.03, wood ~0.3-0.5, rubber ~0.9
        public const float DEFAULT_FRICTION = 0.3f;
        
        /// <summary>Default linear damping - simulates air resistance on linear motion (higher = more damping)</summary>
        // Applied as: velocity *= (1 - damping * deltaTime)
        public const float DEFAULT_LINEAR_DAMPING = 0.001f;
        
        /// <summary>Default angular damping - simulates air resistance on rotation (higher = more damping)</summary>
        // Applied as: angularVelocity *= (1 - damping * deltaTime)
        public const float DEFAULT_ANGULAR_DAMPING = 0.01f;
        #endregion

        #region Simulation Settings
        /// <summary>Default physics time step in seconds - how often physics is updated</summary>
        // 0.02 seconds = 50 updates per second (50 Hz)
        public const float DEFAULT_TIME_STEP = 0.02f;
        
        /// <summary>Default number of solver iterations for constraint resolution</summary>
        // More iterations = more accurate but slower simulation
        public const int DEFAULT_SOLVER_ITERATIONS = 4;
        
        /// <summary>Default number of substeps per frame - divides time step for stability</summary>
        // More substeps = more stable but slower simulation
        public const int DEFAULT_SUBSTEPS = 2;
        
        /// <summary>Separation factor to resolve penetration slightly more than needed</summary>
        // Factor > 1.0 prevents objects from re-penetrating immediately after separation
        // 1.02 means separate objects 2% more than the penetration depth
        public const float SEPARATION_FACTOR = 1.02f;
        #endregion

        #region Velocity Thresholds
        /// <summary>Minimum velocity magnitude in m/s before applying collision response</summary>
        // Collisions with relative velocity below this are ignored to prevent jitter
        public const float MIN_COLLISION_VELOCITY = 0.01f;
        
        /// <summary>Sleep threshold in m/s - objects below this velocity can be put to sleep for optimization</summary>
        // Sleeping objects don't update physics until disturbed by active objects
        public const float SLEEP_VELOCITY_THRESHOLD = 0.1f;
        #endregion
    }
}

