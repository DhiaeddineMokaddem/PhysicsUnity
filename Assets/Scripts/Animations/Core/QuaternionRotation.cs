// Import Unity's core engine functionality
using UnityEngine;

// Namespace for core physics simulation utilities
namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Quaternion-based rotation implementation
    /// Provides stable rotation without gimbal lock
    /// Uses VisualRenderer for visual updates only - no direct transform manipulation for rotation
    /// </summary>
    public class QuaternionRotation : MonoBehaviour
    {
        #region Configuration
        [Header("Rotation Properties")] // UI header in the Unity Inspector
        // Angular velocity vector in radians per second - controls rotation speed and axis
        [SerializeField] private Vector3 angularVelocity = Vector3.zero;
        // Damping factor to gradually slow down rotation over time (0 = no damping, 1 = instant stop)
        [SerializeField] private float angularDamping = 0.01f;
        // If true, applies rotation in local space; if false, applies in world space
        [SerializeField] private bool useLocalSpace = false;
        
        [Header("Torque")] // UI header in the Unity Inspector for torque settings
        // External torque applied each frame to create rotational force
        [SerializeField] private Vector3 appliedTorque = Vector3.zero;
        // Maximum angular speed limit to prevent excessive rotation
        [SerializeField] private float maxAngularSpeed = 10f;
        #endregion

        #region Private Fields
        // Current rotation stored as a quaternion for gimbal-lock-free rotation
        private Quaternion currentRotation;
        // Torque accumulated from AddTorque calls during this frame
        private Vector3 accumulatedTorque = Vector3.zero;
        // Reference to the visual renderer that updates mesh vertices manually
        private VisualRenderer visualRenderer;
        #endregion

        #region Unity Lifecycle
        // Called once when the script instance is being loaded
        void Start()
        {
            // Try to get existing VisualRenderer component attached to this GameObject
            visualRenderer = GetComponent<VisualRenderer>();
            // If no VisualRenderer exists, add one to handle manual mesh transformation
            if (visualRenderer == null)
            {
                visualRenderer = gameObject.AddComponent<VisualRenderer>();
            }
            // Initialize current rotation from the visual renderer's stored rotation
            currentRotation = visualRenderer.GetRotation();
        }

        // Called every fixed framerate frame (typically 50 times per second)
        void FixedUpdate()
        {
            // Integrate rotation physics using the fixed time step
            IntegrateRotation(Time.fixedDeltaTime);
        }
        #endregion

        #region Rotation Integration
        // Performs one physics step of rotation integration
        private void IntegrateRotation(float deltaTime)
        {
            // Combine manually applied torque with any accumulated torque from AddTorque calls
            Vector3 totalTorque = appliedTorque + accumulatedTorque;
            
            // Update angular velocity by applying torque over time (angular acceleration)
            angularVelocity += totalTorque * deltaTime;
            
            // Apply damping to gradually slow down rotation (simulates air resistance)
            angularVelocity = IntegrationUtils.ApplyDamping(angularVelocity, angularDamping, deltaTime);
            
            // Clamp angular velocity magnitude to prevent spinning too fast
            angularVelocity = MathUtils.ClampMagnitude(angularVelocity, maxAngularSpeed);
            
            // Convert angular velocity to appropriate space (local or world)
            // If local space, transform the angular velocity vector by current rotation
            Vector3 rotationAxis = useLocalSpace ? TransformUtils.TransformDirection(angularVelocity, currentRotation) : angularVelocity;
            
            // Integrate the rotation using quaternion math to update current rotation
            // This applies the angular velocity over the time step
            currentRotation = IntegrationUtils.IntegrateRotationQuaternion(
                currentRotation,     // Current rotation state
                rotationAxis,        // Axis and magnitude of rotation
                deltaTime           // Time step for integration
            );
            
            // Update the visual representation by transforming mesh vertices manually
            if (visualRenderer != null)
            {
                visualRenderer.UpdateRotation(currentRotation);
            }
            
            // Reset accumulated torque to zero for the next frame
            accumulatedTorque = Vector3.zero;
        }
        #endregion

        #region Public Methods
        /// <summary>
        /// Add torque to the object
        /// </summary>
        public void AddTorque(Vector3 torque, bool isLocalSpace = false)
        {
            // If torque is in local space, transform it to world space using current rotation
            if (isLocalSpace)
                torque = TransformUtils.TransformDirection(torque, currentRotation);
            
            // Add the torque to accumulated torque (will be applied in next IntegrateRotation call)
            accumulatedTorque += torque;
        }

        /// <summary>
        /// Set angular velocity directly
        /// </summary>
        public void SetAngularVelocity(Vector3 velocity, bool isLocalSpace = false)
        {
            // If velocity is in local space, transform it to world space using current rotation
            if (isLocalSpace)
                velocity = TransformUtils.TransformDirection(velocity, currentRotation);
            
            // Directly set the angular velocity (replaces current value)
            angularVelocity = velocity;
        }

        /// <summary>
        /// Get current angular velocity
        /// </summary>
        public Vector3 GetAngularVelocity(bool inLocalSpace = false)
        {
            // Return angular velocity in world space or transform to local space if requested
            return inLocalSpace ? TransformUtils.InverseTransformDirection(angularVelocity, currentRotation) : angularVelocity;
        }

        /// <summary>
        /// Stop all rotation
        /// </summary>
        public void Stop()
        {
            // Set angular velocity to zero to stop rotation
            angularVelocity = Vector3.zero;
            // Clear any accumulated torque
            accumulatedTorque = Vector3.zero;
        }
        #endregion

        #region Debug
        // Called by Unity to draw debug visualizations in the Scene view
        void OnDrawGizmos()
        {
            // Only draw gizmos when the game is running
            if (!Application.isPlaying) return;

            // Get the position to draw at: from visual renderer if available, otherwise from transform
            Vector3 drawPos = visualRenderer != null ? visualRenderer.GetPosition() : transform.position;
            // Get the rotation to draw: from visual renderer if available, otherwise from transform
            Quaternion drawRot = visualRenderer != null ? visualRenderer.GetRotation() : transform.rotation;

            // Draw an arrow showing the angular velocity direction and magnitude in blue
            DebugDrawUtils.DrawArrow(drawPos, angularVelocity, Color.blue, 0.3f);
            
            // Draw the local coordinate frame (XYZ axes) at the object's position and rotation
            DebugDrawUtils.DrawCoordinateFrame(drawPos, drawRot, 0.5f);
        }
        #endregion
    }
}

