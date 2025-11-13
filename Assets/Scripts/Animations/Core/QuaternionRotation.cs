using UnityEngine;

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
        [Header("Rotation Properties")]
        [SerializeField] private Vector3 angularVelocity = Vector3.zero;
        [SerializeField] private float angularDamping = 0.01f;
        [SerializeField] private bool useLocalSpace = false;
        
        [Header("Torque")]
        [SerializeField] private Vector3 appliedTorque = Vector3.zero;
        [SerializeField] private float maxAngularSpeed = 10f;
        #endregion

        #region Private Fields
        private Quaternion currentRotation;
        private Vector3 accumulatedTorque = Vector3.zero;
        private VisualRenderer visualRenderer;
        #endregion

        #region Unity Lifecycle
        void Start()
        {
            visualRenderer = GetComponent<VisualRenderer>();
            if (visualRenderer == null)
            {
                visualRenderer = gameObject.AddComponent<VisualRenderer>();
            }
            currentRotation = visualRenderer.GetRotation();
        }

        void FixedUpdate()
        {
            IntegrateRotation(Time.fixedDeltaTime);
        }
        #endregion

        #region Rotation Integration
        private void IntegrateRotation(float deltaTime)
        {
            // Add applied torque
            Vector3 totalTorque = appliedTorque + accumulatedTorque;
            
            // Update angular velocity
            angularVelocity += totalTorque * deltaTime;
            
            // Apply damping
            angularVelocity = IntegrationUtils.ApplyDamping(angularVelocity, angularDamping, deltaTime);
            
            // Clamp angular velocity
            angularVelocity = MathUtils.ClampMagnitude(angularVelocity, maxAngularSpeed);
            
            // Get direction for integration
            Vector3 rotationAxis = useLocalSpace ? TransformUtils.TransformDirection(angularVelocity, currentRotation) : angularVelocity;
            
            // Integrate rotation using quaternions
            currentRotation = IntegrationUtils.IntegrateRotationQuaternion(
                currentRotation,
                rotationAxis,
                deltaTime
            );
            
            // Apply to visual renderer
            if (visualRenderer != null)
            {
                visualRenderer.UpdateRotation(currentRotation);
            }
            
            // Clear accumulated torque
            accumulatedTorque = Vector3.zero;
        }
        #endregion

        #region Public Methods
        /// <summary>
        /// Add torque to the object
        /// </summary>
        public void AddTorque(Vector3 torque, bool isLocalSpace = false)
        {
            if (isLocalSpace)
                torque = TransformUtils.TransformDirection(torque, currentRotation);
            
            accumulatedTorque += torque;
        }

        /// <summary>
        /// Set angular velocity directly
        /// </summary>
        public void SetAngularVelocity(Vector3 velocity, bool isLocalSpace = false)
        {
            if (isLocalSpace)
                velocity = TransformUtils.TransformDirection(velocity, currentRotation);
            
            angularVelocity = velocity;
        }

        /// <summary>
        /// Get current angular velocity
        /// </summary>
        public Vector3 GetAngularVelocity(bool inLocalSpace = false)
        {
            return inLocalSpace ? TransformUtils.InverseTransformDirection(angularVelocity, currentRotation) : angularVelocity;
        }

        /// <summary>
        /// Stop all rotation
        /// </summary>
        public void Stop()
        {
            angularVelocity = Vector3.zero;
            accumulatedTorque = Vector3.zero;
        }
        #endregion

        #region Debug
        void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;

            Vector3 drawPos = visualRenderer != null ? visualRenderer.GetPosition() : transform.position;
            Quaternion drawRot = visualRenderer != null ? visualRenderer.GetRotation() : transform.rotation;

            // Draw angular velocity
            DebugDrawUtils.DrawArrow(drawPos, angularVelocity, Color.blue, 0.3f);
            
            // Draw coordinate frame
            DebugDrawUtils.DrawCoordinateFrame(drawPos, drawRot, 0.5f);
        }
        #endregion
    }
}

