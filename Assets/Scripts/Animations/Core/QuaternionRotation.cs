using UnityEngine;

namespace PhysicsUnity.Core
{
    /// <summary>
    /// Quaternion-based rotation implementation
    /// Provides stable rotation without gimbal lock
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
        #endregion

        #region Unity Lifecycle
        void Start()
        {
            currentRotation = transform.rotation;
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
            
            // Integrate rotation using quaternions
            currentRotation = IntegrationUtils.IntegrateRotationQuaternion(
                currentRotation,
                useLocalSpace ? transform.TransformDirection(angularVelocity) : angularVelocity,
                deltaTime
            );
            
            // Apply to transform
            transform.rotation = currentRotation;
            
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
                torque = transform.TransformDirection(torque);
            
            accumulatedTorque += torque;
        }

        /// <summary>
        /// Set angular velocity directly
        /// </summary>
        public void SetAngularVelocity(Vector3 velocity, bool isLocalSpace = false)
        {
            if (isLocalSpace)
                velocity = transform.TransformDirection(velocity);
            
            angularVelocity = velocity;
        }

        /// <summary>
        /// Get current angular velocity
        /// </summary>
        public Vector3 GetAngularVelocity(bool inLocalSpace = false)
        {
            return inLocalSpace ? transform.InverseTransformDirection(angularVelocity) : angularVelocity;
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

            // Draw angular velocity
            DebugDrawUtils.DrawArrow(transform.position, angularVelocity, Color.blue, 0.3f);
            
            // Draw coordinate frame
            DebugDrawUtils.DrawCoordinateFrame(transform.position, transform.rotation, 0.5f);
        }
        #endregion
    }
}

