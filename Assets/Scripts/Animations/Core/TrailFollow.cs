using UnityEngine;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Helper class for trail effects and motion tracking
    /// Useful for visualizing object motion and creating smooth following behaviors
    /// NOTE: This is for camera/visual tracking utilities, not physics simulation.
    /// For physics simulation, use VisualRenderer instead.
    /// </summary>
    public class TrailFollow : MonoBehaviour
    {
        #region Configuration
        [Header("Trail Settings")]
        [SerializeField] private Transform target;
        [SerializeField] private float followSpeed = 5f;
        [SerializeField] private float rotationSpeed = 5f;
        [SerializeField] private float minDistance = 0.1f;
        
        [Header("Smoothing")]
        [SerializeField] private bool useSmoothing = true;
        [SerializeField] private float positionSmoothTime = 0.3f;
        [SerializeField] private float rotationSmoothTime = 0.3f;
        
        [Header("Constraints")]
        [SerializeField] private bool constrainX = false;
        [SerializeField] private bool constrainY = false;
        [SerializeField] private bool constrainZ = false;
        #endregion

        #region Private Fields
        private Vector3 velocity = Vector3.zero;
        private Vector3 angularVelocity = Vector3.zero;
        #endregion

        #region Unity Lifecycle
        void LateUpdate()
        {
            if (target == null) return;

            UpdatePosition();
            UpdateRotation();
        }
        #endregion

        #region Position Update
        private void UpdatePosition()
        {
            Vector3 targetPosition = target.position;
            
            // Apply constraints
            if (constrainX) targetPosition.x = transform.position.x;
            if (constrainY) targetPosition.y = transform.position.y;
            if (constrainZ) targetPosition.z = transform.position.z;

            // Check minimum distance
            float distance = Vector3.Distance(transform.position, targetPosition);
            if (distance < minDistance)
                return;

            if (useSmoothing)
            {
                transform.position = TransformUtils.SmoothMove(
                    transform.position,
                    targetPosition,
                    ref velocity,
                    positionSmoothTime,
                    Time.deltaTime
                );
            }
            else
            {
                transform.position = Vector3.Lerp(
                    transform.position,
                    targetPosition,
                    followSpeed * Time.deltaTime
                );
            }
        }

        private void UpdateRotation()
        {
            if (useSmoothing)
            {
                transform.rotation = TransformUtils.SmoothRotate(
                    transform.rotation,
                    target.rotation,
                    ref angularVelocity,
                    rotationSmoothTime,
                    Time.deltaTime
                );
            }
            else
            {
                transform.rotation = Quaternion.Slerp(
                    transform.rotation,
                    target.rotation,
                    rotationSpeed * Time.deltaTime
                );
            }
        }
        #endregion

        #region Public Methods
        /// <summary>
        /// Set the target to follow
        /// </summary>
        public void SetTarget(Transform newTarget)
        {
            target = newTarget;
        }

        /// <summary>
        /// Instantly snap to target position and rotation
        /// </summary>
        public void SnapToTarget()
        {
            if (target == null) return;

            transform.position = target.position;
            transform.rotation = target.rotation;
            velocity = Vector3.zero;
            angularVelocity = Vector3.zero;
        }
        #endregion
    }
}

