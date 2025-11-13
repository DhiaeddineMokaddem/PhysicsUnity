// Import core physics simulation utilities for mathematical operations
using PhysicsSimulation.Core;
// Import Unity's engine for MonoBehaviour, Vector3, Quaternion
using UnityEngine;

// Namespace for Aziz's individual physics work
namespace PhysicsSimulation.Indiv_Work.Aziz
{
    /// <summary>
    /// Represents a rigid body with all physical properties - PURE MATH VERSION
    /// Stores position, rotation, scale manually without depending on Transform
    /// FIXED: Uses Awake() for proper initialization order
    /// Refactored to use shared utilities from PhysicsUnity.Core
    /// ALL TRANSFORMATIONS DONE MANUALLY - NO transform.position or transform.rotation!
    /// </summary>
    public class RigidBody3D : MonoBehaviour
    {
        #region Inspector Properties
        [Header("Physical Properties")] // Unity Inspector header
        // Mass in kilograms - determines resistance to linear acceleration (F = ma)
        public float mass = 1.0f;
        // Linear velocity in meters per second - rate of position change
        public Vector3 velocity = Vector3.zero;
        // Angular velocity in radians per second - rate of rotation change
        public Vector3 angularVelocity = Vector3.zero;
        // Coefficient of restitution (0 = inelastic, 1 = perfectly elastic) - controls bounciness
        public float restitution = PhysicsConstants.DEFAULT_RESTITUTION;
        // Friction coefficient (higher = more friction) - resists sliding
        public float friction = PhysicsConstants.DEFAULT_FRICTION;
        // Linear damping - simulates air resistance on linear motion
        public float linearDamping = PhysicsConstants.DEFAULT_LINEAR_DAMPING;
        // Angular damping - simulates air resistance on rotation
        public float angularDamping = PhysicsConstants.DEFAULT_ANGULAR_DAMPING;
        
        [Header("Body State")] // Unity Inspector header
        // If true, body doesn't respond to forces (can be moved manually)
        public bool isKinematic = false;
        // If true, gravity force is applied each frame
        public bool useGravity = true;
        
        // Size of the box collider in local space (width, height, depth)
        public Vector3 size = Vector3.one;
        #endregion

        #region Public Transform Data
        // PURE MATH: Position stored manually (NOT using Unity Transform!)
        // This is the center of mass in world space
        [HideInInspector] public Vector3 position;
        // PURE MATH: Rotation stored manually as quaternion (NOT using Unity Transform!)
        // Represents orientation in world space
        [HideInInspector] public Quaternion rotation;
        // PURE MATH: Scale stored manually (NOT using Unity Transform!)
        // Represents size multiplier in each axis
        [HideInInspector] public Vector3 scale = Vector3.one;
        #endregion

        #region Private Fields
        // Accumulated force to be applied in next physics update (Newtons)
        private Vector3 force = Vector3.zero;
        // Accumulated torque to be applied in next physics update (Newton-meters)
        private Vector3 torque = Vector3.zero;
        // Inertia tensor in local space - resistance to angular acceleration
        private Matrix4x4 inertiaTensor;
        // Inverse of inertia tensor - used to calculate angular acceleration from torque
        private Matrix4x4 inertiaTensorInverse;
        
        // Track if this body has been initialized to prevent re-initialization
        private bool isInitialized = false;
        // Reference to visual renderer that updates mesh vertices manually
        private VisualRenderer visualRenderer;
        #endregion

        #region Initialization
        /// <summary>
        /// Unity lifecycle method - called when script instance is being loaded
        /// FIXED: Use Awake() instead of Start() for immediate initialization
        /// Only initialize from VisualRenderer if not already manually set
        /// </summary>
        void Awake()
        {
            // Try to get existing VisualRenderer component
            visualRenderer = GetComponent<VisualRenderer>();
            // If none exists, add one to handle manual mesh transformation
            if (visualRenderer == null)
            {
                visualRenderer = gameObject.AddComponent<VisualRenderer>();
            }

            // Only initialize from VisualRenderer if position hasn't been set manually
            if (!isInitialized)
            {
                // Get initial position from visual renderer (reads from GameObject's initial transform)
                position = visualRenderer.GetPosition();
                // Get initial rotation from visual renderer
                rotation = visualRenderer.GetRotation();
                // Get initial scale from visual renderer
                scale = visualRenderer.GetScale();
                // Mark as initialized to prevent re-initialization
                isInitialized = true;
            }
            
            // Calculate moment of inertia tensor for this rigid body shape
            CalculateInertiaTensor();
        }

        // Calculate inertia tensor based on box shape and mass
        void CalculateInertiaTensor()
        {
            // Calculate inertia tensor for box: I = (m/12) * diag(h²+d², w²+d², w²+h²)
            inertiaTensor = MathUtils.CalculateBoxInertiaTensor(mass, size);
            // Invert the tensor for use in angular acceleration calculations
            inertiaTensorInverse = MathUtils.InvertMatrix3x3(inertiaTensor);
        }

        /// <summary>
        /// Public method to manually initialize position (called before Awake)
        /// FIXED: Allows external scripts to set position before physics starts
        /// </summary>
        public void InitializePosition(Vector3 pos, Quaternion rot, Vector3 scl)
        {
            // Set manual position in world space
            position = pos;
            // Set manual rotation in world space
            rotation = rot;
            // Set manual scale
            scale = scl;
            // Mark as initialized so Awake doesn't override these values
            isInitialized = true;
            
            // Update visual renderer to match if it exists
            if (visualRenderer != null)
            {
                visualRenderer.UpdateTransform(pos, rot, scl);
            }
        }
        #endregion

        #region Transformation Methods
        // PURE MATH: Get right axis (X-axis) in world space from rotation quaternion
        // Returns the direction this body considers "right"
        public Vector3 GetRight() => TransformUtils.GetRight(rotation);
        
        // PURE MATH: Get up axis (Y-axis) in world space from rotation quaternion
        // Returns the direction this body considers "up"
        public Vector3 GetUp() => TransformUtils.GetUp(rotation);
        
        // PURE MATH: Get forward axis (Z-axis) in world space from rotation quaternion
        // Returns the direction this body considers "forward"
        public Vector3 GetForward() => TransformUtils.GetForward(rotation);

        // PURE MATH: Transform point from local space to world space
        // Formula: worldPoint = position + rotation * (scale * localPoint)
        public Vector3 TransformPoint(Vector3 localPoint) => 
            TransformUtils.TransformPoint(localPoint, position, rotation, scale);

        // PURE MATH: Transform point from world space to local space
        // Formula: localPoint = scale^-1 * (rotation^-1 * (worldPoint - position))
        public Vector3 InverseTransformPoint(Vector3 worldPoint) => 
            TransformUtils.InverseTransformPoint(worldPoint, position, rotation, scale);
        #endregion

        #region Force and Impulse Application
        // Add force to be applied in next physics update
        // Force causes acceleration: F = ma, so a = F/m
        public void AddForce(Vector3 f)
        {
            // Only apply force if body is not kinematic (kinematic bodies ignore forces)
            if (!isKinematic)
                force += f;  // Accumulate force vector
        }

        // Add force at a specific point, creating both linear and angular effects
        // Point of application creates torque: τ = r × F
        public void AddForceAtPoint(Vector3 f, Vector3 point)
        {
            // Only apply if not kinematic
            if (!isKinematic)
            {
                // Add to linear force accumulator
                force += f;
                // Calculate lever arm from center of mass to application point
                Vector3 r = point - position;
                // Calculate torque using cross product: τ = r × F
                // Torque causes angular acceleration
                torque += Vector3.Cross(r, f);
            }
        }

        // Add pure torque (rotational force) without linear component
        // Torque causes angular acceleration: τ = I*α, so α = I^-1*τ
        public void AddTorque(Vector3 t)
        {
            // Only apply if not kinematic
            if (!isKinematic)
                torque += t;  // Accumulate torque vector
        }

        // Add impulse - instant velocity change
        // Impulse instantly changes momentum: J = Δp = m*Δv, so Δv = J/m
        public void AddImpulse(Vector3 impulse)
        {
            // Only apply if not kinematic
            if (!isKinematic)
                velocity += impulse / mass;  // Directly modify velocity
        }

        // Add impulse at a point - creates both linear and angular velocity changes
        // Used for instant impacts like collisions
        public void AddImpulseAtPoint(Vector3 impulse, Vector3 point)
        {
            // Only apply if not kinematic
            if (!isKinematic)
            {
                // Calculate world-space inverse inertia tensor for angular response
                Matrix4x4 worldInertiaTensorInv = CalculateWorldInverseInertiaTensor();
                // Apply impulse using utility function (updates both velocity and angularVelocity)
                TransformUtils.ApplyImpulseAtPoint(
                    ref velocity,           // Linear velocity to update (by reference)
                    ref angularVelocity,    // Angular velocity to update (by reference)
                    impulse,                // Impulse vector to apply
                    point,                  // Point of application in world space
                    position,               // Center of mass position in world space
                    1.0f / mass,           // Inverse mass (used for velocity update)
                    worldInertiaTensorInv   // Inverse inertia tensor (used for angular velocity update)
                );
            }
        }
        #endregion

        #region Physics Integration
        public void IntegratePhysics(float deltaTime)
        {
            if (isKinematic) return;

            // Apply gravity
            if (useGravity)
            {
                force += mass * PhysicsConstants.GRAVITY_VECTOR;
            }

            // Calculate acceleration and integrate velocity
            Vector3 acceleration = IntegrationUtils.ForceToAcceleration(force, mass);
            velocity = IntegrationUtils.IntegrateVelocityEuler(velocity, acceleration, deltaTime);
            velocity = IntegrationUtils.ApplyDamping(velocity, linearDamping, deltaTime);

            // PURE MATH: Intégration manuelle de la position
            position = IntegrationUtils.IntegratePositionEuler(position, velocity, deltaTime);

            // Calculate angular acceleration and integrate angular velocity
            Matrix4x4 worldInertiaTensorInv = CalculateWorldInverseInertiaTensor();
            Vector3 angularAcceleration = IntegrationUtils.TorqueToAngularAcceleration(torque, worldInertiaTensorInv);
            angularVelocity = IntegrationUtils.IntegrateVelocityEuler(angularVelocity, angularAcceleration, deltaTime);
            angularVelocity = IntegrationUtils.ApplyDamping(angularVelocity, angularDamping, deltaTime);

            // PURE MATH: Intégration manuelle de la rotation
            rotation = IntegrationUtils.IntegrateRotationQuaternion(rotation, angularVelocity, deltaTime);

            // Mettre à jour le Transform Unity pour le rendu uniquement
            UpdateVisualTransform();

            // Clear accumulated forces
            force = Vector3.zero;
            torque = Vector3.zero;
        }

        /// <summary>
        /// Calculates the world-space inverse inertia tensor
        /// Uses shared MathUtils for matrix operations
        /// </summary>
        private Matrix4x4 CalculateWorldInverseInertiaTensor()
        {
            Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotation);
            return MathUtils.TransformInertiaTensor(inertiaTensorInverse, rotationMatrix);
        }

        // PURE MATH: Mise à jour du Transform Unity pour l'affichage
        public void UpdateVisualTransform()
        {
            if (visualRenderer != null)
            {
                visualRenderer.UpdateTransform(position, rotation, scale);
            }
        }
        #endregion

        #region Velocity Queries
        public Vector3 GetVelocityAtPoint(Vector3 point) => 
            TransformUtils.GetVelocityAtPoint(velocity, angularVelocity, point, position);

        public float GetKineticEnergy()
        {
            float linearKE = IntegrationUtils.CalculateKineticEnergy(velocity, mass);

            Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotation);
            Matrix4x4 worldInertiaTensor = MathUtils.TransformInertiaTensor(inertiaTensor, rotationMatrix);
            float angularKE = IntegrationUtils.CalculateRotationalEnergy(angularVelocity, worldInertiaTensor);

            return linearKE + angularKE;
        }

        public float GetPotentialEnergy() => 
            IntegrationUtils.CalculatePotentialEnergy(position, mass);

        public float GetTotalEnergy() => GetKineticEnergy() + GetPotentialEnergy();
        #endregion

        #region Debug Visualization
        void OnDrawGizmos()
        {
            // Draw velocity vector
            DebugDrawUtils.DrawVelocity(position, velocity, Color.green);

            // Draw angular velocity vector
            DebugDrawUtils.DrawArrow(position, angularVelocity, Color.blue);

            // Draw bounding box
            DebugDrawUtils.DrawWireCube(position, size, rotation, Color.cyan);

            // Draw coordinate frame
            DebugDrawUtils.DrawCoordinateFrame(position, rotation, 0.5f);
        }

        void OnDrawGizmosSelected()
        {
            // Draw detailed physics info when selected
            #if UNITY_EDITOR
            DebugDrawUtils.DrawPhysicsInfo(position + Vector3.up * 2f, gameObject.name, velocity, angularVelocity, mass);
            #endif
        }
        #endregion
    }
}
