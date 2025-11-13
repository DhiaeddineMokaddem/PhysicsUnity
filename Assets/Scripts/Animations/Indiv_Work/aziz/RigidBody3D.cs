using PhysicsSimulation.Core;
using UnityEngine;

namespace PhysicsSimulation.Indiv_Work.Aziz
{
    /// <summary>
    /// Représente un corps rigide avec toutes ses propriétés physiques - VERSION PURE MATH
    /// Stocke position, rotation, échelle manuellement sans dépendre de Transform
    /// FIXED: Uses Awake() for proper initialization order
    /// Refactored to use shared utilities from PhysicsUnity.Core
    /// </summary>
    public class RigidBody3D : MonoBehaviour
    {
        #region Inspector Properties
        [Header("Propriétés Physiques")]
        public float mass = 1.0f;
        public Vector3 velocity = Vector3.zero;
        public Vector3 angularVelocity = Vector3.zero;
        public float restitution = PhysicsConstants.DEFAULT_RESTITUTION;
        public float friction = PhysicsConstants.DEFAULT_FRICTION;
        public float linearDamping = PhysicsConstants.DEFAULT_LINEAR_DAMPING;
        public float angularDamping = PhysicsConstants.DEFAULT_ANGULAR_DAMPING;
        
        [Header("État du Corps")]
        public bool isKinematic = false;
        public bool useGravity = true;
        
        public Vector3 size = Vector3.one;
        #endregion

        #region Public Transform Data
        // PURE MATH: Position et rotation stockées manuellement
        [HideInInspector] public Vector3 position;
        [HideInInspector] public Quaternion rotation;
        [HideInInspector] public Vector3 scale = Vector3.one;
        #endregion

        #region Private Fields
        // Propriétés internes
        private Vector3 force = Vector3.zero;
        private Vector3 torque = Vector3.zero;
        private Matrix4x4 inertiaTensor;
        private Matrix4x4 inertiaTensorInverse;
        
        // Track if this is the first initialization
        private bool isInitialized = false;
        private VisualRenderer visualRenderer;
        #endregion

        #region Initialization
        /// <summary>
        /// FIXED: Use Awake() instead of Start() for immediate initialization
        /// Only initialize from Transform if not already manually set
        /// </summary>
        void Awake()
        {
            visualRenderer = GetComponent<VisualRenderer>();
            if (visualRenderer == null)
            {
                visualRenderer = gameObject.AddComponent<VisualRenderer>();
            }

            // Only initialize from VisualRenderer if position hasn't been set manually
            if (!isInitialized)
            {
                position = visualRenderer.GetPosition();
                rotation = visualRenderer.GetRotation();
                scale = visualRenderer.GetScale();
                isInitialized = true;
            }
            
            CalculateInertiaTensor();
        }

        void CalculateInertiaTensor()
        {
            inertiaTensor = MathUtils.CalculateBoxInertiaTensor(mass, size);
            inertiaTensorInverse = MathUtils.InvertMatrix3x3(inertiaTensor);
        }

        /// <summary>
        /// FIXED: Public method to manually initialize position (called before Awake)
        /// </summary>
        public void InitializePosition(Vector3 pos, Quaternion rot, Vector3 scl)
        {
            position = pos;
            rotation = rot;
            scale = scl;
            isInitialized = true;
            
            // Update visual renderer to match
            if (visualRenderer != null)
            {
                visualRenderer.UpdateTransform(pos, rot, scl);
            }
        }
        #endregion

        #region Transformation Methods
        // PURE MATH: Obtenir les axes locaux depuis le quaternion
        public Vector3 GetRight() => TransformUtils.GetRight(rotation);
        public Vector3 GetUp() => TransformUtils.GetUp(rotation);
        public Vector3 GetForward() => TransformUtils.GetForward(rotation);

        // PURE MATH: Transformation de point (local -> world)
        public Vector3 TransformPoint(Vector3 localPoint) => 
            TransformUtils.TransformPoint(localPoint, position, rotation, scale);

        // PURE MATH: Transformation inverse (world -> local)
        public Vector3 InverseTransformPoint(Vector3 worldPoint) => 
            TransformUtils.InverseTransformPoint(worldPoint, position, rotation, scale);
        #endregion

        #region Force and Impulse Application
        public void AddForce(Vector3 f)
        {
            if (!isKinematic)
                force += f;
        }

        public void AddForceAtPoint(Vector3 f, Vector3 point)
        {
            if (!isKinematic)
            {
                force += f;
                Vector3 r = point - position;
                torque += Vector3.Cross(r, f);
            }
        }

        public void AddTorque(Vector3 t)
        {
            if (!isKinematic)
                torque += t;
        }

        public void AddImpulse(Vector3 impulse)
        {
            if (!isKinematic)
                velocity += impulse / mass;
        }

        public void AddImpulseAtPoint(Vector3 impulse, Vector3 point)
        {
            if (!isKinematic)
            {
                Matrix4x4 worldInertiaTensorInv = CalculateWorldInverseInertiaTensor();
                TransformUtils.ApplyImpulseAtPoint(
                    ref velocity,
                    ref angularVelocity,
                    impulse,
                    point,
                    position,
                    1.0f / mass,
                    worldInertiaTensorInv
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
