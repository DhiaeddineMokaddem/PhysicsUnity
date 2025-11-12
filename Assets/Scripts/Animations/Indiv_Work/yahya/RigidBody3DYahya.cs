using UnityEngine;
using PhysicsUnity.Core;

/// <summary>
/// Représente un corps rigide avec toutes ses propriétés physiques - VERSION PURE MATH
/// Stocke position, rotation, échelle manuellement sans dépendre de Transform
/// FIXED: Uses Awake() for proper initialization order
/// Refactored to use shared utilities from PhysicsUnity.Core
/// </summary>
public class RigidBody3DYahya : MonoBehaviour
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
    [HideInInspector] public Vector3 position;
    [HideInInspector] public Quaternion rotation;
    [HideInInspector] public Vector3 scale = Vector3.one;
    #endregion

    #region Private Fields
    private Vector3 force = Vector3.zero;
    private Vector3 torque = Vector3.zero;
    private Matrix4x4 inertiaTensor;
    private Matrix4x4 inertiaTensorInverse;
    private bool isInitialized = false;
    #endregion

    #region Initialization
    void Awake()
    {
        if (!isInitialized)
        {
            position = transform.position;
            rotation = transform.rotation;
            scale = transform.localScale;
            isInitialized = true;
        }
        
        CalculateInertiaTensor();
    }

    void CalculateInertiaTensor()
    {
        inertiaTensor = MathUtils.CalculateBoxInertiaTensor(mass, size);
        inertiaTensorInverse = MathUtils.InvertMatrix3x3(inertiaTensor);
    }

    public void InitializePosition(Vector3 pos, Quaternion rot, Vector3 scl)
    {
        position = pos;
        rotation = rot;
        scale = scl;
        isInitialized = true;
        
        transform.position = pos;
        transform.rotation = rot;
        transform.localScale = scl;
    }
    #endregion

    #region Transformation Methods
    public Vector3 GetRight() => TransformUtils.GetRight(rotation);
    public Vector3 GetUp() => TransformUtils.GetUp(rotation);
    public Vector3 GetForward() => TransformUtils.GetForward(rotation);
    public Vector3 TransformPoint(Vector3 localPoint) => TransformUtils.TransformPoint(localPoint, position, rotation, scale);
    public Vector3 InverseTransformPoint(Vector3 worldPoint) => TransformUtils.InverseTransformPoint(worldPoint, position, rotation, scale);
    #endregion

    #region Force and Impulse Application
    public void AddForce(Vector3 f)
    {
        if (!isKinematic) force += f;
    }

    public void AddForceAtPoint(Vector3 f, Vector3 point)
    {
        if (!isKinematic)
        {
            force += f;
            torque += Vector3.Cross(point - position, f);
        }
    }

    public void AddTorque(Vector3 t)
    {
        if (!isKinematic) torque += t;
    }

    public void AddImpulse(Vector3 impulse)
    {
        if (!isKinematic) velocity += impulse / mass;
    }

    public void AddImpulseAtPoint(Vector3 impulse, Vector3 point)
    {
        if (!isKinematic)
        {
            Matrix4x4 worldInertiaTensorInv = CalculateWorldInverseInertiaTensor();
            TransformUtils.ApplyImpulseAtPoint(ref velocity, ref angularVelocity, impulse, point, position, 1.0f / mass, worldInertiaTensorInv);
        }
    }
    #endregion

    #region Physics Integration
    public void IntegratePhysics(float deltaTime)
    {
        if (isKinematic) return;

        if (useGravity) force += mass * PhysicsConstants.GRAVITY_VECTOR;

        Vector3 acceleration = IntegrationUtils.ForceToAcceleration(force, mass);
        velocity = IntegrationUtils.IntegrateVelocityEuler(velocity, acceleration, deltaTime);
        velocity = IntegrationUtils.ApplyDamping(velocity, linearDamping, deltaTime);
        position = IntegrationUtils.IntegratePositionEuler(position, velocity, deltaTime);

        Matrix4x4 worldInertiaTensorInv = CalculateWorldInverseInertiaTensor();
        Vector3 angularAcceleration = IntegrationUtils.TorqueToAngularAcceleration(torque, worldInertiaTensorInv);
        angularVelocity = IntegrationUtils.IntegrateVelocityEuler(angularVelocity, angularAcceleration, deltaTime);
        angularVelocity = IntegrationUtils.ApplyDamping(angularVelocity, angularDamping, deltaTime);
        rotation = IntegrationUtils.IntegrateRotationQuaternion(rotation, angularVelocity, deltaTime);

        UpdateVisualTransform();

        force = Vector3.zero;
        torque = Vector3.zero;
    }

    private Matrix4x4 CalculateWorldInverseInertiaTensor()
    {
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotation);
        return MultiplyMatrix3x3(MultiplyMatrix3x3(rotationMatrix, inertiaTensorInverse), TransposeMatrix3x3(rotationMatrix));
    }

    public void UpdateVisualTransform()
    {
        transform.position = position;
        transform.rotation = rotation;
        transform.localScale = scale;
    }
    #endregion

    #region Velocity Queries
    public Vector3 GetVelocityAtPoint(Vector3 point) => TransformUtils.GetVelocityAtPoint(velocity, angularVelocity, point, position);

    public float GetKineticEnergy()
    {
        float linearKE = 0.5f * mass * velocity.sqrMagnitude;
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotation);
        Matrix4x4 worldInertiaTensor = MultiplyMatrix3x3(MultiplyMatrix3x3(rotationMatrix, inertiaTensor), TransposeMatrix3x3(rotationMatrix));
        Vector3 L = MathUtils.MultiplyMatrixVector3(worldInertiaTensor, angularVelocity);
        return linearKE + 0.5f * Vector3.Dot(angularVelocity, L);
    }
    #endregion

    #region Matrix Operations
    Matrix4x4 MultiplyMatrix3x3(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                result[i, j] = a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j];
        result.m33 = 1f;
        return result;
    }

    Matrix4x4 TransposeMatrix3x3(Matrix4x4 m)
    {
        Matrix4x4 result = Matrix4x4.zero;
        result.m00 = m.m00; result.m01 = m.m10; result.m02 = m.m20;
        result.m10 = m.m01; result.m11 = m.m11; result.m12 = m.m21;
        result.m20 = m.m02; result.m21 = m.m12; result.m22 = m.m22;
        result.m33 = 1f;
        return result;
    }
    #endregion

    #region Debug Visualization
    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawLine(position, position + velocity);
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(position, position + angularVelocity);
    }
    #endregion
}