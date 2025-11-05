using UnityEngine;

/// <summary>
/// Représente un corps rigide avec toutes ses propriétés physiques - VERSION PURE MATH
/// Stocke position, rotation, échelle manuellement sans dépendre de Transform
/// FIXED: Uses Awake() for proper initialization order
/// </summary>
public class RigidBody3D : MonoBehaviour
{
    [Header("Propriétés Physiques")]
    public float mass = 1.0f;
    public Vector3 velocity = Vector3.zero;
    public Vector3 angularVelocity = Vector3.zero;
    public float restitution = 0.5f;
    public float friction = 0.3f;
    public float linearDamping = 0.001f;
    public float angularDamping = 0.01f;
    
    [Header("État du Corps")]
    public bool isKinematic = false;
    public bool useGravity = true;
    
    // PURE MATH: Position et rotation stockées manuellement
    [HideInInspector] public Vector3 position;
    [HideInInspector] public Quaternion rotation;
    [HideInInspector] public Vector3 scale = Vector3.one;
    
    // Propriétés internes
    private Vector3 force = Vector3.zero;
    private Vector3 torque = Vector3.zero;
    private Matrix4x4 inertiaTensor;
    private Matrix4x4 inertiaTensorInverse;
    
    public Vector3 size = Vector3.one;
    
    private const float GRAVITY = 9.81f;
    
    // Track if this is the first initialization
    private bool isInitialized = false;

    /// <summary>
    /// FIXED: Use Awake() instead of Start() for immediate initialization
    /// Only initialize from Transform if not already manually set
    /// </summary>
    void Awake()
    {
        // Only initialize from Transform if position hasn't been set manually
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
        float m = mass;
        float w = size.x;
        float h = size.y;
        float d = size.z;
        
        float Ixx = (m / 12.0f) * (h * h + d * d);
        float Iyy = (m / 12.0f) * (w * w + d * d);
        float Izz = (m / 12.0f) * (w * w + h * h);
        
        inertiaTensor = Matrix4x4.identity;
        inertiaTensor.m00 = Ixx;
        inertiaTensor.m11 = Iyy;
        inertiaTensor.m22 = Izz;
        
        inertiaTensorInverse = InvertMatrix3x3(inertiaTensor);
    }

    // PURE MATH: Inversion manuelle de matrice 3x3
    Matrix4x4 InvertMatrix3x3(Matrix4x4 m)
    {
        float det = m.m00 * (m.m11 * m.m22 - m.m12 * m.m21)
                  - m.m01 * (m.m10 * m.m22 - m.m12 * m.m20)
                  + m.m02 * (m.m10 * m.m21 - m.m11 * m.m20);
        
        if (Mathf.Abs(det) < 1e-8f)
            det = 1e-8f;
        
        float invDet = 1.0f / det;
        
        Matrix4x4 inv = Matrix4x4.zero;
        inv.m00 = (m.m11 * m.m22 - m.m12 * m.m21) * invDet;
        inv.m01 = (m.m02 * m.m21 - m.m01 * m.m22) * invDet;
        inv.m02 = (m.m01 * m.m12 - m.m02 * m.m11) * invDet;
        inv.m10 = (m.m12 * m.m20 - m.m10 * m.m22) * invDet;
        inv.m11 = (m.m00 * m.m22 - m.m02 * m.m20) * invDet;
        inv.m12 = (m.m02 * m.m10 - m.m00 * m.m12) * invDet;
        inv.m20 = (m.m10 * m.m21 - m.m11 * m.m20) * invDet;
        inv.m21 = (m.m01 * m.m20 - m.m00 * m.m21) * invDet;
        inv.m22 = (m.m00 * m.m11 - m.m01 * m.m10) * invDet;
        inv.m33 = 1f;
        
        return inv;
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
        
        // Update transform to match
        transform.position = pos;
        transform.rotation = rot;
        transform.localScale = scl;
    }

    // PURE MATH: Obtenir les axes locaux depuis le quaternion
    public Vector3 GetRight()
    {
        return rotation * Vector3.right;
    }
    
    public Vector3 GetUp()
    {
        return rotation * Vector3.up;
    }
    
    public Vector3 GetForward()
    {
        return rotation * Vector3.forward;
    }

    // PURE MATH: Transformation de point (local -> world)
    public Vector3 TransformPoint(Vector3 localPoint)
    {
        return position + rotation * Vector3.Scale(localPoint, scale);
    }

    // PURE MATH: Transformation inverse (world -> local)
    public Vector3 InverseTransformPoint(Vector3 worldPoint)
    {
        Vector3 localPoint = worldPoint - position;
        localPoint = Quaternion.Inverse(rotation) * localPoint;
        return new Vector3(localPoint.x / scale.x, localPoint.y / scale.y, localPoint.z / scale.z);
    }

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
            velocity += impulse / mass;
            Vector3 r = point - position;
            Vector3 angularImpulse = Vector3.Cross(r, impulse);
            
            Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotation);
            Matrix4x4 worldInertiaTensorInv = MultiplyMatrix3x3(
                MultiplyMatrix3x3(rotationMatrix, inertiaTensorInverse),
                TransposeMatrix3x3(rotationMatrix)
            );
            
            angularVelocity += MultiplyMatrix3x3Vector(worldInertiaTensorInv, angularImpulse);
        }
    }

    public void IntegratePhysics(float deltaTime)
    {
        if (isKinematic) return;

        if (useGravity)
        {
            force += mass * Vector3.down * GRAVITY;
        }

        Vector3 acceleration = force / mass;
        velocity += acceleration * deltaTime;
        velocity *= (1f - linearDamping);
        
        // PURE MATH: Intégration manuelle de la position
        position += velocity * deltaTime;

        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotation);
        Matrix4x4 worldInertiaTensorInv = MultiplyMatrix3x3(
            MultiplyMatrix3x3(rotationMatrix, inertiaTensorInverse),
            TransposeMatrix3x3(rotationMatrix)
        );
        
        Vector3 angularAcceleration = MultiplyMatrix3x3Vector(worldInertiaTensorInv, torque);
        angularVelocity += angularAcceleration * deltaTime;
        angularVelocity *= (1f - angularDamping);
        
        // PURE MATH: Intégration manuelle de la rotation
        if (angularVelocity.magnitude > 0.001f)
        {
            float angle = angularVelocity.magnitude * deltaTime;
            Vector3 axis = angularVelocity.normalized;
            Quaternion deltaRotation = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);
            rotation = deltaRotation * rotation;
            rotation.Normalize();
        }

        // Mettre à jour le Transform Unity pour le rendu uniquement
        UpdateVisualTransform();

        force = Vector3.zero;
        torque = Vector3.zero;
    }

    // PURE MATH: Mise à jour du Transform Unity pour l'affichage
    public void UpdateVisualTransform()
    {
        transform.position = position;
        transform.rotation = rotation;
        transform.localScale = scale;
    }

    public Vector3 GetVelocityAtPoint(Vector3 point)
    {
        Vector3 r = point - position;
        return velocity + Vector3.Cross(angularVelocity, r);
    }

    public float GetKineticEnergy()
    {
        float linearKE = 0.5f * mass * velocity.sqrMagnitude;
        
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotation);
        Matrix4x4 worldInertiaTensor = MultiplyMatrix3x3(
            MultiplyMatrix3x3(rotationMatrix, inertiaTensor),
            TransposeMatrix3x3(rotationMatrix)
        );
        Vector3 L = MultiplyMatrix3x3Vector(worldInertiaTensor, angularVelocity);
        float angularKE = 0.5f * Vector3.Dot(angularVelocity, L);
        
        return linearKE + angularKE;
    }

    // PURE MATH: Multiplication matrice 3x3 (stockée dans Matrix4x4)
    Matrix4x4 MultiplyMatrix3x3(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result[i, j] = a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j];
            }
        }
        result.m33 = 1f;
        return result;
    }

    // PURE MATH: Transposée 3x3
    Matrix4x4 TransposeMatrix3x3(Matrix4x4 m)
    {
        Matrix4x4 result = Matrix4x4.zero;
        result.m00 = m.m00; result.m01 = m.m10; result.m02 = m.m20;
        result.m10 = m.m01; result.m11 = m.m11; result.m12 = m.m21;
        result.m20 = m.m02; result.m21 = m.m12; result.m22 = m.m22;
        result.m33 = 1f;
        return result;
    }

    // PURE MATH: Multiplication matrice-vecteur 3x3
    Vector3 MultiplyMatrix3x3Vector(Matrix4x4 m, Vector3 v)
    {
        return new Vector3(
            m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
            m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
            m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
        );
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawLine(position, position + velocity);
        
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(position, position + angularVelocity);
    }
}