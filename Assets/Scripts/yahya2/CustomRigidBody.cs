using UnityEngine;

/// <summary>
/// Corps rigide personnalisé - Toute la physique en mathématiques pures
/// </summary>
public class CustomRigidBody : MonoBehaviour
{
    // Propriétés physiques
    public float mass = 1.0f;
    public Vector3 size = Vector3.one;
    public bool isStatic = false;
    
    // État dynamique (géré manuellement)
    [HideInInspector] public Vector3 position;
    [HideInInspector] public Quaternion rotation;
    [HideInInspector] public Vector3 velocity;
    [HideInInspector] public Vector3 angularVelocity;
    
    // Forces accumulées
    private Vector3 forceAccumulator;
    private Vector3 torqueAccumulator;
    
    // Tenseur d'inertie
    private Matrix4x4 inertiaTensor;
    private Matrix4x4 inertiaTensorInv;
    
    // Amortissement
    public float linearDamping = 0.98f;
    public float angularDamping = 0.95f;
    
    private bool initialized = false;

    void Awake()
    {
        if (!initialized)
        {
            position = transform.position;
            rotation = transform.rotation;
            velocity = Vector3.zero;
            angularVelocity = Vector3.zero;
            initialized = true;
        }
        CalculateInertia();
    }

    /// <summary>
    /// Calcule le tenseur d'inertie pour une boîte
    /// </summary>
    void CalculateInertia()
    {
        float m = mass;
        float w = size.x, h = size.y, d = size.z;
        
        float Ixx = (m / 12.0f) * (h * h + d * d);
        float Iyy = (m / 12.0f) * (w * w + d * d);
        float Izz = (m / 12.0f) * (w * w + h * h);
        
        inertiaTensor = Matrix4x4.identity;
        inertiaTensor.m00 = Ixx;
        inertiaTensor.m11 = Iyy;
        inertiaTensor.m22 = Izz;
        
        inertiaTensorInv = InvertDiagonal(inertiaTensor);
    }

    /// <summary>
    /// Inverse une matrice diagonale 3x3
    /// </summary>
    Matrix4x4 InvertDiagonal(Matrix4x4 m)
    {
        Matrix4x4 inv = Matrix4x4.zero;
        inv.m00 = (m.m00 != 0) ? 1.0f / m.m00 : 0;
        inv.m11 = (m.m11 != 0) ? 1.0f / m.m11 : 0;
        inv.m22 = (m.m22 != 0) ? 1.0f / m.m22 : 0;
        inv.m33 = 1;
        return inv;
    }

    /// <summary>
    /// Ajoute une force au centre de masse
    /// </summary>
    public void AddForce(Vector3 force)
    {
        if (!isStatic)
            forceAccumulator += force;
    }

    /// <summary>
    /// Ajoute une force à un point spécifique
    /// </summary>
    public void AddForceAtPoint(Vector3 force, Vector3 worldPoint)
    {
        if (!isStatic)
        {
            forceAccumulator += force;
            Vector3 r = worldPoint - position;
            torqueAccumulator += Vector3.Cross(r, force);
        }
    }

    /// <summary>
    /// Ajoute un couple (torque)
    /// </summary>
    public void AddTorque(Vector3 torque)
    {
        if (!isStatic)
            torqueAccumulator += torque;
    }

    /// <summary>
    /// Intègre la physique pour un pas de temps
    /// </summary>
    public void IntegratePhysics(float deltaTime)
    {
        if (isStatic) return;

        // Intégration de la vitesse linéaire
        Vector3 acceleration = forceAccumulator / mass;
        velocity += acceleration * deltaTime;
        velocity *= linearDamping;
        
        // Intégration de la position
        position += velocity * deltaTime;
        
        // Intégration de la vitesse angulaire
        Matrix4x4 worldInertiaInv = GetWorldInertiaInverse();
        Vector3 angularAccel = MultiplyMatrixVector(worldInertiaInv, torqueAccumulator);
        angularVelocity += angularAccel * deltaTime;
        angularVelocity *= angularDamping;
        
        // Intégration de la rotation
        if (angularVelocity.sqrMagnitude > 0.0001f)
        {
            float angle = angularVelocity.magnitude * deltaTime;
            Vector3 axis = angularVelocity.normalized;
            Quaternion deltaRotation = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);
            rotation = deltaRotation * rotation;
            rotation.Normalize();
        }
        
        // Mise à jour du Transform pour le rendu
        transform.position = position;
        transform.rotation = rotation;
        
        // Réinitialisation des accumulateurs
        forceAccumulator = Vector3.zero;
        torqueAccumulator = Vector3.zero;
    }

    /// <summary>
    /// Obtient le tenseur d'inertie dans l'espace monde
    /// </summary>
    Matrix4x4 GetWorldInertiaInverse()
    {
        Matrix4x4 R = Matrix4x4.Rotate(rotation);
        Matrix4x4 Rt = TransposeMatrix(R);
        Matrix4x4 temp = MultiplyMatrices(R, inertiaTensorInv);
        return MultiplyMatrices(temp, Rt);
    }

    /// <summary>
    /// Multiplie deux matrices 3x3 (stockées en Matrix4x4)
    /// </summary>
    Matrix4x4 MultiplyMatrices(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result[i, j] = a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j];
            }
        }
        result.m33 = 1;
        return result;
    }

    /// <summary>
    /// Transpose une matrice 3x3
    /// </summary>
    Matrix4x4 TransposeMatrix(Matrix4x4 m)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result[i, j] = m[j, i];
            }
        }
        result.m33 = 1;
        return result;
    }

    /// <summary>
    /// Multiplie une matrice 3x3 par un vecteur
    /// </summary>
    Vector3 MultiplyMatrixVector(Matrix4x4 m, Vector3 v)
    {
        return new Vector3(
            m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
            m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
            m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
        );
    }

    /// <summary>
    /// Obtient la vitesse à un point spécifique
    /// </summary>
    public Vector3 GetVelocityAtPoint(Vector3 worldPoint)
    {
        Vector3 r = worldPoint - position;
        return velocity + Vector3.Cross(angularVelocity, r);
    }

    /// <summary>
    /// Obtient les axes locaux
    /// </summary>
    public Vector3 GetRight() => rotation * Vector3.right;
    public Vector3 GetUp() => rotation * Vector3.up;
    public Vector3 GetForward() => rotation * Vector3.forward;
}