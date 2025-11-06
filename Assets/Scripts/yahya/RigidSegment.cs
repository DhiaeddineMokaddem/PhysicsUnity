using UnityEngine;

/// <summary>
/// Segment rigide de la poutre avec physique complètement manuelle
/// </summary>
public class RigidSegment : MonoBehaviour
{
    // Propriétés physiques
    public float mass = 1f;
    public Vector3 size = new Vector3(2f, 0.2f, 0.2f);
    
    // État du corps rigide (stocké manuellement)
    [HideInInspector] public Vector3 position;
    [HideInInspector] public Quaternion rotation;
    [HideInInspector] public Vector3 velocity;
    [HideInInspector] public Vector3 angularVelocity;
    
    // Forces et couples accumulés
    private Vector3 force;
    private Vector3 torque;
    
    // Tenseur d'inertie
    private Matrix4x4 inertiaTensor;
    private Matrix4x4 inertiaTensorInv;
    
    // État
    public bool isFixed = false; // Si true, ce segment ne bouge pas
    
    private const float DAMPING_LINEAR = 0.01f;
    private const float DAMPING_ANGULAR = 0.02f;
    
    public void Initialize(Vector3 pos, Quaternion rot)
    {
        position = pos;
        rotation = rot;
        velocity = Vector3.zero;
        angularVelocity = Vector3.zero;
        
        // Calculer le tenseur d'inertie pour une boîte
        float w = size.x, h = size.y, d = size.z;
        float Ixx = (mass / 12f) * (h * h + d * d);
        float Iyy = (mass / 12f) * (w * w + d * d);
        float Izz = (mass / 12f) * (w * w + h * h);
        
        inertiaTensor = Matrix4x4.identity;
        inertiaTensor.m00 = Ixx;
        inertiaTensor.m11 = Iyy;
        inertiaTensor.m22 = Izz;
        
        inertiaTensorInv = InvertMatrix3x3(inertiaTensor);
        
        // Mise à jour visuelle
        UpdateVisualTransform();
    }
    
    /// <summary>
    /// Applique une force au centre de masse
    /// </summary>
    public void AddForce(Vector3 f)
    {
        if (!isFixed)
            force += f;
    }
    
    /// <summary>
    /// Applique une force à un point spécifique
    /// </summary>
    public void AddForceAtPoint(Vector3 f, Vector3 worldPoint)
    {
        if (!isFixed)
        {
            force += f;
            Vector3 r = worldPoint - position;
            torque += Vector3.Cross(r, f);
        }
    }
    
    /// <summary>
    /// Applique un couple
    /// </summary>
    public void AddTorque(Vector3 t)
    {
        if (!isFixed)
            torque += t;
    }
    
    /// <summary>
    /// Applique une impulsion (changement instantané de vitesse)
    /// </summary>
    public void AddImpulse(Vector3 impulse)
    {
        if (!isFixed)
            velocity += impulse / mass;
    }
    
    /// <summary>
    /// Applique une impulsion à un point spécifique
    /// </summary>
    public void AddImpulseAtPoint(Vector3 impulse, Vector3 worldPoint)
    {
        if (!isFixed)
        {
            velocity += impulse / mass;
            
            Vector3 r = worldPoint - position;
            Vector3 angularImpulse = Vector3.Cross(r, impulse);
            
            Matrix4x4 worldInertiaTensorInv = GetWorldInertiaTensorInv();
            angularVelocity += Mul3x3(worldInertiaTensorInv, angularImpulse);
        }
    }
    
    /// <summary>
    /// Intègre la physique sur un pas de temps
    /// </summary>
    public void IntegratePhysics(float dt, Vector3 gravity)
    {
        if (isFixed) return;
        
        // Gravité
        force += mass * gravity;
        
        // Amortissement
        force += -DAMPING_LINEAR * velocity * mass;
        torque += -DAMPING_ANGULAR * angularVelocity * mass;
        
        // Intégration de la vélocité linéaire
        Vector3 acceleration = force / mass;
        velocity += acceleration * dt;
        
        // Intégration de la position
        position += velocity * dt;
        
        // Intégration de la vélocité angulaire
        Matrix4x4 worldInertiaTensorInv = GetWorldInertiaTensorInv();
        Vector3 angularAcceleration = Mul3x3(worldInertiaTensorInv, torque);
        angularVelocity += angularAcceleration * dt;
        
        // Intégration de la rotation
        if (angularVelocity.magnitude > 0.001f)
        {
            float angle = angularVelocity.magnitude * dt;
            Vector3 axis = angularVelocity.normalized;
            Quaternion deltaRotation = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);
            rotation = deltaRotation * rotation;
            rotation.Normalize();
        }
        
        // Réinitialiser les accumulateurs
        force = Vector3.zero;
        torque = Vector3.zero;
        
        // Mise à jour visuelle
        UpdateVisualTransform();
    }
    
    /// <summary>
    /// Obtient la vitesse à un point donné dans l'espace
    /// </summary>
    public Vector3 GetVelocityAtPoint(Vector3 worldPoint)
    {
        Vector3 r = worldPoint - position;
        return velocity + Vector3.Cross(angularVelocity, r);
    }
    
    /// <summary>
    /// Transforme un point local en point mondial
    /// </summary>
    public Vector3 TransformPoint(Vector3 localPoint)
    {
        return position + rotation * localPoint;
    }
    
    /// <summary>
    /// Transforme un point mondial en point local
    /// </summary>
    public Vector3 InverseTransformPoint(Vector3 worldPoint)
    {
        return Quaternion.Inverse(rotation) * (worldPoint - position);
    }
    
    /// <summary>
    /// Met à jour le Transform Unity pour le rendu
    /// </summary>
    public void UpdateVisualTransform()
    {
        transform.position = position;
        transform.rotation = rotation;
    }
    
    /// <summary>
    /// Calcule le tenseur d'inertie dans le repère mondial
    /// </summary>
    private Matrix4x4 GetWorldInertiaTensorInv()
    {
        Matrix4x4 R = Matrix4x4.Rotate(rotation);
        Matrix4x4 Rt = Transpose3x3(R);
        Matrix4x4 tmp = Multiply3x3(R, inertiaTensorInv);
        return Multiply3x3(tmp, Rt);
    }
    
    // === Utilitaires mathématiques ===
    
    private static Matrix4x4 InvertMatrix3x3(Matrix4x4 m)
    {
        float det = m.m00 * (m.m11 * m.m22 - m.m12 * m.m21)
                  - m.m01 * (m.m10 * m.m22 - m.m12 * m.m20)
                  + m.m02 * (m.m10 * m.m21 - m.m11 * m.m20);
        
        if (Mathf.Abs(det) < 1e-8f) det = 1e-8f;
        float invDet = 1f / det;
        
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
    
    private static Matrix4x4 Multiply3x3(Matrix4x4 a, Matrix4x4 b)
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
    
    private static Matrix4x4 Transpose3x3(Matrix4x4 m)
    {
        Matrix4x4 result = Matrix4x4.zero;
        result.m00 = m.m00; result.m01 = m.m10; result.m02 = m.m20;
        result.m10 = m.m01; result.m11 = m.m11; result.m12 = m.m21;
        result.m20 = m.m02; result.m21 = m.m12; result.m22 = m.m22;
        result.m33 = 1f;
        return result;
    }
    
    private static Vector3 Mul3x3(Matrix4x4 m, Vector3 v)
    {
        return new Vector3(
            m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
            m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
            m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
        );
    }
    
    void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;
        
        // Dessiner la vélocité
        Gizmos.color = Color.green;
        Gizmos.DrawLine(position, position + velocity * 0.5f);
        
        // Dessiner la vélocité angulaire
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(position, position + angularVelocity * 0.3f);
        
        // Axes locaux
        float axisLength = size.x * 0.3f;
        Gizmos.color = Color.red;
        Gizmos.DrawLine(position, position + rotation * Vector3.right * axisLength);
        Gizmos.color = Color.green;
        Gizmos.DrawLine(position, position + rotation * Vector3.up * axisLength);
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(position, position + rotation * Vector3.forward * axisLength);
    }
}