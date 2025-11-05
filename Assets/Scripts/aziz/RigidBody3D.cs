using UnityEngine;

/// <summary>
/// Représente un corps rigide avec toutes ses propriétés physiques
/// </summary>
public class RigidBody3D : MonoBehaviour
{
    [Header("Propriétés Physiques")]
    public float mass = 1.0f;
    public Vector3 velocity = Vector3.zero;
    public Vector3 angularVelocity = Vector3.zero;
    public float restitution = 0.5f; // Coefficient d'élasticité (0 = inélastique, 1 = parfaitement élastique)
    public float friction = 0.8f; // Augmenté pour plus de stabilité
    public float linearDamping = 0.02f; // Augmenté pour plus de stabilité
    public float angularDamping = 0.15f; // Augmenté pour plus de stabilité
    
    [Header("État du Corps")]
    public bool isKinematic = false; // Si vrai, n'est pas affecté par les forces
    public bool useGravity = true;
    
    // Propriétés internes
    private Vector3 force = Vector3.zero;
    private Vector3 torque = Vector3.zero;
    private Matrix4x4 inertiaTensor;
    private Matrix4x4 inertiaTensorInverse;
    
    // Dimensions du cube (pour calcul du tenseur d'inertie)
    public Vector3 size = Vector3.one;
    
    private const float GRAVITY = 9.81f;

    void Start()
    {
        CalculateInertiaTensor();
    }

    /// <summary>
    /// Calcule le tenseur d'inertie pour un cube
    /// </summary>
    void CalculateInertiaTensor()
    {
        float m = mass;
        float w = Mathf.Max(size.x, 0.1f); // Clamp minimum size
        float h = Mathf.Max(size.y, 0.1f);
        float d = Mathf.Max(size.z, 0.1f);
        
        // Tenseur d'inertie pour un parallélépipède rectangle
        float Ixx = Mathf.Max((m / 12.0f) * (h * h + d * d), 0.01f); // Clamp min inertia
        float Iyy = Mathf.Max((m / 12.0f) * (w * w + d * d), 0.01f);
        float Izz = Mathf.Max((m / 12.0f) * (w * w + h * h), 0.01f);
        
        inertiaTensor = Matrix4x4.identity;
        inertiaTensor.m00 = Ixx;
        inertiaTensor.m11 = Iyy;
        inertiaTensor.m22 = Izz;
        
        inertiaTensorInverse = inertiaTensor.inverse;
    }

    /// <summary>
    /// Applique une force au centre de masse
    /// </summary>
    public void AddForce(Vector3 f)
    {
        if (!isKinematic)
            force += f;
    }

    /// <summary>
    /// Applique une force à un point spécifique (génère aussi un couple)
    /// </summary>
    public void AddForceAtPoint(Vector3 f, Vector3 point)
    {
        if (!isKinematic)
        {
            force += f;
            Vector3 r = point - transform.position;
            torque += Vector3.Cross(r, f);
        }
    }

    /// <summary>
    /// Applique un couple (torque)
    /// </summary>
    public void AddTorque(Vector3 t)
    {
        if (!isKinematic)
            torque += t;
    }

    /// <summary>
    /// Applique une impulsion instantanée
    /// </summary>
    public void AddImpulse(Vector3 impulse)
    {
        if (!isKinematic)
            velocity += impulse / mass;
    }

    /// <summary>
    /// Applique une impulsion à un point spécifique
    /// </summary>
    public void AddImpulseAtPoint(Vector3 impulse, Vector3 point)
    {
        if (!isKinematic)
        {
            velocity += impulse / mass;
            Vector3 r = point - transform.position;
            Vector3 angularImpulse = Vector3.Cross(r, impulse);
            
            // Convertir l'impulsion angulaire en changement de vitesse angulaire
            Matrix4x4 rotationMatrix = Matrix4x4.Rotate(transform.rotation);
            Matrix4x4 worldInertiaTensorInv = rotationMatrix * inertiaTensorInverse * rotationMatrix.transpose;
            
            angularVelocity += worldInertiaTensorInv.MultiplyVector(angularImpulse);
        }
    }

    /// <summary>
    /// Intégration physique (appelé par PhysicsManager)
    /// </summary>
    public void IntegratePhysics(float deltaTime)
    {
        float maxAngularSpeed = 4f; // Réduit pour éviter rotations excessives
        if (angularVelocity.magnitude > maxAngularSpeed)
            angularVelocity = angularVelocity.normalized * maxAngularSpeed;

        if (isKinematic) return;

        // Appliquer la gravité
        if (useGravity)
        {
            force += mass * Vector3.down * GRAVITY;
        }

        // Intégration de la vitesse linéaire (Euler explicite)
        Vector3 acceleration = force / mass;
        velocity += acceleration * deltaTime;
        
        // Appliquer l'amortissement linéaire
        velocity *= (1f - linearDamping);
        
        // Intégration de la position
        transform.position += velocity * deltaTime;

        // Intégration de la vitesse angulaire
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(transform.rotation);
        Matrix4x4 worldInertiaTensorInv = rotationMatrix * inertiaTensorInverse * rotationMatrix.transpose;
        
        Vector3 angularAcceleration = worldInertiaTensorInv.MultiplyVector(torque);
        angularVelocity += angularAcceleration * deltaTime;
        
        // Appliquer l'amortissement angulaire
        angularVelocity *= (1f - angularDamping);
        
        // Intégration de la rotation
        if (angularVelocity.magnitude > 0.001f)
        {
            float angle = angularVelocity.magnitude * deltaTime;
            Vector3 axis = angularVelocity.normalized;
            Quaternion deltaRotation = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);
            transform.rotation = deltaRotation * transform.rotation;
        }

        // Réinitialiser les forces et couples pour la prochaine frame
        force = Vector3.zero;
        torque = Vector3.zero;
    }

    /// <summary>
    /// Obtient la vitesse à un point spécifique du corps
    /// </summary>
    public Vector3 GetVelocityAtPoint(Vector3 point)
    {
        Vector3 r = point - transform.position;
        return velocity + Vector3.Cross(angularVelocity, r);
    }

    /// <summary>
    /// Calcule l'énergie cinétique totale
    /// </summary>
    public float GetKineticEnergy()
    {
        float linearKE = 0.5f * mass * velocity.sqrMagnitude;
        
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(transform.rotation);
        Matrix4x4 worldInertiaTensor = rotationMatrix * inertiaTensor * rotationMatrix.transpose;
        Vector3 L = worldInertiaTensor.MultiplyVector(angularVelocity);
        float angularKE = 0.5f * Vector3.Dot(angularVelocity, L);
        
        return linearKE + angularKE;
    }

    void OnDrawGizmos()
    {
        // Visualisation de la vélocité
        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, transform.position + velocity);
        
        // Visualisation de la vélocité angulaire
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + angularVelocity);
    }
}