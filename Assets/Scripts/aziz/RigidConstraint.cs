using UnityEngine;

/// <summary>
/// Contrainte entre deux corps rigides
/// </summary>
public class RigidConstraint : MonoBehaviour
{
    [Header("Corps Connectés")]
    public RigidBody3D bodyA;
    public RigidBody3D bodyB;
    
    [Header("Paramètres de Contrainte")]
    public float maxDistance = 1.0f; // Distance maximale avant rupture
    public float breakForce = 100.0f; // Force maximale avant rupture
    public float stiffness = 1000.0f; // Rigidité de la contrainte
    public float damping = 50.0f; // Amortissement
    
    [Header("État")]
    public bool isBroken = false;
    
    // Points d'ancrage locaux
    private Vector3 localAnchorA;
    private Vector3 localAnchorB;
    private float restDistance;
    private float accumulatedForce = 0f;

    void Start()
    {
        if (bodyA != null && bodyB != null)
        {
            // Calculer les points d'ancrage locaux
            localAnchorA = bodyA.transform.InverseTransformPoint(transform.position);
            localAnchorB = bodyB.transform.InverseTransformPoint(transform.position);
            restDistance = Vector3.Distance(bodyA.transform.position, bodyB.transform.position);
        }
    }

    /// <summary>
    /// Résout la contrainte (appelé par PhysicsManager)
    /// </summary>
    public void SolveConstraint(float deltaTime)
    {
        if (isBroken || bodyA == null || bodyB == null) return;
        if (bodyA.isKinematic && bodyB.isKinematic) return;

        // Calculer les positions mondiales des points d'ancrage
        Vector3 worldAnchorA = bodyA.transform.TransformPoint(localAnchorA);
        Vector3 worldAnchorB = bodyB.transform.TransformPoint(localAnchorB);
        
        // Vecteur de connexion
        Vector3 delta = worldAnchorB - worldAnchorA;
        float currentDistance = delta.magnitude;
        
        // Vérifier la rupture par distance
        if (currentDistance > maxDistance)
        {
            Break();
            return;
        }
        
        if (currentDistance < 0.0001f) return;
        
        Vector3 direction = delta / currentDistance;
        
        // Calculer l'erreur
        float error = currentDistance - restDistance;
        
        // Calculer les vitesses relatives
        Vector3 velA = bodyA.GetVelocityAtPoint(worldAnchorA);
        Vector3 velB = bodyB.GetVelocityAtPoint(worldAnchorB);
        Vector3 relativeVel = velB - velA;
        float relativeVelAlongConstraint = Vector3.Dot(relativeVel, direction);
        
        // Force de rappel (loi de Hooke)
        float springForce = error * stiffness;
        
        // Force d'amortissement
        float dampingForce = relativeVelAlongConstraint * damping;
        
        // Force totale
        float totalForce = springForce + dampingForce;
        Vector3 force = direction * totalForce;
        
        // Accumuler la force pour vérifier la rupture
        accumulatedForce += Mathf.Abs(totalForce) * deltaTime;
        
        // Vérifier la rupture par force
        if (Mathf.Abs(totalForce) > breakForce)
        {
            Break();
            return;
        }
        
        // Appliquer les forces
        if (!bodyA.isKinematic)
        {
            bodyA.AddForceAtPoint(force, worldAnchorA);
        }
        
        if (!bodyB.isKinematic)
        {
            bodyB.AddForceAtPoint(-force, worldAnchorB);
        }
    }

    /// <summary>
    /// Rompt la contrainte
    /// </summary>
    public void Break()
    {
        if (!isBroken)
        {
            isBroken = true;
            Debug.Log($"Contrainte rompue entre {bodyA.name} et {bodyB.name}");
        }
    }

    /// <summary>
    /// Répare la contrainte
    /// </summary>
    public void Repair()
    {
        isBroken = false;
        accumulatedForce = 0f;
    }

    void OnDrawGizmos()
    {
        if (bodyA == null || bodyB == null) return;
        
        Vector3 worldAnchorA = bodyA.transform.TransformPoint(localAnchorA);
        Vector3 worldAnchorB = bodyB.transform.TransformPoint(localAnchorB);
        
        if (isBroken)
        {
            Gizmos.color = Color.red;
        }
        else
        {
            float stress = accumulatedForce / (breakForce * 10f);
            Gizmos.color = Color.Lerp(Color.green, Color.yellow, stress);
        }
        
        Gizmos.DrawLine(worldAnchorA, worldAnchorB);
        Gizmos.DrawWireSphere(worldAnchorA, 0.1f);
        Gizmos.DrawWireSphere(worldAnchorB, 0.1f);
    }
}