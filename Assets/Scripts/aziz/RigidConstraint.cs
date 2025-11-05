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
    public float maxDistance = 1.0f;
    public float breakForce = 100.0f;
    public float stiffness = 1000.0f;
    public float damping = 50.0f;
    
    [Header("État")]
    public bool isBroken = false;
    
    // Points d'ancrage locaux
    private Vector3 localAnchorA;
    private Vector3 localAnchorB;
    private float restDistance;
    private float accumulatedForce = 0f;
    
    // NOUVEAU: Flag pour désactivation complète
    private bool isActive = true;

    void Start()
    {
        if (bodyA != null && bodyB != null)
        {
            localAnchorA = bodyA.transform.InverseTransformPoint(transform.position);
            localAnchorB = bodyB.transform.InverseTransformPoint(transform.position);
            restDistance = Vector3.Distance(bodyA.transform.position, bodyB.transform.position);
        }
    }

    /// <summary>
    /// CORRECTION MAJEURE: Ne fait RIEN si cassée
    /// </summary>
    public void SolveConstraint(float deltaTime)
    {
        // ARRÊT IMMÉDIAT si cassée ou inactive
        if (isBroken || !isActive || !enabled) return;
        if (bodyA == null || bodyB == null) return;
        if (bodyA.isKinematic && bodyB.isKinematic) return;

        Vector3 worldAnchorA = bodyA.transform.TransformPoint(localAnchorA);
        Vector3 worldAnchorB = bodyB.transform.TransformPoint(localAnchorB);
        
        Vector3 delta = worldAnchorB - worldAnchorA;
        float currentDistance = delta.magnitude;
        
        // Vérifier la rupture par distance
        if (currentDistance > maxDistance)
        {
            Break();
            return; // IMPORTANT: Sortir immédiatement
        }
        
        if (currentDistance < 0.0001f) return;
        
        Vector3 direction = delta / currentDistance;
        
        float error = currentDistance - restDistance;
        
        Vector3 velA = bodyA.GetVelocityAtPoint(worldAnchorA);
        Vector3 velB = bodyB.GetVelocityAtPoint(worldAnchorB);
        Vector3 relativeVel = velB - velA;
        float relativeVelAlongConstraint = Vector3.Dot(relativeVel, direction);
        
        float springForce = error * stiffness;
        float dampingForce = relativeVelAlongConstraint * damping;
        float totalForce = springForce + dampingForce;
        
        Vector3 force = direction * totalForce;
        
        accumulatedForce += Mathf.Abs(totalForce) * deltaTime;
        
        // Vérifier la rupture par force
        if (Mathf.Abs(totalForce) > breakForce)
        {
            Break();
            return; // IMPORTANT: Sortir immédiatement
        }
        
        // Appliquer les forces SEULEMENT si toujours active
        if (isActive && !isBroken)
        {
            if (!bodyA.isKinematic)
            {
                bodyA.AddForceAtPoint(force, worldAnchorA);
            }
            
            if (!bodyB.isKinematic)
            {
                bodyB.AddForceAtPoint(-force, worldAnchorB);
            }
        }
    }

    /// <summary>
    /// AMÉLIORATION: Désactivation totale lors de la rupture
    /// </summary>
    public void Break()
    {
        if (!isBroken)
        {
            isBroken = true;
            isActive = false; // NOUVEAU: Désactiver complètement
            enabled = false; // NOUVEAU: Désactiver le composant
            
            // Optionnel: Détruire le renderer pour visualisation
            Renderer renderer = GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.enabled = false;
            }
            
            Debug.Log($"Contrainte rompue: {bodyA.name} <-> {bodyB.name}");
        }
    }

    /// <summary>
    /// Répare la contrainte
    /// </summary>
    public void Repair()
    {
        isBroken = false;
        isActive = true;
        enabled = true;
        accumulatedForce = 0f;
        
        Renderer renderer = GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.enabled = true;
        }
        
        // Recalculer la distance de repos
        if (bodyA != null && bodyB != null)
        {
            restDistance = Vector3.Distance(bodyA.transform.position, bodyB.transform.position);
        }
    }

    void OnDrawGizmos()
    {
        if (bodyA == null || bodyB == null) return;
        
        // Ne rien dessiner si cassée
        if (isBroken || !isActive)
        {
            return;
        }
        
        Vector3 worldAnchorA = bodyA.transform.TransformPoint(localAnchorA);
        Vector3 worldAnchorB = bodyB.transform.TransformPoint(localAnchorB);
        
        float stress = accumulatedForce / (breakForce * 10f);
        Gizmos.color = Color.Lerp(Color.green, Color.yellow, stress);
        
        Gizmos.DrawLine(worldAnchorA, worldAnchorB);
        Gizmos.DrawWireSphere(worldAnchorA, 0.05f);
        Gizmos.DrawWireSphere(worldAnchorB, 0.05f);
    }

    void OnDrawGizmosSelected()
    {
        if (bodyA == null || bodyB == null) return;
        if (isBroken) return;
        
        Vector3 worldAnchorA = bodyA.transform.TransformPoint(localAnchorA);
        Vector3 worldAnchorB = bodyB.transform.TransformPoint(localAnchorB);
        
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(worldAnchorA, worldAnchorB);
        
        // Afficher info de rupture
        Gizmos.color = new Color(1f, 0f, 0f, 0.2f);
        Gizmos.DrawWireSphere(worldAnchorA, maxDistance);
    }
}