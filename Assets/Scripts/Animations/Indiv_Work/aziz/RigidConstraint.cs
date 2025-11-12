using PhysicsUnity.Indiv_Work.Aziz;
using UnityEngine;

/// <summary>
/// Contrainte rigide entre deux corps - VERSION PURE MATH
/// FIXED: Proper initialization to prevent immediate breaking
/// </summary>
public class RigidConstraint : MonoBehaviour
{
    [Header("Corps Connectés")]
    public RigidBody3D bodyA;
    public RigidBody3D bodyB;

    [Header("Propriétés de la Contrainte")]
    public float stiffness = 500.0f;
    public float damping = 10.0f;
    public float breakForce = 100.0f;
    public float maxDistance = 2.0f;

    [Header("État")]
    public bool isBroken = false;
    public bool showConstraint = true;
    public Color constraintColor = Color.cyan;

    private float restLength;
    private Vector3 localAnchorA;
    private Vector3 localAnchorB;

    void Start()
    {
        // Don't initialize here - will be called from Initialize() method
        if (restLength == 0 && bodyA != null && bodyB != null)
        {
            Initialize();
        }
    }

    /// <summary>
    /// Initialise la contrainte manuellement
    /// </summary>
    public void Initialize()
    {
        if (bodyA != null && bodyB != null)
        {
            // PURE MATH: Calculer la distance initiale
            restLength = Vector3.Distance(bodyA.position, bodyB.position);
            
            // Calculer les points d'ancrage locaux
            Vector3 connectionPoint = (bodyA.position + bodyB.position) * 0.5f;
            localAnchorA = bodyA.InverseTransformPoint(connectionPoint);
            localAnchorB = bodyB.InverseTransformPoint(connectionPoint);
            
            // S'assurer que la contrainte n'est pas cassée
            isBroken = false;
        }
    }

    /// <summary>
    /// Résout la contrainte - PURE MATH
    /// FIXED: Check if properly initialized before solving
    /// </summary>
    public void SolveConstraint(float deltaTime)
    {
        if (isBroken || bodyA == null || bodyB == null) return;

        // Safety check: ensure constraint is initialized
        if (restLength <= 0.001f)
        {
            Initialize();
            return;  // Skip this frame to let initialization settle
        }
        
        // PURE MATH: Points d'ancrage en coordonnées mondiales
        Vector3 anchorWorldA = bodyA.TransformPoint(localAnchorA);
        Vector3 anchorWorldB = bodyB.TransformPoint(localAnchorB);
        
        Vector3 delta = anchorWorldB - anchorWorldA;
        float currentLength = delta.magnitude;
        
        if (currentLength < 0.0001f) return;
        
        // Vérifier si la contrainte doit casser
        float extension = currentLength - restLength;
        if (Mathf.Abs(extension) > maxDistance)
        {
            Break();
            return;
        }
        
        Vector3 direction = delta / currentLength;
        
        // Calculer la force de rappel (loi de Hooke)
        float springForce = -stiffness * extension;
        
        // Calculer la vitesse relative au point d'ancrage
        Vector3 velA = bodyA.isKinematic ? Vector3.zero : bodyA.GetVelocityAtPoint(anchorWorldA);
        Vector3 velB = bodyB.isKinematic ? Vector3.zero : bodyB.GetVelocityAtPoint(anchorWorldB);
        Vector3 relativeVel = velB - velA;
        
        // Amortissement le long de la direction du ressort
        float dampingForce = -damping * Vector3.Dot(relativeVel, direction);
        
        // Force totale
        float totalForce = springForce + dampingForce;
        Vector3 force = direction * totalForce;
        
        // Vérifier la force de rupture
        if (Mathf.Abs(totalForce) > breakForce)
        {
            Break();
            return;
        }
        
        // Appliquer les forces
        if (!bodyA.isKinematic)
        {
            bodyA.AddForceAtPoint(force, anchorWorldA);
        }
        
        if (!bodyB.isKinematic)
        {
            bodyB.AddForceAtPoint(-force, anchorWorldB);
        }
    }

    /// <summary>
    /// Casse la contrainte
    /// </summary>
    public void Break()
    {
        if (!isBroken)
        {
            isBroken = true;
            Debug.Log($"Contrainte cassée entre {bodyA?.name} et {bodyB?.name}");
        }
    }

    /// <summary>
    /// Répare la contrainte
    /// </summary>
    public void Repair()
    {
        isBroken = false;
        
        // Re-initialize to get current positions
        if (bodyA != null && bodyB != null)
        {
            restLength = Vector3.Distance(bodyA.position, bodyB.position);
            
            Vector3 connectionPoint = (bodyA.position + bodyB.position) * 0.5f;
            localAnchorA = bodyA.InverseTransformPoint(connectionPoint);
            localAnchorB = bodyB.InverseTransformPoint(connectionPoint);
        }
    }

    /// <summary>
    /// Obtient la tension actuelle - PURE MATH
    /// </summary>
    public float GetTension()
    {
        if (isBroken || bodyA == null || bodyB == null) return 0f;

        Vector3 anchorWorldA = bodyA.TransformPoint(localAnchorA);
        Vector3 anchorWorldB = bodyB.TransformPoint(localAnchorB);

        float currentLength = Vector3.Distance(anchorWorldA, anchorWorldB);
        float extension = currentLength - restLength;

        return stiffness * Mathf.Abs(extension);
    }

    void OnDrawGizmos()
    {
        if (!showConstraint || bodyA == null || bodyB == null) return;
        
        if (isBroken)
        {
            Gizmos.color = Color.red;
        }
        else
        {
            // PURE MATH: Utiliser les positions stockées
            float tension = GetTension();
            float normalizedTension = Mathf.Clamp01(tension / breakForce);
            Gizmos.color = Color.Lerp(constraintColor, Color.red, normalizedTension);
        }
        
        // Dessiner la ligne de contrainte
        Vector3 startPos = Application.isPlaying ? bodyA.position : bodyA.transform.position;
        Vector3 endPos = Application.isPlaying ? bodyB.position : bodyB.transform.position;
        
        Gizmos.DrawLine(startPos, endPos);
        
        // Dessiner les points d'ancrage
        if (Application.isPlaying)
        {
            Vector3 anchorA = bodyA.TransformPoint(localAnchorA);
            Vector3 anchorB = bodyB.TransformPoint(localAnchorB);
            
            Gizmos.DrawSphere(anchorA, 0.05f);
            Gizmos.DrawSphere(anchorB, 0.05f);
        }
    }

    void OnDrawGizmosSelected()
    {
        if (bodyA == null || bodyB == null) return;
        
        // Afficher les informations de la contrainte
        Vector3 midPoint = Application.isPlaying 
            ? (bodyA.position + bodyB.position) * 0.5f
            : (bodyA.transform.position + bodyB.transform.position) * 0.5f;
        
#if UNITY_EDITOR
        string info = isBroken ? "CASSÉE" : $"Tension: {GetTension():F1}N";
        UnityEditor.Handles.Label(midPoint, info);
#endif
        
        // Dessiner la distance maximale
        if (!isBroken)
        {
            Gizmos.color = new Color(1f, 1f, 0f, 0.3f);
            Vector3 startPos = Application.isPlaying ? bodyA.position : bodyA.transform.position;
            Gizmos.DrawWireSphere(startPos, restLength + maxDistance);
        }
    }
}