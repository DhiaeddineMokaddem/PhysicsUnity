using UnityEngine;

/// <summary>
/// Contrainte élastique entre deux segments avec possibilité de rupture
/// </summary>
public class ElasticConstraint : MonoBehaviour
{
    public RigidSegment segmentA;
    public RigidSegment segmentB;
    
    // Point d'attache en coordonnées locales
    public Vector3 localAnchorA;
    public Vector3 localAnchorB;
    
    // Paramètres de la contrainte
    public float stiffness = 500f;      // Rigidité du ressort (k)
    public float damping = 20f;         // Amortissement
    public float breakForce = 100f;     // Force de rupture
    
    // État
    public bool isBroken = false;
    
    // Longueur au repos
    private float restLength;
    
    // Énergie stockée dans le ressort
    private float storedEnergy = 0f;
    
    public void Initialize()
    {
        if (segmentA != null && segmentB != null)
        {
            Vector3 anchorWorldA = segmentA.TransformPoint(localAnchorA);
            Vector3 anchorWorldB = segmentB.TransformPoint(localAnchorB);
            restLength = Vector3.Distance(anchorWorldA, anchorWorldB);
            isBroken = false;
        }
    }
    
    /// <summary>
    /// Résout la contrainte en appliquant des forces aux segments
    /// </summary>
    public void SolveConstraint(float dt)
    {
        if (isBroken || segmentA == null || segmentB == null) return;
        if (segmentA.isFixed && segmentB.isFixed) return;
        
        // Positions mondiales des points d'attache
        Vector3 anchorWorldA = segmentA.TransformPoint(localAnchorA);
        Vector3 anchorWorldB = segmentB.TransformPoint(localAnchorB);
        
        // Vecteur de séparation
        Vector3 delta = anchorWorldB - anchorWorldA;
        float currentLength = delta.magnitude;
        
        if (currentLength < 0.0001f) return;
        
        Vector3 direction = delta / currentLength;
        
        // Déformation (extension ou compression)
        float extension = currentLength - restLength;
        
        // Force de rappel élastique (loi de Hooke: F = -k * x)
        float springForce = -stiffness * extension;
        
        // Vitesses aux points d'attache
        Vector3 velA = segmentA.isFixed ? Vector3.zero : segmentA.GetVelocityAtPoint(anchorWorldA);
        Vector3 velB = segmentB.isFixed ? Vector3.zero : segmentB.GetVelocityAtPoint(anchorWorldB);
        Vector3 relativeVel = velB - velA;
        
        // Vitesse relative le long de la direction du ressort
        float relativeVelAlongDir = Vector3.Dot(relativeVel, direction);
        
        // Force d'amortissement (proportionnelle à la vitesse relative)
        float dampingForce = -damping * relativeVelAlongDir;
        
        // Force totale
        float totalForce = springForce + dampingForce;
        Vector3 force = direction * totalForce;
        
        // Calculer l'énergie stockée
        storedEnergy = 0.5f * stiffness * extension * extension;
        
        // Vérifier si la force dépasse le seuil de rupture
        if (Mathf.Abs(totalForce) > breakForce)
        {
            Break();
            return;
        }
        
        // Appliquer les forces
        if (!segmentA.isFixed)
        {
            segmentA.AddForceAtPoint(force, anchorWorldA);
        }
        
        if (!segmentB.isFixed)
        {
            segmentB.AddForceAtPoint(-force, anchorWorldB);
        }
    }
    
    /// <summary>
    /// Casse la contrainte et libère l'énergie stockée
    /// </summary>
    public void Break()
    {
        if (isBroken) return;
        
        isBroken = true;
        Debug.Log($"Contrainte cassée! Énergie libérée: {storedEnergy:F2} J");
    }
    
    /// <summary>
    /// Obtient l'énergie actuellement stockée dans le ressort
    /// </summary>
    public float GetStoredEnergy()
    {
        return storedEnergy;
    }
    
    /// <summary>
    /// Obtient la tension actuelle (force)
    /// </summary>
    public float GetTension()
    {
        if (isBroken || segmentA == null || segmentB == null) return 0f;
        
        Vector3 anchorWorldA = segmentA.TransformPoint(localAnchorA);
        Vector3 anchorWorldB = segmentB.TransformPoint(localAnchorB);
        
        float currentLength = Vector3.Distance(anchorWorldA, anchorWorldB);
        float extension = currentLength - restLength;
        
        return Mathf.Abs(stiffness * extension);
    }
    
    void OnDrawGizmos()
    {
        if (segmentA == null || segmentB == null) return;
        
        Vector3 startPos, endPos;
        
        if (Application.isPlaying)
        {
            startPos = segmentA.TransformPoint(localAnchorA);
            endPos = segmentB.TransformPoint(localAnchorB);
        }
        else
        {
            startPos = segmentA.transform.position;
            endPos = segmentB.transform.position;
        }
        
        if (isBroken)
        {
            Gizmos.color = Color.red;
        }
        else
        {
            float tension = GetTension();
            float normalizedTension = Mathf.Clamp01(tension / breakForce);
            Gizmos.color = Color.Lerp(Color.cyan, Color.yellow, normalizedTension);
        }
        
        Gizmos.DrawLine(startPos, endPos);
        
        // Dessiner les points d'attache
        Gizmos.DrawSphere(startPos, 0.05f);
        Gizmos.DrawSphere(endPos, 0.05f);
    }
}