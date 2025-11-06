using UnityEngine;

/// <summary>
/// Contrainte personnalisée entre deux corps rigides
/// Permet de connecter les segments de la poutre
/// </summary>
public class CustomJoint
{
    public CustomRigidBody bodyA;
    public CustomRigidBody bodyB;
    
    public Vector3 anchorA; // Point d'ancrage local à A
    public Vector3 anchorB; // Point d'ancrage local à B
    
    public float stiffness = 2000f;
    public float damping = 100f;
    public float breakForce = 200f;
    
    public bool isBroken = false;
    
    private float restLength;

    public CustomJoint(CustomRigidBody a, CustomRigidBody b, Vector3 localAnchorA, Vector3 localAnchorB)
    {
        bodyA = a;
        bodyB = b;
        anchorA = localAnchorA;
        anchorB = localAnchorB;
        
        // Calculer la distance de repos
        Vector3 worldAnchorA = GetWorldAnchorA();
        Vector3 worldAnchorB = GetWorldAnchorB();
        restLength = Vector3.Distance(worldAnchorA, worldAnchorB);
    }

    /// <summary>
    /// Obtient la position mondiale de l'ancre A
    /// </summary>
    public Vector3 GetWorldAnchorA()
    {
        return bodyA.position + bodyA.rotation * anchorA;
    }

    /// <summary>
    /// Obtient la position mondiale de l'ancre B
    /// </summary>
    public Vector3 GetWorldAnchorB()
    {
        return bodyB.position + bodyB.rotation * anchorB;
    }

    /// <summary>
    /// Résout la contrainte (appelé à chaque pas de physique)
    /// </summary>
    public void SolveConstraint(float deltaTime)
    {
        if (isBroken || bodyA == null || bodyB == null)
            return;

        Vector3 worldAnchorA = GetWorldAnchorA();
        Vector3 worldAnchorB = GetWorldAnchorB();
        
        Vector3 delta = worldAnchorB - worldAnchorA;
        float currentLength = delta.magnitude;
        
        if (currentLength < 0.0001f) return;
        
        Vector3 direction = delta / currentLength;
        float extension = currentLength - restLength;
        
        // Force de ressort (loi de Hooke)
        float springForce = -stiffness * extension;
        
        // Vitesses aux points d'ancrage
        Vector3 velA = bodyA.isStatic ? Vector3.zero : bodyA.GetVelocityAtPoint(worldAnchorA);
        Vector3 velB = bodyB.isStatic ? Vector3.zero : bodyB.GetVelocityAtPoint(worldAnchorB);
        Vector3 relativeVel = velB - velA;
        
        // Amortissement
        float dampingForce = -damping * Vector3.Dot(relativeVel, direction);
        
        // Force totale
        float totalForce = springForce + dampingForce;
        Vector3 force = direction * totalForce;
        
        // Vérifier si la contrainte doit se casser
        if (Mathf.Abs(totalForce) > breakForce)
        {
            Break();
            return;
        }
        
        // Appliquer les forces
        if (!bodyA.isStatic)
        {
            bodyA.AddForceAtPoint(force, worldAnchorA);
        }
        
        if (!bodyB.isStatic)
        {
            bodyB.AddForceAtPoint(-force, worldAnchorB);
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
        
        if (bodyA != null && bodyB != null)
        {
            Vector3 worldAnchorA = GetWorldAnchorA();
            Vector3 worldAnchorB = GetWorldAnchorB();
            restLength = Vector3.Distance(worldAnchorA, worldAnchorB);
        }
    }

    /// <summary>
    /// Obtient la tension actuelle
    /// </summary>
    public float GetTension()
    {
        if (isBroken || bodyA == null || bodyB == null) return 0f;
        
        Vector3 worldAnchorA = GetWorldAnchorA();
        Vector3 worldAnchorB = GetWorldAnchorB();
        float currentLength = Vector3.Distance(worldAnchorA, worldAnchorB);
        float extension = currentLength - restLength;
        
        return Mathf.Abs(stiffness * extension);
    }

    /// <summary>
    /// Dessine un gizmo pour visualiser la contrainte
    /// </summary>
    public void DrawGizmo()
    {
        if (bodyA == null || bodyB == null) return;

        Vector3 worldAnchorA = GetWorldAnchorA();
        Vector3 worldAnchorB = GetWorldAnchorB();

        if (isBroken)
        {
            Gizmos.color = Color.red;
        }
        else
        {
            float tension = GetTension();
            float normalizedTension = Mathf.Clamp01(tension / breakForce);
            Gizmos.color = Color.Lerp(Color.cyan, Color.red, normalizedTension);
        }

        Gizmos.DrawLine(worldAnchorA, worldAnchorB);
        Gizmos.DrawSphere(worldAnchorA, 0.05f);
        Gizmos.DrawSphere(worldAnchorB, 0.05f);
    }
}