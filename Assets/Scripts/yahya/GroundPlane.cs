using UnityEngine;

/// <summary>
/// Plan de sol pour les collisions
/// </summary>
public class GroundPlane : MonoBehaviour
{
    public float height = 0f;
    public Vector3 normal = Vector3.up;
    public float restitution = 0.3f;
    public float friction = 0.5f;
    
    /// <summary>
    /// Détecte une collision avec un projectile
    /// </summary>
    public bool DetectCollision(Vector3 position, float radius, out float penetration)
    {
        penetration = 0f;
        
        // Distance du centre du projectile au plan
        float distanceToPlane = Vector3.Dot(position - new Vector3(0, height, 0), normal);
        
        if (distanceToPlane < radius)
        {
            penetration = radius - distanceToPlane;
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Résout la collision avec un projectile
    /// </summary>
    public void ResolveCollision(ImpactProjectile projectile, float penetration)
    {
        // Séparation
        if (penetration > 0.001f)
        {
            projectile.position += normal * penetration;
        }
        
        // Réponse de vélocité
        float velAlongNormal = Vector3.Dot(projectile.velocity, normal);
        
        if (velAlongNormal < -0.001f)
        {
            // Composante normale
            Vector3 normalVel = normal * velAlongNormal;
            Vector3 tangentVel = projectile.velocity - normalVel;
            
            // Appliquer la restitution
            projectile.velocity = tangentVel - normalVel * restitution;
            
            // Appliquer la friction
            if (tangentVel.magnitude > 0.001f)
            {
                Vector3 frictionForce = -tangentVel.normalized * Mathf.Min(tangentVel.magnitude * friction, tangentVel.magnitude);
                projectile.velocity += frictionForce;
            }
            
            // Réduire la vélocité angulaire
            projectile.angularVelocity *= 0.8f;
        }
    }
    
    /// <summary>
    /// Résout la collision avec un segment rigide
    /// </summary>
    public void ResolveSegmentCollision(RigidSegment segment, float penetration)
    {
        if (segment.isFixed) return;
        
        // Séparation
        if (penetration > 0.001f)
        {
            segment.position += normal * penetration;
        }
        
        // Réponse de vélocité
        float velAlongNormal = Vector3.Dot(segment.velocity, normal);
        
        if (velAlongNormal < -0.001f)
        {
            // Composante normale
            Vector3 normalVel = normal * velAlongNormal;
            Vector3 tangentVel = segment.velocity - normalVel;
            
            // Appliquer la restitution
            segment.velocity = tangentVel - normalVel * restitution;
            
            // Appliquer la friction
            if (tangentVel.magnitude > 0.001f)
            {
                Vector3 frictionForce = -tangentVel.normalized * Mathf.Min(tangentVel.magnitude * friction, tangentVel.magnitude);
                segment.velocity += frictionForce;
            }
            
            // Réduire la vélocité angulaire
            segment.angularVelocity *= 0.8f;
        }
    }
    
    /// <summary>
    /// Détecte une collision avec un segment (point le plus bas)
    /// </summary>
    public bool DetectSegmentCollision(RigidSegment segment, out float penetration)
    {
        penetration = 0f;
        
        // Vérifier les coins du segment
        Vector3 halfSize = segment.size * 0.5f;
        Vector3[] corners = new Vector3[4];
        
        // Coins du bas du segment (local space)
        corners[0] = new Vector3(-halfSize.x, -halfSize.y, -halfSize.z);
        corners[1] = new Vector3(halfSize.x, -halfSize.y, -halfSize.z);
        corners[2] = new Vector3(-halfSize.x, -halfSize.y, halfSize.z);
        corners[3] = new Vector3(halfSize.x, -halfSize.y, halfSize.z);
        
        float maxPenetration = 0f;
        bool hasCollision = false;
        
        foreach (Vector3 localCorner in corners)
        {
            // Transformer en world space
            Vector3 worldCorner = segment.TransformPoint(localCorner);
            
            // Distance au plan
            float distanceToPlane = Vector3.Dot(worldCorner - new Vector3(0, height, 0), normal);
            
            if (distanceToPlane < 0)
            {
                maxPenetration = Mathf.Max(maxPenetration, -distanceToPlane);
                hasCollision = true;
            }
        }
        
        penetration = maxPenetration;
        return hasCollision;
    }
    
    void OnDrawGizmos()
    {
        Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.5f);
        
        // Dessiner un plan au sol
        Vector3 center = new Vector3(0, height, 0);
        float size = 20f;
        
        Vector3 forward = Vector3.forward;
        Vector3 right = Vector3.right;
        
        if (Vector3.Dot(normal, Vector3.up) < 0.99f)
        {
            forward = Vector3.Cross(normal, Vector3.right).normalized;
            right = Vector3.Cross(normal, forward).normalized;
        }
        
        Vector3 p1 = center + forward * size + right * size;
        Vector3 p2 = center + forward * size - right * size;
        Vector3 p3 = center - forward * size - right * size;
        Vector3 p4 = center - forward * size + right * size;
        
        Gizmos.DrawLine(p1, p2);
        Gizmos.DrawLine(p2, p3);
        Gizmos.DrawLine(p3, p4);
        Gizmos.DrawLine(p4, p1);
        
        // Grille
        for (int i = -10; i <= 10; i++)
        {
            if (i == 0) continue;
            Gizmos.DrawLine(
                center + forward * i + right * size,
                center + forward * i - right * size
            );
            Gizmos.DrawLine(
                center + forward * size + right * i,
                center - forward * size + right * i
            );
        }
    }
}

