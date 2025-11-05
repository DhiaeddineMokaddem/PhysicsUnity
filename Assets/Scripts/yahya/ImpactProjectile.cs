using UnityEngine;

/// <summary>
/// Cube projectile lancé vers la poutre avec physique manuelle
/// </summary>
public class ImpactProjectile : MonoBehaviour
{
    // Propriétés physiques
    public float mass = 2f;
    public float size = 0.3f;
    public float launchSpeed = 15f;
    
    // État physique (stocké manuellement)
    [HideInInspector] public Vector3 position;
    [HideInInspector] public Quaternion rotation;
    [HideInInspector] public Vector3 velocity;
    [HideInInspector] public Vector3 angularVelocity;
    
    private Vector3 startPosition;
    private Quaternion startRotation;
    private bool isLaunched = false;
    
    private const float GRAVITY = 9.81f;
    
    public void Initialize(Vector3 pos, Quaternion rot)
    {
        position = pos;
        rotation = rot;
        startPosition = pos;
        startRotation = rot;
        velocity = Vector3.zero;
        angularVelocity = Vector3.zero;
        isLaunched = false;
        
        UpdateVisualTransform();
    }
    
    /// <summary>
    /// Lance le projectile dans une direction donnée
    /// </summary>
    public void Launch(Vector3 direction)
    {
        velocity = direction.normalized * launchSpeed;
        
        // Ajouter une rotation aléatoire pour l'effet visuel
        angularVelocity = Random.onUnitSphere * 2f;
        
        isLaunched = true;
        Debug.Log($"Projectile lancé à {launchSpeed} m/s");
    }
    
    /// <summary>
    /// Intègre la physique du projectile
    /// </summary>
    public void IntegratePhysics(float dt)
    {
        if (!isLaunched) return;
        
        // Gravité
        velocity += Vector3.down * GRAVITY * dt;
        
        // Intégration de la position
        position += velocity * dt;
        
        // Intégration de la rotation
        if (angularVelocity.magnitude > 0.001f)
        {
            float angle = angularVelocity.magnitude * dt;
            Vector3 axis = angularVelocity.normalized;
            Quaternion deltaRotation = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);
            rotation = deltaRotation * rotation;
            rotation.Normalize();
        }
        
        UpdateVisualTransform();
    }
    
    /// <summary>
    /// Détecte la collision avec un segment
    /// </summary>
    public bool DetectCollisionWithSegment(RigidSegment segment, out Vector3 collisionPoint, out Vector3 collisionNormal, out float penetration)
    {
        collisionPoint = Vector3.zero;
        collisionNormal = Vector3.zero;
        penetration = 0f;
        
        // Collision sphère-boîte simplifiée
        // Transformer la position du projectile dans le repère local du segment
        Vector3 localPos = segment.InverseTransformPoint(position);
        Vector3 halfSize = segment.size * 0.5f;
        float radius = size * 0.5f;
        
        // Point le plus proche dans la boîte
        Vector3 closestLocal = new Vector3(
            Mathf.Clamp(localPos.x, -halfSize.x, halfSize.x),
            Mathf.Clamp(localPos.y, -halfSize.y, halfSize.y),
            Mathf.Clamp(localPos.z, -halfSize.z, halfSize.z)
        );
        
        // Retransformer en coordonnées mondiales
        Vector3 closestWorld = segment.TransformPoint(closestLocal);
        
        // Distance entre le centre du projectile et le point le plus proche
        Vector3 delta = position - closestWorld;
        float distSq = delta.sqrMagnitude;
        
        if (distSq < radius * radius)
        {
            float dist = Mathf.Sqrt(distSq);
            
            if (dist < 0.001f)
            {
                // Le centre est à l'intérieur - utiliser la direction du segment au projectile
                collisionNormal = (position - segment.position).normalized;
                penetration = radius;
            }
            else
            {
                collisionNormal = delta / dist;
                penetration = radius - dist;
            }
            
            collisionPoint = closestWorld;
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Résout une collision avec un segment
    /// </summary>
    public void ResolveCollision(RigidSegment segment, Vector3 collisionPoint, Vector3 collisionNormal, float penetration)
    {
        // Séparation des objets
        if (penetration > 0.001f)
        {
            position += collisionNormal * (penetration + 0.01f);
            UpdateVisualTransform();
        }
        
        // Vitesse relative au point de collision
        Vector3 segmentVel = segment.GetVelocityAtPoint(collisionPoint);
        Vector3 relativeVel = velocity - segmentVel;
        float velAlongNormal = Vector3.Dot(relativeVel, collisionNormal);
        
        // Ne résoudre que si les objets s'approchent
        if (velAlongNormal >= -0.001f) return;
        
        // Coefficient de restitution
        float restitution = 0.4f;
        
        // Calcul de l'impulsion
        float invMassProjectile = 1f / mass;
        float invMassSegment = segment.isFixed ? 0f : 1f / segment.mass;
        
        // Position relative pour le calcul du couple
        Vector3 r = collisionPoint - segment.position;
        
        // Terme de l'inertie rotationnelle (simplifié)
        float angularTerm = 0f;
        if (!segment.isFixed)
        {
            // Approximation : on considère que le segment a une inertie uniforme
            float inertiaFactor = 1f / (segment.mass * segment.size.magnitude * 0.5f);
            Vector3 rCrossN = Vector3.Cross(r, collisionNormal);
            angularTerm = rCrossN.sqrMagnitude * inertiaFactor;
        }
        
        float denom = invMassProjectile + invMassSegment + angularTerm;
        denom = Mathf.Max(denom, 1e-6f);
        
        float j = -(1f + restitution) * velAlongNormal / denom;
        j = Mathf.Clamp(j, 0f, 100f); // Limiter l'impulsion
        
        Vector3 impulse = collisionNormal * j;
        
        // Appliquer l'impulsion au projectile
        velocity += impulse * invMassProjectile;
        
        // Appliquer l'impulsion au segment
        if (!segment.isFixed)
        {
            segment.AddImpulseAtPoint(-impulse, collisionPoint);
        }
        
        // Friction
        Vector3 tangentVel = relativeVel - collisionNormal * velAlongNormal;
        if (tangentVel.magnitude > 0.001f)
        {
            Vector3 tangent = tangentVel.normalized;
            float frictionCoeff = 0.3f;
            float frictionMag = Mathf.Min(tangentVel.magnitude * 0.5f, Mathf.Abs(j) * frictionCoeff);
            Vector3 frictionImpulse = -tangent * frictionMag;
            
            velocity += frictionImpulse * invMassProjectile;
            if (!segment.isFixed)
            {
                segment.AddImpulseAtPoint(-frictionImpulse, collisionPoint);
            }
        }
    }
    
    /// <summary>
    /// Réinitialise le projectile
    /// </summary>
    public void Reset()
    {
        position = startPosition;
        rotation = startRotation;
        velocity = Vector3.zero;
        angularVelocity = Vector3.zero;
        isLaunched = false;
        UpdateVisualTransform();
    }
    
    /// <summary>
    /// Met à jour le Transform Unity pour le rendu
    /// </summary>
    private void UpdateVisualTransform()
    {
        transform.position = position;
        transform.rotation = rotation;
    }
    
    /// <summary>
    /// Obtient l'énergie cinétique actuelle
    /// </summary>
    public float GetKineticEnergy()
    {
        return 0.5f * mass * velocity.sqrMagnitude;
    }
    
    // Utilitaires mathématiques
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
        Gizmos.DrawLine(position, position + velocity * 0.3f);
        
        // Dessiner une sphère de collision
        Gizmos.color = isLaunched ? Color.yellow : Color.gray;
        Gizmos.DrawWireSphere(position, size * 0.5f);
    }
}