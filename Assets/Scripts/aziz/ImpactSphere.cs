using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Sphère qui impacte la structure de cubes - VERSION RÉALISTE
/// </summary>
public class ImpactSphere : MonoBehaviour
{
    [Header("Propriétés de la Sphère")]
    public float radius = 1.0f;
    public float mass = 5.0f; // RÉDUIT de 15 à 5 kg pour moins d'impact
    public Vector3 velocity = Vector3.zero;
    public float restitution = 0.3f; // RÉDUIT de 0.6 à 0.3 pour moins de rebond
    
    [Header("Impact")]
    [Tooltip("Multiplicateur de force basé sur l'énergie cinétique")]
    public float impactMultiplier = 0.5f; // RÉDUIT de 2.0 à 0.5
    public float breakRadius = 3.0f;
    public bool autoLaunch = false;
    public Vector3 launchDirection = Vector3.down;
    public float launchSpeed = 10f;
    
    [Header("Limites physiques")]
    [Tooltip("Impulsion maximale par collision (limite la vitesse de transfert)")]
    public float maxImpulsePerCollision = 50f;
    [Tooltip("Perte d'énergie de la sphère à chaque collision")]
    [Range(0f, 1f)]
    public float energyLossPerCollision = 0.3f; // La sphère perd 30% d'énergie par collision
    
    [Header("Visualisation")]
    public Color sphereColor = Color.red;
    public bool showTrajectory = true;
    
    private CollisionDetector collisionDetector;
    private PhysicsManager physicsManager;
    private Vector3 acceleration = Vector3.zero;
    private bool hasImpacted = false;
    private Vector3 startPosition;
    private HashSet<RigidBody3D> collidedThisFrame = new HashSet<RigidBody3D>();
    private int collisionCount = 0;

    void Start()
    {
        physicsManager = FindObjectOfType<PhysicsManager>();
        collisionDetector = physicsManager?.GetComponent<CollisionDetector>();
        
        if (collisionDetector == null)
        {
            Debug.LogError("ImpactSphere: CollisionDetector non trouvé!");
        }
        
        startPosition = transform.position;
        
        if (autoLaunch)
        {
            Launch();
        }
    }

    public void Launch()
    {
        velocity = launchDirection.normalized * launchSpeed;
        hasImpacted = false;
        collisionCount = 0;
    }

    public void LaunchTowards(Vector3 target)
    {
        Vector3 direction = (target - transform.position).normalized;
        velocity = direction * launchSpeed;
        hasImpacted = false;
        collisionCount = 0;
    }

    void FixedUpdate()
    {
        if (physicsManager != null && physicsManager.pauseSimulation) return;
        
        float deltaTime = Time.fixedDeltaTime;
        
        collidedThisFrame.Clear();
        
        // Appliquer la gravité
        acceleration = Vector3.down * 9.81f;
        
        // Intégrer la vitesse et la position
        velocity += acceleration * deltaTime;
        transform.position += velocity * deltaTime;
        
        // Détecter les collisions avec les cubes
        DetectCollisions();
        
        // Collision avec le sol
        HandleGroundCollision();
    }

    void DetectCollisions()
    {
        if (collisionDetector == null) return;
        
        RigidBody3D[] rigidBodies = FindObjectsOfType<RigidBody3D>();
        
        foreach (var body in rigidBodies)
        {
            if (body == null || collidedThisFrame.Contains(body)) continue;
            
            CollisionInfo collision;
            if (collisionDetector.DetectSphereCollision(transform.position, radius, body, out collision))
            {
                collidedThisFrame.Add(body);
                
                // Première collision = impact principal
                if (!hasImpacted)
                {
                    hasImpacted = true;
                    OnImpact(collision.contactPoint);
                }
                
                // Résoudre la collision
                ResolveSphereCollision(collision, body);
                collisionCount++;
            }
        }
    }

    /// <summary>
    /// RÉSOLUTION DE COLLISION RÉALISTE
    /// </summary>
    void ResolveSphereCollision(CollisionInfo collision, RigidBody3D cube)
    {
        if (cube.isKinematic) return;
        
        // Direction de la normale (du cube vers la sphère)
        Vector3 cubeToSphere = transform.position - cube.transform.position;
        Vector3 normal = cubeToSphere.normalized;
        Vector3 contactPoint = collision.contactPoint;
        
        // Séparation des objets
        if (collision.penetrationDepth > 0.001f)
        {
            float totalSeparation = collision.penetrationDepth + 0.02f;
            
            // Répartir la séparation selon les masses
            float totalMass = mass + cube.mass;
            float sphereRatio = cube.mass / totalMass;
            float cubeRatio = mass / totalMass;
            
            transform.position += normal * totalSeparation * sphereRatio;
            cube.transform.position -= normal * totalSeparation * cubeRatio;
        }
        
        // Calcul de vitesse relative
        Vector3 cubeVel = cube.GetVelocityAtPoint(contactPoint);
        Vector3 relativeVel = velocity - cubeVel;
        float velAlongNormal = Vector3.Dot(relativeVel, normal);
        
        // Si les objets s'éloignent déjà, pas de collision
        if (velAlongNormal >= -0.001f) 
        {
            return;
        }
        
        // ========================================
        // CORRECTION PRINCIPALE: Conservation de quantité de mouvement réaliste
        // ========================================
        
        // Coefficient de restitution combiné (plus faible = moins de rebond)
        float e = Mathf.Min(restitution, cube.restitution) * 0.5f; // Prendre le minimum et réduire
        
        // Masses effectives
        float invMassSphere = 1.0f / mass;
        float invMassCube = 1.0f / cube.mass;
        float totalInvMass = invMassSphere + invMassCube;
        
        // Impulsion scalaire (formule standard)
        float j = -(1.0f + e) * velAlongNormal / totalInvMass;
        
        // ========================================
        // LIMITE 1: Impulsion maximale par collision
        // ========================================
        j = Mathf.Clamp(j, 0f, maxImpulsePerCollision);
        
        // ========================================
        // LIMITE 2: L'impulsion ne peut pas dépasser l'énergie disponible
        // ========================================
        // Énergie cinétique de la sphère le long de la normale
        float kineticEnergyAlongNormal = 0.5f * mass * velAlongNormal * velAlongNormal;
        float maxImpulseFromEnergy = Mathf.Sqrt(2f * mass * kineticEnergyAlongNormal);
        j = Mathf.Min(j, maxImpulseFromEnergy);
        
        Vector3 impulse = normal * j;
        
        // Application de l'impulsion
        velocity += impulse * invMassSphere;
        cube.AddImpulseAtPoint(-impulse, contactPoint);
        
        // ========================================
        // PERTE D'ÉNERGIE DE LA SPHÈRE
        // ========================================
        // La sphère perd de l'énergie à chaque collision (déformation, chaleur, etc.)
        velocity *= (1f - energyLossPerCollision);
        
        // Friction tangentielle (réduite)
        Vector3 tangentVel = relativeVel - normal * velAlongNormal;
        if (tangentVel.magnitude > 0.001f)
        {
            Vector3 tangent = tangentVel.normalized;
            float frictionCoeff = 0.2f; // RÉDUIT de 0.3 à 0.2
            float frictionMag = Mathf.Min(tangentVel.magnitude * 0.5f, Mathf.Abs(j) * frictionCoeff);
            Vector3 frictionImpulse = -tangent * frictionMag;
            
            velocity += frictionImpulse * invMassSphere;
            cube.AddImpulseAtPoint(-frictionImpulse, contactPoint);
        }
        
        // Debug info
        if (collisionCount < 5)
        {
            Debug.Log($"Collision #{collisionCount}: Impulsion={j:F1} N·s, Vitesse sphère={velocity.magnitude:F2} m/s");
        }
    }

    /// <summary>
    /// IMPACT CORRIGÉ - Force proportionnelle mais limitée
    /// </summary>
    void OnImpact(Vector3 impactPoint)
    {
        // Énergie cinétique de la sphère
        float kineticEnergy = 0.5f * mass * velocity.sqrMagnitude;
        
        // Force d'explosion proportionnelle mais limitée
        float explosionForce = kineticEnergy * impactMultiplier;
        
        // Limiter la force maximale pour éviter les effets exagérés
        explosionForce = Mathf.Min(explosionForce, 500f); // Maximum 500 N
        
        Debug.Log($"Impact à {impactPoint}");
        Debug.Log($"  Vitesse: {velocity.magnitude:F2} m/s");
        Debug.Log($"  Énergie: {kineticEnergy:F2} J");
        Debug.Log($"  Force explosion: {explosionForce:F2} N");
        
        if (physicsManager != null)
        {
            // Casser les contraintes AVANT l'explosion
            physicsManager.BreakConstraintsInRadius(impactPoint, breakRadius);
        }
        
        // Appliquer l'explosion avec délai
        StartCoroutine(ApplyExplosionDelayed(impactPoint, explosionForce));
    }
    
    System.Collections.IEnumerator ApplyExplosionDelayed(Vector3 impactPoint, float force)
    {
        yield return new WaitForFixedUpdate();
        
        if (physicsManager != null)
        {
            physicsManager.ApplyExplosion(impactPoint, breakRadius, force);
        }
    }

    void HandleGroundCollision()
    {
        float groundLevel = physicsManager != null ? physicsManager.groundLevel : 0f;
        float bottomY = transform.position.y - radius;
        
        if (bottomY <= groundLevel)
        {
            transform.position = new Vector3(
                transform.position.x,
                groundLevel + radius,
                transform.position.z
            );
            
            if (velocity.y < 0)
            {
                float groundRestitution = physicsManager != null ? physicsManager.groundRestitution : 0.3f;
                velocity.y = -velocity.y * groundRestitution;
                
                float groundFriction = physicsManager != null ? physicsManager.groundFriction : 0.5f;
                velocity.x *= (1f - groundFriction);
                velocity.z *= (1f - groundFriction);
            }
        }
    }

    public void Reset()
    {
        transform.position = startPosition;
        velocity = Vector3.zero;
        hasImpacted = false;
        collisionCount = 0;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = sphereColor;
        Gizmos.DrawWireSphere(transform.position, radius);
        
        if (showTrajectory && Application.isPlaying)
        {
            // Visualiser la vitesse
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(transform.position, transform.position + velocity * 0.3f);
            
            // Couleur selon l'énergie
            float ke = 0.5f * mass * velocity.sqrMagnitude;
            Gizmos.color = Color.Lerp(Color.green, Color.red, Mathf.Clamp01(ke / 500f));
            Gizmos.DrawWireSphere(transform.position, radius * 1.1f);
            
            // Afficher le nombre de collisions
            if (collisionCount > 0)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(transform.position, radius * (1.2f + collisionCount * 0.1f));
            }
        }
        
        if (hasImpacted)
        {
            Gizmos.color = new Color(1f, 0.5f, 0f, 0.3f);
            Gizmos.DrawWireSphere(transform.position, breakRadius);
        }
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = new Color(1f, 0f, 0f, 0.2f);
        Gizmos.DrawWireSphere(transform.position, breakRadius);
        
        // Info texte
        if (Application.isPlaying)
        {
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * (radius + 0.5f),
                $"Collisions: {collisionCount}\nVitesse: {velocity.magnitude:F2} m/s"
            );
        }
    }
}