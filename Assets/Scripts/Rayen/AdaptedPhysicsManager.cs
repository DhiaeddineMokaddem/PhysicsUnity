using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Gestionnaire principal de physique
/// ADAPTÉ de PhysicsManager.cs pour gérer sphère statique + plan horizontal
/// 
/// RÉUTILISE : 
/// - Logique d'intégration physique
/// - Gestion contraintes
/// - Collisions cube-cube et cube-sol
/// 
/// MODIFICATIONS :
/// - Ajout collision avec sphère statique
/// - Méthodes pour rupture et explosion adaptées
/// </summary>
public class AdaptedPhysicsManager : MonoBehaviour
{[Header("Configuration Temporelle")]
    public float timeStep = 0.02f; // 50 Hz
    public int substeps = 2;
    
    [Header("Élasticité Globale (Alpha)")]
    [Range(0f, 2f)]
    [Tooltip("Paramètre d'élasticité global - Influence restitution et amortissement")]
    public float globalElasticity = 0.6f;
    
    [Header("Sol")]
    public float groundLevel = 0f;
    public float groundRestitution = 0.35f; // Augmenté pour plus de rebond
    public float groundFriction = 0.8f; // Augmenté pour moins de glissement
    
    [Header("Sphère Statique")]
    [Tooltip("Référence à la sphère d'impact statique")]
    public StaticImpactSphere staticSphere;
    
    [Header("Debugging")]
    public bool showDebugInfo = true;
    public bool pauseSimulation = false;
    
    // Collections internes
    private List<RigidBody3D> rigidBodies = new List<RigidBody3D>();
    private List<RigidConstraint> constraints = new List<RigidConstraint>();
    private CollisionDetector collisionDetector;

    void Start()
    {
        collisionDetector = gameObject.AddComponent<CollisionDetector>();
        
        if (staticSphere == null)
        {
            staticSphere = FindObjectOfType<StaticImpactSphere>();
        }
        
        RegisterAllBodies();
    }

    public void RegisterAllBodies()
    {
        rigidBodies.Clear();
        constraints.Clear();
        
        rigidBodies.AddRange(FindObjectsOfType<RigidBody3D>());
        constraints.AddRange(FindObjectsOfType<RigidConstraint>());
        
        Debug.Log($"PhysicsManager: {rigidBodies.Count} corps, {constraints.Count} contraintes");
    }

    public void RegisterBody(RigidBody3D body)
    {
        if (!rigidBodies.Contains(body))
            rigidBodies.Add(body);
    }

    public void RegisterConstraint(RigidConstraint constraint)
    {
        if (!constraints.Contains(constraint))
            constraints.Add(constraint);
    }

    void FixedUpdate()
    {
        if (pauseSimulation) return;
        
        float deltaTime = timeStep / substeps;
        
        for (int i = 0; i < substeps; i++)
        {
            // 1. Résoudre contraintes ACTIVES uniquement
            SolveConstraints(deltaTime);
            
            // 2. Intégrer physique
            IntegratePhysics(deltaTime);
            
            // 3. Collisions cube-cube
            DetectAndResolveCollisions();
            
            // 4. Collisions avec sphère statique
            HandleStaticSphereCollisions();
            
            // 5. Collisions sol
            HandleGroundCollisions();
        }
    }

    /// <summary>
    /// CORRECTION: Ne résout que les contraintes NON cassées
    /// </summary>
    void SolveConstraints(float deltaTime)
    {
        foreach (var constraint in constraints)
        {
            // IMPORTANT: Ignorer complètement les contraintes cassées
            if (constraint != null && !constraint.isBroken && constraint.enabled)
            {
                constraint.SolveConstraint(deltaTime);
            }
        }
    }

    void IntegratePhysics(float deltaTime)
    {
        foreach (var body in rigidBodies)
        {
            if (body != null)
            {
                body.IntegratePhysics(deltaTime);
            }
        }
    }

    void DetectAndResolveCollisions()
    {
        for (int i = 0; i < rigidBodies.Count; i++)
        {
            for (int j = i + 1; j < rigidBodies.Count; j++)
            {
                if (rigidBodies[i] == null || rigidBodies[j] == null) continue;
                
                CollisionInfo collision;
                if (collisionDetector.DetectCubeCollision(rigidBodies[i], rigidBodies[j], out collision))
                {
                    collisionDetector.ResolveCollision(collision, globalElasticity);
                }
            }
        }
    }

    /// <summary>
    /// CORRECTION: Meilleure gestion collisions sphère
    /// </summary>
    void HandleStaticSphereCollisions()
    {
        if (staticSphere == null) return;
        
        Vector3 sphereCenter = staticSphere.transform.position;
        float sphereRadius = staticSphere.radius;
        
        foreach (var body in rigidBodies)
        {
            if (body == null || body.isKinematic) continue;
            
            CollisionInfo collision;
            if (collisionDetector.DetectSphereCollision(sphereCenter, sphereRadius, body, out collision))
            {
                float combinedRestitution = staticSphere.restitution * globalElasticity;
                ResolveStaticSphereCollision(collision, body, combinedRestitution);
            }
        }
    }

    /// <summary>
    /// CORRECTION: Résolution collision améliorée
    /// </summary>
    void ResolveStaticSphereCollision(CollisionInfo collision, RigidBody3D cube, float restitution)
    {
        Vector3 normal = collision.contactNormal;
        Vector3 contactPoint = collision.contactPoint;
        
        // 1. SÉPARATION IMMÉDIATE pour éviter l'attraction
        if (collision.penetrationDepth > 0.001f)
        {
            // Pousser le cube LOIN de la sphère avec marge de sécurité
            float separationDistance = collision.penetrationDepth + 0.02f;
            cube.transform.position -= normal * separationDistance;
        }
        
        // 2. Vitesse au point de contact
        Vector3 cubeVel = cube.GetVelocityAtPoint(contactPoint);
        float velAlongNormal = Vector3.Dot(cubeVel, normal);
        
        // Si le cube s'éloigne déjà, ne rien faire
        if (velAlongNormal >= 0f) return;
        
        // 3. Impulsion de collision (sphère = masse infinie)
        float j = -(1f + restitution) * velAlongNormal * cube.mass;
        Vector3 impulse = normal * j;
        
        cube.AddImpulseAtPoint(impulse, contactPoint);
        
        // 4. Friction tangentielle
        Vector3 tangentVel = cubeVel - normal * velAlongNormal;
        if (tangentVel.magnitude > 0.001f)
        {
            Vector3 tangent = tangentVel.normalized;
            float frictionCoeff = 0.3f;
            float frictionMag = Mathf.Min(tangentVel.magnitude, Mathf.Abs(j) * frictionCoeff / cube.mass);
            Vector3 frictionImpulse = -tangent * frictionMag * cube.mass;
            cube.AddImpulseAtPoint(frictionImpulse, contactPoint);
        }
    }

    void HandleGroundCollisions()
    {
        foreach (var body in rigidBodies)
        {
            if (body == null || body.isKinematic) continue;
            Vector3 pos = body.transform.position;
            float halfHeight = body.size.y * 0.5f;
            float bottomY = pos.y - halfHeight;
            if (bottomY <= groundLevel)
            {
                // Correction plus forte pour éviter stuck/tunneling
                body.transform.position = new Vector3(pos.x, groundLevel + halfHeight + 0.01f, pos.z);
                if (body.velocity.y < 0)
                {
                    body.velocity.y = -body.velocity.y * groundRestitution * globalElasticity;
                    Vector3 horizontalVel = new Vector3(body.velocity.x, 0, body.velocity.z);
                    horizontalVel *= (1f - groundFriction);
                    body.velocity = new Vector3(horizontalVel.x, body.velocity.y, horizontalVel.z);
                    body.angularVelocity *= (1f - groundFriction);
                }
            }
        }
    }

    public void ApplyExplosion(Vector3 center, float radius, float force)
    {
        foreach (var body in rigidBodies)
        {
            if (body == null || body.isKinematic) continue;
            
            Vector3 direction = body.transform.position - center;
            float distance = direction.magnitude;
            
            if (distance < radius && distance > 0.001f)
            {
                float falloff = 1f - (distance / radius);
                falloff = falloff * falloff;
                
                Vector3 explosionDir = direction.normalized;
                Vector3 explosionForce = explosionDir * force * falloff;
                
                body.AddImpulse(explosionForce / body.mass);
                
                Vector3 randomTorque = new Vector3(
                    Random.Range(-0.5f, 0.5f),
                    Random.Range(-0.5f, 0.5f),
                    Random.Range(-0.5f, 0.5f)
                ) * force * falloff * 0.01f;
                body.AddTorque(randomTorque);
            }
        }
    }

    /// <summary>
    /// CORRECTION: Désactive complètement les contraintes cassées
    /// </summary>
    public void BreakConstraintsInRadius(Vector3 center, float radius)
    {
        int brokenCount = 0;
        foreach (var constraint in constraints)
        {
            if (constraint == null || constraint.isBroken) continue;
            
            Vector3 constraintPos = constraint.transform.position;
            float distance = Vector3.Distance(constraintPos, center);
            
            if (distance < radius)
            {
                constraint.Break();
                constraint.enabled = false; // NOUVEAU: Désactiver complètement
                brokenCount++;
            }
        }
        Debug.Log($"Cassé {brokenCount} contraintes dans rayon {radius}m");
    }

    public string GetSimulationStats()
    {
        int activeBodies = 0;
        int activeConstraints = 0;
        int brokenConstraints = 0;
        float totalEnergy = 0f;
        
        foreach (var body in rigidBodies)
        {
            if (body != null && !body.isKinematic)
            {
                activeBodies++;
                totalEnergy += body.GetKineticEnergy();
            }
        }
        
        foreach (var constraint in constraints)
        {
            if (constraint != null)
            {
                if (!constraint.isBroken)
                    activeConstraints++;
                else
                    brokenConstraints++;
            }
        }
        
        string stats = $"Corps actifs: {activeBodies}\n" +
               $"Contraintes actives: {activeConstraints}\n" +
               $"Contraintes cassées: {brokenConstraints}\n" +
               $"Énergie: {totalEnergy:F2} J\n" +
               $"Élasticité α: {globalElasticity:F2}";
        
        if (staticSphere != null)
        {
            stats += $"\n\n{staticSphere.GetStats()}";
        }
        
        return stats;
    }

    public void ResetSimulation()
    {
        foreach (var constraint in constraints)
        {
            if (constraint != null)
            {
                constraint.Repair();
                constraint.enabled = true; // NOUVEAU: Réactiver
            }
        }
        
        foreach (var body in rigidBodies)
        {
            if (body != null)
            {
                body.velocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
        
        if (staticSphere != null)
        {
            staticSphere.Reset();
        }
    }

    void OnDrawGizmos()
    {
        if (!showDebugInfo) return;
        
        Gizmos.color = Color.gray;
        Gizmos.DrawWireCube(new Vector3(0, groundLevel - 0.5f, 0), new Vector3(50, 1, 50));
    }
}