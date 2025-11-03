using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Gestionnaire principal de la simulation physique
/// </summary>
public class PhysicsManager : MonoBehaviour
{
    [Header("Configuration")]
    public float timeStep = 0.02f; // Pas de temps fixe (50 Hz)
    public int substeps = 2; // Nombre de sous-étapes pour plus de stabilité
    public float globalElasticity = 0.8f; // Paramètre alpha d'élasticité global
    
    [Header("Sol")]
    public float groundLevel = 0f;
    public float groundRestitution = 0.2f;
    public float groundFriction = 0.6f;
    
    [Header("Debugging")]
    public bool showDebugInfo = true;
    public bool pauseSimulation = false;
    
    private List<RigidBody3D> rigidBodies = new List<RigidBody3D>();
    private List<RigidConstraint> constraints = new List<RigidConstraint>();
    private CollisionDetector collisionDetector;
    
    private float accumulator = 0f;

    void Start()
    {
        collisionDetector = gameObject.AddComponent<CollisionDetector>();
        RegisterAllBodies();
    }

    /// <summary>
    /// Enregistre tous les corps rigides de la scène
    /// </summary>
    public void RegisterAllBodies()
    {
        rigidBodies.Clear();
        constraints.Clear();
        
        rigidBodies.AddRange(FindObjectsOfType<RigidBody3D>());
        constraints.AddRange(FindObjectsOfType<RigidConstraint>());
        
        Debug.Log($"PhysicsManager: {rigidBodies.Count} corps rigides et {constraints.Count} contraintes enregistrés");
    }

    /// <summary>
    /// Enregistre un corps rigide
    /// </summary>
    public void RegisterBody(RigidBody3D body)
    {
        if (!rigidBodies.Contains(body))
        {
            rigidBodies.Add(body);
        }
    }

    /// <summary>
    /// Enregistre une contrainte
    /// </summary>
    public void RegisterConstraint(RigidConstraint constraint)
    {
        if (!constraints.Contains(constraint))
        {
            constraints.Add(constraint);
        }
    }

    void FixedUpdate()
    {
        if (pauseSimulation) return;
        
        float deltaTime = timeStep / substeps;
        
        for (int i = 0; i < substeps; i++)
        {
            // 1. Résoudre les contraintes
            SolveConstraints(deltaTime);
            
            // 2. Intégrer la physique
            IntegratePhysics(deltaTime);
            
            // 3. Détecter et résoudre les collisions
            DetectAndResolveCollisions();
            
            // 4. Gérer les collisions avec le sol
            HandleGroundCollisions();
        }
    }

    /// <summary>
    /// Résout toutes les contraintes
    /// </summary>
    void SolveConstraints(float deltaTime)
    {
        foreach (var constraint in constraints)
        {
            if (constraint != null && !constraint.isBroken)
            {
                constraint.SolveConstraint(deltaTime);
            }
        }
    }

    /// <summary>
    /// Intègre la physique pour tous les corps
    /// </summary>
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

    /// <summary>
    /// Détecte et résout les collisions entre les cubes
    /// </summary>
    void DetectAndResolveCollisions()
    {
        // Collision entre cubes
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
    /// Gère les collisions avec le sol
    /// </summary>
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
                // Repositionner l'objet
                body.transform.position = new Vector3(pos.x, groundLevel + halfHeight, pos.z);
                
                // Appliquer la réponse de collision
                if (body.velocity.y < 0)
                {
                    // Réflexion de la vitesse verticale
                    body.velocity.y = -body.velocity.y * groundRestitution * globalElasticity;
                    
                    // Amortissement de la vitesse horizontale (friction)
                    Vector3 horizontalVel = new Vector3(body.velocity.x, 0, body.velocity.z);
                    horizontalVel *= (1f - groundFriction);
                    body.velocity = new Vector3(horizontalVel.x, body.velocity.y, horizontalVel.z);
                    
                    // Amortir la vitesse angulaire
                    body.angularVelocity *= (1f - groundFriction);
                }
            }
        }
    }

    /// <summary>
    /// Applique une force explosive à tous les corps dans un rayon
    /// </summary>
    public void ApplyExplosion(Vector3 center, float radius, float force)
    {
        foreach (var body in rigidBodies)
        {
            if (body == null || body.isKinematic) continue;
            
            Vector3 direction = body.transform.position - center;
            float distance = direction.magnitude;
            
            if (distance < radius && distance > 0.001f)
            {
                // Force diminue avec le carré de la distance (plus réaliste)
                float falloff = 1f - (distance / radius);
                falloff = falloff * falloff; // Courbe quadratique
                
                // Direction normalisée
                Vector3 explosionDir = direction.normalized;
                
                // Appliquer la force explosive principalement vers l'extérieur
                Vector3 explosionForce = explosionDir * force * falloff;
                
                // Appliquer comme impulsion pour un effet instantané
                body.AddImpulse(explosionForce / body.mass);
                
                // Ajouter une rotation aléatoire minimale (réaliste)
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
    /// Casse toutes les contraintes dans un rayon
    /// </summary>
    public void BreakConstraintsInRadius(Vector3 center, float radius)
    {
        foreach (var constraint in constraints)
        {
            if (constraint == null || constraint.isBroken) continue;
            
            Vector3 constraintPos = constraint.transform.position;
            float distance = Vector3.Distance(constraintPos, center);
            
            if (distance < radius)
            {
                constraint.Break();
            }
        }
    }

    void OnDrawGizmos()
    {
        if (!showDebugInfo) return;
        
        // Dessiner le sol
        Gizmos.color = Color.gray;
        Gizmos.DrawWireCube(new Vector3(0, groundLevel - 0.5f, 0), new Vector3(50, 1, 50));
        
        // Statistiques
        if (Application.isPlaying)
        {
            int activeBodies = 0;
            int brokenConstraints = 0;
            
            foreach (var body in rigidBodies)
            {
                if (body != null && !body.isKinematic) activeBodies++;
            }
            
            foreach (var constraint in constraints)
            {
                if (constraint != null && constraint.isBroken) brokenConstraints++;
            }
            
            // Ces informations seraient affichées dans la console ou via UI
        }
    }

    /// <summary>
    /// Obtient des statistiques sur la simulation
    /// </summary>
    public string GetSimulationStats()
    {
        int activeBodies = 0;
        int activeConstraints = 0;
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
            if (constraint != null && !constraint.isBroken)
            {
                activeConstraints++;
            }
        }
        
        return $"Corps actifs: {activeBodies}\n" +
               $"Contraintes actives: {activeConstraints}/{constraints.Count}\n" +
               $"Énergie cinétique totale: {totalEnergy:F2} J\n" +
               $"Élasticité globale: {globalElasticity:F2}";
    }

    /// <summary>
    /// Réinitialise la simulation
    /// </summary>
    public void ResetSimulation()
    {
        foreach (var constraint in constraints)
        {
            if (constraint != null)
            {
                constraint.Repair();
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
    }
}