using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Gestionnaire principal de la simulation physique - VERSION PURE MATH
/// FIXED: Better handling of null references and body registration
/// </summary>
public class PhysicsManagerYahya : MonoBehaviour
{
    [Header("Configuration")]
    public float timeStep = 0.02f;
    public int substeps = 2;
    public float globalElasticity = 0.8f;
    
    [Header("Sol")]
    public float groundLevel = 0f;
    public float groundRestitution = 0.2f;
    public float groundFriction = 0.6f;
    
    [Header("Debugging")]
    public bool showDebugInfo = true;
    public bool pauseSimulation = false;
    
    private List<RigidBody3DYahya> rigidBodies = new List<RigidBody3DYahya>();
    private List<RigidConstraintYahya> constraints = new List<RigidConstraintYahya>();
    private CollisionDetectorYahya collisionDetector;
    
    private float accumulator = 0f;

    void Start()
    {
        collisionDetector = gameObject.AddComponent<CollisionDetectorYahya>();
        RegisterAllBodies();
    }

    /// <summary>
    /// FIXED: Improved registration that handles null references properly
    /// </summary>
    public void RegisterAllBodies()
    {
        // Clear and remove null references
        rigidBodies.RemoveAll(body => body == null);
        constraints.RemoveAll(constraint => constraint == null);
        
        // Find all current bodies and constraints
        RigidBody3DYahya[] foundBodies = FindObjectsOfType<RigidBody3DYahya>();
        RigidConstraintYahya[] foundConstraints = FindObjectsOfType<RigidConstraintYahya>();
        
        // Only add bodies that aren't already registered
        foreach (var body in foundBodies)
        {
            if (!rigidBodies.Contains(body))
            {
                rigidBodies.Add(body);
            }
        }
        
        // Only add constraints that aren't already registered
        foreach (var constraint in foundConstraints)
        {
            if (!constraints.Contains(constraint))
            {
                constraints.Add(constraint);
            }
        }
        
        Debug.Log($"PhysicsManager: {rigidBodies.Count} corps rigides et {constraints.Count} contraintes enregistrés");
    }

    public void RegisterBody(RigidBody3DYahya body)
    {
        if (!rigidBodies.Contains(body))
        {
            rigidBodies.Add(body);
        }
    }

    public void RegisterConstraint(RigidConstraintYahya constraint)
    {
        if (!constraints.Contains(constraint))
        {
            constraints.Add(constraint);
        }
    }

    /// <summary>
    /// FIXED: Cleans up null references at the start of each frame
    /// </summary>
    void FixedUpdate()
    {
        if (pauseSimulation) return;
        
        // Clean up null references at the start of each frame
        rigidBodies.RemoveAll(body => body == null);
        constraints.RemoveAll(constraint => constraint == null);
        
        float deltaTime = timeStep / substeps;
        
        for (int i = 0; i < substeps; i++)
        {
            SolveConstraints(deltaTime);
            IntegratePhysics(deltaTime);
            DetectAndResolveCollisions();
            HandleGroundCollisions();
        }
    }

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
                
                CollisionInfoYahya collision;
                if (collisionDetector.DetectCubeCollision(rigidBodies[i], rigidBodies[j], out collision))
                {
                    collisionDetector.ResolveCollision(collision, globalElasticity);
                }
            }
        }
    }

    /// <summary>
    /// Gère les collisions avec le sol - PURE MATH
    /// </summary>
    void HandleGroundCollisions()
    {
        foreach (var body in rigidBodies)
        {
            if (body == null || body.isKinematic) continue;
            
            // PURE MATH: Utiliser la position stockée
            Vector3 pos = body.position;
            float halfHeight = body.size.y * 0.5f;
            float bottomY = pos.y - halfHeight;
            
            if (bottomY <= groundLevel)
            {
                // PURE MATH: Modifier directement la position
                body.position = new Vector3(pos.x, groundLevel + halfHeight, pos.z);
                body.UpdateVisualTransform();
                
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

    /// <summary>
    /// Applique une force explosive - PURE MATH
    /// </summary>
    public void ApplyExplosion(Vector3 center, float radius, float force)
    {
        foreach (var body in rigidBodies)
        {
            if (body == null || body.isKinematic) continue;
            
            // PURE MATH: Utiliser la position stockée
            Vector3 direction = body.position - center;
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
        
        Gizmos.color = Color.gray;
        Gizmos.DrawWireCube(new Vector3(0, groundLevel - 0.5f, 0), new Vector3(50, 1, 50));
    }

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