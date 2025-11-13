// Import Unity's core engine functionality
using UnityEngine;
// Import C# generic collections
using System.Collections.Generic;
// Import Aziz's physics namespace for RigidBody3D
using PhysicsSimulation.Indiv_Work.Aziz;
// Import core physics utilities
using PhysicsSimulation.Core;

/// <summary>
/// Sphere that impacts the structure of cubes - PURE MATH VERSION
/// FIXED: Proper initialization order using Awake()
/// Uses VisualRenderer for visual updates only - no direct transform manipulation
/// ALL POSITION UPDATES DONE MANUALLY - NO transform.position!
/// </summary>
public class ImpactSphere : MonoBehaviour
{
    [Header("Sphere Properties")] // Unity Inspector header
    // Radius of the sphere in meters - determines size and collision volume
    public float radius = 1.0f;
    // Mass in kilograms - determines momentum and impact force
    public float mass = 5.0f;
    // Linear velocity in meters per second - rate of position change
    public Vector3 velocity = Vector3.zero;
    // Coefficient of restitution (0 = inelastic, 1 = perfectly elastic)
    public float restitution = 0.3f;
    
    [Header("Impact")] // Unity Inspector header for impact settings
    // Multiplier for impact force transferred to cubes on collision
    public float impactMultiplier = 0.5f;
    // Radius within which cubes are affected on impact
    public float breakRadius = 3.0f;
    // If true, sphere launches automatically on Start()
    public bool autoLaunch = false;
    // Direction to launch the sphere when autoLaunch is true
    public Vector3 launchDirection = Vector3.down;
    // Initial speed when launching in meters per second
    public float launchSpeed = 10f;
    
    [Header("Physical Limits")] // Unity Inspector header for safety limits
    // Maximum impulse that can be applied per collision (prevents explosions)
    public float maxImpulsePerCollision = 50f;
    // Fraction of energy lost per collision (0 = no loss, 1 = all energy lost)
    [Range(0f, 1f)]
    public float energyLossPerCollision = 0.3f;
    
    [Header("Visualization")] // Unity Inspector header for visual settings
    // Color of the sphere in debug visualization
    public Color sphereColor = Color.red;
    // If true, shows trajectory and velocity vectors
    public bool showTrajectory = true;
    
    // PURE MATH: Position stored manually (NOT using Unity Transform!)
    // This is the center of the sphere in world space
    [HideInInspector] public Vector3 position;
    
    // Reference to collision detection system
    private CollisionDetector collisionDetector;
    // Reference to physics manager that coordinates simulation
    private PhysicsManager physicsManager;
    // Current acceleration in meters per second squared (from gravity and forces)
    private Vector3 acceleration = Vector3.zero;
    // Flag tracking if sphere has impacted the structure yet
    private bool hasImpacted = false;
    // Original starting position for reset functionality
    private Vector3 startPosition;
    // Set of bodies collided with this frame (prevents duplicate collision responses)
    private HashSet<RigidBody3D> collidedThisFrame = new HashSet<RigidBody3D>();
    // Total number of collisions that have occurred
    private int collisionCount = 0;
    // Reference to visual renderer that updates mesh vertices manually
    private VisualRenderer visualRenderer;

    /// <summary>
    /// Unity lifecycle method - called when script instance is being loaded
    /// FIXED: Use Awake() for immediate initialization before any other scripts run
    /// </summary>
    void Awake()
    {
        // Try to get existing VisualRenderer component
        visualRenderer = GetComponent<VisualRenderer>();
        // If none exists, add one to handle manual mesh transformation
        if (visualRenderer == null)
        {
            visualRenderer = gameObject.AddComponent<VisualRenderer>();
        }

        // PURE MATH: Initialize position from VisualRenderer (reads GameObject's initial position)
        position = visualRenderer.GetPosition();
        // Store starting position for potential reset
        startPosition = position;
    }

    // Unity lifecycle method - called after all Awake() calls
    void Start()
    {
        // Find the PhysicsManager in the scene
        physicsManager = FindObjectOfType<PhysicsManager>();
        // Get CollisionDetector from the PhysicsManager
        collisionDetector = physicsManager?.GetComponent<CollisionDetector>();
        
        // Safety check: verify collision detector exists
        if (collisionDetector == null)
        {
            Debug.LogError("ImpactSphere: CollisionDetector not found!");
        }
        
        // If autoLaunch enabled, start moving immediately
        if (autoLaunch)
        {
            Launch();
        }
        
        // Update visual mesh to match initial position
        UpdateVisualTransform();
    }

    // Launch sphere with configured direction and speed
    public void Launch()
    {
        // Set velocity: normalize direction and multiply by speed
        velocity = launchDirection.normalized * launchSpeed;
        // Reset impact flag
        hasImpacted = false;
        // Reset collision counter
        collisionCount = 0;
    }

    // Launch sphere towards a specific target position
    public void LaunchTowards(Vector3 target)
    {
        // Calculate direction from current position to target
        Vector3 direction = (target - position).normalized;
        // Set velocity towards target at configured speed
        velocity = direction * launchSpeed;
        // Reset impact flag
        hasImpacted = false;
        // Reset collision counter
        collisionCount = 0;
    }

    // Unity fixed update - called at fixed time intervals for physics
    void FixedUpdate()
    {
        // Skip physics if simulation is paused
        if (physicsManager != null && physicsManager.pauseSimulation) return;
        
        // Get physics time step
        float deltaTime = Time.fixedDeltaTime;
        
        collidedThisFrame.Clear();
        
        // PURE MATH: Gravité
        acceleration = Vector3.down * 9.81f;
        
        // PURE MATH: Intégration manuelle
        velocity += acceleration * deltaTime;
        position += velocity * deltaTime;
        
        // Mettre à jour le Transform Unity pour le rendu
        UpdateVisualTransform();
        
        DetectCollisions();
        HandleGroundCollision();
    }

    // PURE MATH: Mise à jour du Transform Unity pour l'affichage
    void UpdateVisualTransform()
    {
        if (visualRenderer != null)
        {
            visualRenderer.UpdatePosition(position);
        }
    }

    void DetectCollisions()
    {
        if (collisionDetector == null) return;
        
        RigidBody3D[] rigidBodies = FindObjectsOfType<RigidBody3D>();
        
        foreach (var body in rigidBodies)
        {
            if (body == null || collidedThisFrame.Contains(body)) continue;
            
            CollisionInfo collision;
            if (collisionDetector.DetectSphereCollision(position, radius, body, out collision))
            {
                collidedThisFrame.Add(body);
                
                if (!hasImpacted)
                {
                    hasImpacted = true;
                    OnImpact(collision.contactPoint);
                }
                
                ResolveSphereCollision(collision, body);
                collisionCount++;
            }
        }
    }

    void ResolveSphereCollision(CollisionInfo collision, RigidBody3D cube)
    {
        if (cube.isKinematic) return;
        
        // PURE MATH: Calculs manuels de collision
        Vector3 cubeToSphere = position - cube.position;
        Vector3 normal = cubeToSphere.normalized;
        Vector3 contactPoint = collision.contactPoint;
        
        // Séparation des objets
        if (collision.penetrationDepth > 0.001f)
        {
            float totalSeparation = collision.penetrationDepth + 0.02f;
            float totalMass = mass + cube.mass;
            float sphereRatio = cube.mass / totalMass;
            float cubeRatio = mass / totalMass;
            
            // PURE MATH: Modifier directement les positions
            position += normal * totalSeparation * sphereRatio;
            cube.position -= normal * totalSeparation * cubeRatio;
            cube.UpdateVisualTransform();
            UpdateVisualTransform();
        }
        
        Vector3 cubeVel = cube.GetVelocityAtPoint(contactPoint);
        Vector3 relativeVel = velocity - cubeVel;
        float velAlongNormal = Vector3.Dot(relativeVel, normal);
        
        if (velAlongNormal >= -0.001f) 
        {
            return;
        }
        
        float e = Mathf.Min(restitution, cube.restitution) * 0.5f;
        
        float invMassSphere = 1.0f / mass;
        float invMassCube = 1.0f / cube.mass;
        float totalInvMass = invMassSphere + invMassCube;
        
        float j = -(1.0f + e) * velAlongNormal / totalInvMass;
        j = Mathf.Clamp(j, 0f, maxImpulsePerCollision);
        
        float kineticEnergyAlongNormal = 0.5f * mass * velAlongNormal * velAlongNormal;
        float maxImpulseFromEnergy = Mathf.Sqrt(2f * mass * kineticEnergyAlongNormal);
        j = Mathf.Min(j, maxImpulseFromEnergy);
        
        Vector3 impulse = normal * j;
        
        // PURE MATH: Appliquer directement à la vitesse
        velocity += impulse * invMassSphere;
        cube.AddImpulseAtPoint(-impulse, contactPoint);
        
        velocity *= (1f - energyLossPerCollision);
        
        // Friction
        Vector3 tangentVel = relativeVel - normal * velAlongNormal;
        if (tangentVel.magnitude > 0.001f)
        {
            Vector3 tangent = tangentVel.normalized;
            float frictionCoeff = 0.2f;
            float frictionMag = Mathf.Min(tangentVel.magnitude * 0.5f, Mathf.Abs(j) * frictionCoeff);
            Vector3 frictionImpulse = -tangent * frictionMag;
            
            velocity += frictionImpulse * invMassSphere;
            cube.AddImpulseAtPoint(-frictionImpulse, contactPoint);
        }
        
        if (collisionCount < 5)
        {
            Debug.Log($"Collision #{collisionCount}: Impulsion={j:F1} N·s, Vitesse sphère={velocity.magnitude:F2} m/s");
        }
    }

    void OnImpact(Vector3 impactPoint)
    {
        float kineticEnergy = 0.5f * mass * velocity.sqrMagnitude;
        float explosionForce = kineticEnergy * impactMultiplier;
        explosionForce = Mathf.Min(explosionForce, 500f);
        
        Debug.Log($"Impact à {impactPoint}");
        Debug.Log($"  Vitesse: {velocity.magnitude:F2} m/s");
        Debug.Log($"  Énergie: {kineticEnergy:F2} J");
        Debug.Log($"  Force explosion: {explosionForce:F2} N");
        
        if (physicsManager != null)
        {
            physicsManager.BreakConstraintsInRadius(impactPoint, breakRadius);
        }
        
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
        float bottomY = position.y - radius;
        
        if (bottomY <= groundLevel)
        {
            // PURE MATH: Modifier directement la position
            position = new Vector3(position.x, groundLevel + radius, position.z);
            UpdateVisualTransform();
            
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
        position = startPosition;
        velocity = Vector3.zero;
        hasImpacted = false;
        collisionCount = 0;
        UpdateVisualTransform();
    }

    void OnDrawGizmos()
    {
        // Utiliser la position stockée si en jeu, sinon Transform
        Vector3 drawPos = Application.isPlaying ? position : transform.position;
        
        Gizmos.color = sphereColor;
        Gizmos.DrawWireSphere(drawPos, radius);
        
        if (showTrajectory && Application.isPlaying)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(drawPos, drawPos + velocity * 0.3f);
            
            float ke = 0.5f * mass * velocity.sqrMagnitude;
            Gizmos.color = Color.Lerp(Color.green, Color.red, Mathf.Clamp01(ke / 500f));
            Gizmos.DrawWireSphere(drawPos, radius * 1.1f);
            
            if (collisionCount > 0)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(drawPos, radius * (1.2f + collisionCount * 0.1f));
            }
        }
        
        if (hasImpacted)
        {
            Gizmos.color = new Color(1f, 0.5f, 0f, 0.3f);
            Gizmos.DrawWireSphere(drawPos, breakRadius);
        }
    }

    void OnDrawGizmosSelected()
    {
        Vector3 drawPos = Application.isPlaying ? position : transform.position;
        
        Gizmos.color = new Color(1f, 0f, 0f, 0.2f);
        Gizmos.DrawWireSphere(drawPos, breakRadius);
        
#if UNITY_EDITOR
        if (Application.isPlaying)
        {
            UnityEditor.Handles.Label(
                drawPos + Vector3.up * (radius + 0.5f),
                $"Collisions: {collisionCount}\nVitesse: {velocity.magnitude:F2} m/s"
            );
        }
#endif
    }
}

