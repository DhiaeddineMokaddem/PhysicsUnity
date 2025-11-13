using UnityEngine;
using System.Collections.Generic;
using PhysicsSimulation.Core;

/// <summary>
/// Sphère qui impacte la structure de cubes - VERSION PURE MATH
/// MODIFIÉ: Impact horizontal avec force importante
/// FIXED: Collision detection and high-velocity impact
/// Uses VisualRenderer for visual updates only - no direct transform manipulation
/// </summary>
public class ImpactSphereYahya : MonoBehaviour
{
    [Header("Proprits de la Sphre")]
    public float radius = 1.0f;
    public float mass = 10.0f; // Augment pour plus d'impact
    public Vector3 velocity = Vector3.zero;
    public float restitution = 0.4f;
    
    [Header("Gravit")]
    public bool useGravity = true;
    public float gravityStrength = 9.81f; // Gravit (m/s²)
    
    [Header("Impact Horizontal")]
    public float impactMultiplier = 0.8f; // Rduit pour impact moins violent
    public float breakRadius = 4.0f; // Augment
    public bool autoLaunch = false; // Dsactiv par dfaut - attendre Space
    public enum LaunchSide { Left, Right }
    public LaunchSide launchFrom = LaunchSide.Right;
    public float launchSpeed = 75f; // Vitesse TRS RAPIDE pour longue distance
    public float launchHeight = 5.0f; // Higher to avoid immediate ground collision
    public float launchDistance = 15f; // Plus loin pour mieux acclrer
    
    [Header("Air Resistance")]
    public bool useAirResistance = false; // Disabled by default for maximum speed
    public float airResistanceCoeff = 0.01f; // Very low air resistance
    
    [Header("Limites physiques")]
    public float maxImpulsePerCollision = 150f; // Rduit pour moins de violence
    [Range(0f, 1f)]
    public float energyLossPerCollision = 0.02f; // Trs faible pour distances longues
    
    [Header("Visualisation")]
    public Color sphereColor = Color.red;
    public bool showTrajectory = true;
    public bool showDebugInfo = true;
    
    // PURE MATH: Position stockée manuellement
    [HideInInspector] public Vector3 position;
    
    private CollisionDetectorYahya collisionDetector;
    private PhysicsManagerYahya physicsManager;
    private Vector3 acceleration = Vector3.zero;
    private bool hasImpacted = false;
    private Vector3 startPosition;
    private HashSet<RigidBody3DYahya> collidedThisFrame = new HashSet<RigidBody3DYahya>();
    private int collisionCount = 0;
    private bool isLaunched = false;
    private VisualRenderer visualRenderer;

    void Awake()
    {
        visualRenderer = GetComponent<VisualRenderer>();
        if (visualRenderer == null)
        {
            visualRenderer = gameObject.AddComponent<VisualRenderer>();
        }

        // Position de départ horizontale
        float xPos = launchFrom == LaunchSide.Right ? launchDistance : -launchDistance;
        position = new Vector3(xPos, launchHeight, 0f);
        visualRenderer.UpdatePosition(position);
        startPosition = position;
        
        Debug.Log($"Sphère initialisée à position: {position}");
    }

    void Start()
    {
        physicsManager = FindObjectOfType<PhysicsManagerYahya>();
        
        if (physicsManager != null)
        {
            collisionDetector = physicsManager.GetComponent<CollisionDetectorYahya>();
        }
        
        if (collisionDetector == null)
        {
            Debug.LogError("CollisionDetector non trouvé! Ajout automatique...");
            collisionDetector = physicsManager.gameObject.AddComponent<CollisionDetectorYahya>();
        }
        
        UpdateVisualTransform();
        
        if (autoLaunch)
        {
            // Lancer après un court délai
            Invoke("Launch", 0.5f);
        }
    }

    public void Launch()
    {
        Vector3 direction = launchFrom == LaunchSide.Right ? Vector3.left : Vector3.right;
        velocity = direction * launchSpeed;
        hasImpacted = false;
        collisionCount = 0;
        isLaunched = true;
        
        Debug.Log("••••••••••••••••••••••••••••••••••••••••••••••");
        Debug.Log($"ŠŸŠŸ LANCEMENT SUPER RAPIDE! ŠŸŠŸ");
        Debug.Log($"  Direction: {direction}");
        Debug.Log($"  Vitesse: {launchSpeed} m/s");
        Debug.Log($"  ‰nergie cintique: {0.5f * mass * launchSpeed * launchSpeed:F0} J");
        Debug.Log($"  Gravit active: {useGravity}");
        Debug.Log("••••••••••••••••••••••••••••••••••••••••••••••");
    }

    public void LaunchTowards(Vector3 target)
    {
        Vector3 targetHorizontal = new Vector3(target.x, position.y, target.z);
        Vector3 direction = (targetHorizontal - position).normalized;
        velocity = direction * launchSpeed;
        hasImpacted = false;
        collisionCount = 0;
        isLaunched = true;
        
        Debug.Log($"Lancement vers {target}, Direction: {direction}");
    }

    void FixedUpdate()
    {
        if (physicsManager != null && physicsManager.pauseSimulation) return;
        
        float deltaTime = Time.fixedDeltaTime;
        
        collidedThisFrame.Clear();
        
        // Gravit (mme si la sphre n'est pas lance)
        if (useGravity)
        {
            acceleration = Vector3.down * gravityStrength;
        }
        else
        {
            acceleration = Vector3.zero;
        }
        
        // Intgration
        velocity += acceleration * deltaTime;
        
        // Optional air resistance (disabled by default for max speed)
        if (useAirResistance && velocity.magnitude > 0.1f)
        {
            Vector3 airResistance = -velocity.normalized * velocity.sqrMagnitude * airResistanceCoeff;
            velocity += airResistance * deltaTime;
        }
        
        position += velocity * deltaTime;
        
        UpdateVisualTransform();
        
        // CRITIQUE: Dtection de collision mme si pas lance (pour gravit)
        if (isLaunched)
        {
            DetectCollisions();
            
            // Debug: Log velocity every 0.5 seconds
            if (showDebugInfo && Time.frameCount % 30 == 0)
            {
                Debug.Log($"Sphere velocity: {velocity.magnitude:F2} m/s, Position: {position}");
            }
        }
        HandleGroundCollision();
        
        if (showDebugInfo && isLaunched)
        {
            Debug.DrawLine(position, position + velocity.normalized * 2f, Color.yellow, deltaTime);
        }
    }

    void UpdateVisualTransform()
    {
        if (visualRenderer != null)
        {
            visualRenderer.UpdatePosition(position);
        }
    }

    void DetectCollisions()
    {
        if (collisionDetector == null)
        {
            Debug.LogError("CollisionDetector est null!");
            return;
        }
        
        RigidBody3DYahya[] rigidBodies = FindObjectsOfType<RigidBody3DYahya>();
        
        if (showDebugInfo && rigidBodies.Length == 0)
        {
            Debug.LogWarning("Aucun RigidBody3DYahya trouvé!");
        }
        
        foreach (var body in rigidBodies)
        {
            if (body == null || collidedThisFrame.Contains(body)) continue;
            
            CollisionInfoYahya collision;
            bool hasCollision = collisionDetector.DetectSphereCollision(position, radius, body, out collision);
            
            if (hasCollision)
            {
                collidedThisFrame.Add(body);
                
                if (!hasImpacted)
                {
                    hasImpacted = true;
                    Debug.Log($"PREMIER IMPACT détecté à {collision.contactPoint}!");
                    OnImpact(collision.contactPoint);
                }
                
                ResolveSphereCollision(collision, body);
                collisionCount++;
                
                Debug.Log($"Collision #{collisionCount} avec {body.name}");
            }
        }
    }

    void ResolveSphereCollision(CollisionInfoYahya collision, RigidBody3DYahya cube)
    {
        if (cube.isKinematic) return;
        
        // Store velocity magnitude before collision
        float velocityBeforeCollision = velocity.magnitude;
        
        Vector3 cubeToSphere = position - cube.position;
        Vector3 normal = cubeToSphere.normalized;
        Vector3 contactPoint = collision.contactPoint;
        
        // Séparation
        if (collision.penetrationDepth > 0.001f)
        {
            float totalSeparation = collision.penetrationDepth + 0.05f;
            float totalMass = mass + cube.mass;
            float sphereRatio = cube.mass / totalMass;
            float cubeRatio = mass / totalMass;
            
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
        
        float e = Mathf.Min(restitution, cube.restitution);
        
        float invMassSphere = 1.0f / mass;
        float invMassCube = 1.0f / cube.mass;
        float totalInvMass = invMassSphere + invMassCube;
        
        // Calcul de l'impulsion avec force maximale
        float j = -(1.0f + e) * velAlongNormal / totalInvMass;
        j = Mathf.Clamp(j, 0f, maxImpulsePerCollision);
        
        Vector3 impulse = normal * j;
        
        // Application de l'impulsion
        velocity += impulse * invMassSphere;
        cube.AddImpulseAtPoint(-impulse * 0.8f, contactPoint); // Rduit pour viter que les cubes volent trop loin
        
        // Perte d'énergie minimale pour longues distances
        if (velocityBeforeCollision < 10f) // Only apply energy loss at low speeds
        {
            velocity *= (1f - energyLossPerCollision);
        }
        else
        {
            velocity *= (1f - energyLossPerCollision * 0.1f); // Presque aucune perte à haute vitesse
        }
        
        // Ensure we maintain velocity - preserve at least 85% of original speed at high velocities
        float speedAfterLoss = velocity.magnitude;
        if (velocityBeforeCollision > 20f && speedAfterLoss < velocityBeforeCollision * 0.85f)
        {
            velocity = velocity.normalized * (velocityBeforeCollision * 0.85f);
        }
        else if (velocityBeforeCollision > 10f && speedAfterLoss < velocityBeforeCollision * 0.75f)
        {
            velocity = velocity.normalized * (velocityBeforeCollision * 0.75f);
        }
        
        // Friction minimale pour distances maximales
        Vector3 tangentVel = relativeVel - normal * velAlongNormal;
        if (tangentVel.magnitude > 0.001f)
        {
            Vector3 tangent = tangentVel.normalized;
            // Friction extrmement rduite pour longues distances
            float frictionCoeff = velocityBeforeCollision > 20f ? 0.01f : 0.05f;
            float frictionMag = Mathf.Min(tangentVel.magnitude * 0.1f, Mathf.Abs(j) * frictionCoeff);
            Vector3 frictionImpulse = -tangent * frictionMag;
            
            velocity += frictionImpulse * invMassSphere;
            cube.AddImpulseAtPoint(-frictionImpulse, contactPoint);
        }
        
        Debug.Log($"Collision: Vitesse avant={velocityBeforeCollision:F1}, après={velocity.magnitude:F1} m/s");
    }

    void OnImpact(Vector3 impactPoint)
    {
        float kineticEnergy = 0.5f * mass * velocity.sqrMagnitude;
        float explosionForce = kineticEnergy * impactMultiplier;
        explosionForce = Mathf.Clamp(explosionForce, 50f, 1500f); // Force rduite pour impact moins violent
        
        Debug.Log("═══════════════════════════════════");
        Debug.Log($"🎯 IMPACT HORIZONTAL à {impactPoint}");
        Debug.Log($"  Vitesse: {velocity.magnitude:F2} m/s");
        Debug.Log($"  Direction: {velocity.normalized}");
        Debug.Log($"  Énergie: {kineticEnergy:F2} J");
        Debug.Log($"  Force explosion: {explosionForce:F2} N");
        Debug.Log($"  Rayon de rupture: {breakRadius:F1} m");
        Debug.Log("═══════════════════════════════════");
        
        if (physicsManager != null)
        {
            // Casser les contraintes
            physicsManager.BreakConstraintsInRadius(impactPoint, breakRadius);
            
            // Appliquer l'explosion immédiatement
            physicsManager.ApplyExplosion(impactPoint, breakRadius, explosionForce);
        }
    }

    void HandleGroundCollision()
    {
        float groundLevel = physicsManager != null ? physicsManager.groundLevel : 0f;
        float bottomY = position.y - radius;
        
        if (bottomY <= groundLevel)
        {
            position = new Vector3(position.x, groundLevel + radius, position.z);
            UpdateVisualTransform();
            
            if (velocity.y < 0)
            {
                float groundRestitution = physicsManager != null ? physicsManager.groundRestitution : 0.3f;
                velocity.y = -velocity.y * groundRestitution;
                
                // MINIMAL FRICTION: Only apply when sphere is almost stopped
                if (Mathf.Abs(velocity.y) < 0.2f && velocity.magnitude < 5f)
                {
                    float groundFriction = physicsManager != null ? physicsManager.groundFriction : 0.5f;
                    velocity.x *= (1f - groundFriction * 0.02f); // Minimal friction - 2%
                    velocity.z *= (1f - groundFriction * 0.02f);
                }
            }
        }
    }

    public void Reset()
    {
        float xPos = launchFrom == LaunchSide.Right ? launchDistance : -launchDistance;
        position = new Vector3(xPos, launchHeight, 0f);
        startPosition = position;
        velocity = Vector3.zero;
        hasImpacted = false;
        collisionCount = 0;
        isLaunched = false;
        UpdateVisualTransform();
        
        Debug.Log($"Sphère réinitialisée à {position}");
    }

    void OnDrawGizmos()
    {
        Vector3 drawPos = Application.isPlaying ? position : transform.position;
        
        // Sphère principale
        Gizmos.color = sphereColor;
        Gizmos.DrawWireSphere(drawPos, radius);
        
        if (showTrajectory && Application.isPlaying && isLaunched)
        {
            // Trajectoire
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(drawPos, drawPos + velocity * 0.5f);
            
            // Indicateur d'énergie
            float ke = 0.5f * mass * velocity.sqrMagnitude;
            Gizmos.color = Color.Lerp(Color.green, Color.red, Mathf.Clamp01(ke / 2000f));
            Gizmos.DrawWireSphere(drawPos, radius * 1.2f);
            
            // Collisions
            if (collisionCount > 0)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(drawPos, radius * (1.3f + collisionCount * 0.1f));
            }
        }
        
        // Zone d'impact
        if (hasImpacted)
        {
            Gizmos.color = new Color(1f, 0.5f, 0f, 0.3f);
            Gizmos.DrawWireSphere(drawPos, breakRadius);
        }
        
        // Position de départ
        if (!Application.isPlaying || !isLaunched)
        {
            float xPos = launchFrom == LaunchSide.Right ? launchDistance : -launchDistance;
            Vector3 startPos = new Vector3(xPos, launchHeight, 0f);
            Gizmos.color = new Color(0f, 1f, 0f, 0.5f);
            Gizmos.DrawWireSphere(startPos, radius);
            
            // Flèche de direction
            Vector3 direction = launchFrom == LaunchSide.Right ? Vector3.left : Vector3.right;
            Gizmos.color = Color.green;
            Gizmos.DrawLine(startPos, startPos + direction * 3f);
            
            // Ligne vers la cible
            Vector3 targetPos = new Vector3(0, launchHeight, 0);
            Gizmos.color = new Color(0f, 1f, 0f, 0.3f);
            Gizmos.DrawLine(startPos, targetPos);
        }
    }

    void OnDrawGizmosSelected()
    {
        Vector3 drawPos = Application.isPlaying ? position : transform.position;
        
        // Zone de rupture
        Gizmos.color = new Color(1f, 0f, 0f, 0.3f);
        Gizmos.DrawWireSphere(drawPos, breakRadius);
        
        // Sphère de collision élargie
        Gizmos.color = new Color(1f, 1f, 0f, 0.5f);
        Gizmos.DrawWireSphere(drawPos, radius * 1.5f);
        
#if UNITY_EDITOR
        if (Application.isPlaying)
        {
            string info = $"Collisions: {collisionCount}\n" +
                         $"Vitesse: {velocity.magnitude:F2} m/s\n" +
                         $"Énergie: {0.5f * mass * velocity.sqrMagnitude:F0} J\n" +
                         $"Direction: {(launchFrom == LaunchSide.Right ? "→" : "←")}\n" +
                         $"Lancée: {isLaunched}";
            UnityEditor.Handles.Label(drawPos + Vector3.up * (radius + 1f), info);
        }
        else
        {
            float xPos = launchFrom == LaunchSide.Right ? launchDistance : -launchDistance;
            Vector3 startPos = new Vector3(xPos, launchHeight, 0f);
            string info = $"Position de départ\n" +
                         $"Direction: {(launchFrom == LaunchSide.Right ? "Droite → Gauche" : "Gauche → Droite")}\n" +
                         $"Vitesse: {launchSpeed:F1} m/s\n" +
                         $"Masse: {mass:F1} kg";
            UnityEditor.Handles.Label(startPos + Vector3.up * (radius + 1f), info);
        }
#endif
    }
}