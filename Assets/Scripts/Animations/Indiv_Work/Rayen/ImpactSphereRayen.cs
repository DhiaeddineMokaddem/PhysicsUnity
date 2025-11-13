using UnityEngine;
using System.Collections.Generic;
using PhysicsSimulation.Indiv_Work.Aziz;

/// <summary>
/// Sphère qui impacte la structure de cubes - VERSION PURE MATH
/// FIXED: Proper initialization order using Awake()
/// </summary>
public class ImpactSphereRayen : MonoBehaviour
{
    [Header("Propriétés de la Sphère")]
    public float radius = 1.0f;
    public float mass = 5.0f;
    public Vector3 velocity = Vector3.zero;
    public float restitution = 0.3f;
    
    [Header("Impact")]
    public float impactMultiplier = 0.5f;
    public float breakRadius = 3.0f;
    public bool autoLaunch = false;
    public Vector3 launchDirection = Vector3.down;
    public float launchSpeed = 10f;
    
    [Header("Limites physiques")]
    public float maxImpulsePerCollision = 50f;
    [Range(0f, 1f)]
    public float energyLossPerCollision = 0.3f;
    
    [Header("Visualisation")]
    public Color sphereColor = Color.red;
    public bool showTrajectory = true;
    
    // PURE MATH: Position stockée manuellement
    [HideInInspector] public Vector3 position;
    
    private CollisionDetectorRayen CollisionDetectorRayen;
    private PhysicsManagerRayen PhysicsManagerRayen;
    private Vector3 acceleration = Vector3.zero;
    private bool hasImpacted = false;
    private Vector3 startPosition;
    private HashSet<RigidBody3D> collidedThisFrame = new HashSet<RigidBody3D>();
    private int collisionCount = 0;

    /// <summary>
    /// FIXED: Use Awake() for immediate initialization
    /// </summary>
    void Awake()
    {
        // PURE MATH: Initialiser la position depuis Transform
        position = transform.position;
        startPosition = position;
    }

    void Start()
    {
        PhysicsManagerRayen = FindObjectOfType<PhysicsManagerRayen>();
        CollisionDetectorRayen = PhysicsManagerRayen?.GetComponent<CollisionDetectorRayen>();
        
        if (CollisionDetectorRayen == null)
        {
            Debug.LogError("ImpactSphere: CollisionDetectorRayen non trouvé!");
        }
        
        if (autoLaunch)
        {
            Launch();
        }
        
        // Mettre à jour le Transform visuel
        UpdateVisualTransform();
    }

    public void Launch()
    {
        velocity = launchDirection.normalized * launchSpeed;
        hasImpacted = false;
        collisionCount = 0;
    }

    public void LaunchTowards(Vector3 target)
    {
        Vector3 direction = (target - position).normalized;
        velocity = direction * launchSpeed;
        hasImpacted = false;
        collisionCount = 0;
    }

    void FixedUpdate()
    {
        if (PhysicsManagerRayen != null && PhysicsManagerRayen.pauseSimulation) return;
        
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
        transform.position = position;
    }

    void DetectCollisions()
    {
        if (CollisionDetectorRayen == null) return;
        
        RigidBody3D[] rigidBodies = FindObjectsOfType<RigidBody3D>();
        
        foreach (var body in rigidBodies)
        {
            if (body == null || collidedThisFrame.Contains(body)) continue;
            
            CollisionInfo collision;
            if (CollisionDetectorRayen.TryDetectSphereCubeCollision(position, radius, body, out collision))
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
        
        if (PhysicsManagerRayen != null)
        {
            PhysicsManagerRayen.BreakConstraintsInRadius(impactPoint, breakRadius);
        }
        
        StartCoroutine(ApplyExplosionDelayed(impactPoint, explosionForce));
    }
    
    System.Collections.IEnumerator ApplyExplosionDelayed(Vector3 impactPoint, float force)
    {
        yield return new WaitForFixedUpdate();
        
        if (PhysicsManagerRayen != null)
        {
            PhysicsManagerRayen.ApplyExplosion(impactPoint, breakRadius, force);
        }
    }

    void HandleGroundCollision()
    {
        float groundLevel = PhysicsManagerRayen != null ? PhysicsManagerRayen.groundLevel : 0f;
        float bottomY = position.y - radius;
        
        if (bottomY <= groundLevel)
        {
            // PURE MATH: Modifier directement la position
            position = new Vector3(position.x, groundLevel + radius, position.z);
            UpdateVisualTransform();
            
            if (velocity.y < 0)
            {
                float groundRestitution = PhysicsManagerRayen != null ? PhysicsManagerRayen.groundRestitution : 0.3f;
                velocity.y = -velocity.y * groundRestitution;
                
                float groundFriction = PhysicsManagerRayen != null ? PhysicsManagerRayen.groundFriction : 0.5f;
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