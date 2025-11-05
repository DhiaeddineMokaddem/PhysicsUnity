using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Sphère d'impact STATIQUE au sol
/// ADAPTÉ de ImpactSphere.cs - Version simplifiée sans mouvement
/// Les cubes tombent SUR la sphère (pas l'inverse)
/// </summary>
public class StaticImpactSphere : MonoBehaviour
{
    [Header("Propriétés de la Sphère")]
    public float radius = 1.0f;
    public float mass = 10.0f; // Masse pour calculs de collision
    
    [Tooltip("Coefficient de restitution (rebond)")]
    [Range(0f, 1f)]
    public float restitution = 0.7f; // Augmenté pour plus de rebond
    
    [Header("Position")]
    [Tooltip("Positionner automatiquement au niveau du sol")]
    public bool autoPositionOnGround = true;
    
    [Header("Impact et Rupture")]
    [Tooltip("Rayon dans lequel les contraintes se brisent au premier contact")]
    public float breakRadius = 3.0f;
    
    [Tooltip("Multiplicateur de force pour l'explosion initiale")]
    public float impactMultiplier = 0.5f;
    
    [Header("Visualisation")]
    public Color sphereColor = Color.red;
    public bool showBreakRadius = true;
    
    // État interne
    private CollisionDetector collisionDetector;
    private PhysicsManager physicsManager;
    private HashSet<RigidBody3D> collidedBodies = new HashSet<RigidBody3D>();
    private bool hasTriggeredBreak = false;
    private int totalCollisions = 0;

    void Start()
    {
        physicsManager = FindObjectOfType<PhysicsManager>();
        collisionDetector = physicsManager?.GetComponent<CollisionDetector>();
        
        if (collisionDetector == null)
        {
            Debug.LogError("StaticImpactSphere: CollisionDetector non trouvé!");
        }
        
        // Positionner au sol si demandé
        if (autoPositionOnGround && physicsManager != null)
        {
            float groundY = physicsManager.groundLevel;
            transform.position = new Vector3(
                transform.position.x,
                groundY + radius, // Poser la sphère sur le sol
                transform.position.z
            );
        }
        
        Debug.Log($"Sphère statique positionnée à {transform.position}");
    }

    void FixedUpdate()
    {
        if (physicsManager != null && physicsManager.pauseSimulation) return;
        
        // La sphère reste statique, on détecte juste les collisions avec cubes tombants
        DetectAndResolveCollisions();
    }

    /// <summary>
    /// NOUVELLE APPROCHE : Détection passive des collisions
    /// Les cubes tombent sur la sphère statique
    /// </summary>
    void DetectAndResolveCollisions()
    {
        if (collisionDetector == null) return;
        
        RigidBody3D[] rigidBodies = FindObjectsOfType<RigidBody3D>();
        
        foreach (var body in rigidBodies)
        {
            if (body == null || body.isKinematic) continue;
            
            CollisionInfo collision;
            if (collisionDetector.DetectSphereCollision(transform.position, radius, body, out collision))
            {
                // Premier contact avec un cube → Déclencher rupture des contraintes
                if (!hasTriggeredBreak)
                {
                    hasTriggeredBreak = true;
                    OnFirstImpact(collision.contactPoint);
                }
                
                // Résoudre la collision (rebond)
                ResolveCubeCollision(collision, body);
                
                // Compter collisions uniques
                if (!collidedBodies.Contains(body))
                {
                    collidedBodies.Add(body);
                    totalCollisions++;
                }
            }
        }
    }

    /// <summary>
    /// Résolution de collision cube → sphère statique
    /// ADAPTÉ de ImpactSphere.cs - Sphère ne bouge pas
    /// </summary>
    void ResolveCubeCollision(CollisionInfo collision, RigidBody3D cube)
    {
        Vector3 normal = collision.contactNormal; // Pointe du cube vers la sphère
        Vector3 contactPoint = collision.contactPoint;
        
        // 1. Séparer les objets si pénétration
        if (collision.penetrationDepth > 0.001f)
        {
            // Seul le cube bouge (sphère statique)
            cube.transform.position -= normal * (collision.penetrationDepth + 0.01f);
        }
        
        // 2. Vitesse relative au point de contact
        Vector3 cubeVel = cube.GetVelocityAtPoint(contactPoint);
        float velAlongNormal = Vector3.Dot(cubeVel, normal);
        
        // Si le cube s'éloigne déjà, pas de collision
        if (velAlongNormal >= -0.001f) return;
        
        // 3. Calcul d'impulsion (formule standard avec sphère infinie masse)
        // La sphère statique = masse infinie → 1/m_sphere ≈ 0
        float invMassCube = 1.0f / cube.mass;
        
        // Impulsion scalaire : j = -(1 + e) * v_n / (1/m)
        float j = -(1.0f + restitution) * velAlongNormal * cube.mass;
        
        Vector3 impulse = normal * j;
        
        // Appliquer l'impulsion uniquement au cube
        cube.AddImpulseAtPoint(impulse, contactPoint);
        
        // 4. Friction tangentielle
        Vector3 tangentVel = cubeVel - normal * velAlongNormal;
        if (tangentVel.magnitude > 0.001f)
        {
            Vector3 tangent = tangentVel.normalized;
            float frictionCoeff = 0.3f;
            float frictionMag = Mathf.Min(tangentVel.magnitude * 0.5f, Mathf.Abs(j) * frictionCoeff / cube.mass);
            Vector3 frictionImpulse = -tangent * frictionMag * cube.mass;
            
            cube.AddImpulseAtPoint(frictionImpulse, contactPoint);
        }
    }

    /// <summary>
    /// Premier impact → Rupture des contraintes
    /// ADAPTÉ de ImpactSphere.OnImpact()
    /// </summary>
    void OnFirstImpact(Vector3 impactPoint)
    {
        Debug.Log($"═══ PREMIER IMPACT À {impactPoint} ═══");
        
        if (physicsManager != null)
        {
            // Casser toutes les contraintes dans le rayon
            physicsManager.BreakConstraintsInRadius(impactPoint, breakRadius);
            
            // Explosion plus puissante pour séparer les cubes
            float explosionForce = 500f * impactMultiplier; // Augmenté
            physicsManager.ApplyExplosion(impactPoint, breakRadius, explosionForce);
            
            Debug.Log($"  → Contraintes cassées dans rayon {breakRadius}m");
            Debug.Log($"  → Force d'explosion : {explosionForce}N");
        }
    }

    /// <summary>
    /// Réinitialiser l'état
    /// </summary>
    public void Reset()
    {
        hasTriggeredBreak = false;
        collidedBodies.Clear();
        totalCollisions = 0;
    }

    /// <summary>
    /// Obtenir statistiques
    /// </summary>
    public string GetStats()
    {
        return $"Collisions totales : {totalCollisions}\n" +
               $"Cubes uniques : {collidedBodies.Count}\n" +
               $"Rupture déclenchée : {(hasTriggeredBreak ? "OUI" : "NON")}";
    }

    void OnDrawGizmos()
    {
        // Sphère principale
        Gizmos.color = sphereColor;
        Gizmos.DrawWireSphere(transform.position, radius);
        
        // Rayon de rupture
        if (showBreakRadius)
        {
            Gizmos.color = new Color(1f, 0.5f, 0f, 0.3f);
            Gizmos.DrawWireSphere(transform.position, breakRadius);
        }
        
        // Indicateur visuel si rupture activée
        if (Application.isPlaying && hasTriggeredBreak)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, radius * 1.2f);
        }
    }

    void OnDrawGizmosSelected()
    {
        // Zone de rupture détaillée
        Gizmos.color = new Color(1f, 0f, 0f, 0.2f);
        Gizmos.DrawSphere(transform.position, breakRadius);
        
        // Afficher info texte
        if (Application.isPlaying)
        {
            #if UNITY_EDITOR
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * (radius + 0.5f),
                $"Collisions: {totalCollisions}\nCubes: {collidedBodies.Count}"
            );
            #endif
        }
    }
}