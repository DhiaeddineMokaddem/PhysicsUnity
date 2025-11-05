using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Gestionnaire principal de la simulation de poutre avec fracture
/// </summary>
public class BeamSimulation : MonoBehaviour
{
    [Header("Configuration de la poutre")]
    public float beamLength = 4f;
    public float beamWidth = 0.2f;
    public float beamHeight = 0.2f;
    public float segmentMass = 1f;
    
    [Header("Positions")]
    public float beamHeightFromGround = 2f;
    public float postHeight = 3f;
    public float postRadius = 0.15f;
    
    [Header("Paramètre α (élasticité)")]
    [Range(0f, 2f)]
    public float alpha = 0.5f;
    
    [Header("Contraintes")]
    public float baseStiffness = 500f;
    public float baseDamping = 20f;
    public float breakForce = 100f;
    
    [Header("Projectile")]
    public float projectileSize = 0.3f;
    public float projectileMass = 2f;
    public float projectileSpeed = 15f;
    public Vector3 projectileStartOffset = new Vector3(-5f, 0.5f, 0f);
    
    [Header("Physique")]
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public int substeps = 2;
    
    [Header("Sol")]
    public float groundHeight = 0f;
    public float groundRestitution = 0.3f;
    public float groundFriction = 0.5f;
    
    // Composants
    private GameObject leftPost, rightPost;
    private RigidSegment leftSegment, rightSegment;
    private ElasticConstraint centerConstraint;
    private ImpactProjectile projectile;
    private GroundPlane ground;
    private GameObject groundMesh;
    
    // État de la simulation
    private bool isSimulationRunning = false;
    private bool hasFractured = false;
    
    // Matériaux
    private Material postMaterial;
    private Material beamMaterial;
    private Material projectileMaterial;
    private Material groundMaterial;
    
    void Start()
    {
        CreateMaterials();
        BuildBeamStructure();
    }
    
    /// <summary>
    /// Crée les matériaux pour les différents objets
    /// </summary>
    void CreateMaterials()
    {
        // Bright, distinct colors
        postMaterial = MeshGenerator.CreateColorMaterial(new Color(1.0f, 0.0f, 1.0f));     // Magenta/Pink - Posts
        beamMaterial = MeshGenerator.CreateColorMaterial(new Color(1.0f, 0.0f, 0.0f));      // Red - Beam segments
        projectileMaterial = MeshGenerator.CreateColorMaterial(new Color(0.0f, 1.0f, 0.0f)); // Green - Projectile
        groundMaterial = MeshGenerator.CreateColorMaterial(new Color(1.0f, 0.8f, 0.0f));    // Yellow/Gold - Ground
        
        Debug.Log("Materials created with distinct colors: Posts=Magenta, Beam=Red, Projectile=Green, Ground=Yellow");
    }
    
    /// <summary>
    /// Construit toute la structure de la poutre
    /// </summary>
    void BuildBeamStructure()
    {
        float halfBeamLength = beamLength * 0.5f;
        float segmentLength = beamLength * 0.5f;
        
        // === Créer les poteaux ===
        Mesh cylinderMesh = MeshGenerator.CreateCylinderMesh(postRadius, postHeight, 12);
        
        // Poteau gauche
        leftPost = MeshGenerator.CreateMeshObject(cylinderMesh, postMaterial, "LeftPost");
        leftPost.transform.position = new Vector3(-halfBeamLength, postHeight * 0.5f, 0f);
        leftPost.transform.parent = transform;
        
        // Poteau droit
        rightPost = MeshGenerator.CreateMeshObject(cylinderMesh, postMaterial, "RightPost");
        rightPost.transform.position = new Vector3(halfBeamLength, postHeight * 0.5f, 0f);
        rightPost.transform.parent = transform;
        
        // === Créer les segments de la poutre ===
        Vector3 segmentSize = new Vector3(segmentLength, beamHeight, beamWidth);
        Mesh beamMesh = MeshGenerator.CreateCubeMesh(segmentSize);
        
        // Segment gauche
        GameObject leftSegmentObj = MeshGenerator.CreateMeshObject(beamMesh, beamMaterial, "LeftSegment");
        leftSegmentObj.transform.parent = transform;
        leftSegment = leftSegmentObj.AddComponent<RigidSegment>();
        leftSegment.mass = segmentMass;
        leftSegment.size = segmentSize;
        
        Vector3 leftSegmentPos = new Vector3(-segmentLength * 0.5f, beamHeightFromGround, 0f);
        leftSegment.Initialize(leftSegmentPos, Quaternion.identity);
        
        // Segment droit
        GameObject rightSegmentObj = MeshGenerator.CreateMeshObject(beamMesh, beamMaterial, "RightSegment");
        rightSegmentObj.transform.parent = transform;
        rightSegment = rightSegmentObj.AddComponent<RigidSegment>();
        rightSegment.mass = segmentMass;
        rightSegment.size = segmentSize;
        
        Vector3 rightSegmentPos = new Vector3(segmentLength * 0.5f, beamHeightFromGround, 0f);
        rightSegment.Initialize(rightSegmentPos, Quaternion.identity);
        
        // === Créer la contrainte centrale ===
        GameObject constraintObj = new GameObject("CenterConstraint");
        constraintObj.transform.parent = transform;
        constraintObj.transform.position = new Vector3(0f, beamHeightFromGround, 0f);
        
        centerConstraint = constraintObj.AddComponent<ElasticConstraint>();
        centerConstraint.segmentA = leftSegment;
        centerConstraint.segmentB = rightSegment;
        
        // Points d'attache : extrémité droite du segment gauche, extrémité gauche du segment droit
        centerConstraint.localAnchorA = new Vector3(segmentLength * 0.5f, 0f, 0f);
        centerConstraint.localAnchorB = new Vector3(-segmentLength * 0.5f, 0f, 0f);
        
        UpdateConstraintParameters();
        centerConstraint.Initialize();
        
        // === Créer le projectile ===
        Mesh projectileMesh = MeshGenerator.CreateCubeMesh(Vector3.one * projectileSize);
        GameObject projectileObj = MeshGenerator.CreateMeshObject(projectileMesh, projectileMaterial, "Projectile");
        projectileObj.transform.parent = transform;
        
        projectile = projectileObj.AddComponent<ImpactProjectile>();
        projectile.mass = projectileMass;
        projectile.size = projectileSize;
        projectile.launchSpeed = projectileSpeed;
        
        Vector3 projectileStartPos = new Vector3(0f, beamHeightFromGround, 0f) + projectileStartOffset;
        projectile.Initialize(projectileStartPos, Quaternion.identity);
        
        // === Créer le sol ===
        GameObject groundObj = new GameObject("Ground");
        groundObj.transform.parent = transform;
        groundObj.transform.position = new Vector3(0f, groundHeight, 0f);
        
        ground = groundObj.AddComponent<GroundPlane>();
        ground.height = groundHeight;
        ground.restitution = groundRestitution;
        ground.friction = groundFriction;
        
        // Créer un mesh visuel pour le sol
        Mesh groundMeshData = MeshGenerator.CreateCubeMesh(new Vector3(20f, 0.1f, 10f));
        groundMesh = MeshGenerator.CreateMeshObject(groundMeshData, groundMaterial, "GroundMesh");
        groundMesh.transform.parent = groundObj.transform;
        groundMesh.transform.localPosition = new Vector3(0f, -0.05f, 0f);
        
        Debug.Log("Structure de poutre construite avec succès");
    }
    
    /// <summary>
    /// Met à jour les paramètres de la contrainte en fonction de α
    /// </summary>
    void UpdateConstraintParameters()
    {
        if (centerConstraint != null)
        {
            // α contrôle l'élasticité : plus α est grand, plus la poutre est flexible
            centerConstraint.stiffness = baseStiffness / (1f + alpha);
            centerConstraint.damping = baseDamping / (1f + alpha * 0.5f);
            centerConstraint.breakForce = breakForce * (1f + alpha * 0.5f);
        }
    }
    
    /// <summary>
    /// Démarre la simulation (lance le projectile)
    /// </summary>
    public void StartSimulation()
    {
        if (isSimulationRunning) return;
        
        // Direction de tir : vers le centre de la poutre
        Vector3 beamCenter = new Vector3(0f, beamHeightFromGround, 0f);
        Vector3 direction = (beamCenter - projectile.position).normalized;
        
        projectile.Launch(direction);
        isSimulationRunning = true;
        hasFractured = false;
        
        Debug.Log("Simulation démarrée - Projectile lancé!");
    }
    
    /// <summary>
    /// Réinitialise la simulation
    /// </summary>
    public void ResetSimulation()
    {
        isSimulationRunning = false;
        hasFractured = false;
        
        // Réinitialiser les segments
        float segmentLength = beamLength * 0.5f;
        Vector3 leftSegmentPos = new Vector3(-segmentLength * 0.5f, beamHeightFromGround, 0f);
        Vector3 rightSegmentPos = new Vector3(segmentLength * 0.5f, beamHeightFromGround, 0f);
        
        leftSegment.Initialize(leftSegmentPos, Quaternion.identity);
        rightSegment.Initialize(rightSegmentPos, Quaternion.identity);
        
        // Réinitialiser la contrainte
        centerConstraint.isBroken = false;
        UpdateConstraintParameters();
        centerConstraint.Initialize();
        
        // Réinitialiser le projectile
        projectile.Reset();
        
        Debug.Log("Simulation réinitialisée");
    }
    
    void FixedUpdate()
    {
        if (!isSimulationRunning) return;
        
        float dt = Time.fixedDeltaTime / substeps;
        
        for (int i = 0; i < substeps; i++)
        {
            // 1. Résoudre les contraintes
            if (!centerConstraint.isBroken)
            {
                centerConstraint.SolveConstraint(dt);
            }
            
            // 2. Intégrer la physique des segments
            leftSegment.IntegratePhysics(dt, gravity);
            rightSegment.IntegratePhysics(dt, gravity);
            
            // 3. Intégrer la physique du projectile
            projectile.IntegratePhysics(dt);
            
            // 4. Détecter et résoudre les collisions
            DetectAndResolveCollisions();
            
            // 5. Vérifier la fracture
            CheckFracture();
        }
    }
    
    /// <summary>
    /// Détecte et résout les collisions entre le projectile et les segments
    /// </summary>
    void DetectAndResolveCollisions()
    {
        // Collision avec le segment gauche
        Vector3 collisionPoint, collisionNormal;
        float penetration;
        
        if (projectile.DetectCollisionWithSegment(leftSegment, out collisionPoint, out collisionNormal, out penetration))
        {
            projectile.ResolveCollision(leftSegment, collisionPoint, collisionNormal, penetration);
            
            if (!hasFractured)
            {
                Debug.Log($"Impact sur le segment gauche! Énergie: {projectile.GetKineticEnergy():F2} J");
            }
        }
        
        // Collision avec le segment droit
        if (projectile.DetectCollisionWithSegment(rightSegment, out collisionPoint, out collisionNormal, out penetration))
        {
            projectile.ResolveCollision(rightSegment, collisionPoint, collisionNormal, penetration);
            
            if (!hasFractured)
            {
                Debug.Log($"Impact sur le segment droit! Énergie: {projectile.GetKineticEnergy():F2} J");
            }
        }
        
        // Collision du projectile avec le sol
        if (ground.DetectCollision(projectile.position, projectile.size * 0.5f, out penetration))
        {
            ground.ResolveCollision(projectile, penetration);
        }
        
        // Collision des segments avec le sol
        if (ground.DetectSegmentCollision(leftSegment, out penetration))
        {
            ground.ResolveSegmentCollision(leftSegment, penetration);
        }
        
        if (ground.DetectSegmentCollision(rightSegment, out penetration))
        {
            ground.ResolveSegmentCollision(rightSegment, penetration);
        }
    }
    
    /// <summary>
    /// Vérifie si la contrainte doit se rompre et applique l'énergie de fracture
    /// </summary>
    void CheckFracture()
    {
        if (hasFractured || centerConstraint.isBroken) return;
        
        float tension = centerConstraint.GetTension();
        
        // Vérifier si la contrainte est cassée
        if (centerConstraint.isBroken)
        {
            hasFractured = true;
            
            // Récupérer l'énergie stockée
            float storedEnergy = centerConstraint.GetStoredEnergy();
            
            // Convertir une fraction de l'énergie en énergie cinétique
            float energyFraction = 0.3f * alpha; // α influence l'effet de fracture
            float energyToRelease = storedEnergy * energyFraction;
            
            // Appliquer des impulsions aux segments
            ApplyFractureImpulses(energyToRelease);
            
            Debug.Log($"FRACTURE! Énergie libérée: {energyToRelease:F2} J (α={alpha:F2})");
        }
    }
    
    /// <summary>
    /// Applique des impulsions aux segments lors de la fracture
    /// </summary>
    void ApplyFractureImpulses(float energy)
    {
        // Direction : perpendiculaire à la poutre (vers le haut et légèrement vers l'extérieur)
        Vector3 leftDir = (Vector3.up + Vector3.left * 0.3f).normalized;
        Vector3 rightDir = (Vector3.up + Vector3.right * 0.3f).normalized;
        
        // Calculer l'impulsion à partir de l'énergie
        // E = 0.5 * m * v^2 => v = sqrt(2*E/m)
        float leftSpeed = Mathf.Sqrt(2f * energy / leftSegment.mass);
        float rightSpeed = Mathf.Sqrt(2f * energy / rightSegment.mass);
        
        // Limiter les vitesses
        leftSpeed = Mathf.Min(leftSpeed, 5f);
        rightSpeed = Mathf.Min(rightSpeed, 5f);
        
        // Appliquer les impulsions
        leftSegment.AddImpulse(leftDir * leftSpeed * leftSegment.mass);
        rightSegment.AddImpulse(rightDir * rightSpeed * rightSegment.mass);
        
        // Ajouter un peu de rotation
        leftSegment.AddTorque(Vector3.forward * energy * 0.1f);
        rightSegment.AddTorque(Vector3.back * energy * 0.1f);
    }
    
    void Update()
    {
        // Permettre de changer α en temps réel
        UpdateConstraintParameters();
    }
    
    void OnDrawGizmos()
    {
        if (leftPost != null && rightPost != null)
        {
            // Dessiner les limites de la structure
            Gizmos.color = Color.gray;
            Gizmos.DrawLine(leftPost.transform.position, rightPost.transform.position);
        }
        
        // Dessiner l'état de la simulation
        if (isSimulationRunning)
        {
            Gizmos.color = Color.green;
        }
        else
        {
            Gizmos.color = Color.red;
        }
        
        Gizmos.DrawWireSphere(transform.position, 0.2f);
    }
}