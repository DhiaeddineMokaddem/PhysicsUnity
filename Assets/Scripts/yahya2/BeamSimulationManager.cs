using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Gestionnaire principal de la simulation de poutre
/// Crée et gère tous les objets et la physique
/// </summary>
public class BeamSimulationManager : MonoBehaviour
{
    [Header("Configuration de la scène")]
    public Vector3 groundSize = new Vector3(20, 0.2f, 20);
    public Color groundColor = new Color(0.3f, 0.3f, 0.3f);
    
    [Header("Configuration de la poutre")]
    public Vector3 beamSegmentSize = new Vector3(3f, 0.3f, 0.3f);
    public float beamHeight = 1.5f;
    public Color segment1Color = new Color(1f, 0.9f, 0.2f); // Jaune
    public Color segment2Color = new Color(1f, 0.2f, 0.2f); // Rouge
    
    [Header("Configuration des poteaux")]
    public float postRadius = 0.15f;
    public float postHeight = 2.5f;
    public Color postColor = new Color(0.4f, 0.25f, 0.15f); // Marron
    
    [Header("Configuration du cube d'impact")]
    public Vector3 cubeSize = new Vector3(0.5f, 0.5f, 0.5f);
    public Vector3 cubeStartPosition = new Vector3(-5f, 0.3f, 0f);
    public Color cubeColor = new Color(0.2f, 0.8f, 0.2f); // Vert
    public float impactVelocity = 25f;
    
    [Header("Paramètres physiques")]
    public float gravity = -9.81f;
    public float timeStep = 0.015f;
    [Range(0f, 2f)]
    public float elasticity = 0.6f; // α - élasticité de la poutre
    public float beamMass = 2f;
    public float cubeMass = 3f;
    
    [Header("Paramètres de contrainte")]
    public float jointStiffness = 3000f;
    public float jointDamping = 150f;
    public float jointBreakForce = 300f;
    
    // Références internes
    private GameObject ground;
    private GameObject leftPost;
    private GameObject rightPost;
    private GameObject segment1;
    private GameObject segment2;
    private GameObject impactCube;
    
    private CustomRigidBody segment1Body;
    private CustomRigidBody segment2Body;
    private CustomRigidBody cubeBody;
    
    private CustomJoint centerJoint; // Contrainte entre les deux segments
    private List<CustomJoint> postJoints = new List<CustomJoint>(); // Contraintes avec les poteaux
    
    private CustomCollisionSystem collisionSystem;
    private List<CustomRigidBody> allBodies = new List<CustomRigidBody>();
    
    private bool simulationStarted = false;
    private float accumulator = 0f;

    void Start()
    {
        CreateScene();
        collisionSystem = new CustomCollisionSystem();
    }

    /// <summary>
    /// Crée toute la scène par code
    /// </summary>
    void CreateScene()
    {
        // Sol
        CreateGround();
        
        // Poteaux
        CreatePosts();
        
        // Segments de la poutre
        CreateBeamSegments();
        
        // Cube d'impact
        CreateImpactCube();
        
        // Contraintes
        CreateJoints();
        
        Debug.Log("Scène créée. Appuyez sur ESPACE pour démarrer la simulation.");
    }

    /// <summary>
    /// Crée le sol
    /// </summary>
    void CreateGround()
    {
        Mesh groundMesh = CustomMeshGenerator.CreateBox(groundSize);
        Material groundMat = CustomMeshGenerator.CreateColorMaterial(groundColor);
        
        ground = CustomMeshGenerator.CreateMeshObject(
            "Ground",
            groundMesh,
            groundMat,
            Vector3.zero,
            Quaternion.identity
        );
    }

    /// <summary>
    /// Crée les poteaux fixes
    /// </summary>
    void CreatePosts()
    {
        Mesh postMesh = CustomMeshGenerator.CreateCylinder(postRadius, postHeight, 12);
        Material postMat = CustomMeshGenerator.CreateColorMaterial(postColor);
        
        float beamLength = beamSegmentSize.x * 2;
        float postSpacing = beamLength * 0.55f;
        
        // Poteau gauche
        leftPost = CustomMeshGenerator.CreateMeshObject(
            "LeftPost",
            postMesh,
            postMat,
            new Vector3(-postSpacing, postHeight * 0.5f, 0),
            Quaternion.identity
        );
        
        CustomRigidBody leftPostBody = leftPost.AddComponent<CustomRigidBody>();
        leftPostBody.mass = 100f;
        leftPostBody.size = new Vector3(postRadius * 2, postHeight, postRadius * 2);
        leftPostBody.isStatic = true;
        allBodies.Add(leftPostBody);
        
        // Poteau droit
        rightPost = CustomMeshGenerator.CreateMeshObject(
            "RightPost",
            postMesh,
            postMat,
            new Vector3(postSpacing, postHeight * 0.5f, 0),
            Quaternion.identity
        );
        
        CustomRigidBody rightPostBody = rightPost.AddComponent<CustomRigidBody>();
        rightPostBody.mass = 100f;
        rightPostBody.size = new Vector3(postRadius * 2, postHeight, postRadius * 2);
        rightPostBody.isStatic = true;
        allBodies.Add(rightPostBody);
    }

    /// <summary>
    /// Crée les deux segments de la poutre
    /// </summary>
    void CreateBeamSegments()
    {
        Mesh beamMesh = CustomMeshGenerator.CreateBox(beamSegmentSize);
        
        // Segment 1 (jaune, gauche)
        Material seg1Mat = CustomMeshGenerator.CreateColorMaterial(segment1Color);
        Vector3 seg1Pos = new Vector3(-beamSegmentSize.x * 0.5f, beamHeight, 0);
        
        segment1 = CustomMeshGenerator.CreateMeshObject(
            "BeamSegment1",
            beamMesh,
            seg1Mat,
            seg1Pos,
            Quaternion.identity
        );
        
        segment1Body = segment1.AddComponent<CustomRigidBody>();
        segment1Body.mass = beamMass;
        segment1Body.size = beamSegmentSize;
        segment1Body.isStatic = false;
        segment1Body.linearDamping = 0.98f;
        segment1Body.angularDamping = 0.95f;
        allBodies.Add(segment1Body);
        
        // Segment 2 (rouge, droite)
        Material seg2Mat = CustomMeshGenerator.CreateColorMaterial(segment2Color);
        Vector3 seg2Pos = new Vector3(beamSegmentSize.x * 0.5f, beamHeight, 0);
        
        segment2 = CustomMeshGenerator.CreateMeshObject(
            "BeamSegment2",
            beamMesh,
            seg2Mat,
            seg2Pos,
            Quaternion.identity
        );
        
        segment2Body = segment2.AddComponent<CustomRigidBody>();
        segment2Body.mass = beamMass;
        segment2Body.size = beamSegmentSize;
        segment2Body.isStatic = false;
        segment2Body.linearDamping = 0.98f;
        segment2Body.angularDamping = 0.95f;
        allBodies.Add(segment2Body);
    }

    /// <summary>
    /// Crée le cube d'impact
    /// </summary>
    void CreateImpactCube()
    {
        Mesh cubeMesh = CustomMeshGenerator.CreateBox(cubeSize);
        Material cubeMat = CustomMeshGenerator.CreateColorMaterial(cubeColor);
        
        impactCube = CustomMeshGenerator.CreateMeshObject(
            "ImpactCube",
            cubeMesh,
            cubeMat,
            cubeStartPosition,
            Quaternion.identity
        );
        
        cubeBody = impactCube.AddComponent<CustomRigidBody>();
        cubeBody.mass = cubeMass;
        cubeBody.size = cubeSize;
        cubeBody.isStatic = false;
        cubeBody.linearDamping = 0.99f;
        cubeBody.angularDamping = 0.95f;
        allBodies.Add(cubeBody);
    }

    /// <summary>
    /// Crée les contraintes entre segments et poteaux
    /// </summary>
    void CreateJoints()
    {
        // Ajuster la rigidité selon l'élasticité
        float adjustedStiffness = jointStiffness * (1f - elasticity * 0.5f);
        float adjustedDamping = jointDamping * (1f - elasticity * 0.3f);
        
        // Contrainte centrale entre les deux segments
        Vector3 anchorA = new Vector3(beamSegmentSize.x * 0.5f, 0, 0);
        Vector3 anchorB = new Vector3(-beamSegmentSize.x * 0.5f, 0, 0);
        
        centerJoint = new CustomJoint(segment1Body, segment2Body, anchorA, anchorB);
        centerJoint.stiffness = adjustedStiffness;
        centerJoint.damping = adjustedDamping;
        centerJoint.breakForce = jointBreakForce;
        
        // Contraintes avec les poteaux (simuler un support)
        // On va créer des contraintes "souples" pour permettre un peu de mouvement
        CustomRigidBody leftPostBody = leftPost.GetComponent<CustomRigidBody>();
        CustomRigidBody rightPostBody = rightPost.GetComponent<CustomRigidBody>();
        
        // Contrainte segment1 - poteau gauche
        Vector3 seg1LeftAnchor = new Vector3(-beamSegmentSize.x * 0.5f, 0, 0);
        Vector3 postLeftAnchor = new Vector3(0, postHeight * 0.5f - beamHeight, 0);
        
        CustomJoint leftJoint = new CustomJoint(segment1Body, leftPostBody, seg1LeftAnchor, postLeftAnchor);
        leftJoint.stiffness = adjustedStiffness * 1.5f;
        leftJoint.damping = adjustedDamping * 1.2f;
        leftJoint.breakForce = jointBreakForce * 2f;
        postJoints.Add(leftJoint);
        
        // Contrainte segment2 - poteau droit
        Vector3 seg2RightAnchor = new Vector3(beamSegmentSize.x * 0.5f, 0, 0);
        Vector3 postRightAnchor = new Vector3(0, postHeight * 0.5f - beamHeight, 0);
        
        CustomJoint rightJoint = new CustomJoint(segment2Body, rightPostBody, seg2RightAnchor, postRightAnchor);
        rightJoint.stiffness = adjustedStiffness * 1.5f;
        rightJoint.damping = adjustedDamping * 1.2f;
        rightJoint.breakForce = jointBreakForce * 2f;
        postJoints.Add(rightJoint);
    }

    void Update()
    {
        // Contrôles clavier
        if (Input.GetKeyDown(KeyCode.Space))
        {
            StartSimulation();
        }
        
        if (Input.GetKeyDown(KeyCode.R))
        {
            RestartSimulation();
        }
        
        // Ajuster l'élasticité en temps réel
        if (Input.GetKey(KeyCode.E) && elasticity < 2f)
        {
            elasticity += 0.01f;
            UpdateJointParameters();
        }
        
        if (Input.GetKey(KeyCode.Q) && elasticity > 0f)
        {
            elasticity -= 0.01f;
            UpdateJointParameters();
        }
    }

    void FixedUpdate()
    {
        if (!simulationStarted) return;
        
        accumulator += Time.fixedDeltaTime;
        
        while (accumulator >= timeStep)
        {
            PhysicsStep(timeStep);
            accumulator -= timeStep;
        }
    }

    /// <summary>
    /// Un pas de simulation physique
    /// </summary>
    void PhysicsStep(float dt)
    {
        // Résoudre les contraintes
        if (centerJoint != null && !centerJoint.isBroken)
        {
            centerJoint.SolveConstraint(dt);
        }
        
        foreach (var joint in postJoints)
        {
            if (!joint.isBroken)
            {
                joint.SolveConstraint(dt);
            }
        }
        
        // Appliquer la gravité
        foreach (var body in allBodies)
        {
            if (!body.isStatic)
            {
                body.AddForce(Vector3.up * gravity * body.mass);
            }
        }
        
        // Intégrer la physique
        foreach (var body in allBodies)
        {
            body.IntegratePhysics(dt);
        }
        
        // Détecter et résoudre les collisions
        DetectAndResolveCollisions();
        
        // Collision avec le sol
        HandleGroundCollisions();
    }

    /// <summary>
    /// Détecte et résout toutes les collisions
    /// </summary>
    void DetectAndResolveCollisions()
    {
        for (int i = 0; i < allBodies.Count; i++)
        {
            for (int j = i + 1; j < allBodies.Count; j++)
            {
                if (allBodies[i].isStatic && allBodies[j].isStatic)
                    continue;
                
                CustomCollisionSystem.CollisionContact contact;
                if (collisionSystem.DetectBoxCollision(allBodies[i], allBodies[j], out contact))
                {
                    collisionSystem.ResolveCollision(contact);
                }
            }
        }
    }

    /// <summary>
    /// Gère les collisions avec le sol
    /// </summary>
    void HandleGroundCollisions()
    {
        float groundTop = groundSize.y * 0.5f;
        
        foreach (var body in allBodies)
        {
            if (body.isStatic) continue;
            
            float bottom = body.position.y - body.size.y * 0.5f;
            
            if (bottom < groundTop)
            {
                // Correction de position
                body.position = new Vector3(
                    body.position.x,
                    groundTop + body.size.y * 0.5f,
                    body.position.z
                );
                body.transform.position = body.position;
                
                // Rebond et friction
                if (body.velocity.y < 0)
                {
                    body.velocity.y = -body.velocity.y * 0.3f;
                    body.velocity.x *= 0.8f;
                    body.velocity.z *= 0.8f;
                    body.angularVelocity *= 0.8f;
                }
            }
        }
    }

    /// <summary>
    /// Démarre la simulation (lance le cube)
    /// </summary>
    public void StartSimulation()
    {
        if (simulationStarted) return;
        
        simulationStarted = true;
        
        // Lancer le cube vers la poutre
        Vector3 targetPoint = Vector3.zero;
        targetPoint.y = beamHeight;
        
        Vector3 direction = (targetPoint - cubeBody.position).normalized;
        cubeBody.velocity = direction * impactVelocity;
        
        Debug.Log("Simulation démarrée ! Le cube fonce vers la poutre.");
    }

    /// <summary>
    /// Redémarre la simulation
    /// </summary>
    public void RestartSimulation()
    {
        simulationStarted = false;
        
        // Réinitialiser les positions et vitesses
        segment1Body.position = new Vector3(-beamSegmentSize.x * 0.5f, beamHeight, 0);
        segment1Body.rotation = Quaternion.identity;
        segment1Body.velocity = Vector3.zero;
        segment1Body.angularVelocity = Vector3.zero;
        segment1.transform.position = segment1Body.position;
        segment1.transform.rotation = segment1Body.rotation;
        
        segment2Body.position = new Vector3(beamSegmentSize.x * 0.5f, beamHeight, 0);
        segment2Body.rotation = Quaternion.identity;
        segment2Body.velocity = Vector3.zero;
        segment2Body.angularVelocity = Vector3.zero;
        segment2.transform.position = segment2Body.position;
        segment2.transform.rotation = segment2Body.rotation;
        
        cubeBody.position = cubeStartPosition;
        cubeBody.rotation = Quaternion.identity;
        cubeBody.velocity = Vector3.zero;
        cubeBody.angularVelocity = Vector3.zero;
        impactCube.transform.position = cubeBody.position;
        impactCube.transform.rotation = cubeBody.rotation;
        
        // Réparer les contraintes
        if (centerJoint != null)
        {
            centerJoint.Repair();
        }
        
        foreach (var joint in postJoints)
        {
            joint.Repair();
        }
        
        accumulator = 0f;
        
        Debug.Log("Simulation réinitialisée. Appuyez sur ESPACE pour relancer.");
    }

    /// <summary>
    /// Met à jour les paramètres des contraintes selon l'élasticité
    /// </summary>
    void UpdateJointParameters()
    {
        float adjustedStiffness = jointStiffness * (1f - elasticity * 0.5f);
        float adjustedDamping = jointDamping * (1f - elasticity * 0.3f);
        
        if (centerJoint != null)
        {
            centerJoint.stiffness = adjustedStiffness;
            centerJoint.damping = adjustedDamping;
        }
        
        foreach (var joint in postJoints)
        {
            joint.stiffness = adjustedStiffness * 1.5f;
            joint.damping = adjustedDamping * 1.2f;
        }
    }

    void OnDrawGizmos()
    {
        if (centerJoint != null)
        {
            centerJoint.DrawGizmo();
        }
        
        foreach (var joint in postJoints)
        {
            joint.DrawGizmo();
        }
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        
        GUILayout.Box("Simulation de Poutre - Physique Pure", GUILayout.Width(280));
        
        GUILayout.Space(10);
        
        if (!simulationStarted)
        {
            if (GUILayout.Button("Démarrer (ESPACE)", GUILayout.Height(40)))
            {
                StartSimulation();
            }
        }
        else
        {
            GUILayout.Label("Simulation en cours...");
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button("Redémarrer (R)", GUILayout.Height(30)))
        {
            RestartSimulation();
        }
        
        GUILayout.Space(10);
        
        GUILayout.Label($"Élasticité (α): {elasticity:F2} (Q/E)");
        elasticity = GUILayout.HorizontalSlider(elasticity, 0f, 2f);
        
        GUILayout.Space(5);
        
        if (centerJoint != null)
        {
            string status = centerJoint.isBroken ? "CASSÉE" : $"Tension: {centerJoint.GetTension():F0}N";
            GUILayout.Label($"Contrainte centrale: {status}");
        }
        
        GUILayout.EndArea();
    }
}