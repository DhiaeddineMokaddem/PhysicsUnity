using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Génère un plan HORIZONTAL de cubes suspendus au-dessus du sol
/// ADAPTÉ de StructureBuilder.cs pour créer une structure 2D (grille XZ)
/// </summary>
public class HorizontalPlateBuilder : MonoBehaviour
{[Header("Dimensions du Plan Horizontal")]
    [Tooltip("Nombre de cubes en largeur (axe X)")]
    public int width = 10;
    
    [Tooltip("Nombre de cubes en profondeur (axe Z)")]
    public int depth = 10;
    
    [Tooltip("Hauteur de suspension au-dessus du sol")]
    public float suspensionHeight = 5.0f;
    
    [Tooltip("Espacement entre cubes (multiplicateur de la taille X)")]
    public float spacing = 1.1f;
    
    [Header("Dimensions des Cubes (PERSONNALISABLES)")]
    [Tooltip("Largeur du cube (axe X)")]
    public float cubeWidth = 1.0f;
    
    [Tooltip("Hauteur du cube (axe Y) - ex: 0.1 pour plaques fines")]
    public float cubeHeight = 1.0f;
    
    [Tooltip("Profondeur du cube (axe Z)")]
    public float cubeDepth = 1.0f;
    
    [Header("Propriétés des Cubes")]
    public float cubeMass = 1.0f;
    public Material cubeMaterial;
    
    [Header("Propriétés des Contraintes")]
    [Tooltip("Rigidité des ressorts entre cubes")]
    public float constraintStiffness = 500.0f;
    
    [Tooltip("Amortissement des contraintes")]
    public float constraintDamping = 30.0f;
    
    [Tooltip("Force maximale avant rupture")]
    public float constraintBreakForce = 100.0f;
    
    [Tooltip("Distance maximale avant rupture")]
    public float constraintMaxDistance = 2.0f;
    
    [Header("Options de Connexion")]
    [Tooltip("Connecter aussi les diagonales pour plus de rigidité")]
    public bool connectDiagonals = true;
    
    [Header("Presets de Dimensions")]
    public bool useThinPlatePreset = false; // Ex: 1x0.1x1
    public bool useThickPlatePreset = false; // Ex: 1x0.5x1
    
    [Header("Construction")]
    public bool buildOnStart = true;
    
    // Données internes
    private List<GameObject> cubes = new List<GameObject>();
    private List<GameObject> constraints = new List<GameObject>();
    private PhysicsManager physicsManager;
    private RigidBody3D[,] cubeGrid;

    void Start()
    {
        physicsManager = FindObjectOfType<PhysicsManager>();
        
        // Appliquer les presets si demandés
        ApplyPresets();
        
        if (buildOnStart)
        {
            BuildHorizontalPlate();
        }
    }

    void ApplyPresets()
    {
        if (useThinPlatePreset)
        {
            cubeWidth = 1.0f;
            cubeHeight = 0.1f;
            cubeDepth = 1.0f;
            Debug.Log("Preset appliqué: Plaque fine (1x0.1x1)");
        }
        else if (useThickPlatePreset)
        {
            cubeWidth = 1.0f;
            cubeHeight = 0.5f;
            cubeDepth = 1.0f;
            Debug.Log("Preset appliqué: Plaque épaisse (1x0.5x1)");
        }
    }

    /// <summary>
    /// AMÉLIORÉ: Construction avec dimensions personnalisables
    /// </summary>
    public void BuildHorizontalPlate()
    {
        ClearStructure();
        
        Debug.Log($"Construction d'un plan horizontal {width}x{depth} à hauteur {suspensionHeight}m");
        Debug.Log($"  Dimensions cubes: {cubeWidth}x{cubeHeight}x{cubeDepth}");
        
        cubeGrid = new RigidBody3D[width, depth];
        
        Vector3 plateCenter = transform.position + Vector3.up * suspensionHeight;
        
        // Calculer l'espacement basé sur la largeur du cube
        float spacingX = cubeWidth * spacing;
        float spacingZ = cubeDepth * spacing;
        
        // Créer les cubes avec dimensions personnalisées
        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < depth; z++)
            {
                Vector3 position = plateCenter + new Vector3(
                    (x - width / 2f) * spacingX,
                    0f,
                    (z - depth / 2f) * spacingZ
                );
                
                // NOUVEAU: Passer les dimensions personnalisées
                Vector3 cubeSize = new Vector3(cubeWidth, cubeHeight, cubeDepth);
                GameObject cube = CreateCube(position, cubeSize);
                cubes.Add(cube);
                
                RigidBody3D body = cube.GetComponent<RigidBody3D>();
                cubeGrid[x, z] = body;
            }
        }
        
        CreateHorizontalConstraints();
        
        if (physicsManager != null)
        {
            physicsManager.RegisterAllBodies();
        }
        
        Debug.Log($"✓ Plan construit : {cubes.Count} cubes, {constraints.Count} contraintes");
    }

    /// <summary>
    /// AMÉLIORÉ: Création de cube avec dimensions personnalisées
    /// </summary>
    GameObject CreateCube(Vector3 position, Vector3 size)
    {
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = position;
        cube.transform.localScale = size;
        cube.transform.parent = transform;
        cube.name = $"Cube_{cubes.Count}";
        
        // Retirer le Collider Unity par défaut
        Collider collider = cube.GetComponent<Collider>();
        if (collider != null)
        {
            Destroy(collider);
        }
        
        // Ajouter notre RigidBody personnalisé
        RigidBody3D body = cube.AddComponent<RigidBody3D>();
        body.mass = cubeMass;
        body.size = size; // IMPORTANT: Utiliser la taille personnalisée
        body.useGravity = true;
        
        // Matériau visuel
        if (cubeMaterial != null)
        {
            cube.GetComponent<Renderer>().material = cubeMaterial;
        }
        else
        {
            float hue = (float)cubes.Count / (width * depth);
            cube.GetComponent<Renderer>().material.color = Color.HSVToRGB(hue, 0.7f, 0.9f);
        }
        
        return cube;
    }

    void CreateHorizontalConstraints()
    {
        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < depth; z++)
            {
                RigidBody3D current = cubeGrid[x, z];
                
                // Connexions orthogonales
                ConnectIfValid(current, x + 1, z);
                ConnectIfValid(current, x, z + 1);
                
                if (connectDiagonals)
                {
                    ConnectIfValid(current, x + 1, z + 1);
                    ConnectIfValid(current, x + 1, z - 1);
                }
            }
        }
    }

    void ConnectIfValid(RigidBody3D bodyA, int x, int z)
    {
        if (x >= 0 && x < width && z >= 0 && z < depth)
        {
            RigidBody3D bodyB = cubeGrid[x, z];
            CreateConstraint(bodyA, bodyB);
        }
    }

    void CreateConstraint(RigidBody3D bodyA, RigidBody3D bodyB)
    {
        Vector3 connectionPoint = (bodyA.transform.position + bodyB.transform.position) * 0.5f;
        
        GameObject constraintObj = new GameObject($"Constraint_{bodyA.name}_{bodyB.name}");
        constraintObj.transform.position = connectionPoint;
        constraintObj.transform.parent = transform;
        
        RigidConstraint constraint = constraintObj.AddComponent<RigidConstraint>();
        constraint.bodyA = bodyA;
        constraint.bodyB = bodyB;
        constraint.stiffness = constraintStiffness;
        constraint.damping = constraintDamping;
        constraint.breakForce = constraintBreakForce;
        constraint.maxDistance = constraintMaxDistance;
        
        constraints.Add(constraintObj);
    }

    public void ClearStructure()
    {
        foreach (GameObject cube in cubes)
        {
            if (cube != null) Destroy(cube);
        }
        cubes.Clear();
        
        foreach (GameObject constraintObj in constraints)
        {
            if (constraintObj != null) Destroy(constraintObj);
        }
        constraints.Clear();
        
        cubeGrid = null;
    }

    public Vector3 GetPlateCenter()
    {
        return transform.position + Vector3.up * suspensionHeight;
    }

    public Vector3 GetPlateDimensions()
    {
        float spacingX = cubeWidth * spacing;
        float spacingZ = cubeDepth * spacing;
        return new Vector3(width * spacingX, cubeHeight, depth * spacingZ);
    }

    /// <summary>
    /// NOUVEAU: Changer rapidement les dimensions des cubes
    /// </summary>
    public void SetCubeDimensions(float width, float height, float depth)
    {
        cubeWidth = width;
        cubeHeight = height;
        cubeDepth = depth;
        Debug.Log($"Dimensions changées: {width}x{height}x{depth}");
    }

    /// <summary>
    /// NOUVEAU: Presets rapides
    /// </summary>
    public void ApplyThinPlatePreset()
    {
        SetCubeDimensions(1.0f, 0.1f, 1.0f);
        BuildHorizontalPlate();
    }

    public void ApplyMediumPlatePreset()
    {
        SetCubeDimensions(1.0f, 0.5f, 1.0f);
        BuildHorizontalPlate();
    }

    public void ApplyThickPlatePreset()
    {
        SetCubeDimensions(1.0f, 1.0f, 1.0f);
        BuildHorizontalPlate();
    }

    void OnDrawGizmos()
    {
        Gizmos.color = new Color(0f, 1f, 1f, 0.3f);
        Vector3 center = transform.position + Vector3.up * suspensionHeight;
        float spacingX = cubeWidth * spacing;
        float spacingZ = cubeDepth * spacing;
        Vector3 size = new Vector3(width * spacingX, cubeHeight, depth * spacingZ);
        Gizmos.DrawWireCube(center, size);
        
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, center);
    }

    void OnDrawGizmosSelected()
    {
        Vector3 center = transform.position + Vector3.up * suspensionHeight;
        float spacingX = cubeWidth * spacing;
        float spacingZ = cubeDepth * spacing;
        
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(center, new Vector3(width * spacingX, cubeHeight, depth * spacingZ));
        
        float hw = width * spacingX * 0.5f;
        float hd = depth * spacingZ * 0.5f;
        Gizmos.DrawSphere(center + new Vector3(hw, 0, hd), 0.2f);
        Gizmos.DrawSphere(center + new Vector3(-hw, 0, hd), 0.2f);
        Gizmos.DrawSphere(center + new Vector3(hw, 0, -hd), 0.2f);
        Gizmos.DrawSphere(center + new Vector3(-hw, 0, -hd), 0.2f);
    }
}