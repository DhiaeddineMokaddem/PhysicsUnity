using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Génère une structure de cubes connectés
/// </summary>
public class StructureBuilder : MonoBehaviour
{
    [Header("Dimensions de la Structure")]
    public int width = 5;
    public int height = 5;
    public int depth = 5;
    public float spacing = 1.1f; // Espacement entre les cubes
    
    [Header("Propriétés des Cubes")]
    public float cubeSize = 1.0f;
    public float cubeMass = 0.5f;
    public Material cubeMaterial;
    
    [Header("Propriétés des Contraintes")]
    public float constraintStiffness = 800.0f;
    public float constraintDamping = 40.0f;
    public float constraintBreakForce = 80.0f;
    public float constraintMaxDistance = 1.5f;
    
    [Header("Construction")]
    public bool buildOnStart = true;
    public bool connectDiagonals = false; // Connecter aussi les diagonales
    
    private List<GameObject> cubes = new List<GameObject>();
    private PhysicsManager physicsManager;

    void Start()
    {
        physicsManager = FindObjectOfType<PhysicsManager>();
        
        if (buildOnStart)
        {
            BuildStructure();
        }
    }

    /// <summary>
    /// Construit la structure complète
    /// </summary>
    public void BuildStructure()
    {
        ClearStructure();
        
        Debug.Log($"Construction d'une structure {width}x{height}x{depth}...");
        
        // Créer les cubes
        RigidBody3D[,,] cubeGrid = new RigidBody3D[width, height, depth];
        
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                for (int z = 0; z < depth; z++)
                {
                    Vector3 position = new Vector3(
                        (x - width / 2f) * spacing,
                        y * spacing,
                        (z - depth / 2f) * spacing
                    );
                    
                    GameObject cube = CreateCube(position, new Vector3(cubeSize, cubeSize, cubeSize));
                    cubes.Add(cube);
                    
                    RigidBody3D body = cube.GetComponent<RigidBody3D>();
                    cubeGrid[x, y, z] = body;
                }
            }
        }
        
        // Créer les contraintes
        CreateConstraints(cubeGrid);
        
        // Enregistrer dans le PhysicsManager
        if (physicsManager != null)
        {
            physicsManager.RegisterAllBodies();
        }
        
        Debug.Log($"Structure construite: {cubes.Count} cubes créés");
    }

    /// <summary>
    /// Crée un cube individuel
    /// </summary>
    GameObject CreateCube(Vector3 position, Vector3 size)
    {
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = position;
        cube.transform.localScale = size;
        cube.transform.parent = transform;
        cube.name = $"Cube_{cubes.Count}";
        
        // Retirer le Collider par défaut
        Collider collider = cube.GetComponent<Collider>();
        if (collider != null)
        {
            Destroy(collider);
        }
        
        // Ajouter notre RigidBody personnalisé
        RigidBody3D body = cube.AddComponent<RigidBody3D>();
        body.mass = cubeMass;
        body.size = size;
        
        // Appliquer le matériau
        if (cubeMaterial != null)
        {
            cube.GetComponent<Renderer>().material = cubeMaterial;
        }
        else
        {
            // Matériau par défaut avec une couleur aléatoire
            cube.GetComponent<Renderer>().material.color = Random.ColorHSV(0f, 1f, 0.5f, 1f, 0.5f, 1f);
        }
        
        return cube;
    }

    /// <summary>
    /// Crée les contraintes entre les cubes
    /// </summary>
    void CreateConstraints(RigidBody3D[,,] cubeGrid)
    {
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                for (int z = 0; z < depth; z++)
                {
                    RigidBody3D current = cubeGrid[x, y, z];
                    
                    // Connexions orthogonales (6 directions)
                    ConnectIfValid(current, cubeGrid, x + 1, y, z); // Droite
                    ConnectIfValid(current, cubeGrid, x, y + 1, z); // Haut
                    ConnectIfValid(current, cubeGrid, x, y, z + 1); // Avant
                    
                    if (connectDiagonals)
                    {
                        // Connexions diagonales dans le plan horizontal
                        ConnectIfValid(current, cubeGrid, x + 1, y, z + 1);
                        ConnectIfValid(current, cubeGrid, x + 1, y, z - 1);
                        
                        // Connexions diagonales verticales
                        ConnectIfValid(current, cubeGrid, x + 1, y + 1, z);
                        ConnectIfValid(current, cubeGrid, x, y + 1, z + 1);
                        ConnectIfValid(current, cubeGrid, x + 1, y + 1, z + 1);
                        ConnectIfValid(current, cubeGrid, x + 1, y + 1, z - 1);
                        ConnectIfValid(current, cubeGrid, x - 1, y + 1, z + 1);
                        ConnectIfValid(current, cubeGrid, x - 1, y + 1, z - 1);
                    }
                }
            }
        }
    }

    /// <summary>
    /// Crée une contrainte si les indices sont valides
    /// </summary>
    void ConnectIfValid(RigidBody3D bodyA, RigidBody3D[,,] grid, int x, int y, int z)
    {
        if (x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth)
        {
            RigidBody3D bodyB = grid[x, y, z];
            CreateConstraint(bodyA, bodyB);
        }
    }

    /// <summary>
    /// Crée une contrainte entre deux corps
    /// </summary>
    void CreateConstraint(RigidBody3D bodyA, RigidBody3D bodyB)
    {
        // Point de connexion (milieu entre les deux cubes)
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
    }

    /// <summary>
    /// Efface la structure existante
    /// </summary>
    public void ClearStructure()
    {
        foreach (GameObject cube in cubes)
        {
            if (cube != null)
            {
                Destroy(cube);
            }
        }
        cubes.Clear();
        
        // Détruire aussi les contraintes
        RigidConstraint[] constraints = GetComponentsInChildren<RigidConstraint>();
        foreach (var constraint in constraints)
        {
            if (constraint != null)
            {
                Destroy(constraint.gameObject);
            }
        }
    }

    /// <summary>
    /// Construit une structure pyramidale
    /// </summary>
    public void BuildPyramid(int baseSize, float cubeSpacing)
    {
        ClearStructure();
        spacing = cubeSpacing;
        
        List<RigidBody3D> allBodies = new List<RigidBody3D>();
        
        for (int level = 0; level < baseSize; level++)
        {
            int levelSize = baseSize - level;
            float yPos = level * spacing;
            
            for (int x = 0; x < levelSize; x++)
            {
                for (int z = 0; z < levelSize; z++)
                {
                    Vector3 position = new Vector3(
                        (x - levelSize / 2f) * spacing,
                        yPos,
                        (z - levelSize / 2f) * spacing
                    );
                    
                    GameObject cube = CreateCube(position, new Vector3(cubeSize, cubeSize, cubeSize));
                    cubes.Add(cube);
                    allBodies.Add(cube.GetComponent<RigidBody3D>());
                }
            }
        }
        
        // Créer les contraintes entre cubes proches
        for (int i = 0; i < allBodies.Count; i++)
        {
            for (int j = i + 1; j < allBodies.Count; j++)
            {
                float dist = Vector3.Distance(allBodies[i].transform.position, allBodies[j].transform.position);
                if (dist < spacing * 1.5f)
                {
                    CreateConstraint(allBodies[i], allBodies[j]);
                }
            }
        }
        
        if (physicsManager != null)
        {
            physicsManager.RegisterAllBodies();
        }
        
        Debug.Log($"Pyramide construite: {cubes.Count} cubes");
    }

    /// <summary>
    /// Construit une tour
    /// </summary>
    public void BuildTower(int levels, int cubesPerLevel)
    {
        ClearStructure();
        
        List<RigidBody3D> allBodies = new List<RigidBody3D>();
        
        for (int y = 0; y < levels; y++)
        {
            float angle = (y % 2 == 0) ? 0f : 90f;
            
            for (int i = 0; i < cubesPerLevel; i++)
            {
                float offset = (i - cubesPerLevel / 2f) * spacing;
                Vector3 position = new Vector3(
                    Mathf.Cos(angle * Mathf.Deg2Rad) * offset,
                    y * spacing,
                    Mathf.Sin(angle * Mathf.Deg2Rad) * offset
                );
                
                GameObject cube = CreateCube(position, new Vector3(cubeSize, cubeSize, cubeSize));
                cubes.Add(cube);
                allBodies.Add(cube.GetComponent<RigidBody3D>());
            }
        }
        
        // Connecter les cubes proches
        for (int i = 0; i < allBodies.Count; i++)
        {
            for (int j = i + 1; j < allBodies.Count; j++)
            {
                float dist = Vector3.Distance(allBodies[i].transform.position, allBodies[j].transform.position);
                if (dist < spacing * 2f)
                {
                    CreateConstraint(allBodies[i], allBodies[j]);
                }
            }
        }
        
        if (physicsManager != null)
        {
            physicsManager.RegisterAllBodies();
        }
        
        Debug.Log($"Tour construite: {cubes.Count} cubes");
    }

    void OnDrawGizmos()
    {
        // Visualiser la zone de construction
        Gizmos.color = new Color(0f, 1f, 0f, 0.2f);
        Vector3 center = new Vector3(0, height * spacing / 2f, 0);
        Vector3 size = new Vector3(width * spacing, height * spacing, depth * spacing);
        Gizmos.DrawWireCube(center, size);
    }
}