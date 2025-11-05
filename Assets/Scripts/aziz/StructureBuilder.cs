using System.Collections;
using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Génère une structure de cubes connectés - VERSION PURE MATH
/// FIXED: Proper cleanup, initialization, and settling for rebuilding
/// </summary>
public class StructureBuilder : MonoBehaviour
{
    [Header("Dimensions de la Structure")]
    public int width = 5;
    public int height = 5;
    public int depth = 5;
    public float spacing = 1.1f;
    
    [Header("Propriétés des Cubes")]
    public float cubeSize = 1.0f;
    public float cubeMass = 0.5f;
    public Material cubeMaterial;
    
    [Header("Propriétés des Contraintes")]
    public float constraintStiffness = 1200.0f;  // Increased from 800
    public float constraintDamping = 60.0f;      // Increased from 40
    public float constraintBreakForce = 150.0f;  // Increased from 80
    public float constraintMaxDistance = 1.5f;
    
    [Header("Construction")]
    public bool buildOnStart = true;
    public bool connectDiagonals = false;
    
    private List<GameObject> cubes = new List<GameObject>();
    private PhysicsManager physicsManager;

    void Start()
    {
        physicsManager = FindObjectOfType<PhysicsManager>();
    
        if (buildOnStart)
        {
            StartCoroutine(DelayedBuild());
        }
    }

    IEnumerator DelayedBuild()
    {
        yield return new WaitForSeconds(0.02f); // wait 1 second
        BuildStructure();
    }


    /// <summary>
    /// Construit la structure complète - PURE MATH
    /// FIXED: Pauses physics during construction and ensures proper initialization
    /// </summary>
    public void BuildStructure()
    {
        // Force pause physics during construction
        bool wasPaused = false;
        if (physicsManager != null)
        {
            wasPaused = physicsManager.pauseSimulation;
            physicsManager.pauseSimulation = true;
        }
        
        ClearStructure();
        
        Debug.Log($"Construction d'une structure {width}x{height}x{depth}...");
        
        RigidBody3D[,,] cubeGrid = new RigidBody3D[width, height, depth];
        
        // Create all cubes first
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                for (int z = 0; z < depth; z++)
                {
                    // PURE MATH: Calcul manuel de position
                    Vector3 position = new Vector3(
                        (x - width / 2f) * spacing,
                        y * spacing,
                        (z - depth / 2f) * spacing
                    );
                    
                    GameObject cube = CreateCube(position, new Vector3(cubeSize, cubeSize, cubeSize));
                    cubes.Add(cube);
                    
                    RigidBody3D body = cube.GetComponent<RigidBody3D>();
                    cubeGrid[x, y, z] = body;
                    
                    // Ensure zero velocity
                    body.velocity = Vector3.zero;
                    body.angularVelocity = Vector3.zero;
                }
            }
        }
        
        // Register all bodies before creating constraints
        if (physicsManager != null)
        {
            physicsManager.RegisterAllBodies();
        }
        
        // Now create constraints with proper initialization
        CreateConstraints(cubeGrid);
        
        // Wait one frame before resuming physics
        StartCoroutine(ResumePhysicsAfterBuild(wasPaused));
        
        Debug.Log($"Structure construite: {cubes.Count} cubes créés");
    }
    
    /// <summary>
    /// Resume physics after a short delay to ensure everything is settled
    /// </summary>
    System.Collections.IEnumerator ResumePhysicsAfterBuild(bool wasPaused)
    {
        // Wait for physics to settle
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();
        
        // Ensure all velocities are zero
        foreach (GameObject cube in cubes)
        {
            if (cube != null)
            {
                RigidBody3D body = cube.GetComponent<RigidBody3D>();
                if (body != null)
                {
                    body.velocity = Vector3.zero;
                    body.angularVelocity = Vector3.zero;
                }
            }
        }
        
        // Resume physics if it wasn't paused before
        if (physicsManager != null)
        {
            physicsManager.pauseSimulation = wasPaused;
        }
    }

    /// <summary>
    /// Crée un cube individuel - PURE MATH
    /// FIXED: Uses InitializePosition to ensure proper setup before Awake()
    /// </summary>
    GameObject CreateCube(Vector3 position, Vector3 size)
    {
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        
        // Set transform first
        cube.transform.position = position;
        cube.transform.localScale = size;
        cube.transform.parent = transform;
        cube.name = $"Cube_{cubes.Count}";
        
        Collider collider = cube.GetComponent<Collider>();
        if (collider != null)
        {
            DestroyImmediate(collider);
        }
        
        RigidBody3D body = cube.AddComponent<RigidBody3D>();
        body.mass = cubeMass;
        body.size = size;
        
        // CRITICAL FIX: Use InitializePosition method to set everything at once
        // This happens before Awake(), preventing initialization issues
        body.InitializePosition(position, Quaternion.identity, size);
        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
        
        if (cubeMaterial != null)
        {
            cube.GetComponent<Renderer>().material = cubeMaterial;
        }
        else
        {
            cube.GetComponent<Renderer>().material.color = Random.ColorHSV(0f, 1f, 0.5f, 1f, 0.5f, 1f);
        }
        
        return cube;
    }

    void CreateConstraints(RigidBody3D[,,] cubeGrid)
    {
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                for (int z = 0; z < depth; z++)
                {
                    RigidBody3D current = cubeGrid[x, y, z];
                    
                    ConnectIfValid(current, cubeGrid, x + 1, y, z);
                    ConnectIfValid(current, cubeGrid, x, y + 1, z);
                    ConnectIfValid(current, cubeGrid, x, y, z + 1);
                    
                    if (connectDiagonals)
                    {
                        ConnectIfValid(current, cubeGrid, x + 1, y, z + 1);
                        ConnectIfValid(current, cubeGrid, x + 1, y, z - 1);
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

    void ConnectIfValid(RigidBody3D bodyA, RigidBody3D[,,] grid, int x, int y, int z)
    {
        if (x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth)
        {
            RigidBody3D bodyB = grid[x, y, z];
            CreateConstraint(bodyA, bodyB);
        }
    }

    /// <summary>
    /// Crée une contrainte - PURE MATH
    /// FIXED: Properly initialize constraint after setting parameters
    /// </summary>
    void CreateConstraint(RigidBody3D bodyA, RigidBody3D bodyB)
    {
        // PURE MATH: Calcul manuel du point de connexion
        Vector3 connectionPoint = (bodyA.position + bodyB.position) * 0.5f;
        
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
        
        // Initialize the constraint immediately with correct rest length
        constraint.Initialize();
    }

    /// <summary>
    /// FIXED: Properly clears and waits before starting new construction
    /// </summary>
    public void ClearStructure()
    {
        // Pause physics during clearing
        if (physicsManager != null)
        {
            physicsManager.pauseSimulation = true;
        }
        
        // Clear constraints first
        RigidConstraint[] constraints = GetComponentsInChildren<RigidConstraint>();
        foreach (var constraint in constraints)
        {
            if (constraint != null)
            {
                DestroyImmediate(constraint.gameObject);
            }
        }
        
        // Then clear cubes
        foreach (GameObject cube in cubes)
        {
            if (cube != null)
            {
                DestroyImmediate(cube);
            }
        }
        cubes.Clear();
        
        // Force physics manager to update its lists
        if (physicsManager != null)
        {
            physicsManager.RegisterAllBodies();
        }
    }

    /// <summary>
    /// Construit une pyramide - PURE MATH
    /// FIXED: Pauses physics during construction and ensures proper initialization
    /// </summary>
    public void BuildPyramid(int baseSize, float cubeSpacing)
    {
        // Pause physics during construction
        bool wasPaused = false;
        if (physicsManager != null)
        {
            wasPaused = physicsManager.pauseSimulation;
            physicsManager.pauseSimulation = true;
        }
        
        ClearStructure();
        spacing = cubeSpacing;
        
        List<RigidBody3D> allBodies = new List<RigidBody3D>();
        
        // Create all cubes first
        for (int level = 0; level < baseSize; level++)
        {
            int levelSize = baseSize - level;
            float yPos = level * spacing;
            
            for (int x = 0; x < levelSize; x++)
            {
                for (int z = 0; z < levelSize; z++)
                {
                    // PURE MATH: Calcul manuel
                    Vector3 position = new Vector3(
                        (x - levelSize / 2f) * spacing,
                        yPos,
                        (z - levelSize / 2f) * spacing
                    );
                    
                    GameObject cube = CreateCube(position, new Vector3(cubeSize, cubeSize, cubeSize));
                    cubes.Add(cube);
                    
                    RigidBody3D body = cube.GetComponent<RigidBody3D>();
                    allBodies.Add(body);
                    
                    // Ensure zero velocity
                    body.velocity = Vector3.zero;
                    body.angularVelocity = Vector3.zero;
                }
            }
        }
        
        // Register bodies before creating constraints
        if (physicsManager != null)
        {
            physicsManager.RegisterAllBodies();
        }
        
        // Create constraints with proper distances
        for (int i = 0; i < allBodies.Count; i++)
        {
            for (int j = i + 1; j < allBodies.Count; j++)
            {
                float dist = Vector3.Distance(allBodies[i].position, allBodies[j].position);
                if (dist < spacing * 1.5f)
                {
                    CreateConstraint(allBodies[i], allBodies[j]);
                }
            }
        }
        
        // Wait before resuming physics
        StartCoroutine(ResumePhysicsAfterBuild(wasPaused));
        
        Debug.Log($"Pyramide construite: {cubes.Count} cubes");
    }

    /// <summary>
    /// Construit une tour - PURE MATH
    /// FIXED: Pauses physics during construction and ensures proper initialization
    /// </summary>
    public void BuildTower(int levels, int cubesPerLevel)
    {
        // Pause physics during construction
        bool wasPaused = false;
        if (physicsManager != null)
        {
            wasPaused = physicsManager.pauseSimulation;
            physicsManager.pauseSimulation = true;
        }
        
        ClearStructure();
        
        List<RigidBody3D> allBodies = new List<RigidBody3D>();
        
        // Create all cubes first
        for (int y = 0; y < levels; y++)
        {
            float angle = (y % 2 == 0) ? 0f : 90f;
            
            for (int i = 0; i < cubesPerLevel; i++)
            {
                float offset = (i - cubesPerLevel / 2f) * spacing;
                
                // PURE MATH: Calcul trigonométrique manuel
                Vector3 position = new Vector3(
                    Mathf.Cos(angle * Mathf.Deg2Rad) * offset,
                    y * spacing,
                    Mathf.Sin(angle * Mathf.Deg2Rad) * offset
                );
                
                GameObject cube = CreateCube(position, new Vector3(cubeSize, cubeSize, cubeSize));
                cubes.Add(cube);
                
                RigidBody3D body = cube.GetComponent<RigidBody3D>();
                allBodies.Add(body);
                
                // Ensure zero velocity
                body.velocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
        
        // Register bodies before creating constraints
        if (physicsManager != null)
        {
            physicsManager.RegisterAllBodies();
        }
        
        // Create constraints
        for (int i = 0; i < allBodies.Count; i++)
        {
            for (int j = i + 1; j < allBodies.Count; j++)
            {
                float dist = Vector3.Distance(allBodies[i].position, allBodies[j].position);
                if (dist < spacing * 2f)
                {
                    CreateConstraint(allBodies[i], allBodies[j]);
                }
            }
        }
        
        // Wait before resuming physics
        StartCoroutine(ResumePhysicsAfterBuild(wasPaused));
        
        Debug.Log($"Tour construite: {cubes.Count} cubes");
    }

    void OnDrawGizmos()
    {
        Gizmos.color = new Color(0f, 1f, 0f, 0.2f);
        Vector3 center = new Vector3(0, height * spacing / 2f, 0);
        Vector3 size = new Vector3(width * spacing, height * spacing, depth * spacing);
        Gizmos.DrawWireCube(center, size);
    }
}