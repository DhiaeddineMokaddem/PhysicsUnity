// filepath: c:\Users\Rayen\Documents\GitHub\PhysicsUnity\Assets\Scripts\Animations\Indiv_Work\aziz\CubesOverSphereDemo.cs
using UnityEngine;
using System.Collections.Generic;
using PhysicsUnity.Indiv_Work.Aziz;

/// <summary>
/// Demo bootstrap: builds a static sphere and spawns a group of custom RigidBody3D cubes above it.
/// Uses only the custom physics/math code (no Unity physics components). Unity Transform is used for rendering only.
/// </summary>
public class CubesOverSphereDemo : MonoBehaviour
{
    [Header("Sphere")]
    public float sphereRadius = 2.0f;
    public Vector3 sphereCenter = new Vector3(0f, 2.0f, 0f);
    public Material sphereMaterial;

    [Header("Cubes Grid")]
    public int gridX = 6;
    public int gridY = 5;
    public int gridZ = 6;
    public float spacing = 1.05f;
    public Vector3 cubeSize = Vector3.one;
    public float cubeMass = 1.0f;
    public Material cubeMaterial;

    [Header("Ground/Physics")]
    public float groundLevel = 0f;
    public float groundRestitution = 0.2f;
    public float groundFriction = 0.6f;
    public float globalElasticity = 0.9f;

    [Header("Visuals")]
    public bool createVisualGroundPlane = true;
    public Vector2 groundPlaneSize = new Vector2(30f, 30f);
    public Color groundColor = new Color(0.85f, 0.85f, 0.85f, 1f);

    private PhysicsManagerRayen _physicsManager;
    private DynamicSphere3D _sphere;

    void Start()
    {
        SetupPhysicsManager();
        CreateSphere();
        SpawnCubesAboveSphere();
        CreateGroundVisual();

        // Register all bodies for the physics manager
        _physicsManager.RegisterAllBodies();
    }

    private void SetupPhysicsManager()
    {
        _physicsManager = FindFirstObjectByType<PhysicsManagerRayen>();
        if (_physicsManager == null)
        {
            var go = new GameObject("PhysicsManager");
            _physicsManager = go.AddComponent<PhysicsManagerRayen>();
        }

        _physicsManager.groundLevel = groundLevel;
        _physicsManager.groundRestitution = groundRestitution;
        _physicsManager.groundFriction = groundFriction;
        _physicsManager.globalElasticity = globalElasticity;
        _physicsManager.pauseSimulation = false;
    }

    private void CreateSphere()
    {
        // Create a render sphere primitive as visual shell, then add DynamicSphere3D
        var sphereGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphereGo.name = "DynamicSphere3D";
        sphereGo.transform.SetParent(transform, worldPositionStays: true);
        sphereGo.transform.position = sphereCenter;
        sphereGo.transform.localScale = Vector3.one * (sphereRadius * 2f);

        var col = sphereGo.GetComponent<Collider>();
        if (col != null) DestroyImmediate(col);

        _sphere = sphereGo.AddComponent<DynamicSphere3D>();
        _sphere.radius = sphereRadius;
        _sphere.mass = 8.0f;
        _sphere.restitution = 0.3f;
        _sphere.friction = 0.6f;
        _sphere.linearDamping = 0.02f;
        _sphere.isKinematic = false;
        _sphere.useGravity = true;
        _sphere.sphereMaterial = sphereMaterial;
        _sphere.position = sphereCenter;
        _sphere.velocity = Vector3.zero;
        _sphere.UpdateVisualTransform();
    }

    private void SpawnCubesAboveSphere()
    {
        // Compute the base height so the lowest cube starts above the sphere
        float gridHeight = (gridY - 1) * spacing + cubeSize.y; // approximate stack height
        float baseY = sphereCenter.y + sphereRadius + 0.5f + gridHeight * 0.25f;

        int count = 0;
        for (int ix = 0; ix < gridX; ix++)
        {
            for (int iy = 0; iy < gridY; iy++)
            {
                for (int iz = 0; iz < gridZ; iz++)
                {
                    Vector3 pos = new Vector3(
                        (ix - (gridX - 1) * 0.5f) * spacing,
                        baseY + iy * spacing,
                        (iz - (gridZ - 1) * 0.5f) * spacing
                    );

                    CreateCube(pos, cubeSize, cubeMass, count);
                    count++;
                }
            }
        }
    }

    private GameObject CreateCube(Vector3 position, Vector3 size, float mass, int index)
    {
        // Create render-only primitive, then strip collider
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.name = $"Cube_{index}";
        cube.transform.SetParent(transform, worldPositionStays: true);
        cube.transform.position = position;
        cube.transform.localScale = size;

        var col = cube.GetComponent<Collider>();
        if (col != null) DestroyImmediate(col);

        var body = cube.AddComponent<RigidBody3D>();
        body.mass = mass;
        body.size = size;
        body.useGravity = true;
        body.isKinematic = false;
        body.restitution = 0.3f;
        body.friction = 0.6f;
        body.linearDamping = 0.02f;
        body.angularDamping = 0.02f;
        body.InitializePosition(position, Quaternion.identity, size);
        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;

        // Assign material/color
        var mr = cube.GetComponent<Renderer>();
        if (mr != null)
        {
            if (cubeMaterial != null) mr.material = cubeMaterial;
            else mr.material.color = Random.ColorHSV(0f, 1f, 0.6f, 1f, 0.6f, 1f);
        }

        return cube;
    }

    private void CreateGroundVisual()
    {
        if (!createVisualGroundPlane) return;

        var ground = GameObject.CreatePrimitive(PrimitiveType.Quad);
        ground.name = "GroundVisual";
        ground.transform.SetParent(transform, worldPositionStays: true);
        ground.transform.position = new Vector3(0f, groundLevel, 0f);
        ground.transform.rotation = Quaternion.Euler(90f, 0f, 0f); // lay flat
        ground.transform.localScale = new Vector3(groundPlaneSize.x, groundPlaneSize.y, 1f);

        var col = ground.GetComponent<Collider>();
        if (col != null) DestroyImmediate(col);

        var mr = ground.GetComponent<Renderer>();
        if (mr != null)
        {
            if (mr.material != null)
            {
                mr.material.color = groundColor;
            }
        }
    }
}
