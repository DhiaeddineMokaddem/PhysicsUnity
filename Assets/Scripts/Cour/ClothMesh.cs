using System.Collections.Generic;
using UnityEngine;
using PhysicsSimulation.Core;

/// <summary>
/// A simple mass-spring cloth simulation that builds a rectangular grid of
/// masses connected by springs. Refactored to use shared utilities from PhysicsUnity.Core
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ClothSimulation : MonoBehaviour
{
    #region Inspector Properties
    [Header("Grid size")]
    [Tooltip("Number of rows in the cloth grid.")]
    public int numberOfLines = 5;
    [Tooltip("Number of columns in the cloth grid.")]
    public int vertexesPerLine = 5;
    [Tooltip("Distance between adjacent vertices in metres.")]
    public float distanceBetweenVertexes = 0.2f;

    [Header("Spring parameters")]
    [Tooltip("Hooke's law stiffness constant for all springs.")]
    public float springStiffness = 50f;
    [Tooltip("Damping constant applied along each spring.")]
    public float springDamping = 1f;
    [Tooltip("Mass of each particle (kg).")]
    public float mass = 1f;
    [Tooltip("Gravity applied to each particle.")]
    public Vector3 gravity = PhysicsConstants.GRAVITY_VECTOR;
    [Tooltip("Whether to pin two corners instead of the whole top row.")]
    public bool TwoCorner = false;
    #endregion

    #region Private Fields
    private Vector3[] basePositions;
    private Vector3[] positions;
    private Vector3[] velocities;
    private Vector3[] forces;
    private bool[] isPinned;
    private Mesh mesh;
    private int[] meshTriangles;
    private List<ClothSpring> springs;
    #endregion

    #region Initialization
    void Awake()
    {
        int count = numberOfLines * vertexesPerLine;
        basePositions = new Vector3[count];
        positions = new Vector3[count];
        velocities = new Vector3[count];
        forces = new Vector3[count];
        isPinned = new bool[count];
        springs = new List<ClothSpring>();

        for (int row = 0; row < numberOfLines; row++)
        {
            for (int col = 0; col < vertexesPerLine; col++)
            {
                int index = row * vertexesPerLine + col;
                Vector3 pos = new Vector3(row * distanceBetweenVertexes, 0f, col * distanceBetweenVertexes);
                basePositions[index] = pos;
                positions[index] = pos;
                velocities[index] = Vector3.zero;
                forces[index] = Vector3.zero;
                
                if (TwoCorner)
                    isPinned[index] = (row == 0 || row == numberOfLines - 1);
                else
                    isPinned[index] = (row == 0);
            }
        }

        List<int> triList = new List<int>();
        for (int row = 0; row < numberOfLines - 1; row++)
        {
            for (int col = 0; col < vertexesPerLine; col++)
            {
                if (col != vertexesPerLine - 1)
                {
                    triList.Add(col + row * vertexesPerLine);
                    triList.Add(col + row * vertexesPerLine + 1);
                    triList.Add(col + (row + 1) * vertexesPerLine);
                }
                if (col != 0)
                {
                    triList.Add(col + row * vertexesPerLine);
                    triList.Add(col + (row + 1) * vertexesPerLine);
                    triList.Add(col + (row + 1) * vertexesPerLine - 1);
                }
            }
        }
        meshTriangles = triList.ToArray();

        BuildSprings();
        InitializeMesh();
    }

    void BuildSprings()
    {
        for (int row = 0; row < numberOfLines; row++)
        {
            for (int col = 0; col < vertexesPerLine; col++)
            {
                int index = row * vertexesPerLine + col;
                
                if (col < vertexesPerLine - 1)
                    springs.Add(new ClothSpring(index, index + 1, distanceBetweenVertexes));
                
                if (row < numberOfLines - 1)
                    springs.Add(new ClothSpring(index, index + vertexesPerLine, distanceBetweenVertexes));
                
                if (row < numberOfLines - 1 && col < vertexesPerLine - 1)
                    springs.Add(new ClothSpring(index, index + vertexesPerLine + 1, distanceBetweenVertexes * Mathf.Sqrt(2f)));
                
                if (row < numberOfLines - 1 && col > 0)
                    springs.Add(new ClothSpring(index, index + vertexesPerLine - 1, distanceBetweenVertexes * Mathf.Sqrt(2f)));
                
                if (col < vertexesPerLine - 2)
                    springs.Add(new ClothSpring(index, index + 2, distanceBetweenVertexes * 2f));
                
                if (row < numberOfLines - 2)
                    springs.Add(new ClothSpring(index, index + 2 * vertexesPerLine, distanceBetweenVertexes * 2f));
            }
        }
    }

    void InitializeMesh()
    {
        mesh = new Mesh();
        mesh.vertices = positions;
        mesh.triangles = meshTriangles;
        mesh.RecalculateNormals();
        GetComponent<MeshFilter>().mesh = mesh;
    }
    #endregion

    #region Physics Update
    void Update()
    {
        float dt = Time.deltaTime;
        ComputeForces();
        IntegrateParticles(dt);
        UpdateMesh();
    }

    void ComputeForces()
    {
        for (int i = 0; i < forces.Length; i++)
            forces[i] = mass * gravity;

        foreach (var spring in springs)
        {
            Vector3 pos1 = positions[spring.indexA];
            Vector3 pos2 = positions[spring.indexB];
            Vector3 vel1 = velocities[spring.indexA];
            Vector3 vel2 = velocities[spring.indexB];

            Vector3 delta = pos2 - pos1;
            float currentLength = delta.magnitude;
            
            if (currentLength < PhysicsConstants.EPSILON_SMALL) continue;

            Vector3 direction = delta / currentLength;
            float extension = currentLength - spring.restLength;

            Vector3 springForce = springStiffness * extension * direction;
            Vector3 dampingForce = springDamping * Vector3.Dot(vel2 - vel1, direction) * direction;
            Vector3 totalForce = springForce + dampingForce;

            forces[spring.indexA] += totalForce;
            forces[spring.indexB] -= totalForce;
        }
    }

    void IntegrateParticles(float dt)
    {
        for (int i = 0; i < positions.Length; i++)
        {
            if (isPinned[i])
            {
                positions[i] = basePositions[i];
                velocities[i] = Vector3.zero;
                continue;
            }

            Vector3 acceleration = IntegrationUtils.ForceToAcceleration(forces[i], mass);
            velocities[i] = IntegrationUtils.IntegrateVelocityEuler(velocities[i], acceleration, dt);
            positions[i] = IntegrationUtils.IntegratePositionEuler(positions[i], velocities[i], dt);
        }
    }

    void UpdateMesh()
    {
        mesh.vertices = positions;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }
    #endregion
}

[System.Serializable]
public class ClothSpring
{
    public int indexA;
    public int indexB;
    public float restLength;

    public ClothSpring(int a, int b, float rest)
    {
        indexA = a;
        indexB = b;
        restLength = rest;
    }
}