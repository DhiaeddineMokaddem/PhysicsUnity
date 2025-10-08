using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A simple mass–spring cloth simulation that builds a rectangular grid of
/// masses connected by springs.  Each mass is connected to its neighbours
/// with structural, shear and bending springs.  The forces on each mass
/// follow the equations presented in the lecture notes (page 29):
///  • Spring force  F⃗ˢ = kₛₚᵣᵢₙ₉ Δ⃗x
///  • Damping force F⃗ᴰ = –k_damping v⃗
/// The total force on a mass is the sum of spring and damping forces from
/// all connected springs plus gravity.  A simple explicit Euler integrator
/// updates the velocity and position of each mass on every frame.  The
/// top row of vertices is pinned in place so the cloth hangs under gravity.
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ClothSimulation : MonoBehaviour
{
    // Grid dimensions
    [Header("Grid size")]
    [Tooltip("Number of rows in the cloth grid.")]
    public int numberOfLines = 5;
    [Tooltip("Number of columns in the cloth grid.")]
    public int vertexesPerLine = 5;
    [Tooltip("Distance between adjacent vertices in metres.")]
    public float distanceBetweenVertexes = 0.2f;

    // Physics parameters
    [Header("Spring parameters")]
    [Tooltip("Hooke's law stiffness constant for all springs.")]
    public float springStiffness = 50f;
    [Tooltip("Damping constant applied along each spring.")]
    public float springDamping = 1f;
    [Tooltip("Mass of each particle (kg). All particles share the same mass.")]
    public float mass = 1f;
    [Tooltip("Gravity applied to each particle.")]
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    [Tooltip("Whether to pin two corners instead of the whole top row.")]
    public bool TwoCorner = false;

    // Internal data structures
    private Vector3[] basePositions;   // initial positions for each particle (used for pinning)
    private Vector3[] positions;       // current positions of particles
    private Vector3[] velocities;      // current velocities of particles
    private Vector3[] forces;          // accumulated forces per particle
    private bool[] isPinned;           // whether a given particle is fixed in space
    private Mesh mesh;
    private int[] meshTriangles;

    /// <summary>
    /// Represents a connection between two particles.  The rest length is the
    /// distance at which the spring is neither compressed nor stretched.
    /// </summary>
    private struct Spring
    {
        public int indexA;
        public int indexB;
        public float restLength;

        public Spring(int a, int b, float rest)
        {
            indexA = a;
            indexB = b;
            restLength = rest;
        }
    }

    private List<Spring> springs;

    void Awake()
    {
        int count = numberOfLines * vertexesPerLine;
        basePositions = new Vector3[count];
        positions = new Vector3[count];
        velocities = new Vector3[count];
        forces = new Vector3[count];
        isPinned = new bool[count];
        springs = new List<Spring>();

        // Generate the grid of particles.
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
                // Pin the top row so that it acts as an anchor.
                if(TwoCorner)
                {
                isPinned[index] = (row == 0 || row == numberOfLines - 1) ;
                }
                else
                {
                    isPinned[index] = (row == 0);
                }
            }
        }

        // Build the mesh triangles (two triangles per quad).
        List<int> triList = new List<int>();
        for (int row = 0; row < numberOfLines - 1; row++)
        {
            for (int col = 0; col < vertexesPerLine; col++)
            {
                if (col != vertexesPerLine - 1)
                {
                    // first triangle: (row, col) -> (row, col+1) -> (row+1, col)
                    triList.Add(col + row * vertexesPerLine);
                    triList.Add(col + row * vertexesPerLine + 1);
                    triList.Add(col + (row + 1) * vertexesPerLine);
                }
                if (col != 0)
                {
                    // second triangle: (row, col) -> (row+1, col) -> (row+1, col-1)
                    triList.Add(col + row * vertexesPerLine);
                    triList.Add(col + (row + 1) * vertexesPerLine);
                    triList.Add(col + (row + 1) * vertexesPerLine - 1);
                }
            }
        }
        meshTriangles = triList.ToArray();

        // Build springs connecting neighbouring particles.  We include
        // structural (adjacent), shear (diagonal) and bending (two away)
        // springs.  Each spring stores its rest length so that the cloth
        // naturally tries to return to its original shape.
        for (int row = 0; row < numberOfLines; row++)
        {
            for (int col = 0; col < vertexesPerLine; col++)
            {
                int index = row * vertexesPerLine + col;
                // Right neighbour (structural)
                if (col < vertexesPerLine - 1)
                {
                    int right = index + 1;
                    float rest = distanceBetweenVertexes;
                    springs.Add(new Spring(index, right, rest));
                }
                // Down neighbour (structural)
                if (row < numberOfLines - 1)
                {
                    int down = index + vertexesPerLine;
                    float rest = distanceBetweenVertexes;
                    springs.Add(new Spring(index, down, rest));
                }
                // Shear springs (diagonals)
                if (row < numberOfLines - 1 && col < vertexesPerLine - 1)
                {
                    int downRight = index + vertexesPerLine + 1;
                    float rest = distanceBetweenVertexes * Mathf.Sqrt(2f);
                    springs.Add(new Spring(index, downRight, rest));
                }
                if (row < numberOfLines - 1 && col > 0)
                {
                    int downLeft = index + vertexesPerLine - 1;
                    float rest = distanceBetweenVertexes * Mathf.Sqrt(2f);
                    springs.Add(new Spring(index, downLeft, rest));
                }
                // Bending springs (two units apart)
                if (col < vertexesPerLine - 2)
                {
                    int twoRight = index + 2;
                    float rest = distanceBetweenVertexes * 2f;
                    springs.Add(new Spring(index, twoRight, rest));
                }
                if (row < numberOfLines - 2)
                {
                    int twoDown = index + 2 * vertexesPerLine;
                    float rest = distanceBetweenVertexes * 2f;
                    springs.Add(new Spring(index, twoDown, rest));
                }
            }
        }

        // Create the mesh and assign it to the MeshFilter.
        mesh = new Mesh();
        mesh.name = "Cloth Mesh";
        GetComponent<MeshFilter>().mesh = mesh;
        // Assign initial geometry
        mesh.vertices = positions;
        mesh.triangles = meshTriangles;
        mesh.RecalculateNormals();
    }

    void Update()
    {
        // Time step for this frame
        float dt = Time.deltaTime;
        int particleCount = positions.Length;
        // Reset forces
        for (int i = 0; i < particleCount; i++)
        {
            forces[i] = Vector3.zero;
        }
        // Compute spring forces and accumulate them
        for (int s = 0; s < springs.Count; s++)
        {
            Spring sp = springs[s];
            int a = sp.indexA;
            int b = sp.indexB;
            Vector3 pA = positions[a];
            Vector3 pB = positions[b];
            Vector3 delta = pB - pA;
            float currentLength = delta.magnitude;
            if (currentLength > Mathf.Epsilon)
            {
                Vector3 dir = delta / currentLength;
                float displacement = currentLength - sp.restLength;
                // Hooke's spring force: F = k * displacement * direction
                Vector3 springForce = springStiffness * displacement * dir;
                // Relative velocity along the spring
                Vector3 relVel = velocities[b] - velocities[a];
                float velAlongSpring = Vector3.Dot(relVel, dir);
                // Damping force: F = k_damping * v_parallel * direction
                Vector3 dampingForce = springDamping * velAlongSpring * dir;
                // Total force applied to A and opposite on B
                Vector3 total = springForce + dampingForce;
                forces[a] += total;
                forces[b] -= total;
            }
        }
        // Add gravity
        for (int i = 0; i < particleCount; i++)
        {
            forces[i] += gravity * mass;
        }
        // Integrate acceleration -> velocity -> position using explicit Euler
        for (int i = 0; i < particleCount; i++)
        {
            if (isPinned[i])
            {
                // Keep pinned particles fixed at their base positions
                velocities[i] = Vector3.zero;
                positions[i] = basePositions[i];
                continue;
            }
            // Acceleration = F/m
            Vector3 accel = forces[i] / mass;
            // Update velocity and apply some global damping to stabilise
            velocities[i] += accel * dt;
            // Apply position update
            positions[i] += velocities[i] * dt;
        }
        // Update the mesh with new vertex positions
        mesh.vertices = positions;
        mesh.triangles = meshTriangles;
        mesh.RecalculateNormals();
    }
}