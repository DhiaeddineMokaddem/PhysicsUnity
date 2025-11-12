using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A self-contained particle–spring chain simulation.  
/// Each link is represented by a particle connected to its neighbour
/// with a spring that can stretch and optionally break.
/// The simulation runs entirely in code (no Rigidbody, no Joints).
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ChainSimulation : MonoBehaviour
{
    [Header("Chain Parameters")]
    [Tooltip("Number of links in the chain.")]
    public int chainLength = 10;
    [Tooltip("Distance between each link.")]
    public float restLength = 0.3f;
    [Tooltip("Whether the first link is pinned in place.")]
    public bool pinFirst = true;

    [Header("Physics Parameters")]
    [Tooltip("Hooke's spring stiffness.")]
    public float springStiffness = 200f;
    [Tooltip("Damping factor to stabilise motion.")]
    public float damping = 0.98f;
    [Tooltip("Mass of each particle.")]
    public float mass = 0.5f;
    [Tooltip("Gravity applied to each link.")]
    public Vector3 gravity = new Vector3(0, -9.81f, 0);

    [Header("Break Settings")]
    [Tooltip("Factor of rest length at which the link breaks.")]
    public float breakStretchFactor = 1.5f;
    [Tooltip("Whether to visually break links or just simulate stretching.")]
    public bool removeBrokenLinks = true;

    private Vector3[] positions;
    private Vector3[] velocities;
    private Vector3[] forces;
    private bool[] isPinned;
    private List<Spring> springs;
    private Mesh mesh;
    private int[] meshTriangles;

    void Awake()
    {
        int count = chainLength;
        positions = new Vector3[count];
        velocities = new Vector3[count];
        forces = new Vector3[count];
        isPinned = new bool[count];
        springs = new List<Spring>();

        // Create a vertical chain layout
        for (int i = 0; i < count; i++)
        {
            positions[i] = new Vector3(0, -i * restLength, 0);
            velocities[i] = Vector3.zero;
            forces[i] = Vector3.zero;
            isPinned[i] = (i == 0 && pinFirst);
        }

        // Build springs between consecutive links
        for (int i = 0; i < count - 1; i++)
        {
            springs.Add(new Spring(i, i + 1, restLength));
        }

        // Simple mesh representation (a line of quads)
        mesh = new Mesh();
        mesh.name = "Chain Mesh";
        GetComponent<MeshFilter>().mesh = mesh;
        BuildMesh();
    }

    void BuildMesh()
    {
        // Represent each link as a small rectangular segment between particles
        List<Vector3> verts = new List<Vector3>();
        List<int> tris = new List<int>();
        float width = 0.05f;

        for (int i = 0; i < chainLength - 1; i++)
        {
            Vector3 p1 = positions[i];
            Vector3 p2 = positions[i + 1];
            Vector3 dir = (p2 - p1).normalized;
            Vector3 side = Vector3.Cross(dir, Vector3.forward).normalized * width;

            int vertStart = verts.Count;
            verts.Add(p1 + side);
            verts.Add(p1 - side);
            verts.Add(p2 + side);
            verts.Add(p2 - side);

            tris.Add(vertStart + 0);
            tris.Add(vertStart + 2);
            tris.Add(vertStart + 1);
            tris.Add(vertStart + 2);
            tris.Add(vertStart + 3);
            tris.Add(vertStart + 1);
        }

        mesh.Clear();
        mesh.SetVertices(verts);
        mesh.SetTriangles(tris, 0);
        mesh.RecalculateNormals();
        meshTriangles = tris.ToArray();
    }

    void Update()
    {
        Simulate(Time.deltaTime);
        BuildMesh();
    }

    void Simulate(float dt)
    {
        int count = positions.Length;

        // Reset forces
        for (int i = 0; i < count; i++)
        {
            forces[i] = Vector3.zero;
        }

        // Apply gravity
        for (int i = 0; i < count; i++)
        {
            if (!isPinned[i])
                forces[i] += gravity * mass;
        }

        // Spring forces
        for (int s = 0; s < springs.Count; s++)
        {
            Spring sp = springs[s];
            int a = sp.indexA;
            int b = sp.indexB;
            Vector3 delta = positions[b] - positions[a];
            float length = delta.magnitude;

            if (length < Mathf.Epsilon)
                continue;

            // Check for breaking
            if (length > sp.restLength * breakStretchFactor)
            {
                if (removeBrokenLinks)
                {
                    springs.RemoveAt(s);
                    s--;
                    continue;
                }
            }

            Vector3 dir = delta / length;
            float displacement = length - sp.restLength;
            Vector3 force = springStiffness * displacement * dir;

            forces[a] += force;
            forces[b] -= force;
        }

        // Integrate motion
        for (int i = 0; i < count; i++)
        {
            if (isPinned[i]) continue;

            Vector3 accel = forces[i] / mass;
            velocities[i] = (velocities[i] + accel * dt) * damping;
            positions[i] += velocities[i] * dt;
        }
    }

    private void OnDrawGizmos()
    {
        if (positions == null) return;

        Gizmos.color = Color.yellow;
        for (int i = 0; i < springs.Count; i++)
        {
            var sp = springs[i];
            Gizmos.DrawLine(positions[sp.indexA] + transform.position, positions[sp.indexB] +transform.position);
        }
    }
}