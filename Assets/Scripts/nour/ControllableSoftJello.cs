using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Super-stable soft-body "jello" cube using position-based spring constraints.
/// Includes visible springs via Gizmos and full collision with floor and OBBs.
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ControllableSoftJello : MonoBehaviour
{
    [Header("Soft Body Settings")]
    public int gridSize = 4;
    public float cellSize = 0.3f;
    public float pointMass = 0.1f;
    public float stiffness = 25f;         // lower stiffness for stability
    public float damping = 2.5f;
    public float gravity = -9.81f;
    public float restitution = 0.3f;
    public float friction = 0.9f;
    public int solverIterations = 8;

    [Header("Movement Settings")]
    public float moveForce = 30f;
    public float jumpForce = 80f;

    [Header("Visuals")]
    public Material jelloMaterial;
    public bool drawDebug = true;

    private MassPoint[,,] points;
    private List<Spring> springs;
    private Mesh mesh;
    private Vector3[] baseVertices;
    private Vector3[] deformedVertices;
    private bool grounded = false;

    // ---------------- MASS POINT ----------------
    public class MassPoint
    {
        public Vector3 position;
        public Vector3 previousPosition;
        public Vector3 force;
        public float mass;

        public MassPoint(Vector3 pos, float m)
        {
            position = pos;
            previousPosition = pos;
            force = Vector3.zero;
            mass = m;
        }

        public void AddForce(Vector3 f) => force += f;

        public void Integrate(float dt, float damping)
        {
            Vector3 velocity = position - previousPosition;
            Vector3 newPos = position + velocity * (1f - damping * dt) + (force / mass) * dt * dt;
            previousPosition = position;
            position = newPos;
            force = Vector3.zero;
        }
    }

    // ---------------- SPRING (position constraint) ----------------
    public class Spring
    {
        public MassPoint a, b;
        public float restLength;
        public float stiffness;

        public Spring(MassPoint p1, MassPoint p2, float k)
        {
            a = p1;
            b = p2;
            restLength = Vector3.Distance(p1.position, p2.position);
            stiffness = k;
        }

        public void Apply()
        {
            Vector3 delta = b.position - a.position;
            float dist = delta.magnitude;
            if (dist <= 1e-6f) return;

            float diff = (dist - restLength) / dist;
            // Stable position-based correction
            Vector3 correction = delta * 0.5f * stiffness * 0.01f * diff;

            a.position += correction;
            b.position -= correction;
        }
    }

    // ---------------- UNITY LIFECYCLE ----------------
    void Start()
    {
        InitSoftBody();
        CreateMesh();
    }

    void InitSoftBody()
    {
        points = new MassPoint[gridSize, gridSize, gridSize];
        springs = new List<Spring>();

        Vector3 start = transform.position - Vector3.one * (gridSize - 1) * cellSize * 0.5f;

        // Create points
        for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
                for (int z = 0; z < gridSize; z++)
                    points[x, y, z] = new MassPoint(start + new Vector3(x, y, z) * cellSize, pointMass);

        // Create springs between close neighbors only (structural + shear)
        for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
                for (int z = 0; z < gridSize; z++)
                {
                    var p = points[x, y, z];
                    for (int i = -1; i <= 1; i++)
                        for (int j = -1; j <= 1; j++)
                            for (int k = -1; k <= 1; k++)
                            {
                                if (i == 0 && j == 0 && k == 0) continue;
                                int nx = x + i, ny = y + j, nz = z + k;
                                if (nx < 0 || ny < 0 || nz < 0 || nx >= gridSize || ny >= gridSize || nz >= gridSize)
                                    continue;

                                var np = points[nx, ny, nz];
                                float dist = Vector3.Distance(p.position, np.position);
                                if (dist <= cellSize * 1.5f && !SpringExists(p, np))
                                    springs.Add(new Spring(p, np, stiffness));
                            }
                }
    }

    bool SpringExists(MassPoint a, MassPoint b)
    {
        foreach (var s in springs)
            if ((s.a == a && s.b == b) || (s.a == b && s.b == a))
                return true;
        return false;
    }

    void CreateMesh()
    {
        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;
        if (jelloMaterial)
            GetComponent<MeshRenderer>().material = jelloMaterial;

        Mesh cube = BuildCubeMesh();
        mesh.vertices = cube.vertices;
        mesh.triangles = cube.triangles;
        mesh.RecalculateNormals();

        baseVertices = mesh.vertices;
        deformedVertices = new Vector3[baseVertices.Length];
    }

    Mesh BuildCubeMesh()
    {
        Mesh m = new Mesh();
        m.vertices = new Vector3[]
        {
            new Vector3(-1,-1,-1), new Vector3(1,-1,-1), new Vector3(1,-1,1), new Vector3(-1,-1,1),
            new Vector3(-1,1,-1),  new Vector3(1,1,-1),  new Vector3(1,1,1),  new Vector3(-1,1,1)
        };
        m.triangles = new int[]
        {
            0,2,1, 0,3,2,
            4,5,6, 4,6,7,
            0,1,5, 0,5,4,
            1,2,6, 1,6,5,
            2,3,7, 2,7,6,
            3,0,4, 3,4,7
        };
        return m;
    }

    void FixedUpdate()
    {
        float dt = Mathf.Min(Time.fixedDeltaTime, 0.02f);

        // gravity
        foreach (var p in points)
            p.AddForce(Vector3.up * gravity * p.mass);

        // movement input
        Vector3 input = new Vector3(Input.GetAxis("Horizontal"), 0, Input.GetAxis("Vertical"));
        if (input.sqrMagnitude > 0)
        {
            Vector3 moveForceVec = input.normalized * moveForce;
            foreach (var p in points)
                p.AddForce(moveForceVec);
        }

        // jump
        if (grounded && Input.GetKeyDown(KeyCode.Space))
        {
            foreach (var p in points)
                p.AddForce(Vector3.up * jumpForce);
        }

        // integrate
        foreach (var p in points)
            p.Integrate(dt, damping);

        // apply springs (solver)
        for (int i = 0; i < solverIterations; i++)
            foreach (var s in springs)
                s.Apply();

        // floor collision
        grounded = false;
        foreach (var p in points)
        {
            if (p.position.y < 0)
            {
                p.position.y = 0;
                Vector3 vel = (p.position - p.previousPosition) / dt;
                if (vel.y < 0) vel.y *= -restitution;
                vel.x *= friction;
                vel.z *= friction;
                p.previousPosition = p.position - vel * dt;
                grounded = true;
            }
        }

        HandleOBBCollisions(dt);
    }

    void HandleOBBCollisions(float dt)
    {
        foreach (var obb in FindObjectsOfType<SimpleOBB>())
        {
            foreach (var p in points)
            {
                Vector3 localPos = obb.WorldToLocalPoint(p.position);
                Vector3 penetration = obb.GetPenetration(localPos);
                if (penetration != Vector3.zero)
                {
                    Vector3 correction = obb.LocalToWorldVector(penetration);
                    p.position += correction;

                    Vector3 vel = (p.position - p.previousPosition) / dt;
                    Vector3 normal = correction.normalized;
                    vel -= (1f + restitution) * Vector3.Dot(vel, normal) * normal;
                    vel *= friction;
                    p.previousPosition = p.position - vel * dt;
                }
            }
        }
    }

    void LateUpdate()
    {
        // deform mesh based on outer corner points
        Vector3[] corners =
        {
            points[0,0,0].position, points[gridSize-1,0,0].position,
            points[gridSize-1,0,gridSize-1].position, points[0,0,gridSize-1].position,
            points[0,gridSize-1,0].position, points[gridSize-1,gridSize-1,0].position,
            points[gridSize-1,gridSize-1,gridSize-1].position, points[0,gridSize-1,gridSize-1].position
        };

        for (int i = 0; i < baseVertices.Length; i++)
        {
            Vector3 local = (baseVertices[i] + Vector3.one) * 0.5f;
            float fx = local.x, fy = local.y, fz = local.z;

            Vector3 interp =
                Vector3.Lerp(
                    Vector3.Lerp(
                        Vector3.Lerp(corners[0], corners[1], fx),
                        Vector3.Lerp(corners[3], corners[2], fx), fz),
                    Vector3.Lerp(
                        Vector3.Lerp(corners[4], corners[5], fx),
                        Vector3.Lerp(corners[7], corners[6], fx), fz),
                    fy);

            deformedVertices[i] = interp - transform.position;
        }

        mesh.vertices = deformedVertices;
        mesh.RecalculateNormals();
    }

    void OnDrawGizmos()
    {
        if (!Application.isPlaying || !drawDebug || springs == null) return;

        Gizmos.color = Color.yellow;
        foreach (var s in springs)
            Gizmos.DrawLine(s.a.position, s.b.position);

        Gizmos.color = Color.cyan;
        foreach (var p in points)
            Gizmos.DrawSphere(p.position, 0.02f);
    }
}
