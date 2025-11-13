using UnityEngine;
using System.Collections.Generic;
using PhysicsUnity.Core; // Added to use MeshUtils and CollisionUtils
// using PhysicsUnity.Indiv_Work.Aziz; // To access RigidBody3D and CollisionDetector
/// <summary>
/// Super-stable soft-body "jello" cube using position-based spring constraints.
/// Includes visible springs via Gizmos and full collision with floor and OBBs.
/// Refactored to use MeshUtils for mesh creation and CollisionUtils for ground collision.
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
    public float pointRadius = 0.15f; // Collision sphere radius per mass point (increased for better detection)

    [Header("Movement Settings")]
    public float moveForce = 1f;
    public float jumpForce = 80f;
    [Tooltip("External input direction to move the jello (x,z). Set from your own input system.")]
    public Vector3 inputDirection = Vector3.zero; // x,z components used

    [Header("Visuals")]
    public Material jelloMaterial;
    public bool drawDebug = true;
    public bool drawCollisionSpheres = true; // Added: visualize collision spheres

    private MassPoint[,,] points;
    private List<Spring> springs;
    private Mesh mesh;
    private Vector3[] baseVertices;
    private Vector3[] deformedVertices;
    private bool grounded = false;
    // Removed _obbsBuffer / _obbProviders – using SimpleOBB static registry

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
            restLength = MathUtils.Distance(p1.position, p2.position);
            stiffness = k;
        }

        public void Apply()
        {
            Vector3 delta = b.position - a.position;
            float dist = MathUtils.Magnitude(delta);
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
        // Ensure any SimpleOBB in scene recompute matrices once
        var all = SimpleOBB.All;
        for (int i = 0; i < all.Count; i++)
        {
            if (all[i] != null) all[i].RecomputeMatrices();
        }
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
                                float dist = MathUtils.Distance(p.position, np.position);
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
        mesh.MarkDynamic(); // dynamic deforming mesh
        GetComponent<MeshFilter>().mesh = mesh;
        if (jelloMaterial)
            GetComponent<MeshRenderer>().material = jelloMaterial;

        // Use MeshUtils instead of local cube builder. Size 2 because original cube vertices were from -1 to +1.
        Mesh cube = MeshUtils.CreateCubeMesh(new Vector3(2f, 2f, 2f));
        mesh.vertices = cube.vertices;
        mesh.triangles = cube.triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds(); // ensure initial bounds are valid

        baseVertices = mesh.vertices;
        deformedVertices = new Vector3[baseVertices.Length];
    }

    // Removed local BuildCubeMesh (now using MeshUtils)

    void FixedUpdate()
    {
        float dt = Mathf.Min(Time.fixedDeltaTime, 0.02f);

        // gravity
        foreach (var p in points)
            p.AddForce(Vector3.up * gravity * p.mass);

        // movement input (no Unity Input APIs)
        Vector3 moveDir = new Vector3(inputDirection.x, 0f, inputDirection.z);
        if (MathUtils.SqrMagnitude(moveDir) > PhysicsConstants.EPSILON_SMALL * PhysicsConstants.EPSILON_SMALL)
        {
            // Safe normalize using MathUtils to avoid Unity .normalized
            Vector3 dir = MathUtils.SafeNormalize(moveDir);
            Vector3 moveForceVec = dir * moveForce;
            foreach (var p in points)
                p.AddForce(moveForceVec);
        }

        // jump: gated by external toggle (no Unity key query)
        if (grounded && inputDirection.y > 0)
        {
            foreach (var p in points)
                p.AddForce(Vector3.up * jumpForce);
        }

        // integrate
        foreach (var p in points)
            p.Integrate(dt, damping);

        // FIRST: collisions (pure math) BEFORE springs
        grounded = false;
        var colliders = SimpleOBB.All;
        if (colliders != null && colliders.Count > 0)
        {
            for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
            for (int z = 0; z < gridSize; z++)
            {
                MassPoint mp = points[x, y, z];
                for (int ci = 0; ci < colliders.Count; ci++)
                {
                    var obb = colliders[ci];
                    if (obb == null) continue;

                    // Sphere vs OBB: closest point on OBB to sphere center
                    Vector3 local = obb.WorldToLocalPoint(mp.position);
                    Vector3 clamped = new Vector3(
                        Mathf.Clamp(local.x, -obb.halfExtents.x, obb.halfExtents.x),
                        Mathf.Clamp(local.y, -obb.halfExtents.y, obb.halfExtents.y),
                        Mathf.Clamp(local.z, -obb.halfExtents.z, obb.halfExtents.z)
                    );
                    Vector3 closestWorld = obb.LocalToWorldPoint(clamped);

                    Vector3 d = mp.position - closestWorld;
                    float distSq = MathUtils.SqrMagnitude(d);
                    float rSq = pointRadius * pointRadius;

                    if (distSq <= rSq)
                    {
                        float dist = Mathf.Sqrt(Mathf.Max(distSq, PhysicsConstants.EPSILON));
                        Vector3 n = dist > 1e-5f ? d / dist : Vector3.up; // fallback normal if nearly coincident
                        float pen = pointRadius - dist + 1e-4f; // small slop to prevent re-penetration

                        mp.position += n * pen;

                        Vector3 vel = (mp.position - mp.previousPosition) / dt;
                        float vn = MathUtils.Dot(vel, n);
                        if (vn < 0f)
                        {
                            vel -= vn * n * (1f + restitution);
                            Vector3 vt = vel - MathUtils.Dot(vel, n) * n;
                            vel = vt * friction + MathUtils.Dot(vel, n) * n;
                        }
                        mp.previousPosition = mp.position - vel * dt;
                        grounded = grounded || n.y > 0.3f; // only consider as grounded if normal points upward
                        break; // first collider per point
                    }
                }
            }
        }

        // SECOND: springs
        for (int i = 0; i < solverIterations; i++)
            foreach (var s in springs)
                s.Apply();

        // Consume jump input (single press behavior)
        inputDirection.y = 0f;
    }

    // Removed old CheckSphereOBBCollision – using SimpleOBB math directly

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
                MathUtils.Lerp(
                    MathUtils.Lerp(
                        MathUtils.Lerp(corners[0], corners[1], fx),
                        MathUtils.Lerp(corners[3], corners[2], fx), fz),
                    MathUtils.Lerp(
                        MathUtils.Lerp(corners[4], corners[5], fx),
                        MathUtils.Lerp(corners[7], corners[6], fx), fz),
                    fy);

            deformedVertices[i] = interp - transform.position;
        }

        mesh.vertices = deformedVertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds(); // keep bounds current to avoid frustum-culling when camera is close
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

        // Draw collision spheres for debugging
        if (drawCollisionSpheres)
        {
            Gizmos.color = Color.red;
            foreach (var p in points)
                Gizmos.DrawWireSphere(p.position, pointRadius);
        }
    }
}
