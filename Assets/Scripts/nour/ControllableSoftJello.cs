using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Soft-body controllable "jello" cube using a spring-mass system.
/// Mesh deformation is handled via MatrixCubeMesh.
/// This version keeps mesh and gizmos aligned and prevents scale explosion.
/// </summary>
[RequireComponent(typeof(MatrixCubeMesh))]
public class ControllableSoftJello : MonoBehaviour
{
    [Header("Soft Body Settings")]
    public int gridSize = 4;
    public float cellSize = 0.3f;
    public float pointMass = 0.12f;
    public float stiffness = 80f;
    public float damping = 1.5f;               // integrator damping factor
    public float gravity = -9.81f;
    public float restitution = 0.4f;
    public float friction = 0.85f;
    [Tooltip("Enable physical spring breaking (may fragment the cube).")]
    public bool enableBreaks = false;
    [Tooltip("Break threshold as relative strain (e.g. 2.0 = 200% extension)")]
    public float breakThreshold = 2.0f;
    public int solverIterations = 3;

    [Header("Movement Settings")]
    public float moveForce = 40f;
    public float jumpForce = 80f;

    [Header("Debug Visualization")]
    public bool drawSprings = true;
    public Color springColor = Color.yellow;
    public Color brokenSpringColor = Color.red;

    // Internal
    private MassPoint[,,] points;
    private List<Spring> springs;
    private MatrixCubeMesh cubeMesh;
    private bool grounded = false;

    // Stored rest size (used for stable mesh scale)
    private Vector3 restScale; // world-space size (x,y,z) = (cellSize*(gridSize-1)) etc.

    // ------------------ MASS POINT ------------------
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
            // semi-Verlet style (Verlet-like)
            Vector3 vel = (position - previousPosition) * (1f - damping * dt);
            Vector3 newPos = position + vel + (force / mass) * dt * dt;
            previousPosition = position;
            position = newPos;
            force = Vector3.zero;
        }
    }

    // ------------------ SPRING ------------------
    public class Spring
    {
        public MassPoint pointA;
        public MassPoint pointB;
        public float restLength;
        public float stiffness;
        public bool isBroken = false;

        public Spring(MassPoint a, MassPoint b, float k)
        {
            pointA = a;
            pointB = b;
            restLength = Vector3.Distance(a.position, b.position);
            stiffness = k;
        }

        public void Apply(bool enableBreaks, float breakThreshold)
        {
            if (isBroken) return;

            Vector3 delta = pointB.position - pointA.position;
            float dist = delta.magnitude;
            if (dist <= 1e-8f) return;

            Vector3 dir = delta / dist;
            float displacement = dist - restLength;
            Vector3 force = dir * (stiffness * displacement);

            pointA.AddForce(force);
            pointB.AddForce(-force);

            if (enableBreaks)
            {
                float strain = Mathf.Abs(displacement / restLength);
                if (strain > breakThreshold)
                    isBroken = true;
            }
        }
    }

    // ------------------ UNITY LIFECYCLE ------------------
    void Start()
    {
        cubeMesh = GetComponent<MatrixCubeMesh>();
        InitSoftBody();
    }

    void InitSoftBody()
    {
        // Precompute rest scale (fixed) so mesh scale won't explode
        restScale = new Vector3(cellSize * (gridSize - 1),
                                cellSize * (gridSize - 1),
                                cellSize * (gridSize - 1));

        points = new MassPoint[gridSize, gridSize, gridSize];
        springs = new List<Spring>();

        // Create points in world-space centered on GameObject position
        Vector3 halfExtent = restScale * 0.5f;
        Vector3 start = transform.position - halfExtent;

        for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
                for (int z = 0; z < gridSize; z++)
                {
                    Vector3 pos = start + new Vector3(x, y, z) * cellSize;
                    points[x, y, z] = new MassPoint(pos, pointMass);
                }

        // Create structural springs (you can extend with shear/bend if desired)
        for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
                for (int z = 0; z < gridSize; z++)
                {
                    MassPoint p = points[x, y, z];
                    if (x < gridSize - 1) springs.Add(new Spring(p, points[x + 1, y, z], stiffness));
                    if (y < gridSize - 1) springs.Add(new Spring(p, points[x, y + 1, z], stiffness));
                    if (z < gridSize - 1) springs.Add(new Spring(p, points[x, y, z + 1], stiffness));
                }
    }


    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // reset forces + gravity
        foreach (var p in points)
            p.AddForce(Vector3.up * gravity * p.mass);

        // movement input (apply force to every point to move the body)
        Vector3 input = new Vector3(Input.GetAxis("Horizontal"), 0f, Input.GetAxis("Vertical"));
        if (input.sqrMagnitude > 0.0001f)
        {
            Vector3 moveVec = input.normalized * moveForce;
            foreach (var p in points) p.AddForce(moveVec);
        }

        // jump: only if grounded
        if (grounded && Input.GetKeyDown(KeyCode.Space))
        {
            foreach (var p in points) p.AddForce(Vector3.up * jumpForce);
        }

        // springs apply forces (multiple solver iterations)
        for (int it = 0; it < solverIterations; it++)
            foreach (var s in springs)
                s.Apply(enableBreaks, breakThreshold);

        // integrate points
        foreach (var p in points) p.Integrate(dt, damping);

        // floor collision at y = 0
        grounded = false;
        for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
                for (int z = 0; z < gridSize; z++)
                {
                    var p = points[x, y, z];
                    if (p.position.y < 0f)
                    {
                        p.position.y = 0f;
                        Vector3 vel = (p.position - p.previousPosition) / dt;
                        if (vel.y < 0f) vel.y *= -restitution;
                        vel.x *= friction;
                        vel.z *= friction;
                        p.previousPosition = p.position - vel * dt;
                        grounded = true;
                    }
                }

        // optional: containment OBB collisions (SimpleOBB if present)
        HandleOBBCollisions(dt);

        // update mesh transform (use restScale for scale so mesh doesn't blow up)
        UpdateMeshTransform();
    }

    void HandleOBBCollisions(float dt)
    {
        // sample OBBs in scene (if you have a SimpleOBB component)
        var obbs = FindObjectsOfType<SimpleOBB>();
        if (obbs == null || obbs.Length == 0) return;

        foreach (var obb in obbs)
        {
            Matrix4x4 worldToLocal = obb.transform.worldToLocalMatrix;
            Matrix4x4 localToWorld = obb.transform.localToWorldMatrix;
            Vector3 half = obb.halfExtents;

            // Test all points (could be optimized by sampling surface points only)
            for (int x = 0; x < gridSize; x++)
                for (int y = 0; y < gridSize; y++)
                    for (int z = 0; z < gridSize; z++)
                    {
                        var p = points[x, y, z];
                        // transform to obb local space
                        Vector3 local = worldToLocal.MultiplyPoint3x4(p.position);

                        // penetration test (point inside OBB)
                        if (Mathf.Abs(local.x) < half.x && Mathf.Abs(local.y) < half.y && Mathf.Abs(local.z) < half.z)
                        {
                            // compute minimum penetration axis in local space
                            float px = half.x - Mathf.Abs(local.x);
                            float py = half.y - Mathf.Abs(local.y);
                            float pz = half.z - Mathf.Abs(local.z);

                            Vector3 penLocal;
                            if (px < py && px < pz)
                                penLocal = new Vector3(local.x < 0 ? -px : px, 0, 0);
                            else if (py < pz)
                                penLocal = new Vector3(0, local.y < 0 ? -py : py, 0);
                            else
                                penLocal = new Vector3(0, 0, local.z < 0 ? -pz : pz);

                            // convert correction back to world
                            Vector3 correction = localToWorld.MultiplyVector(penLocal);

                            // push point out
                            p.position += correction;

                            // reflect velocity along normal
                            Vector3 normal = correction.normalized;
                            Vector3 vel = (p.position - p.previousPosition) / dt;
                            float vn = Vector3.Dot(vel, normal);
                            if (vn < 0f)
                            {
                                vel -= (1f + restitution) * vn * normal;
                                vel *= friction;
                                p.previousPosition = p.position - vel * dt;
                            }
                        }
                    }
        }
    }

    // ------------------ MESH TRANSFORMATION ------------------
    void UpdateMeshTransform()
    {
        // Compute actual center of mass
        Vector3 center = Vector3.zero;
        int count = 0;
        foreach (var p in points)
        {
            if (float.IsNaN(p.position.x)) continue;
            center += p.position;
            count++;
        }
        if (count == 0) return;
        center /= count;

        // Safe direction sampling
        Vector3 rightVec = points[gridSize - 1, 0, 0].position - points[0, 0, 0].position;
        Vector3 upVec = points[0, gridSize - 1, 0].position - points[0, 0, 0].position;
        Vector3 fwdVec = points[0, 0, gridSize - 1].position - points[0, 0, 0].position;

        // Avoid degenerate (zero-length) vectors
        if (rightVec.sqrMagnitude < 1e-8f) rightVec = Vector3.right * cellSize;
        if (upVec.sqrMagnitude < 1e-8f) upVec = Vector3.up * cellSize;
        if (fwdVec.sqrMagnitude < 1e-8f) fwdVec = Vector3.forward * cellSize;

        // Build rotation from forward/up safely
        Quaternion rot = Quaternion.LookRotation(fwdVec.normalized, upVec.normalized);

        // Approximate scale
        Vector3 scale = new Vector3(
            Mathf.Max(0.001f, rightVec.magnitude),
            Mathf.Max(0.001f, upVec.magnitude),
            Mathf.Max(0.001f, fwdVec.magnitude)
        );

        // Build matrix in *local space* (no double transform)
        Matrix4x4 localTransform = Matrix4x4.TRS(Vector3.zero, rot, scale);

        // Update mesh around center in world space
        Matrix4x4 worldTransform = Matrix4x4.TRS(center, rot, scale);
        cubeMesh.UpdateMesh(worldTransform);

        // Also move GameObject so Gizmos and Mesh align visually
        transform.position = center;
        transform.rotation = rot;
    }


    // ------------------ GIZMOS ------------------
    void OnDrawGizmos()
    {
        if (!Application.isPlaying || !drawSprings || springs == null) return;

        foreach (var s in springs)
        {
            Gizmos.color = s.isBroken ? brokenSpringColor : springColor;
            if (!s.isBroken)
                Gizmos.DrawLine(s.pointA.position, s.pointB.position);
            else
            {
                // draw broken springs faded
                Color c = brokenSpringColor;
                c.a = 0.6f;
                Gizmos.color = c;
                Gizmos.DrawLine(s.pointA.position, s.pointB.position);
            }
        }

        // draw corner spheres for debugging to compare with mesh
        Gizmos.color = Color.cyan;
        if (points != null)
        {
            // highlight the 8 corners used to orient the mesh
            if (gridSize > 1)
            {
                Vector3[] corners = new Vector3[]
                {
                    points[0,0,0].position,
                    points[gridSize-1,0,0].position,
                    points[0,gridSize-1,0].position,
                    points[0,0,gridSize-1].position,
                    points[gridSize-1,gridSize-1,gridSize-1].position
                };
                foreach (var c in corners) Gizmos.DrawSphere(c, 0.03f);
            }
        }
    }
}
