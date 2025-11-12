using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Energized fracture simulation following Baraff-style rigid-body math.
/// - Each small cube is a real rigid body with v and w, body inertia, and orientation.
/// - Springs connect adjacent cubes; spring forces are applied at attachment points (face centers),
///   producing linear forces and torques (r x F).
/// - Sphere collision is impulse-based updating both v and w (Baraff effective-mass formula).
/// - On fracture, stored spring energy is converted to impulses applied at constraint midpoints
///   (affecting v and w).
/// 
/// Tuning parameters are at the top. Read the inline comments for where to tweak values.
/// </summary>
public class EnergizedPlateFractureBaraff : MonoBehaviour
{
    [Header("Plate layout")]
    public int cols = 5, rows = 4;              // 5x4 = 20 cubes
    public float cubeSize = 0.5f;
    public Vector3 plateCenterOffset = new Vector3(0f, 3f, 0f);

    [Header("Rigid body")]
    public float cubeMass = 1.0f;
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public float linearDamping = 0.02f;         // small damping to stabilize
    public float angularDamping = 0.02f;

    [Header("Springs / fracture")]
    public float springK = 400f;                // spring stiffness
    public float springDamping = 5f;            // damping along spring (reduces oscillation)
    public float fractureAlpha = 0.6f;          // fraction of stored energy to convert to kinetic
    public bool breakAllOnFirstHit = true;      // break all constraints at first collision (simpler)

    [Header("Sphere collider")]
    public Vector3 sphereCenter = new Vector3(0f, -1f, 0f);
    public float sphereRadius = 0.5f;
    public float restitution = 0.35f;           // bounce factor

    [Header("Clamping / safety")]
    public float maxImpulse = 40f;              // clamp contact impulses
    public float maxFractureImpulse = 30f;      // clamp fracture impulses per cube
    public float minDenom = 1e-6f;

    // Internal types
    class RigidCube
    {
        public GameObject go;
        public Vector3 p;            // world position (COM)
        public Quaternion q;        // orientation
        public Vector3 v;           // linear velocity
        public Vector3 w;           // angular velocity (world)
        public float mass;
        public Matrix4x4 Ibody;     // 3x3 body inertia (4x4 storage)
        public Matrix4x4 IbodyInv;  // inverse of Ibody
    }

    class SpringConstraint
    {
        public int a, b;            // indices of cubes
        public Vector3 attachA_local; // attachment point in local coords of A (relative to its COM)
        public Vector3 attachB_local; // attachment point in local coords of B
        public float restLength;
        public bool broken = false;
    }

    List<RigidCube> cubes;
    List<SpringConstraint> springs;
    bool fractured = false;
    Vector3 plateWorldCenter;

    // ---------- Utilities ----------
    static Matrix4x4 MakeInertiaMatrix(float Ixx, float Iyy, float Izz)
    {
        Matrix4x4 M = Matrix4x4.zero;
        M.m00 = Ixx; M.m11 = Iyy; M.m22 = Izz; M.m33 = 1;
        return M;
    }

    static Matrix4x4 Invert3x3(Matrix4x4 A)
    {
        float a = A.m00, b = A.m01, c = A.m02;
        float d = A.m10, e = A.m11, f = A.m12;
        float g = A.m20, h = A.m21, i = A.m22;
        float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
        if (Mathf.Abs(det) < 1e-8f) det = Mathf.Sign(det) * 1e-8f;
        float inv = 1.0f / det;
        Matrix4x4 R = Matrix4x4.zero;
        R.m00 = (e*i - f*h) * inv;
        R.m01 = (c*h - b*i) * inv;
        R.m02 = (b*f - c*e) * inv;
        R.m10 = (f*g - d*i) * inv;
        R.m11 = (a*i - c*g) * inv;
        R.m12 = (c*d - a*f) * inv;
        R.m20 = (d*h - e*g) * inv;
        R.m21 = (b*g - a*h) * inv;
        R.m22 = (a*e - b*d) * inv;
        R.m33 = 1;
        return R;
    }

    static Matrix4x4 Multiply3x3(Matrix4x4 A, Matrix4x4 B)
    {
        Matrix4x4 C = Matrix4x4.zero;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                C[r, c] = A[r, 0] * B[0, c] + A[r, 1] * B[1, c] + A[r, 2] * B[2, c];
        C.m33 = 1;
        return C;
    }

    static Vector3 Mul3x3(Matrix4x4 M, Vector3 v)
    {
        return new Vector3(
            M.m00 * v.x + M.m01 * v.y + M.m02 * v.z,
            M.m10 * v.x + M.m11 * v.y + M.m12 * v.z,
            M.m20 * v.x + M.m21 * v.y + M.m22 * v.z
        );
    }

    static Matrix4x4 RotationMatrixQuaternion(Quaternion q)
    {
        return Matrix4x4.Rotate(q);
    }

    static Matrix4x4 WorldInverseInertia(Quaternion q, Matrix4x4 IbodyInv)
    {
        Matrix4x4 R = RotationMatrixQuaternion(q);
        Matrix4x4 Rt = R.transpose;
        Matrix4x4 tmp = Multiply3x3(R, IbodyInv);
        Matrix4x4 outM = Multiply3x3(tmp, Rt);
        outM.m33 = 1;
        return outM;
    }

    // Exponential map quaternion integration: stable rotation integration from angular velocity w
    static Quaternion IntegrateRotation(Quaternion q, Vector3 w, float dt)
    {
        float theta = w.magnitude * dt;
        if (theta < 1e-8f)
        {
            // small-angle: q' ~ q + 0.5 * dt * (0,w) * q
            Vector3 half = 0.5f * w * dt;
            Quaternion dq = new Quaternion(half.x, half.y, half.z, 0f);
            Quaternion qNew = new Quaternion(
                q.x + dq.x * q.w + dq.w * q.x + dq.y * q.z - dq.z * q.y,
                q.y + dq.y * q.w + dq.w * q.y + dq.z * q.x - dq.x * q.z,
                q.z + dq.z * q.w + dq.w * q.z + dq.x * q.y - dq.y * q.x,
                q.w + dq.w * q.w - dq.x * q.x - dq.y * q.y - dq.z * q.z
            );
            return qNew.normalized;
        }
        else
        {
            Vector3 axis = w.normalized;
            float half = 0.5f * theta;
            float s = Mathf.Sin(half);
            Quaternion dq = new Quaternion(axis.x * s, axis.y * s, axis.z * s, Mathf.Cos(half));
            Quaternion qNew = dq * q;
            return qNew.normalized;
        }
    }

    // ---------- Initialization ----------
    void Start()
    {
        BuildPlate();
        CreateSphereVisual();
    }

    void BuildPlate()
    {
        cubes = new List<RigidCube>();
        springs = new List<SpringConstraint>();

        Vector3 center = transform.position + plateCenterOffset;
        float width = cols * cubeSize;
        float depth = rows * cubeSize;
        Vector3 basee = center - new Vector3(width, 0f, depth) * 0.5f;

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                Vector3 pos = basee + new Vector3(c * cubeSize + cubeSize * 0.5f, 0f, r * cubeSize + cubeSize * 0.5f);
                GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.transform.localScale = Vector3.one * cubeSize * 0.98f;
                go.transform.position = pos;
                go.transform.rotation = Quaternion.identity;
                go.transform.parent = transform;
                Destroy(go.GetComponent<Collider>());

                RigidCube rc = new RigidCube();
                rc.go = go;
                rc.p = pos;
                rc.q = Quaternion.identity;
                rc.v = Vector3.zero;
                rc.w = Vector3.zero;
                rc.mass = cubeMass;

                // box inertia about center: I = (1/12) m * (h^2 + d^2), for each axis
                float s = cubeSize;
                float I = (1f / 12f) * rc.mass * (s * s + s * s);
                rc.Ibody = MakeInertiaMatrix(I, I, I);
                rc.IbodyInv = Invert3x3(rc.Ibody);
                cubes.Add(rc);
            }
        }

        // plate center
        Vector3 sum = Vector3.zero;
        foreach (var rc in cubes) sum += rc.p;
        plateWorldCenter = sum / cubes.Count;

        // build structural springs between neighbors; attachment point is face center in local coords
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                int idx = r * cols + c;
                // right neighbor
                if (c < cols - 1)
                {
                    SpringConstraint sC = new SpringConstraint();
                    sC.a = idx;
                    sC.b = idx + 1;
                    // attachment local points: on +x face of A and -x face of B
                    sC.attachA_local = new Vector3(cubeSize * 0.5f, 0f, 0f);
                    sC.attachB_local = new Vector3(-cubeSize * 0.5f, 0f, 0f);
                    sC.restLength = cubeSize;
                    springs.Add(sC);
                }
                // forward neighbor (z)
                if (r < rows - 1)
                {
                    SpringConstraint sC = new SpringConstraint();
                    sC.a = idx;
                    sC.b = idx + cols;
                    sC.attachA_local = new Vector3(0f, 0f, cubeSize * 0.5f);
                    sC.attachB_local = new Vector3(0f, 0f, -cubeSize * 0.5f);
                    sC.restLength = cubeSize;
                    springs.Add(sC);
                }
            }
        }
    }

    void CreateSphereVisual()
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.position = sphereCenter;
        sphere.transform.localScale = Vector3.one * (sphereRadius * 2f);
        sphere.GetComponent<Renderer>().material.color = Color.yellow;
        Destroy(sphere.GetComponent<Collider>());
    }

    // ---------- Main physics loop ----------
    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        int N = cubes.Count;

        // accumulators for linear force and torque for each cube
        Vector3[] F_acc = new Vector3[N];
        Vector3[] Tau_acc = new Vector3[N];

        // 1) spring forces applied at attachment points (converted to linear force + torque r x F)
        if (!fractured)
        {
            foreach (var sc in springs)
            {
                if (sc.broken) continue;
                var A = cubes[sc.a];
                var B = cubes[sc.b];

                // world-space attachment points:
                Vector3 attachA_world = A.p + A.q * sc.attachA_local;
                Vector3 attachB_world = B.p + B.q * sc.attachB_local;

                Vector3 d = attachB_world - attachA_world;
                float dist = d.magnitude;
                if (dist < 1e-6f) continue;
                Vector3 n = d / dist;
                float x = dist - sc.restLength;

                // relative velocity at attachment points (for damping along spring)
                Vector3 vA = A.v + Vector3.Cross(A.w, attachA_world - A.p);
                Vector3 vB = B.v + Vector3.Cross(B.w, attachB_world - B.p);
                Vector3 relV = vB - vA;
                float relAlong = Vector3.Dot(relV, n);

                // spring force with damping: F = -k * x - c * relAlong
                Vector3 F = -springK * x * n - springDamping * relAlong * n;

                // apply to A (and -F to B)
                F_acc[sc.a] += F;
                F_acc[sc.b] -= F;

                // torques: tau = r x F (r relative to COM)
                Vector3 rA = attachA_world - A.p;
                Vector3 rB = attachB_world - B.p;
                Tau_acc[sc.a] += Vector3.Cross(rA, F);
                Tau_acc[sc.b] += Vector3.Cross(rB, -F);
            }
        }

        // 2) add gravity and damping
        for (int i = 0; i < N; i++)
        {
            F_acc[i] += gravity * cubes[i].mass;
            // simple linear damping force (opposes velocity)
            F_acc[i] += -linearDamping * cubes[i].v * cubes[i].mass;
            // small angular damping torque
            Tau_acc[i] += -angularDamping * cubes[i].w * cubes[i].mass;
        }

        // 3) integrate velocities (explicit Euler on v and w)
        for (int i = 0; i < N; i++)
        {
            var rc = cubes[i];

            // linear acceleration
            Vector3 a = F_acc[i] / rc.mass;
            rc.v += a * dt;

            // angular acceleration: w' += I_world^{-1} * tau * dt
            Matrix4x4 IwInv = WorldInverseInertia(rc.q, rc.IbodyInv);
            Vector3 angAcc = Mul3x3(IwInv, Tau_acc[i]);
            rc.w += angAcc * dt;
        }

        // 4) integrate positions and orientations
        for (int i = 0; i < N; i++)
        {
            var rc = cubes[i];
            rc.p += rc.v * dt;
            rc.q = IntegrateRotation(rc.q, rc.w, dt);

            // safety clamp - avoid NaNs
            if (float.IsNaN(rc.p.x) || float.IsInfinity(rc.p.x))
                continue;
            rc.go.transform.position = rc.p;
            rc.go.transform.rotation = rc.q;
        }

        // 5) sphere collision impulse resolution (per-cube)
        // We'll treat sphere as static. For each cube, check approximate contact at nearest surface point (cube center to sphere).
        for (int i = 0; i < N; i++)
        {
            var rc = cubes[i];
            Vector3 toCenter = rc.p - sphereCenter;
            float dist = toCenter.magnitude;
            float minDist = sphereRadius + cubeSize * 0.5f;

            if (dist < minDist)
            {
                Vector3 normal = (dist > 1e-6f) ? (toCenter / dist) : Vector3.up;
                float penetration = minDist - dist;

                // positional correction (push out)
                rc.p += normal * penetration;
                rc.go.transform.position = rc.p;

                // approximate contact point on cube surface
                Vector3 contactPoint = rc.p - normal * (cubeSize * 0.5f);
                Vector3 r = contactPoint - rc.p; // approx zero vector but left for torque computation

                // relative velocity at contact point (sphere stationary)
                Vector3 vRel = rc.v + Vector3.Cross(rc.w, r);
                float vRelN = Vector3.Dot(vRel, normal);

                // only resolve if penetrating velocity toward sphere
                if (vRelN < 0f)
                {
                    // compute impulse scalar j using effective mass:
                    // denom = 1/m + n · ( (I_worldInv * (r × n)) × r )
                    Matrix4x4 IwInv = WorldInverseInertia(rc.q, rc.IbodyInv);
                    Vector3 rCrossN = Vector3.Cross(r, normal);
                    Vector3 tmp = Mul3x3(IwInv, rCrossN);
                    Vector3 crossTerm = Vector3.Cross(tmp, r);
                    float denom = (1.0f / rc.mass) + Vector3.Dot(normal, crossTerm);
                    denom = Mathf.Max(denom, minDenom);
                    float j = -(1f + restitution) * vRelN / denom;
                    j = Mathf.Clamp(j, -maxImpulse, maxImpulse);
                    Vector3 J = j * normal;

                    // apply linear impulse
                    rc.v += J / rc.mass;

                    // apply angular impulse: Δω = IwInv * (r × J)
                    Vector3 dL = Vector3.Cross(r, J);
                    Vector3 deltaW = Mul3x3(IwInv, dL);
                    rc.w += deltaW;
                }
            }
        }

        // 6) fracture detection + energized impulses
        if (!fractured)
        {
            bool hit = false;
            for (int i = 0; i < N; i++)
            {
                if ((cubes[i].p - sphereCenter).magnitude < sphereRadius + cubeSize * 0.5f)
                {
                    hit = true; break;
                }
            }

            if (hit)
            {
                // compute stored elastic energy in springs
                float Estored = 0f;
                foreach (var sc in springs)
                {
                    if (sc.broken) continue;
                    var A = cubes[sc.a];
                    var B = cubes[sc.b];
                    Vector3 aWorld = A.p + A.q * sc.attachA_local;
                    Vector3 bWorld = B.p + B.q * sc.attachB_local;
                    float d = (bWorld - aWorld).magnitude;
                    float x = d - sc.restLength;
                    Estored += 0.5f * springK * x * x;
                }

                // break springs (either all or selectively)
                if (breakAllOnFirstHit)
                {
                    foreach (var sc in springs) sc.broken = true;
                }
                else
                {
                    // optionally implement selective breaking (not done here)
                    foreach (var sc in springs) sc.broken = true;
                }

                fractured = true;

                // distribute energized impulses at each cube (we use equal energy allocation per cube)
                float Eadd = fractureAlpha * Estored;
                if (Eadd <= 0f) return;
                int M = N;
                float perE = Eadd / M;

                for (int i = 0; i < M; i++)
                {
                    var rc = cubes[i];

                    // choose impulse direction: outward from plate center plus upward bias
                    Vector3 dir = (rc.p - plateWorldCenter).normalized;
                    if (dir.sqrMagnitude < 1e-6f) dir = Random.onUnitSphere;
                    dir += Vector3.up * 0.15f;
                    dir.Normalize();

                    // apply impulse a bit offset from COM to generate torque
                    Vector3 appPoint = rc.p + dir * (cubeSize * 0.25f);
                    Vector3 r = appPoint - rc.p;

                    // compute energy quadratic term:
                    // energy = 0.5 * (mu^2) * (1/m + (r×dir)^T * IwInv * (r×dir))
                    Matrix4x4 IwInv = WorldInverseInertia(rc.q, rc.IbodyInv);
                    float a_lin = 1.0f / rc.mass;
                    Vector3 Lc = Vector3.Cross(r, dir);
                    float angQ = Vector3.Dot(Lc, Mul3x3(IwInv, Lc));
                    float denomEnergy = Mathf.Max(a_lin + angQ, 1e-8f);

                    float mu = Mathf.Sqrt(2f * perE / denomEnergy);
                    mu = Mathf.Clamp(mu, 0f, maxFractureImpulse);

                    Vector3 impulse = mu * dir;

                    // apply impulse to update v and w
                    rc.v += impulse / rc.mass;
                    Vector3 dL = Vector3.Cross(r, impulse);
                    Vector3 deltaW = Mul3x3(IwInv, dL);
                    rc.w += deltaW;
                }
            }
        }
    }
}
