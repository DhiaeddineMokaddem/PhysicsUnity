using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Energized fracture simulation following Baraff-style rigid-body math.
/// Alternative implementation with spring-based fracture
/// </summary>
public class EnergizedPlateFractureYahya : MonoBehaviour
{
    [Header("Plate layout")]
    public int cols = 5, rows = 4;
    public float cubeSize = 0.5f;
    public Vector3 plateCenterOffset = new Vector3(0f, 3f, 0f);

    [Header("Rigid body")]
    public float cubeMass = 1.0f;
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public float linearDamping = 0.02f;
    public float angularDamping = 0.02f;

    [Header("Springs / fracture")]
    public float springK = 400f;
    public float springDamping = 5f;
    public float fractureAlpha = 0.6f;
    public bool breakAllOnFirstHit = true;

    [Header("Sphere collider")]
    public Vector3 sphereCenter = new Vector3(0f, -1f, 0f);
    public float sphereRadius = 0.5f;
    public float restitution = 0.35f;

    [Header("Clamping / safety")]
    public float maxImpulse = 40f;
    public float maxFractureImpulse = 30f;
    public float minDenom = 1e-6f;

    class RigidCube
    {
        public GameObject go;
        public Vector3 p;
        public Quaternion q;
        public Vector3 v;
        public Vector3 w;
        public float mass;
        public Matrix4x4 Ibody;
        public Matrix4x4 IbodyInv;
    }

    class SpringConstraint
    {
        public int a, b;
        public Vector3 attachA_local;
        public Vector3 attachB_local;
        public float restLength;
        public bool broken = false;
    }

    List<RigidCube> cubes;
    List<SpringConstraint> springs;
    bool fractured = false;
    Vector3 plateWorldCenter;

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

    static Quaternion IntegrateRotation(Quaternion q, Vector3 w, float dt)
    {
        float theta = w.magnitude * dt;
        if (theta < 1e-8f)
        {
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

                float s = cubeSize;
                float I = (1f / 12f) * rc.mass * (s * s + s * s);
                rc.Ibody = MakeInertiaMatrix(I, I, I);
                rc.IbodyInv = Invert3x3(rc.Ibody);
                cubes.Add(rc);
            }
        }

        Vector3 sum = Vector3.zero;
        foreach (var rc in cubes) sum += rc.p;
        plateWorldCenter = sum / cubes.Count;

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                int idx = r * cols + c;
                if (c < cols - 1)
                {
                    SpringConstraint sC = new SpringConstraint();
                    sC.a = idx;
                    sC.b = idx + 1;
                    sC.attachA_local = new Vector3(cubeSize * 0.5f, 0f, 0f);
                    sC.attachB_local = new Vector3(-cubeSize * 0.5f, 0f, 0f);
                    sC.restLength = cubeSize;
                    springs.Add(sC);
                }
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

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        int N = cubes.Count;

        Vector3[] F_acc = new Vector3[N];
        Vector3[] Tau_acc = new Vector3[N];

        if (!fractured)
        {
            foreach (var sc in springs)
            {
                if (sc.broken) continue;
                var A = cubes[sc.a];
                var B = cubes[sc.b];

                Vector3 attachA_world = A.p + A.q * sc.attachA_local;
                Vector3 attachB_world = B.p + B.q * sc.attachB_local;

                Vector3 d = attachB_world - attachA_world;
                float dist = d.magnitude;
                if (dist < 1e-6f) continue;
                Vector3 n = d / dist;
                float x = dist - sc.restLength;

                Vector3 vA = A.v + Vector3.Cross(A.w, attachA_world - A.p);
                Vector3 vB = B.v + Vector3.Cross(B.w, attachB_world - B.p);
                Vector3 relV = vB - vA;
                float relAlong = Vector3.Dot(relV, n);

                Vector3 F = -springK * x * n - springDamping * relAlong * n;

                F_acc[sc.a] += F;
                F_acc[sc.b] -= F;

                Vector3 rA = attachA_world - A.p;
                Vector3 rB = attachB_world - B.p;
                Tau_acc[sc.a] += Vector3.Cross(rA, F);
                Tau_acc[sc.b] += Vector3.Cross(rB, -F);
            }
        }

        for (int i = 0; i < N; i++)
        {
            F_acc[i] += gravity * cubes[i].mass;
            F_acc[i] += -linearDamping * cubes[i].v * cubes[i].mass;
            Tau_acc[i] += -angularDamping * cubes[i].w * cubes[i].mass;
        }

        for (int i = 0; i < N; i++)
        {
            var rc = cubes[i];
            Vector3 a = F_acc[i] / rc.mass;
            rc.v += a * dt;

            Matrix4x4 IwInv = WorldInverseInertia(rc.q, rc.IbodyInv);
            Vector3 angAcc = Mul3x3(IwInv, Tau_acc[i]);
            rc.w += angAcc * dt;
        }

        for (int i = 0; i < N; i++)
        {
            var rc = cubes[i];
            rc.p += rc.v * dt;
            rc.q = IntegrateRotation(rc.q, rc.w, dt);

            if (float.IsNaN(rc.p.x) || float.IsInfinity(rc.p.x))
                continue;
            rc.go.transform.position = rc.p;
            rc.go.transform.rotation = rc.q;
        }

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

                rc.p += normal * penetration;
                rc.go.transform.position = rc.p;

                Vector3 contactPoint = rc.p - normal * (cubeSize * 0.5f);
                Vector3 r = contactPoint - rc.p;

                Vector3 vRel = rc.v + Vector3.Cross(rc.w, r);
                float vRelN = Vector3.Dot(vRel, normal);

                if (vRelN < 0f)
                {
                    Matrix4x4 IwInv = WorldInverseInertia(rc.q, rc.IbodyInv);
                    Vector3 rCrossN = Vector3.Cross(r, normal);
                    Vector3 tmp = Mul3x3(IwInv, rCrossN);
                    Vector3 crossTerm = Vector3.Cross(tmp, r);
                    float denom = (1.0f / rc.mass) + Vector3.Dot(normal, crossTerm);
                    denom = Mathf.Max(denom, minDenom);
                    float j = -(1f + restitution) * vRelN / denom;
                    j = Mathf.Clamp(j, -maxImpulse, maxImpulse);
                    Vector3 J = j * normal;

                    rc.v += J / rc.mass;
                    Vector3 dL = Vector3.Cross(r, J);
                    Vector3 deltaW = Mul3x3(IwInv, dL);
                    rc.w += deltaW;
                }
            }
        }

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

                if (breakAllOnFirstHit)
                {
                    foreach (var sc in springs) sc.broken = true;
                }

                fractured = true;

                float Eadd = fractureAlpha * Estored;
                if (Eadd <= 0f) return;
                int M = N;
                float perE = Eadd / M;

                for (int i = 0; i < M; i++)
                {
                    var rc = cubes[i];
                    Vector3 dir = (rc.p - plateWorldCenter).normalized;
                    if (dir.sqrMagnitude < 1e-6f) dir = Random.onUnitSphere;
                    dir += Vector3.up * 0.15f;
                    dir.Normalize();

                    Vector3 appPoint = rc.p + dir * (cubeSize * 0.25f);
                    Vector3 r = appPoint - rc.p;

                    Matrix4x4 IwInv = WorldInverseInertia(rc.q, rc.IbodyInv);
                    float a_lin = 1.0f / rc.mass;
                    Vector3 Lc = Vector3.Cross(r, dir);
                    float angQ = Vector3.Dot(Lc, Mul3x3(IwInv, Lc));
                    float denomEnergy = Mathf.Max(a_lin + angQ, 1e-8f);

                    float mu = Mathf.Sqrt(2f * perE / denomEnergy);
                    mu = Mathf.Clamp(mu, 0f, maxFractureImpulse);

                    Vector3 impulse = mu * dir;

                    rc.v += impulse / rc.mass;
                    Vector3 dL = Vector3.Cross(r, impulse);
                    Vector3 deltaW = Mul3x3(IwInv, dL);
                    rc.w += deltaW;
                }
            }
        }
    }
}