using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RaycastTP : MonoBehaviour
{
    [Header("Cube Settings")]
    public Vector3 cubeWorldPosition = Vector3.zero; // Cube fixed in world
    public float cubeSize = 1f;

    [Header("Capsule Settings")]
    public Vector3 capsulePosition = new Vector3(0f, 0f, -2f);
    public float capsuleRadius = 0.25f;
    public float capsuleHeight = 1.2f;

    [Header("Controls")]
    public float moveSpeed = 2f;
    public float yawSpeed = 2f;
    public float pitchSpeed = 2f;

    [Header("Material")]
    public Material cubeMaterial;

    private Mesh mesh;
    private Vector3[] cubeVertices;
    private int[] cubeTriangles;
    private int[][] cubeFaces;

    private float yaw = 0f;
    private float pitch = 0f;

    private Vector3 rayDir = Vector3.forward;

    private bool hitDetected = false;
    private Vector3 hitPoint;
    private Vector3 hitNormal;
    [Header("Capsule Mesh")]
    public Material capsuleMaterial; // assign a material for the capsule
    private GameObject capsuleObject;
    void Awake()
    {
        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;
        if (cubeMaterial != null)
            GetComponent<MeshRenderer>().material = cubeMaterial;

        BuildCubeMesh();

        cubeFaces = new int[][]
        {
            new int[]{0,2,1,3}, // back
            new int[]{4,5,6,7}, // front
            new int[]{0,1,5,4}, // bottom
            new int[]{2,3,7,6}, // top
            new int[]{0,4,7,3}, // left
            new int[]{1,2,6,5}  // right
        };
    }
    void Start()
    {
        // Create a capsule GameObject
        capsuleObject = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        capsuleObject.transform.localScale = new Vector3(capsuleRadius * 2f, capsuleHeight / 2f, capsuleRadius * 2f);
        capsuleObject.GetComponent<Collider>().enabled = false; // optional: disable collider
        if (capsuleMaterial != null)
            capsuleObject.GetComponent<MeshRenderer>().material = capsuleMaterial;
    }
    void Update()
    {
        HandleMovement();
        HandleMouseManual();
        UpdateRaycast();
        UpdateCubeMesh();

        // Move the capsule mesh
        if (capsuleObject != null)
            capsuleObject.transform.position = capsulePosition;
    }

    // ---------------- INPUT ----------------
    void HandleMovement()
    {
        Vector3 move = Vector3.zero;

        if (Input.GetKey(KeyCode.Z)) move += new Vector3(0, 0, 1);
        if (Input.GetKey(KeyCode.S)) move += new Vector3(0, 0, -1);
        if (Input.GetKey(KeyCode.Q)) move += new Vector3(-1, 0, 0);
        if (Input.GetKey(KeyCode.D)) move += new Vector3(1, 0, 0);
        if (Input.GetKey(KeyCode.A)) move += new Vector3(0, 1, 0);
        if (Input.GetKey(KeyCode.E)) move += new Vector3(0, -1, 0);

        capsulePosition += move * moveSpeed * Time.deltaTime;
    }

    void HandleMouseManual()
    {
        float mouseX = Input.GetAxis("Mouse X");
        float mouseY = Input.GetAxis("Mouse Y");

        yaw += mouseX * yawSpeed;
        pitch -= mouseY * pitchSpeed;

        if (pitch > 80f) pitch = 80f;
        if (pitch < -80f) pitch = -80f;

        float yawRad = yaw * Mathf.Deg2Rad;
        float pitchRad = pitch * Mathf.Deg2Rad;

        float cosPitch = Mathf.Cos(pitchRad);
        rayDir.x = Mathf.Sin(yawRad) * cosPitch;
        rayDir.y = Mathf.Sin(pitchRad);
        rayDir.z = Mathf.Cos(yawRad) * cosPitch;

        float length = Mathf.Sqrt(rayDir.x * rayDir.x + rayDir.y * rayDir.y + rayDir.z * rayDir.z);
        rayDir.x /= length;
        rayDir.y /= length;
        rayDir.z /= length;
    }

    // ---------------- RAYCAST ----------------
    void UpdateRaycast()
    {
        hitDetected = false;
        Vector3 origin = capsulePosition;
        Vector3[] verts = GetTransformedCubeVertices();

        float closestT = float.PositiveInfinity;

        foreach (var face in cubeFaces)
        {
            Vector3[] faceVerts = { verts[face[0]], verts[face[1]], verts[face[2]], verts[face[3]] };
            if (RayQuadIntersection(origin, rayDir, faceVerts, out Vector3 intersection, out float t))
            {
                hitDetected = true;
                if (t < closestT)
                {
                    closestT = t;
                    hitPoint = intersection;
                    hitNormal = Cross(faceVerts[1] - faceVerts[0], faceVerts[2] - faceVerts[0]).normalized;
                }
            }
        }

        if (cubeMaterial != null)
            cubeMaterial.color = hitDetected ? Color.red : Color.gray;
    }

    bool RayQuadIntersection(Vector3 origin, Vector3 dir, Vector3[] quad, out Vector3 intersection, out float t)
    {
        intersection = Vector3.zero;
        t = 0f;

        Vector3 p0 = quad[0], p1 = quad[1], p2 = quad[2];
        Vector3 normal = Cross(p1 - p0, p2 - p0).normalized;
        float denom = Dot(normal, dir);
        if (Mathf.Abs(denom) < 1e-6f) return false;

        t = Dot(p0 - origin, normal) / denom;
        if (t < 0) return false;

        Vector3 hit = origin + dir * t;
        if (PointInQuad(hit, quad))
        {
            intersection = hit;
            return true;
        }
        return false;
    }

    bool PointInQuad(Vector3 p, Vector3[] quad)
    {
        return PointInTriangle(p, quad[0], quad[1], quad[2]) || PointInTriangle(p, quad[0], quad[2], quad[3]);
    }

    bool PointInTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 v0 = c - a;
        Vector3 v1 = b - a;
        Vector3 v2 = p - a;
        float dot00 = Dot(v0, v0);
        float dot01 = Dot(v0, v1);
        float dot02 = Dot(v0, v2);
        float dot11 = Dot(v1, v1);
        float dot12 = Dot(v1, v2);
        float inv = 1f / (dot00 * dot11 - dot01 * dot01);
        float u = (dot11 * dot02 - dot01 * dot12) * inv;
        float v = (dot00 * dot12 - dot01 * dot02) * inv;
        return u >= 0 && v >= 0 && (u + v <= 1);
    }

    // ---------------- MESH ----------------
    void BuildCubeMesh()
    {
        float half = cubeSize * 0.5f;
        cubeVertices = new Vector3[]
        {
            new Vector3(-half,-half,-half),
            new Vector3( half,-half,-half),
            new Vector3( half, half,-half),
            new Vector3(-half, half,-half),
            new Vector3(-half,-half, half),
            new Vector3( half,-half, half),
            new Vector3( half, half, half),
            new Vector3(-half, half, half)
        };

        cubeTriangles = new int[]
        {
            0,2,1,0,3,2,
            4,5,6,4,6,7,
            0,1,5,0,5,4,
            2,3,7,2,7,6,
            0,4,7,0,7,3,
            1,2,6,1,6,5
        };

        mesh.Clear();
        mesh.vertices = GetTransformedCubeVertices();
        mesh.triangles = cubeTriangles;
        mesh.RecalculateNormals();
    }

    void UpdateCubeMesh()
    {
        mesh.vertices = GetTransformedCubeVertices();
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();
    }

    Vector3[] GetTransformedCubeVertices()
    {
        Vector3[] transformed = new Vector3[cubeVertices.Length];
        for (int i = 0; i < cubeVertices.Length; i++)
            transformed[i] = cubeVertices[i] + cubeWorldPosition; 
        return transformed;
    }

    // ---------------- GIZMOS ----------------
    void OnDrawGizmos()
    {
        // Capsule as vertical line
        Gizmos.color = Color.blue;
        Vector3 bottom = capsulePosition - new Vector3(0, capsuleHeight / 2f, 0);
        Vector3 top = capsulePosition + new Vector3(0, capsuleHeight / 2f, 0);
        Gizmos.DrawLine(bottom, top);
        Gizmos.DrawSphere(bottom, capsuleRadius);
        Gizmos.DrawSphere(top, capsuleRadius);

        // Ray
        Gizmos.color = Color.green;
        Gizmos.DrawRay(capsulePosition, rayDir * 10f);

        // Hit circle
        if (hitDetected)
        {
            Gizmos.color = Color.red;
            DrawCircle(hitPoint, hitNormal, 0.1f);
        }
    }

    void DrawCircle(Vector3 center, Vector3 normal, float radius, int segments = 24)
    {
        Vector3 tangent = Cross(normal, new Vector3(0, 1, 0));
        if (tangent.sqrMagnitude < 0.01f)
            tangent = Cross(normal, new Vector3(1, 0, 0));
        tangent.Normalize();
        Vector3 bitangent = Cross(normal, tangent);

        Vector3 prev = center + tangent * radius;
        for (int i = 1; i <= segments; i++)
        {
            float angle = i * Mathf.PI * 2f / segments;
            Vector3 next = center + tangent * Mathf.Cos(angle) * radius + bitangent * Mathf.Sin(angle) * radius;
            Gizmos.DrawLine(prev, next);
            prev = next;
        }
    }

    // ---------------- MANUAL VECTOR MATH ----------------
    Vector3 Cross(Vector3 a, Vector3 b)
    {
        return new Vector3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    float Dot(Vector3 a, Vector3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
}
