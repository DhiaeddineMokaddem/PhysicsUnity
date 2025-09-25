using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class MatrixCube : MonoBehaviour
{
    [Header("Free fall settings")]
    [SerializeField] float gravity = 9.81f;   // Gravity strength
    [SerializeField] float drag = 0.1f;       // Drag coefficient
    [SerializeField] float dt = 0.02f;        // Fixed timestep for Euler

    private Vector3 velocity;   
    private Vector3 position;  

    [Header("Rotation speeds (degrees/sec)")]
    [SerializeField] Vector3 rotationSpeedXYZ = Vector3.zero;
    [Header("Transform type")]
    [SerializeField] bool isLocal = true;

    private Vector3[] baseVertices;
    private int[] triangles;
    private Matrix4x4 accumulatedTransform = Matrix4x4.identity;

    void Start()
    {
        // Initial conditions
        velocity = Vector3.zero;
        position = Vector3.zero;

        // Define cube vertices
        baseVertices = new Vector3[]
        {
            new Vector3(-0.5f, -0.5f, -0.5f),
            new Vector3( 0.5f, -0.5f, -0.5f),
            new Vector3( 0.5f,  0.5f, -0.5f),
            new Vector3(-0.5f,  0.5f, -0.5f),
            new Vector3(-0.5f, -0.5f,  0.5f),
            new Vector3( 0.5f, -0.5f,  0.5f),
            new Vector3( 0.5f,  0.5f,  0.5f),
            new Vector3(-0.5f,  0.5f,  0.5f)
        };

        // Define faces
        triangles = new int[]
        {
            0, 2, 1, 0, 3, 2,   // Back
            4, 5, 6, 4, 6, 7,   // Front
            0, 1, 5, 0, 5, 4,   // Bottom
            2, 3, 7, 2, 7, 6,   // Top
            0, 4, 7, 0, 7, 3,   // Left
            1, 2, 6, 1, 6, 5    // Right
        };
    }

    void FixedUpdate()
    {
        // Freefall drag 
        Vector3 acceleration = Vector3.down * gravity - drag * velocity;
        velocity += acceleration * dt;   // v(t+dt) = v(t) + a*dt
        position += velocity * dt;       // p(t+dt) = p(t) + v*dt

        // Build translation from position
        Matrix4x4 T = Matrix4x4.identity;
        T.m03 = position.x;
        T.m13 = position.y;
        T.m23 = position.z;

        // Rotation handling
        if (isLocal)
            ApplyLocalTransform(T);
        else
            ApplyGlobalTransform(T);

        // Build final mesh
        Mesh mesh = new Mesh();
        Vector3[] transformed = new Vector3[baseVertices.Length];
        for (int i = 0; i < baseVertices.Length; i++)
            transformed[i] = accumulatedTransform.MultiplyPoint3x4(baseVertices[i]);

        mesh.vertices = transformed;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        GetComponent<MeshFilter>().mesh = mesh;
    }

    // --------------------
    // LOCAL TRANSFORM
    // --------------------
    void ApplyLocalTransform(Matrix4x4 T)
    {
        float dtFrame = Time.fixedDeltaTime;
        float alpha = Mathf.Deg2Rad * rotationSpeedXYZ.y * dtFrame;
        float beta = Mathf.Deg2Rad * rotationSpeedXYZ.x * dtFrame;
        float gamma = Mathf.Deg2Rad * rotationSpeedXYZ.z * dtFrame;

        Matrix4x4 Rx = new Matrix4x4(
            new Vector4(1, 0, 0, 0),
            new Vector4(0, Mathf.Cos(beta), -Mathf.Sin(beta), 0),
            new Vector4(0, Mathf.Sin(beta), Mathf.Cos(beta), 0),
            new Vector4(0, 0, 0, 1)
        );

        Matrix4x4 Ry = new Matrix4x4(
            new Vector4(Mathf.Cos(alpha), 0, Mathf.Sin(alpha), 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(-Mathf.Sin(alpha), 0, Mathf.Cos(alpha), 0),
            new Vector4(0, 0, 0, 1)
        );

        Matrix4x4 Rz = new Matrix4x4(
            new Vector4(Mathf.Cos(gamma), -Mathf.Sin(gamma), 0, 0),
            new Vector4(Mathf.Sin(gamma), Mathf.Cos(gamma), 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );

        accumulatedTransform = T * accumulatedTransform * (Ry * Rx * Rz);
    }

    // --------------------
    // GLOBAL TRANSFORM
    // --------------------
    void ApplyGlobalTransform(Matrix4x4 T)
    {
        float dtFrame = Time.fixedDeltaTime;
        float alpha = Mathf.Deg2Rad * rotationSpeedXYZ.y * dtFrame;
        float beta = Mathf.Deg2Rad * rotationSpeedXYZ.x * dtFrame;
        float gamma = Mathf.Deg2Rad * rotationSpeedXYZ.z * dtFrame;

        Matrix4x4 Rx = new Matrix4x4(
            new Vector4(1, 0, 0, 0),
            new Vector4(0, Mathf.Cos(beta), -Mathf.Sin(beta), 0),
            new Vector4(0, Mathf.Sin(beta), Mathf.Cos(beta), 0),
            new Vector4(0, 0, 0, 1)
        );

        Matrix4x4 Ry = new Matrix4x4(
            new Vector4(Mathf.Cos(alpha), 0, Mathf.Sin(alpha), 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(-Mathf.Sin(alpha), 0, Mathf.Cos(alpha), 0),
            new Vector4(0, 0, 0, 1)
        );

        Matrix4x4 Rz = new Matrix4x4(
            new Vector4(Mathf.Cos(gamma), -Mathf.Sin(gamma), 0, 0),
            new Vector4(Mathf.Sin(gamma), Mathf.Cos(gamma), 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );

        accumulatedTransform = T * (Ry * Rx * Rz) * accumulatedTransform;
    }

    // Get transformed vertices in world coordinates
    public Vector3[] GetTransformedVertices()
    {
        Vector3[] transformed = new Vector3[baseVertices.Length];
        for (int i = 0; i < baseVertices.Length; i++)
            transformed[i] = accumulatedTransform.MultiplyPoint3x4(baseVertices[i]);
        return transformed;
    }
}
