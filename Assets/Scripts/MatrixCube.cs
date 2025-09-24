using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class MatrixCube : MonoBehaviour
{
    [Header("Translation speeds (units/sec)")]
    [SerializeField] Vector3 translationSpeedXYZ = Vector3.zero;
    [Header("Rotation speeds (degrees/sec)")]
    [SerializeField] Vector3 rotationSpeedXYZ = Vector3.zero;
    [Header("Transform type")]
    [SerializeField] bool isLocal = true;

    private Vector3[] baseVertices; // original cube shape
    private int[] triangles;

    // Current accumulated transformation
    private Matrix4x4 accumulatedTransform = Matrix4x4.identity;

    void Start()
    {
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

    void Update()
    {
        // Switch between local/global by commenting one
        if (isLocal)
            ApplyLocalTransform();
        else 
            ApplyGlobalTransform();

        // Build the final mesh
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
    void ApplyLocalTransform()
    {
        float dt = Time.deltaTime;

        // Convert rotation speeds (degrees/sec) to radians
        float alpha = Mathf.Deg2Rad * rotationSpeedXYZ.y * dt;
        float beta  = Mathf.Deg2Rad * rotationSpeedXYZ.x * dt;
        float gamma = Mathf.Deg2Rad * rotationSpeedXYZ.z * dt;

        // Build rotation matrices
        Matrix4x4 Rx = new Matrix4x4(
            new Vector4(1, 0, 0, 0),
            new Vector4(0, Mathf.Cos(beta), -Mathf.Sin(beta), 0),
            new Vector4(0, Mathf.Sin(beta),  Mathf.Cos(beta), 0),
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
            new Vector4(Mathf.Sin(gamma),  Mathf.Cos(gamma), 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );

        // Translation (incremental)
        Matrix4x4 T = Matrix4x4.identity;
        T.m03 = translationSpeedXYZ.x * dt;
        T.m13 = translationSpeedXYZ.y * dt;
        T.m23 = translationSpeedXYZ.z * dt;

        // LOCAL: rotate first (about object center), then translate
        accumulatedTransform = T * accumulatedTransform * (Ry * Rx * Rz);
    }

    // --------------------
    // GLOBAL TRANSFORM
    // --------------------
    void ApplyGlobalTransform()
    {
        float dt = Time.deltaTime;

        // Convert to radians
        float alpha = Mathf.Deg2Rad * rotationSpeedXYZ.y * dt;
        float beta  = Mathf.Deg2Rad * rotationSpeedXYZ.x * dt;
        float gamma = Mathf.Deg2Rad * rotationSpeedXYZ.z * dt;

        // Build rotations
        Matrix4x4 Rx = new Matrix4x4(
            new Vector4(1, 0, 0, 0),
            new Vector4(0, Mathf.Cos(beta), -Mathf.Sin(beta), 0),
            new Vector4(0, Mathf.Sin(beta),  Mathf.Cos(beta), 0),
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
            new Vector4(Mathf.Sin(gamma),  Mathf.Cos(gamma), 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );

        // Translation
        Matrix4x4 T = Matrix4x4.identity;
        T.m03 = translationSpeedXYZ.x * dt;
        T.m13 = translationSpeedXYZ.y * dt;
        T.m23 = translationSpeedXYZ.z * dt;

        // GLOBAL: rotate around world, then translate
        accumulatedTransform = T * (Ry * Rx * Rz) * accumulatedTransform;
    }
    
    // Get the transformed vertices in world coordinates
    public Vector3[] GetTransformedVertices()
    {
        Vector3[] transformed = new Vector3[baseVertices.Length];
        for (int i = 0; i < baseVertices.Length; i++)
            transformed[i] = accumulatedTransform.MultiplyPoint3x4(baseVertices[i]);
        return transformed;
    }
}
