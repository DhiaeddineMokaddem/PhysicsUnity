using UnityEngine;

// Handles mesh creation and transformation for the cube
// Separates mesh logic from rotation and physics
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class MatrixCubeMesh : MonoBehaviour
{
    // Base vertices of the cube (local space)
    public Vector3[] baseVertices;
    // Triangle indices for mesh faces
    public int[] triangles;
    // Current transformation matrix applied to the mesh
    public Matrix4x4 meshTransform = Matrix4x4.identity;

    // Called when the script instance is being loaded
    // Initializes the cube's geometry
    void Awake()
    {
        // Define cube vertices (8 corners)
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
        // Define cube faces (triangles)
        triangles = new int[]
        {
            0, 2, 1, 0, 3, 2,
            4, 5, 6, 4, 6, 7,
            0, 1, 5, 0, 5, 4,
            2, 3, 7, 2, 7, 6,
            0, 4, 7, 0, 7, 3,
            1, 2, 6, 1, 6, 5
        };
    }

    // Updates the mesh vertices using the given transformation matrix
    // This is called every frame to animate the cube
    public void UpdateMesh(Matrix4x4 transform)
    {
        meshTransform = transform;
        Mesh mesh = new Mesh();
        Vector3[] transformed = new Vector3[baseVertices.Length];
        // Transform each vertex by the current matrix
        for (int i = 0; i < baseVertices.Length; i++)
            transformed[i] = meshTransform.MultiplyPoint3x4(baseVertices[i]);
        mesh.vertices = transformed;
        mesh.triangles = triangles;
        mesh.RecalculateNormals(); // For correct lighting
        GetComponent<MeshFilter>().mesh = mesh;
    }

    // Returns the transformed vertices of the mesh
    // Useful for other scripts (e.g., trail)
    public Vector3[] GetTransformedVertices()
    {
        Vector3[] transformed = new Vector3[baseVertices.Length];
        for (int i = 0; i < baseVertices.Length; i++)
            transformed[i] = meshTransform.MultiplyPoint3x4(baseVertices[i]);
        return transformed;
    }
}
