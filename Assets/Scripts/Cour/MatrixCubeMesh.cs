using UnityEngine;

/// <summary>
/// Handles mesh creation and transformation for the cube.
/// Separates mesh logic from soft-body simulation.
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class MatrixCubeMesh : MonoBehaviour
{
    public Vector3[] baseVertices;
    public int[] triangles;
    public Matrix4x4 meshTransform = Matrix4x4.identity; // world-space TRS used last frame

    void Awake()
    {
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

        triangles = new int[]
        {
            0,2,1, 0,3,2,
            4,5,6, 4,6,7,
            0,1,5, 0,5,4,
            2,3,7, 2,7,6,
            0,4,7, 0,7,3,
            1,2,6, 1,6,5
        };
    }

    /// <summary>
    /// Update the mesh using a **world-space** TRS matrix.
    /// The method converts world-space transformed vertices into the mesh's local space
    /// before assigning them to mesh.vertices so the rendered mesh appears exactly
    /// at the positions defined by the world TRS.
    /// </summary>
    public void UpdateMesh(Matrix4x4 worldTRS)
    {
        meshTransform = worldTRS;

        // Transform base vertices -> world space
        Vector3[] worldVerts = new Vector3[baseVertices.Length];
        for (int i = 0; i < baseVertices.Length; i++)
        {
            Vector3 v = worldTRS.MultiplyPoint3x4(baseVertices[i]);

            // safety guard
            if (float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) ||
                float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z))
            {
                v = transform.position; // fallback to object's origin
            }
            worldVerts[i] = v;
        }

        // Convert world-space verts into this GameObject's local space
        Matrix4x4 worldToLocal = transform.worldToLocalMatrix;
        Vector3[] localVerts = new Vector3[worldVerts.Length];
        for (int i = 0; i < worldVerts.Length; i++)
            localVerts[i] = worldToLocal.MultiplyPoint3x4(worldVerts[i]);

        // Build mesh safely
        Mesh mesh = new Mesh();
        mesh.name = "MatrixCubeMesh_Generated";
        mesh.vertices = localVerts;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        // safety: if bounds are invalid, clamp to small bounds
        var b = mesh.bounds;
        if (float.IsNaN(b.min.x) || float.IsInfinity(b.size.magnitude))
        {
            // fallback simple cube
            Vector3[] fallbackVerts = new Vector3[baseVertices.Length];
            for (int i = 0; i < baseVertices.Length; i++) fallbackVerts[i] = baseVertices[i] * 0.5f;
            mesh.vertices = fallbackVerts;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
        }
        else
        {
            mesh.RecalculateBounds();
        }

        GetComponent<MeshFilter>().mesh = mesh;
    }

    /// <summary>Returns transformed vertices using meshTransform (world-space positions).</summary>
    public Vector3[] GetTransformedVertices()
    {
        Vector3[] transformed = new Vector3[baseVertices.Length];
        for (int i = 0; i < baseVertices.Length; i++)
            transformed[i] = meshTransform.MultiplyPoint3x4(baseVertices[i]);
        return transformed;
    }

    /// <summary>Returns transformed vertices in world-space including the GameObject transform.</summary>
    public Vector3[] GetTransformedVerticesWorld()
    {
        Vector3[] transformed = new Vector3[baseVertices.Length];
        Matrix4x4 worldMat = transform.localToWorldMatrix * meshTransform;
        for (int i = 0; i < baseVertices.Length; i++)
            transformed[i] = worldMat.MultiplyPoint3x4(baseVertices[i]);
        return transformed;
    }
}
