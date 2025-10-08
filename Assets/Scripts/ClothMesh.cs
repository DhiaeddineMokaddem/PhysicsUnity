
using System.Collections.Generic;
using UnityEngine;

public class ClothMesh : MonoBehaviour
{
    // Base vertices of the cube (local space)
    public List<Vector3> baseVertices;
    // Triangle indices for mesh faces
    public List<int> triangles;
    public float distanceBetweenVertexes = 0.2f;
    public int vertexesPerLine = 5;
    public int numberOfLines = 5;
    void Awake()
    {
        baseVertices = new List<Vector3>();
        for (int i = 0; i < numberOfLines; i++)
        {
            for (int j = 0; j < vertexesPerLine; j++)
            {
                baseVertices.Add(new Vector3(i*distanceBetweenVertexes,0,j*distanceBetweenVertexes));
            }
        }
        triangles = new List<int>();
        for (int i = 0; i < numberOfLines-1; i++)
        {
            for (int j = 0; j < vertexesPerLine; j++)
            {
                if(j != vertexesPerLine - 1)
                {
                    triangles.Add(j + (i * vertexesPerLine));
                    triangles.Add(j + (i * vertexesPerLine) + 1);
                    triangles.Add(j + ((i + 1) * vertexesPerLine));
                }
                if (j != 0)
                {
                    triangles.Add(j + (i * vertexesPerLine));
                    triangles.Add(j + ((i + 1) * vertexesPerLine));
                    triangles.Add(j + ((i+1) * vertexesPerLine) - 1);
                }
            }
        }
    }
    void Start()
    {
        
    }
    void Update()
    {
        Mesh mesh = new Mesh();
        mesh.vertices = baseVertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals(); // For correct lighting
        GetComponent<MeshFilter>().mesh = mesh;
    }
}
