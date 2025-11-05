using UnityEngine;

public class CubeObject2
{
    public GameObject cube;
    public Material material;

    public CubeObject2(Vector3 position, Color color)
    {
        cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = position;

        material = new Material(Shader.Find("Standard"));
        material.color = color;
        cube.GetComponent<Renderer>().material = material;
    }

    public void ApplyMatrix(float[,] M)
    {
        // Appliquer matrice de transformation manuellement
        Vector3[] vertices = cube.GetComponent<MeshFilter>().mesh.vertices;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 v = vertices[i];
            float x = v.x, y = v.y, z = v.z;

            float newX = M[0, 0] * x + M[0, 1] * y + M[0, 2] * z + M[0, 3];
            float newY = M[1, 0] * x + M[1, 1] * y + M[1, 2] * z + M[1, 3];
            float newZ = M[2, 0] * x + M[2, 1] * y + M[2, 2] * z + M[2, 3];

            vertices[i] = new Vector3(newX, newY, newZ);
        }
        cube.GetComponent<MeshFilter>().mesh.vertices = vertices;
        cube.GetComponent<MeshFilter>().mesh.RecalculateNormals();
    }
}
