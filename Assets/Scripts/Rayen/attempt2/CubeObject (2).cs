using UnityEngine;

public class CubeObject
{
    public GameObject cube;
    public Material material;
    private Mesh mesh;
    private Vector3[] originalVertices;

    // position ici n'est appliquée que via la matrice, on garde transform.position = Vector3.zero
    public CubeObject(Vector3 position, Color color, string name = "CubeObject")
    {
        // Crée un primitive cube (contient MeshFilter + MeshRenderer)
        cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.name = name;

        // S'assurer que le transform du GameObject est propre
        cube.transform.position = Vector3.zero;
        cube.transform.rotation = Quaternion.identity;
        cube.transform.localScale = Vector3.one;

        // Cloner le mesh pour éviter de modifier le sharedMesh du primitive
        MeshFilter mf = cube.GetComponent<MeshFilter>();
        mesh = Object.Instantiate(mf.sharedMesh);
        mf.mesh = mesh;

        // Matériau propre
        material = new Material(Shader.Find("Standard"));
        material.color = color;
        cube.GetComponent<Renderer>().material = material;

        // Sauvegarder les sommets originaux (coordonnées locales du cube)
        originalVertices = mesh.vertices;
    }

    // Applique la matrice 4x4 M (format float[4,4]) aux sommets originaux
    public void ApplyMatrix(float[,] M)
    {
        if (mesh == null || originalVertices == null) return;

        Vector3[] transformed = new Vector3[originalVertices.Length];

        for (int i = 0; i < originalVertices.Length; i++)
        {
            Vector3 v = originalVertices[i];
            float x = v.x, y = v.y, z = v.z;

            float newX = M[0, 0] * x + M[0, 1] * y + M[0, 2] * z + M[0, 3];
            float newY = M[1, 0] * x + M[1, 1] * y + M[1, 2] * z + M[1, 3];
            float newZ = M[2, 0] * x + M[2, 1] * y + M[2, 2] * z + M[2, 3];

            transformed[i] = new Vector3(newX, newY, newZ);
        }

        mesh.vertices = transformed;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds(); // important pour éviter le frustum culling
    }

    // Remet le mesh à l'état initial
    public void ResetToOriginal()
    {
        if (mesh == null || originalVertices == null) return;
        mesh.vertices = (Vector3[])originalVertices.Clone();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    public void Destroy()
    {
        if (cube != null) Object.Destroy(cube);
        if (material != null) Object.Destroy(material);
        // mesh cloné sera détruit avec le GameObject
    }
}
