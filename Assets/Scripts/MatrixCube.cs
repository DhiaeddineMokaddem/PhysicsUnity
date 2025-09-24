using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class MatrixCube : MonoBehaviour
{
    [Header("Translation speeds (units/sec)")]
    [SerializeField]Vector3 translationSpeedXYZ= Vector3.zero;
    [Header("Rotation speeds (degrees/sec)")]
    [SerializeField]Vector3 rotationSpeedXYZ= Vector3.zero;
    
    private Vector3[] vertices;
    private int[] triangles;
    void Start()
    {
        // Définition des sommets du cube
         vertices = new Vector3[]
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

        // Définition des faces (12 triangles)
         triangles = new int[] {
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
        // Angles en radians
        
        float alpha = Mathf.Deg2Rad * rotationSpeedXYZ.y*Time.deltaTime; // Rotation Y
        float beta  = Mathf.Deg2Rad * rotationSpeedXYZ.x*Time.deltaTime; // Rotation X
        float gamma = Mathf.Deg2Rad * rotationSpeedXYZ.z*Time.deltaTime;  // Rotation Z

        // Construction des matrices de rotation
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
        T.m03 = translationSpeedXYZ.x*Time.deltaTime; // x
        T.m13 = translationSpeedXYZ.y*Time.deltaTime; // y
        T.m23 = translationSpeedXYZ.z*Time.deltaTime; // z
        

        // Composition : Translation * Ry * Rx * Rz
        Matrix4x4 transformMatrix = T * Ry * Rx * Rz;

        // Application aux sommets
        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = transformMatrix.MultiplyPoint3x4(vertices[i]);
        }

        // Assignation
        Mesh mesh = new Mesh();
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        GetComponent<MeshFilter>().mesh = mesh;
    }
}
