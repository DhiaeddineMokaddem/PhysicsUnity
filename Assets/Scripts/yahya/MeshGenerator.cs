using UnityEngine;

/// <summary>
/// Génère des maillages procéduraux sans utiliser de primitives Unity
/// </summary>
public static class MeshGenerator
{
    /// <summary>
    /// Crée un maillage de cube avec dimensions personnalisées
    /// </summary>
    public static Mesh CreateCubeMesh(Vector3 size)
    {
        Mesh mesh = new Mesh();
        mesh.name = "ProceduralCube";
        
        float x = size.x * 0.5f;
        float y = size.y * 0.5f;
        float z = size.z * 0.5f;
        
        // 24 vertices (4 per face)
        Vector3[] vertices = new Vector3[]
        {
            // Front face
            new Vector3(-x, -y, z), new Vector3(x, -y, z), new Vector3(x, y, z), new Vector3(-x, y, z),
            // Back face
            new Vector3(x, -y, -z), new Vector3(-x, -y, -z), new Vector3(-x, y, -z), new Vector3(x, y, -z),
            // Top face
            new Vector3(-x, y, z), new Vector3(x, y, z), new Vector3(x, y, -z), new Vector3(-x, y, -z),
            // Bottom face
            new Vector3(-x, -y, -z), new Vector3(x, -y, -z), new Vector3(x, -y, z), new Vector3(-x, -y, z),
            // Right face
            new Vector3(x, -y, z), new Vector3(x, -y, -z), new Vector3(x, y, -z), new Vector3(x, y, z),
            // Left face
            new Vector3(-x, -y, -z), new Vector3(-x, -y, z), new Vector3(-x, y, z), new Vector3(-x, y, -z)
        };
        
        // Triangles (2 per face, 6 faces)
        int[] triangles = new int[]
        {
            0, 2, 1, 0, 3, 2,       // Front
            4, 6, 5, 4, 7, 6,       // Back
            8, 10, 9, 8, 11, 10,    // Top
            12, 14, 13, 12, 15, 14, // Bottom
            16, 18, 17, 16, 19, 18, // Right
            20, 22, 21, 20, 23, 22  // Left
        };
        
        // Normales
        Vector3[] normals = new Vector3[]
        {
            // Front
            Vector3.forward, Vector3.forward, Vector3.forward, Vector3.forward,
            // Back
            Vector3.back, Vector3.back, Vector3.back, Vector3.back,
            // Top
            Vector3.up, Vector3.up, Vector3.up, Vector3.up,
            // Bottom
            Vector3.down, Vector3.down, Vector3.down, Vector3.down,
            // Right
            Vector3.right, Vector3.right, Vector3.right, Vector3.right,
            // Left
            Vector3.left, Vector3.left, Vector3.left, Vector3.left
        };
        
        // UVs
        Vector2[] uvs = new Vector2[24];
        for (int i = 0; i < 6; i++)
        {
            int idx = i * 4;
            uvs[idx] = new Vector2(0, 0);
            uvs[idx + 1] = new Vector2(1, 0);
            uvs[idx + 2] = new Vector2(1, 1);
            uvs[idx + 3] = new Vector2(0, 1);
        }
        
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.normals = normals;
        mesh.uv = uvs;
        
        return mesh;
    }
    
    /// <summary>
    /// Crée un cylindre (pour les poteaux)
    /// </summary>
    public static Mesh CreateCylinderMesh(float radius, float height, int segments = 16)
    {
        Mesh mesh = new Mesh();
        mesh.name = "ProceduralCylinder";
        
        int vertCount = segments * 4 + 2; // sides + top + bottom + centers
        Vector3[] vertices = new Vector3[vertCount];
        int[] triangles = new int[segments * 12];
        Vector3[] normals = new Vector3[vertCount];
        
        float halfHeight = height * 0.5f;
        
        // Center vertices
        vertices[0] = new Vector3(0, halfHeight, 0);  // top center
        vertices[1] = new Vector3(0, -halfHeight, 0); // bottom center
        normals[0] = Vector3.up;
        normals[1] = Vector3.down;
        
        // Generate vertices
        int vertIdx = 2;
        for (int i = 0; i < segments; i++)
        {
            float angle = (float)i / segments * Mathf.PI * 2f;
            float x = Mathf.Cos(angle) * radius;
            float z = Mathf.Sin(angle) * radius;
            Vector3 normal = new Vector3(x, 0, z).normalized;
            
            // Top rim
            vertices[vertIdx] = new Vector3(x, halfHeight, z);
            normals[vertIdx] = Vector3.up;
            vertIdx++;
            
            // Side top
            vertices[vertIdx] = new Vector3(x, halfHeight, z);
            normals[vertIdx] = normal;
            vertIdx++;
            
            // Side bottom
            vertices[vertIdx] = new Vector3(x, -halfHeight, z);
            normals[vertIdx] = normal;
            vertIdx++;
            
            // Bottom rim
            vertices[vertIdx] = new Vector3(x, -halfHeight, z);
            normals[vertIdx] = Vector3.down;
            vertIdx++;
        }
        
        // Generate triangles
        int triIdx = 0;
        for (int i = 0; i < segments; i++)
        {
            int current = i * 4 + 2;
            int next = ((i + 1) % segments) * 4 + 2;
            
            // Top cap
            triangles[triIdx++] = 0;
            triangles[triIdx++] = current;
            triangles[triIdx++] = next;
            
            // Side faces
            triangles[triIdx++] = current + 1;
            triangles[triIdx++] = next + 1;
            triangles[triIdx++] = current + 2;
            
            triangles[triIdx++] = next + 1;
            triangles[triIdx++] = next + 2;
            triangles[triIdx++] = current + 2;
            
            // Bottom cap
            triangles[triIdx++] = 1;
            triangles[triIdx++] = next + 3;
            triangles[triIdx++] = current + 3;
        }
        
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.normals = normals;
        mesh.RecalculateBounds();
        
        return mesh;
    }
    
    /// <summary>
    /// Crée un GameObject avec un maillage personnalisé
    /// </summary>
    public static GameObject CreateMeshObject(Mesh mesh, Material material, string name)
    {
        GameObject obj = new GameObject(name);
        
        MeshFilter filter = obj.AddComponent<MeshFilter>();
        filter.mesh = mesh;
        
        MeshRenderer renderer = obj.AddComponent<MeshRenderer>();
        renderer.material = material;
        
        return obj;
    }
    
    /// <summary>
    /// Crée un matériau avec une couleur
    /// </summary>
    public static Material CreateColorMaterial(Color color)
    {
        // Try different shader paths for different render pipelines
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null) shader = Shader.Find("Standard");
        if (shader == null) shader = Shader.Find("Diffuse");
        if (shader == null) shader = Shader.Find("Legacy Shaders/Diffuse");
        if (shader == null)
        {
            Debug.LogWarning("No suitable shader found, using default");
            shader = Shader.Find("UI/Default");
        }
        
        Material mat = new Material(shader);
        
        // Set color using different property names depending on shader
        if (mat.HasProperty("_BaseColor"))
            mat.SetColor("_BaseColor", color);
        else if (mat.HasProperty("_Color"))
            mat.SetColor("_Color", color);
        else
            mat.color = color;
            
        return mat;
    }
}