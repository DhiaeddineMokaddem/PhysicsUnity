using UnityEngine;

/// <summary>
/// Génère des maillages personnalisés par code mathématique pur
/// </summary>
public static class CustomMeshGenerator
{
    /// <summary>
    /// Crée un cube avec dimensions personnalisées
    /// </summary>
    public static Mesh CreateBox(Vector3 size)
    {
        Mesh mesh = new Mesh();
        mesh.name = "CustomBox";

        float x = size.x * 0.5f;
        float y = size.y * 0.5f;
        float z = size.z * 0.5f;

        // 24 vertices (4 par face pour les normales correctes)
        Vector3[] vertices = new Vector3[24]
        {
            // Face avant (z+)
            new Vector3(-x, -y,  z), new Vector3( x, -y,  z), new Vector3( x,  y,  z), new Vector3(-x,  y,  z),
            // Face arrière (z-)
            new Vector3( x, -y, -z), new Vector3(-x, -y, -z), new Vector3(-x,  y, -z), new Vector3( x,  y, -z),
            // Face gauche (x-)
            new Vector3(-x, -y, -z), new Vector3(-x, -y,  z), new Vector3(-x,  y,  z), new Vector3(-x,  y, -z),
            // Face droite (x+)
            new Vector3( x, -y,  z), new Vector3( x, -y, -z), new Vector3( x,  y, -z), new Vector3( x,  y,  z),
            // Face haut (y+)
            new Vector3(-x,  y,  z), new Vector3( x,  y,  z), new Vector3( x,  y, -z), new Vector3(-x,  y, -z),
            // Face bas (y-)
            new Vector3(-x, -y, -z), new Vector3( x, -y, -z), new Vector3( x, -y,  z), new Vector3(-x, -y,  z)
        };

        // Normales
        Vector3[] normals = new Vector3[24];
        for (int i = 0; i < 4; i++)
        {
            normals[i] = Vector3.forward;      // Face avant
            normals[i + 4] = Vector3.back;     // Face arrière
            normals[i + 8] = Vector3.left;     // Face gauche
            normals[i + 12] = Vector3.right;   // Face droite
            normals[i + 16] = Vector3.up;      // Face haut
            normals[i + 20] = Vector3.down;    // Face bas
        }

        // UVs
        Vector2[] uvs = new Vector2[24];
        for (int i = 0; i < 6; i++)
        {
            int offset = i * 4;
            uvs[offset] = new Vector2(0, 0);
            uvs[offset + 1] = new Vector2(1, 0);
            uvs[offset + 2] = new Vector2(1, 1);
            uvs[offset + 3] = new Vector2(0, 1);
        }

        // Triangles (2 par face, 6 faces)
        int[] triangles = new int[36];
        for (int i = 0; i < 6; i++)
        {
            int offset = i * 4;
            int triOffset = i * 6;
            
            triangles[triOffset] = offset;
            triangles[triOffset + 1] = offset + 2;
            triangles[triOffset + 2] = offset + 1;
            
            triangles[triOffset + 3] = offset;
            triangles[triOffset + 4] = offset + 3;
            triangles[triOffset + 5] = offset + 2;
        }

        mesh.vertices = vertices;
        mesh.normals = normals;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.RecalculateBounds();

        return mesh;
    }

    /// <summary>
    /// Crée un plan horizontal
    /// </summary>
    public static Mesh CreatePlane(float width, float depth, int subdivisions = 1)
    {
        Mesh mesh = new Mesh();
        mesh.name = "CustomPlane";

        int vertCountX = subdivisions + 2;
        int vertCountZ = subdivisions + 2;
        int vertCount = vertCountX * vertCountZ;

        Vector3[] vertices = new Vector3[vertCount];
        Vector3[] normals = new Vector3[vertCount];
        Vector2[] uvs = new Vector2[vertCount];

        float halfW = width * 0.5f;
        float halfD = depth * 0.5f;

        // Générer les vertices
        int vertIndex = 0;
        for (int z = 0; z < vertCountZ; z++)
        {
            for (int x = 0; x < vertCountX; x++)
            {
                float xPos = -halfW + (width * x / (float)(vertCountX - 1));
                float zPos = -halfD + (depth * z / (float)(vertCountZ - 1));
                
                vertices[vertIndex] = new Vector3(xPos, 0, zPos);
                normals[vertIndex] = Vector3.up;
                uvs[vertIndex] = new Vector2((float)x / (vertCountX - 1), (float)z / (vertCountZ - 1));
                
                vertIndex++;
            }
        }

        // Générer les triangles
        int triCount = (vertCountX - 1) * (vertCountZ - 1) * 6;
        int[] triangles = new int[triCount];
        int triIndex = 0;

        for (int z = 0; z < vertCountZ - 1; z++)
        {
            for (int x = 0; x < vertCountX - 1; x++)
            {
                int i = z * vertCountX + x;
                
                // Premier triangle
                triangles[triIndex++] = i;
                triangles[triIndex++] = i + vertCountX;
                triangles[triIndex++] = i + 1;
                
                // Second triangle
                triangles[triIndex++] = i + 1;
                triangles[triIndex++] = i + vertCountX;
                triangles[triIndex++] = i + vertCountX + 1;
            }
        }

        mesh.vertices = vertices;
        mesh.normals = normals;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.RecalculateBounds();

        return mesh;
    }

    /// <summary>
    /// Crée un cylindre (pour les poteaux)
    /// </summary>
    public static Mesh CreateCylinder(float radius, float height, int segments = 16)
    {
        Mesh mesh = new Mesh();
        mesh.name = "CustomCylinder";

        int vertCount = (segments + 1) * 2 + segments * 2;
        Vector3[] vertices = new Vector3[vertCount];
        Vector3[] normals = new Vector3[vertCount];
        Vector2[] uvs = new Vector2[vertCount];

        float halfH = height * 0.5f;
        int vertIndex = 0;

        // Cercle du bas et du haut
        for (int ring = 0; ring < 2; ring++)
        {
            float y = (ring == 0) ? -halfH : halfH;
            
            for (int i = 0; i <= segments; i++)
            {
                float angle = (float)i / segments * Mathf.PI * 2f;
                float x = Mathf.Cos(angle) * radius;
                float z = Mathf.Sin(angle) * radius;
                
                vertices[vertIndex] = new Vector3(x, y, z);
                normals[vertIndex] = new Vector3(x, 0, z).normalized;
                uvs[vertIndex] = new Vector2((float)i / segments, ring);
                vertIndex++;
            }
        }

        // Caps (centres + bords)
        int capCenterBottom = vertIndex;
        vertices[vertIndex] = new Vector3(0, -halfH, 0);
        normals[vertIndex] = Vector3.down;
        uvs[vertIndex++] = new Vector2(0.5f, 0.5f);

        for (int i = 0; i < segments; i++)
        {
            float angle = (float)i / segments * Mathf.PI * 2f;
            float x = Mathf.Cos(angle) * radius;
            float z = Mathf.Sin(angle) * radius;
            
            vertices[vertIndex] = new Vector3(x, -halfH, z);
            normals[vertIndex] = Vector3.down;
            uvs[vertIndex++] = new Vector2(0.5f + x / (radius * 2), 0.5f + z / (radius * 2));
        }

        int capCenterTop = vertIndex;
        vertices[vertIndex] = new Vector3(0, halfH, 0);
        normals[vertIndex] = Vector3.up;
        uvs[vertIndex++] = new Vector2(0.5f, 0.5f);

        for (int i = 0; i < segments; i++)
        {
            float angle = (float)i / segments * Mathf.PI * 2f;
            float x = Mathf.Cos(angle) * radius;
            float z = Mathf.Sin(angle) * radius;
            
            vertices[vertIndex] = new Vector3(x, halfH, z);
            normals[vertIndex] = Vector3.up;
            uvs[vertIndex++] = new Vector2(0.5f + x / (radius * 2), 0.5f + z / (radius * 2));
        }

        // Triangles des côtés
        int triCount = segments * 6 + segments * 6;
        int[] triangles = new int[triCount];
        int triIndex = 0;

        for (int i = 0; i < segments; i++)
        {
            int bottomCurrent = i;
            int bottomNext = i + 1;
            int topCurrent = (segments + 1) + i;
            int topNext = (segments + 1) + i + 1;

            triangles[triIndex++] = bottomCurrent;
            triangles[triIndex++] = topCurrent;
            triangles[triIndex++] = bottomNext;

            triangles[triIndex++] = bottomNext;
            triangles[triIndex++] = topCurrent;
            triangles[triIndex++] = topNext;
        }

        // Triangles du cap bas
        for (int i = 0; i < segments; i++)
        {
            triangles[triIndex++] = capCenterBottom;
            triangles[triIndex++] = capCenterBottom + 1 + ((i + 1) % segments);
            triangles[triIndex++] = capCenterBottom + 1 + i;
        }

        // Triangles du cap haut
        for (int i = 0; i < segments; i++)
        {
            triangles[triIndex++] = capCenterTop;
            triangles[triIndex++] = capCenterTop + 1 + i;
            triangles[triIndex++] = capCenterTop + 1 + ((i + 1) % segments);
        }

        mesh.vertices = vertices;
        mesh.normals = normals;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.RecalculateBounds();

        return mesh;
    }

    /// <summary>
    /// Crée un GameObject avec un maillage personnalisé
    /// </summary>
    public static GameObject CreateMeshObject(string name, Mesh mesh, Material material, Vector3 position, Quaternion rotation)
    {
        GameObject obj = new GameObject(name);
        obj.transform.position = position;
        obj.transform.rotation = rotation;

        MeshFilter meshFilter = obj.AddComponent<MeshFilter>();
        meshFilter.mesh = mesh;

        MeshRenderer renderer = obj.AddComponent<MeshRenderer>();
        renderer.material = material;

        return obj;
    }

    /// <summary>
    /// Crée un matériau avec couleur
    /// </summary>
    public static Material CreateColorMaterial(Color color)
    {
        Material mat = new Material(Shader.Find("Standard"));
        mat.color = color;
        return mat;
    }
}