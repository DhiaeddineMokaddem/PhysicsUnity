using UnityEngine;
using System.Collections.Generic;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Utility class for procedural mesh generation and manipulation
    /// Useful for soft body physics, cloth simulation, and dynamic meshes
    /// </summary>
    public static class MeshUtils
    {
        #region Mesh Generation
        /// <summary>
        /// Creates a plane mesh with specified dimensions and resolution
        /// </summary>
        public static Mesh CreatePlaneMesh(float width, float height, int widthSegments, int heightSegments)
        {
            Mesh mesh = new Mesh();
            mesh.name = "Procedural Plane";

            int vertexCount = (widthSegments + 1) * (heightSegments + 1);
            Vector3[] vertices = new Vector3[vertexCount];
            Vector2[] uvs = new Vector2[vertexCount];
            int[] triangles = new int[widthSegments * heightSegments * 6];

            // Generate vertices and UVs
            int vertIndex = 0;
            for (int y = 0; y <= heightSegments; y++)
            {
                for (int x = 0; x <= widthSegments; x++)
                {
                    float xPos = (x / (float)widthSegments - 0.5f) * width;
                    float yPos = (y / (float)heightSegments - 0.5f) * height;
                    
                    vertices[vertIndex] = new Vector3(xPos, 0, yPos);
                    uvs[vertIndex] = new Vector2(x / (float)widthSegments, y / (float)heightSegments);
                    vertIndex++;
                }
            }

            // Generate triangles
            int triIndex = 0;
            for (int y = 0; y < heightSegments; y++)
            {
                for (int x = 0; x < widthSegments; x++)
                {
                    int bottomLeft = y * (widthSegments + 1) + x;
                    int bottomRight = bottomLeft + 1;
                    int topLeft = bottomLeft + widthSegments + 1;
                    int topRight = topLeft + 1;

                    triangles[triIndex++] = bottomLeft;
                    triangles[triIndex++] = topLeft;
                    triangles[triIndex++] = bottomRight;

                    triangles[triIndex++] = bottomRight;
                    triangles[triIndex++] = topLeft;
                    triangles[triIndex++] = topRight;
                }
            }

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            return mesh;
        }

        /// <summary>
        /// Creates a sphere mesh with specified radius and resolution
        /// </summary>
        public static Mesh CreateSphereMesh(float radius, int segments)
        {
            Mesh mesh = new Mesh();
            mesh.name = "Procedural Sphere";

            List<Vector3> vertices = new List<Vector3>();
            List<int> triangles = new List<int>();
            List<Vector2> uvs = new List<Vector2>();

            for (int lat = 0; lat <= segments; lat++)
            {
                float theta = lat * Mathf.PI / segments;
                float sinTheta = Mathf.Sin(theta);
                float cosTheta = Mathf.Cos(theta);

                for (int lon = 0; lon <= segments; lon++)
                {
                    float phi = lon * 2 * Mathf.PI / segments;
                    float sinPhi = Mathf.Sin(phi);
                    float cosPhi = Mathf.Cos(phi);

                    Vector3 vertex = new Vector3(
                        radius * sinTheta * cosPhi,
                        radius * cosTheta,
                        radius * sinTheta * sinPhi
                    );

                    vertices.Add(vertex);
                    uvs.Add(new Vector2(lon / (float)segments, lat / (float)segments));
                }
            }

            for (int lat = 0; lat < segments; lat++)
            {
                for (int lon = 0; lon < segments; lon++)
                {
                    int current = lat * (segments + 1) + lon;
                    int next = current + segments + 1;

                    triangles.Add(current);
                    triangles.Add(next);
                    triangles.Add(current + 1);

                    triangles.Add(current + 1);
                    triangles.Add(next);
                    triangles.Add(next + 1);
                }
            }

            mesh.vertices = vertices.ToArray();
            mesh.triangles = triangles.ToArray();
            mesh.uv = uvs.ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            return mesh;
        }

        /// <summary>
        /// Creates a cube mesh with specified dimensions
        /// </summary>
        public static Mesh CreateCubeMesh(Vector3 size)
        {
            Mesh mesh = new Mesh();
            mesh.name = "Procedural Cube";

            Vector3 halfSize = size * 0.5f;

            Vector3[] vertices = new Vector3[]
            {
                // Front
                new Vector3(-halfSize.x, -halfSize.y, halfSize.z),
                new Vector3(halfSize.x, -halfSize.y, halfSize.z),
                new Vector3(halfSize.x, halfSize.y, halfSize.z),
                new Vector3(-halfSize.x, halfSize.y, halfSize.z),
                // Back
                new Vector3(-halfSize.x, -halfSize.y, -halfSize.z),
                new Vector3(halfSize.x, -halfSize.y, -halfSize.z),
                new Vector3(halfSize.x, halfSize.y, -halfSize.z),
                new Vector3(-halfSize.x, halfSize.y, -halfSize.z),
            };

            int[] triangles = new int[]
            {
                0, 2, 1, 0, 3, 2, // Front
                5, 6, 4, 4, 6, 7, // Back
                4, 7, 0, 0, 7, 3, // Left
                1, 2, 5, 5, 2, 6, // Right
                3, 6, 2, 3, 7, 6, // Top
                4, 1, 5, 4, 0, 1  // Bottom
            };

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            return mesh;
        }
        #endregion

        #region Mesh Manipulation
        /// <summary>
        /// Subdivides a mesh for higher resolution
        /// </summary>
        public static void SubdivideMesh(Mesh mesh)
        {
            Vector3[] oldVertices = mesh.vertices;
            int[] oldTriangles = mesh.triangles;
            Vector2[] oldUVs = mesh.uv;

            List<Vector3> newVertices = new List<Vector3>(oldVertices);
            List<int> newTriangles = new List<int>();
            List<Vector2> newUVs = new List<Vector2>(oldUVs);

            Dictionary<string, int> midPointCache = new Dictionary<string, int>();

            for (int i = 0; i < oldTriangles.Length; i += 3)
            {
                int i1 = oldTriangles[i];
                int i2 = oldTriangles[i + 1];
                int i3 = oldTriangles[i + 2];

                int a = GetMidPoint(i1, i2, oldVertices, oldUVs, newVertices, newUVs, midPointCache);
                int b = GetMidPoint(i2, i3, oldVertices, oldUVs, newVertices, newUVs, midPointCache);
                int c = GetMidPoint(i3, i1, oldVertices, oldUVs, newVertices, newUVs, midPointCache);

                newTriangles.AddRange(new int[] { i1, a, c });
                newTriangles.AddRange(new int[] { i2, b, a });
                newTriangles.AddRange(new int[] { i3, c, b });
                newTriangles.AddRange(new int[] { a, b, c });
            }

            mesh.vertices = newVertices.ToArray();
            mesh.triangles = newTriangles.ToArray();
            mesh.uv = newUVs.ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
        }

        private static int GetMidPoint(int i1, int i2, Vector3[] oldVerts, Vector2[] oldUVs, 
            List<Vector3> newVerts, List<Vector2> newUVs, Dictionary<string, int> cache)
        {
            string key = Mathf.Min(i1, i2) + "_" + Mathf.Max(i1, i2);

            if (cache.ContainsKey(key))
                return cache[key];

            Vector3 midPoint = (oldVerts[i1] + oldVerts[i2]) * 0.5f;
            Vector2 midUV = (oldUVs[i1] + oldUVs[i2]) * 0.5f;

            int index = newVerts.Count;
            newVerts.Add(midPoint);
            newUVs.Add(midUV);
            cache[key] = index;

            return index;
        }

        /// <summary>
        /// Calculates tangents for a mesh (needed for normal mapping)
        /// </summary>
        public static void CalculateTangents(Mesh mesh)
        {
            int[] triangles = mesh.triangles;
            Vector3[] vertices = mesh.vertices;
            Vector2[] uv = mesh.uv;
            Vector3[] normals = mesh.normals;

            int triangleCount = triangles.Length / 3;
            int vertexCount = vertices.Length;

            Vector3[] tan1 = new Vector3[vertexCount];
            Vector3[] tan2 = new Vector3[vertexCount];
            Vector4[] tangents = new Vector4[vertexCount];

            for (int a = 0; a < triangleCount; a++)
            {
                int i1 = triangles[a * 3 + 0];
                int i2 = triangles[a * 3 + 1];
                int i3 = triangles[a * 3 + 2];

                Vector3 v1 = vertices[i1];
                Vector3 v2 = vertices[i2];
                Vector3 v3 = vertices[i3];

                Vector2 w1 = uv[i1];
                Vector2 w2 = uv[i2];
                Vector2 w3 = uv[i3];

                float x1 = v2.x - v1.x;
                float x2 = v3.x - v1.x;
                float y1 = v2.y - v1.y;
                float y2 = v3.y - v1.y;
                float z1 = v2.z - v1.z;
                float z2 = v3.z - v1.z;

                float s1 = w2.x - w1.x;
                float s2 = w3.x - w1.x;
                float t1 = w2.y - w1.y;
                float t2 = w3.y - w1.y;

                float r = 1.0f / (s1 * t2 - s2 * t1);
                Vector3 sdir = new Vector3((t2 * x1 - t1 * x2) * r, (t2 * y1 - t1 * y2) * r, (t2 * z1 - t1 * z2) * r);
                Vector3 tdir = new Vector3((s1 * x2 - s2 * x1) * r, (s1 * y2 - s2 * y1) * r, (s1 * z2 - s2 * z1) * r);

                tan1[i1] += sdir;
                tan1[i2] += sdir;
                tan1[i3] += sdir;

                tan2[i1] += tdir;
                tan2[i2] += tdir;
                tan2[i3] += tdir;
            }

            for (int a = 0; a < vertexCount; a++)
            {
                Vector3 n = normals[a];
                Vector3 t = tan1[a];

                Vector3.OrthoNormalize(ref n, ref t);
                tangents[a].x = t.x;
                tangents[a].y = t.y;
                tangents[a].z = t.z;

                tangents[a].w = (Vector3.Dot(Vector3.Cross(n, t), tan2[a]) < 0.0f) ? -1.0f : 1.0f;
            }

            mesh.tangents = tangents;
        }
        #endregion

        #region Mesh Analysis
        /// <summary>
        /// Gets all triangle centers in a mesh
        /// </summary>
        public static Vector3[] GetTriangleCenters(Mesh mesh)
        {
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            int triangleCount = triangles.Length / 3;
            Vector3[] centers = new Vector3[triangleCount];

            for (int i = 0; i < triangleCount; i++)
            {
                int idx = i * 3;
                Vector3 v1 = vertices[triangles[idx]];
                Vector3 v2 = vertices[triangles[idx + 1]];
                Vector3 v3 = vertices[triangles[idx + 2]];
                centers[i] = (v1 + v2 + v3) / 3f;
            }

            return centers;
        }

        /// <summary>
        /// Calculates the surface area of a mesh
        /// </summary>
        public static float CalculateSurfaceArea(Mesh mesh)
        {
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            float totalArea = 0f;

            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 v1 = vertices[triangles[i]];
                Vector3 v2 = vertices[triangles[i + 1]];
                Vector3 v3 = vertices[triangles[i + 2]];

                Vector3 side1 = v2 - v1;
                Vector3 side2 = v3 - v1;
                totalArea += Vector3.Cross(side1, side2).magnitude * 0.5f;
            }

            return totalArea;
        }

        /// <summary>
        /// Calculates the volume of a closed mesh
        /// </summary>
        public static float CalculateVolume(Mesh mesh)
        {
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            float volume = 0f;

            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 v1 = vertices[triangles[i]];
                Vector3 v2 = vertices[triangles[i + 1]];
                Vector3 v3 = vertices[triangles[i + 2]];

                volume += SignedVolumeOfTriangle(v1, v2, v3);
            }

            return Mathf.Abs(volume);
        }

        private static float SignedVolumeOfTriangle(Vector3 p1, Vector3 p2, Vector3 p3)
        {
            return Vector3.Dot(p1, Vector3.Cross(p2, p3)) / 6.0f;
        }
        #endregion
    }
}

