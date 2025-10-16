using UnityEngine;

public class RayonIntersection : MonoBehaviour
{
    private GameObject cube;
    private GameObject capsule;

    void Start()
    {
        cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = new Vector3(0, 0, 0);

        capsule = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        capsule.transform.position = new Vector3(0, 2, -5);

        Mesh mesh = cube.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        Vector3 p0 = cube.transform.TransformPoint(vertices[triangles[0]]);
        Vector3 p1 = cube.transform.TransformPoint(vertices[triangles[1]]);
        Vector3 p2 = cube.transform.TransformPoint(vertices[triangles[2]]);

        Debug.Log("Triangle choisi :");
        Debug.Log("p0 = " + p0);
        Debug.Log("p1 = " + p1);
        Debug.Log("p2 = " + p2);

        Vector3 normal = Vector3.Cross(p1 - p0, p2 - p0).normalized;
        float d = -Vector3.Dot(normal, p0);
        Debug.Log($"Equation du plan : {normal.x}x + {normal.y}y + {normal.z}z + {d} = 0");

        Vector3 S = capsule.transform.position;             
        Vector3 V = new Vector3(0, 0, 1).normalized;        

        float denom = Vector3.Dot(normal, V);
        if (Mathf.Abs(denom) > 1e-6f)
        {
            float t = -(Vector3.Dot(normal, S) + d) / denom;
            if (t >= 0)
            {
                Vector3 P = S + t * V;
                Debug.Log(" Point d’intersection : " + P);

                if (PointInTriangle(P, p0, p1, p2))
                    Debug.Log("Le point est a  du polygone.");
                else
                    Debug.Log("Le point est a  du polygone.");
            }
            else
            {
                Debug.Log(" Le rayon ne touche pas le plan (intersection derrière la capsule).");
            }
        }
        else
        {
            Debug.Log(" Rayon parallèle au plan : aucune intersection.");
        }
    }

    //Fonction : Test barycentrique
    bool PointInTriangle(Vector3 P, Vector3 A, Vector3 B, Vector3 C)
    {
        Vector3 v0 = C - A;
        Vector3 v1 = B - A;
        Vector3 v2 = P - A;

        float dot00 = Vector3.Dot(v0, v0);
        float dot01 = Vector3.Dot(v0, v1);
        float dot02 = Vector3.Dot(v0, v2);
        float dot11 = Vector3.Dot(v1, v1);
        float dot12 = Vector3.Dot(v1, v2);

        float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
        float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }
}
