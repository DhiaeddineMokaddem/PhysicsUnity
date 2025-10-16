using UnityEngine;

public class MatrixRaycastTesterDhiaeddineMokaddem : MonoBehaviour
{
    [Header("Ray Settings")]
    public Vector3 direction = Vector3.forward;
    public Vector3 start = Vector3.zero;
    public float rayLength = 10f;

    [Header("References")]
    public MatrixCubeMeshDhiadeddineMokaddem cubeMesh;

    private Vector3? hitPoint;

    void OnDrawGizmos()
    {
        if (cubeMesh == null) return;

        Vector3 dir = direction.normalized;
        Vector3 finish = start + dir * rayLength;

        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(start, finish);

        hitPoint = FindRayIntersection(start, dir);

        if (hitPoint.HasValue)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(hitPoint.Value, 0.1f);
        }
    }

    Vector3? FindRayIntersection(Vector3 origin, Vector3 dir)
    {
        Vector3[] verts = cubeMesh.GetTransformedVertices();
        int[] faces = cubeMesh.triangles;
        float nearestDist = float.MaxValue;
        Vector3? closestHit = null;

        for (int i = 0; i < faces.Length; i += 3)
        {
            Vector3 p0 = verts[faces[i]];
            Vector3 p1 = verts[faces[i + 1]];
            Vector3 p2 = verts[faces[i + 2]];

            if (RayHitsTriangle(origin, dir, p0, p1, p2, out Vector3 intersection))
            {
                float dist = ((intersection - origin).x * (intersection - origin).x +
                              (intersection - origin).y * (intersection - origin).y +
                              (intersection - origin).z * (intersection - origin).z);
                if (dist < nearestDist)
                {
                    nearestDist = dist;
                    closestHit = intersection;
                }
            }
        }

        return closestHit;
    }

    bool RayHitsTriangle(Vector3 rayStart, Vector3 rayDir, Vector3 a, Vector3 b, Vector3 c, out Vector3 hit)
    {
        hit = Vector3.zero;

        // Plane normal
        Vector3 edge1 = b - a;
        Vector3 edge2 = c - a;
        Vector3 normal = new Vector3(
            edge1.y * edge2.z - edge1.z * edge2.y,
            edge1.z * edge2.x - edge1.x * edge2.z,
            edge1.x * edge2.y - edge1.y * edge2.x
        );

        float denom = normal.x * rayDir.x + normal.y * rayDir.y + normal.z * rayDir.z;
        if (Mathf.Abs(denom) < 1e-6f) return false; // Ray parallel to plane

        float d = -(normal.x * a.x + normal.y * a.y + normal.z * a.z);
        float t = -(normal.x * rayStart.x + normal.y * rayStart.y + normal.z * rayStart.z + d) /
                  denom;
        if (t < 0f) return false;

        Vector3 pointOnPlane = rayStart + rayDir * t;

        if (IsInsideTriangle(pointOnPlane, a, b, c))
        {
            hit = pointOnPlane;
            return true;
        }

        return false;
    }

    // Checks if a point is inside a triangle
    bool IsInsideTriangle(Vector3 p, Vector3 v0, Vector3 v1, Vector3 v2)
    {
        Vector3 side0 = v2 - v0;
        Vector3 side1 = v1 - v0;
        Vector3 toPoint = p - v0;

        float dot00 = side0.x * side0.x + side0.y * side0.y + side0.z * side0.z;
        float dot01 = side0.x * side1.x + side0.y * side1.y + side0.z * side1.z;
        float dot02 = side0.x * toPoint.x + side0.y * toPoint.y + side0.z * toPoint.z;
        float dot11 = side1.x * side1.x + side1.y * side1.y + side1.z * side1.z;
        float dot12 = side1.x * toPoint.x + side1.y * toPoint.y + side1.z * toPoint.z;

        float invDet = 1f / (dot00 * dot11 - dot01 * dot01);

        float u = (dot11 * dot02 - dot01 * dot12) * invDet;
        float v = (dot00 * dot12 - dot01 * dot02) * invDet;

        return u >= 0f && v >= 0f && (u + v) <= 1f;
    }
}
