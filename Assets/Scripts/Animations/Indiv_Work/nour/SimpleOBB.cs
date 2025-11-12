using UnityEngine;

/// <summary>
/// Simple OBB collider: can be attached to any GameObject.
/// Other scripts can query for collisions manually.
/// </summary>
public class SimpleOBB : MonoBehaviour
{
    public Vector3 halfExtents = new Vector3(1, 1, 1);

    public Matrix4x4 WorldToLocalMatrix => transform.worldToLocalMatrix;
    public Matrix4x4 LocalToWorldMatrix => transform.localToWorldMatrix;

    public Vector3 WorldToLocalPoint(Vector3 worldPoint) => WorldToLocalMatrix.MultiplyPoint3x4(worldPoint);
    public Vector3 LocalToWorldVector(Vector3 localVec) => LocalToWorldMatrix.MultiplyVector(localVec);

    public Vector3 GetPenetration(Vector3 localPoint)
    {
        Vector3 pen = Vector3.zero;
        if (Mathf.Abs(localPoint.x) < halfExtents.x &&
            Mathf.Abs(localPoint.y) < halfExtents.y &&
            Mathf.Abs(localPoint.z) < halfExtents.z)
        {
            float px = halfExtents.x - Mathf.Abs(localPoint.x);
            float py = halfExtents.y - Mathf.Abs(localPoint.y);
            float pz = halfExtents.z - Mathf.Abs(localPoint.z);

            if (px < py && px < pz)
                pen = new Vector3(localPoint.x < 0 ? -px : px, 0, 0);
            else if (py < pz)
                pen = new Vector3(0, localPoint.y < 0 ? -py : py, 0);
            else
                pen = new Vector3(0, 0, localPoint.z < 0 ? -pz : pz);
        }
        return pen;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.DrawWireCube(Vector3.zero, halfExtents * 2);
    }
}
