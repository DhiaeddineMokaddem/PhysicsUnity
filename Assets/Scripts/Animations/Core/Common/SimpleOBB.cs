using UnityEngine;
using System.Collections.Generic;
using PhysicsSimulation.Core; // for PhysicsConstants / MathUtils if needed

/// <summary>
/// Pure math oriented OBB collider with manual rigid transform (rotation + translation only).
/// Keeps a static registry for collision queries (no Unity physics used).
/// </summary>
public class SimpleOBB : MonoBehaviour
{
    [Header("OBB Shape")]
    public Vector3 halfExtents = new Vector3(1, 1, 1); // half sizes

    private ManualMatrix localToWorld;
    private ManualMatrix worldToLocal;

    private static readonly List<SimpleOBB> registry = new List<SimpleOBB>();
    public static IReadOnlyList<SimpleOBB> All => registry;

    void OnEnable()
    {
        if (!registry.Contains(this)) registry.Add(this);
        RecomputeMatrices();
    }

    void OnDisable()
    {
        registry.Remove(this);
    }

    void Update()
    {
        if (transform.hasChanged)
        {
            RecomputeMatrices();
            transform.hasChanged = false;
        }
    }

    public void RecomputeMatrices()
    {
        localToWorld = ManualMatrix.TR(transform.position, transform.rotation);
        worldToLocal = localToWorld.InverseRigid();
    }

    public Vector3 WorldToLocalPoint(Vector3 world)
    {
        return worldToLocal.MultiplyPoint(world);
    }

    public Vector3 LocalToWorldVector(Vector3 local)
    {
        return localToWorld.MultiplyVector(local);
    }

    public Vector3 LocalToWorldPoint(Vector3 local)
    {
        return localToWorld.MultiplyPoint(local);
    }

    /// <summary>
    /// Compute penetration vector (in local space) for a point inside the box. Zero if outside.
    /// Returns axis-aligned minimum translation vector to exit box.
    /// </summary>
    public Vector3 GetPenetrationLocal(Vector3 localPoint)
    {
        if (Mathf.Abs(localPoint.x) <= halfExtents.x &&
            Mathf.Abs(localPoint.y) <= halfExtents.y &&
            Mathf.Abs(localPoint.z) <= halfExtents.z)
        {
            float px = halfExtents.x - Mathf.Abs(localPoint.x);
            float py = halfExtents.y - Mathf.Abs(localPoint.y);
            float pz = halfExtents.z - Mathf.Abs(localPoint.z);
            if (px < py && px < pz)
                return new Vector3(localPoint.x < 0 ? -px : px, 0, 0);
            if (py < pz)
                return new Vector3(0, localPoint.y < 0 ? -py : py, 0);
            return new Vector3(0, 0, localPoint.z < 0 ? -pz : pz);
        }
        return Vector3.zero;
    }

    void OnDrawGizmos()
    {
        if (!Application.isPlaying) RecomputeMatrices();
        // Build a debug Matrix4x4 for drawing only
        Matrix4x4 gm = new Matrix4x4();
        gm.m00 = localToWorld.m00; gm.m01 = localToWorld.m01; gm.m02 = localToWorld.m02; gm.m03 = localToWorld.m03;
        gm.m10 = localToWorld.m10; gm.m11 = localToWorld.m11; gm.m12 = localToWorld.m12; gm.m13 = localToWorld.m13;
        gm.m20 = localToWorld.m20; gm.m21 = localToWorld.m21; gm.m22 = localToWorld.m22; gm.m23 = localToWorld.m23;
        gm.m30 = localToWorld.m30; gm.m31 = localToWorld.m31; gm.m32 = localToWorld.m32; gm.m33 = localToWorld.m33;
        Gizmos.matrix = gm;
        Gizmos.color = Color.red;
        Gizmos.DrawWireCube(Vector3.zero, halfExtents * 2f);
        Gizmos.matrix = Matrix4x4.identity;
    }
}
