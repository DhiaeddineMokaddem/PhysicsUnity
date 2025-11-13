using UnityEngine;
using System.Collections.Generic;
using PhysicsUnity.Core; // for PhysicsConstants / MathUtils if needed

/// <summary>
/// Pure math oriented OBB collider with manual rigid transform (rotation + translation only).
/// Keeps a static registry for collision queries (no Unity physics used).
/// </summary>
public class SimpleOBB : MonoBehaviour
{
    #region Manual Matrix (Rigid Transform)
    private struct RigidMatrix
    {
        public float m00, m01, m02, m03;
        public float m10, m11, m12, m13;
        public float m20, m21, m22, m23;
        public float m30, m31, m32, m33; // always 0,0,0,1 for rigid

        public static RigidMatrix Identity => new RigidMatrix { m00 = 1, m11 = 1, m22 = 1, m33 = 1 };

        public static RigidMatrix TR(Vector3 position, Quaternion q)
        {
            // Normalize quaternion
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (mag < PhysicsConstants.EPSILON) q = Quaternion.identity; else { q.x /= mag; q.y /= mag; q.z /= mag; q.w /= mag; }
            float x = q.x, y = q.y, z = q.z, w = q.w;
            float xx = x * x, yy = y * y, zz = z * z;
            float xy = x * y, xz = x * z, yz = y * z;
            float wx = w * x, wy = w * y, wz = w * z;
            float r00 = 1f - 2f * (yy + zz);
            float r01 = 2f * (xy - wz);
            float r02 = 2f * (xz + wy);
            float r10 = 2f * (xy + wz);
            float r11 = 1f - 2f * (xx + zz);
            float r12 = 2f * (yz - wx);
            float r20 = 2f * (xz - wy);
            float r21 = 2f * (yz + wx);
            float r22 = 1f - 2f * (xx + yy);
            return new RigidMatrix
            {
                m00 = r00, m01 = r01, m02 = r02, m03 = position.x,
                m10 = r10, m11 = r11, m12 = r12, m13 = position.y,
                m20 = r20, m21 = r21, m22 = r22, m23 = position.z,
                m30 = 0,   m31 = 0,   m32 = 0,   m33 = 1
            };
        }

        public RigidMatrix InverseRigid()
        {
            // Transpose rotation part
            RigidMatrix inv = Identity;
            inv.m00 = m00; inv.m01 = m10; inv.m02 = m20;
            inv.m10 = m01; inv.m11 = m11; inv.m12 = m21;
            inv.m20 = m02; inv.m21 = m12; inv.m22 = m22;
            // Invert translation: -R^T * t
            Vector3 t = new Vector3(m03, m13, m23);
            Vector3 it = MultiplyRotationOnly(inv, -t);
            inv.m03 = it.x; inv.m13 = it.y; inv.m23 = it.z;
            inv.m30 = 0; inv.m31 = 0; inv.m32 = 0; inv.m33 = 1;
            return inv;
        }

        private static Vector3 MultiplyRotationOnly(RigidMatrix m, Vector3 v)
        {
            return new Vector3(
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
            );
        }

        public Vector3 MultiplyPoint(Vector3 p)
        {
            return new Vector3(
                m00 * p.x + m01 * p.y + m02 * p.z + m03,
                m10 * p.x + m11 * p.y + m12 * p.z + m13,
                m20 * p.x + m21 * p.y + m22 * p.z + m23
            );
        }

        public Vector3 MultiplyVector(Vector3 v)
        {
            return new Vector3(
                m00 * v.x + m01 * v.y + m02 * v.z,
                m10 * v.x + m11 * v.y + m12 * v.z,
                m20 * v.x + m21 * v.y + m22 * v.z
            );
        }
    }
    #endregion

    [Header("OBB Shape")]
    public Vector3 halfExtents = new Vector3(1, 1, 1); // half sizes

    private RigidMatrix localToWorld;
    private RigidMatrix worldToLocal;

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
        localToWorld = RigidMatrix.TR(transform.position, transform.rotation);
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

