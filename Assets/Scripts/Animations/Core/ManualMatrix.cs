using UnityEngine;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Manual 4x4 rigid transform matrix (rotation + translation) without using Unity's built-in matrix helpers.
    /// Supports orthonormal rotation composition from Euler angles (Z * Y * X) or Quaternion and affine inversion for rigid transforms.
    /// </summary>
    public struct ManualMatrix
    {
        // Row-major elements
        public float m00, m01, m02, m03;
        public float m10, m11, m12, m13;
        public float m20, m21, m22, m23;
        public float m30, m31, m32, m33; // normally 0,0,0,1

        public static ManualMatrix Identity => new ManualMatrix
        {
            m00 = 1, m11 = 1, m22 = 1, m33 = 1
        };

        /// <summary>
        /// Build a TR rigid transform matrix from position and euler XYZ (degrees) using manual cos/sin.
        /// Rotation order applied: Z, then Y, then X (Rz * Ry * Rx).
        /// </summary>
        public static ManualMatrix TR(Vector3 position, Vector3 eulerDeg)
        {
            float rx = eulerDeg.x * Mathf.Deg2Rad;
            float ry = eulerDeg.y * Mathf.Deg2Rad;
            float rz = eulerDeg.z * Mathf.Deg2Rad;

            float cx = Mathf.Cos(rx); float sx = Mathf.Sin(rx);
            float cy = Mathf.Cos(ry); float sy = Mathf.Sin(ry);
            float cz = Mathf.Cos(rz); float sz = Mathf.Sin(rz);

            // Individual rotation matrices (row-major)
            // Rz
            float rzz00 = cz; float rzz01 = -sz; float rzz02 = 0;
            float rzz10 = sz; float rzz11 = cz;  float rzz12 = 0;
            float rzz20 = 0;  float rzz21 = 0;   float rzz22 = 1;

            // Ry
            float ryy00 = cy;  float ryy01 = 0; float ryy02 = sy;
            float ryy10 = 0;   float ryy11 = 1; float ryy12 = 0;
            float ryy20 = -sy; float ryy21 = 0; float ryy22 = cy;

            // Rx
            float rxx00 = 1;  float rxx01 = 0;   float rxx02 = 0;
            float rxx10 = 0;  float rxx11 = cx;  float rxx12 = -sx;
            float rxx20 = 0;  float rxx21 = sx;  float rxx22 = cx;

            // Compose R = Rz * Ry * Rx
            // First temp = Rz * Ry
            float t00 = rzz00 * ryy00 + rzz01 * ryy10 + rzz02 * ryy20;
            float t01 = rzz00 * ryy01 + rzz01 * ryy11 + rzz02 * ryy21;
            float t02 = rzz00 * ryy02 + rzz01 * ryy12 + rzz02 * ryy22;

            float t10 = rzz10 * ryy00 + rzz11 * ryy10 + rzz12 * ryy20;
            float t11 = rzz10 * ryy01 + rzz11 * ryy11 + rzz12 * ryy21;
            float t12 = rzz10 * ryy02 + rzz11 * ryy12 + rzz12 * ryy22;

            float t20 = rzz20 * ryy00 + rzz21 * ryy10 + rzz22 * ryy20;
            float t21 = rzz20 * ryy01 + rzz21 * ryy11 + rzz22 * ryy21;
            float t22 = rzz20 * ryy02 + rzz21 * ryy12 + rzz22 * ryy22;

            // Final R = temp * Rx
            float r00 = t00 * rxx00 + t01 * rxx10 + t02 * rxx20;
            float r01 = t00 * rxx01 + t01 * rxx11 + t02 * rxx21;
            float r02 = t00 * rxx02 + t01 * rxx12 + t02 * rxx22;

            float r10 = t10 * rxx00 + t11 * rxx10 + t12 * rxx20;
            float r11 = t10 * rxx01 + t11 * rxx11 + t12 * rxx21;
            float r12 = t10 * rxx02 + t11 * rxx12 + t12 * rxx22;

            float r20 = t20 * rxx00 + t21 * rxx10 + t22 * rxx20;
            float r21 = t20 * rxx01 + t21 * rxx11 + t22 * rxx21;
            float r22 = t20 * rxx02 + t21 * rxx12 + t22 * rxx22;

            return new ManualMatrix
            {
                m00 = r00, m01 = r01, m02 = r02, m03 = position.x,
                m10 = r10, m11 = r11, m12 = r12, m13 = position.y,
                m20 = r20, m21 = r21, m22 = r22, m23 = position.z,
                m30 = 0,   m31 = 0,   m32 = 0,   m33 = 1
            };
        }

        /// <summary>
        /// Build a TR rigid transform matrix from position and quaternion rotation, using manual quaternion->matrix formula.
        /// </summary>
        public static ManualMatrix TR(Vector3 position, Quaternion q)
        {
            // Normalize quaternion to avoid drift
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            float x = q.x / mag;
            float y = q.y / mag;
            float z = q.z / mag;
            float w = q.w / mag;

            float xx = x * x; float yy = y * y; float zz = z * z;
            float xy = x * y; float xz = x * z; float yz = y * z;
            float wx = w * x; float wy = w * y; float wz = w * z;

            float r00 = 1f - 2f * (yy + zz);
            float r01 = 2f * (xy - wz);
            float r02 = 2f * (xz + wy);

            float r10 = 2f * (xy + wz);
            float r11 = 1f - 2f * (xx + zz);
            float r12 = 2f * (yz - wx);

            float r20 = 2f * (xz - wy);
            float r21 = 2f * (yz + wx);
            float r22 = 1f - 2f * (xx + yy);

            return new ManualMatrix
            {
                m00 = r00, m01 = r01, m02 = r02, m03 = position.x,
                m10 = r10, m11 = r11, m12 = r12, m13 = position.y,
                m20 = r20, m21 = r21, m22 = r22, m23 = position.z,
                m30 = 0,   m31 = 0,   m32 = 0,   m33 = 1
            };
        }

        /// <summary>
        /// Invert a rigid TR matrix (assumes upper 3x3 is rotation and orthonormal).
        /// </summary>
        public ManualMatrix InverseRigid()
        {
            // Rotation transpose
            ManualMatrix inv = Identity;
            inv.m00 = m00; inv.m01 = m10; inv.m02 = m20;
            inv.m10 = m01; inv.m11 = m11; inv.m12 = m21;
            inv.m20 = m02; inv.m21 = m12; inv.m22 = m22;
            // Translation inverse: -R^T * t
            Vector3 t = new Vector3(m03, m13, m23);
            Vector3 it = MultiplyRotationOnly(inv, -t); // using newly set rotation part
            inv.m03 = it.x; inv.m13 = it.y; inv.m23 = it.z;
            inv.m30 = 0; inv.m31 = 0; inv.m32 = 0; inv.m33 = 1;
            return inv;
        }

        private static Vector3 MultiplyRotationOnly(ManualMatrix m, Vector3 v)
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
}

