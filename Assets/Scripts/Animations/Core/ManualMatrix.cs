// Import Unity's Vector3, Quaternion, and Mathf utilities
using UnityEngine;

// Namespace for core physics simulation utilities
namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Manual 4x4 rigid transform matrix (rotation + translation) without using Unity's built-in matrix helpers.
    /// Supports orthonormal rotation composition from Euler angles (Z * Y * X) or Quaternion and affine inversion for rigid transforms.
    /// </summary>
    public struct ManualMatrix
    {
        // Row 0 of the 4x4 matrix: [m00 m01 m02 m03] where m03 is X translation
        public float m00, m01, m02, m03;
        // Row 1 of the 4x4 matrix: [m10 m11 m12 m13] where m13 is Y translation
        public float m10, m11, m12, m13;
        // Row 2 of the 4x4 matrix: [m20 m21 m22 m23] where m23 is Z translation
        public float m20, m21, m22, m23;
        // Row 3 of the 4x4 matrix: [m30 m31 m32 m33] - normally [0 0 0 1] for affine transforms
        public float m30, m31, m32, m33;

        // Static property that returns an identity matrix (no rotation, no translation, unit scale)
        public static ManualMatrix Identity => new ManualMatrix
        {
            m00 = 1, m11 = 1, m22 = 1, m33 = 1 // Diagonal elements set to 1, rest are 0 by default
        };

        /// <summary>
        /// Build a TR rigid transform matrix from position and euler XYZ (degrees) using manual cos/sin.
        /// Rotation order applied: Z, then Y, then X (Rz * Ry * Rx).
        /// </summary>
        public static ManualMatrix TR(Vector3 position, Vector3 eulerDeg)
        {
            // Convert X rotation from degrees to radians for trig functions
            float rx = eulerDeg.x * Mathf.Deg2Rad;
            // Convert Y rotation from degrees to radians for trig functions
            float ry = eulerDeg.y * Mathf.Deg2Rad;
            // Convert Z rotation from degrees to radians for trig functions
            float rz = eulerDeg.z * Mathf.Deg2Rad;

            // Calculate cosine and sine for X rotation
            float cx = Mathf.Cos(rx); float sx = Mathf.Sin(rx);
            // Calculate cosine and sine for Y rotation
            float cy = Mathf.Cos(ry); float sy = Mathf.Sin(ry);
            // Calculate cosine and sine for Z rotation
            float cz = Mathf.Cos(rz); float sz = Mathf.Sin(rz);

            // Build individual rotation matrices (row-major layout)
            // Z-axis rotation matrix Rz: rotates around Z axis
            // Row 0 of Rz: [cos(z) -sin(z) 0]
            float rzz00 = cz; float rzz01 = -sz; float rzz02 = 0;
            // Row 1 of Rz: [sin(z) cos(z) 0]
            float rzz10 = sz; float rzz11 = cz;  float rzz12 = 0;
            // Row 2 of Rz: [0 0 1]
            float rzz20 = 0;  float rzz21 = 0;   float rzz22 = 1;

            // Y-axis rotation matrix Ry: rotates around Y axis
            // Row 0 of Ry: [cos(y) 0 sin(y)]
            float ryy00 = cy;  float ryy01 = 0; float ryy02 = sy;
            // Row 1 of Ry: [0 1 0]
            float ryy10 = 0;   float ryy11 = 1; float ryy12 = 0;
            // Row 2 of Ry: [-sin(y) 0 cos(y)]
            float ryy20 = -sy; float ryy21 = 0; float ryy22 = cy;

            // X-axis rotation matrix Rx: rotates around X axis
            // Row 0 of Rx: [1 0 0]
            float rxx00 = 1;  float rxx01 = 0;   float rxx02 = 0;
            // Row 1 of Rx: [0 cos(x) -sin(x)]
            float rxx10 = 0;  float rxx11 = cx;  float rxx12 = -sx;
            // Row 2 of Rx: [0 sin(x) cos(x)]
            float rxx20 = 0;  float rxx21 = sx;  float rxx22 = cx;

            // Compose final rotation matrix R = Rz * Ry * Rx
            // First intermediate result: temp = Rz * Ry
            // Calculate temp row 0, column 0: dot product of Rz row 0 with Ry column 0
            float t00 = rzz00 * ryy00 + rzz01 * ryy10 + rzz02 * ryy20;
            // Calculate temp row 0, column 1: dot product of Rz row 0 with Ry column 1
            float t01 = rzz00 * ryy01 + rzz01 * ryy11 + rzz02 * ryy21;
            // Calculate temp row 0, column 2: dot product of Rz row 0 with Ry column 2
            float t02 = rzz00 * ryy02 + rzz01 * ryy12 + rzz02 * ryy22;

            // Calculate temp row 1, column 0: dot product of Rz row 1 with Ry column 0
            float t10 = rzz10 * ryy00 + rzz11 * ryy10 + rzz12 * ryy20;
            // Calculate temp row 1, column 1: dot product of Rz row 1 with Ry column 1
            float t11 = rzz10 * ryy01 + rzz11 * ryy11 + rzz12 * ryy21;
            // Calculate temp row 1, column 2: dot product of Rz row 1 with Ry column 2
            float t12 = rzz10 * ryy02 + rzz11 * ryy12 + rzz12 * ryy22;

            // Calculate temp row 2, column 0: dot product of Rz row 2 with Ry column 0
            float t20 = rzz20 * ryy00 + rzz21 * ryy10 + rzz22 * ryy20;
            // Calculate temp row 2, column 1: dot product of Rz row 2 with Ry column 1
            float t21 = rzz20 * ryy01 + rzz21 * ryy11 + rzz22 * ryy21;
            // Calculate temp row 2, column 2: dot product of Rz row 2 with Ry column 2
            float t22 = rzz20 * ryy02 + rzz21 * ryy12 + rzz22 * ryy22;

            // Final rotation matrix: R = temp * Rx
            // Calculate R row 0, column 0: dot product of temp row 0 with Rx column 0
            float r00 = t00 * rxx00 + t01 * rxx10 + t02 * rxx20;
            // Calculate R row 0, column 1: dot product of temp row 0 with Rx column 1
            float r01 = t00 * rxx01 + t01 * rxx11 + t02 * rxx21;
            // Calculate R row 0, column 2: dot product of temp row 0 with Rx column 2
            float r02 = t00 * rxx02 + t01 * rxx12 + t02 * rxx22;

            // Calculate R row 1, column 0: dot product of temp row 1 with Rx column 0
            float r10 = t10 * rxx00 + t11 * rxx10 + t12 * rxx20;
            // Calculate R row 1, column 1: dot product of temp row 1 with Rx column 1
            float r11 = t10 * rxx01 + t11 * rxx11 + t12 * rxx21;
            // Calculate R row 1, column 2: dot product of temp row 1 with Rx column 2
            float r12 = t10 * rxx02 + t11 * rxx12 + t12 * rxx22;

            // Calculate R row 2, column 0: dot product of temp row 2 with Rx column 0
            float r20 = t20 * rxx00 + t21 * rxx10 + t22 * rxx20;
            // Calculate R row 2, column 1: dot product of temp row 2 with Rx column 1
            float r21 = t20 * rxx01 + t21 * rxx11 + t22 * rxx21;
            // Calculate R row 2, column 2: dot product of temp row 2 with Rx column 2
            float r22 = t20 * rxx02 + t21 * rxx12 + t22 * rxx22;

            // Create and return the final 4x4 transformation matrix
            return new ManualMatrix
            {
                // Row 0: rotation components r00, r01, r02 and X translation
                m00 = r00, m01 = r01, m02 = r02, m03 = position.x,
                // Row 1: rotation components r10, r11, r12 and Y translation
                m10 = r10, m11 = r11, m12 = r12, m13 = position.y,
                // Row 2: rotation components r20, r21, r22 and Z translation
                m20 = r20, m21 = r21, m22 = r22, m23 = position.z,
                // Row 3: homogeneous coordinates [0 0 0 1]
                m30 = 0,   m31 = 0,   m32 = 0,   m33 = 1
            };
        }

        /// <summary>
        /// Build a TR rigid transform matrix from position and quaternion rotation, using manual quaternion->matrix formula.
        /// </summary>
        public static ManualMatrix TR(Vector3 position, Quaternion q)
        {
            // Calculate the magnitude (length) of the quaternion for normalization
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            // Normalize X component by dividing by magnitude to avoid drift
            float x = q.x / mag;
            // Normalize Y component by dividing by magnitude to avoid drift
            float y = q.y / mag;
            // Normalize Z component by dividing by magnitude to avoid drift
            float z = q.z / mag;
            // Normalize W component by dividing by magnitude to avoid drift
            float w = q.w / mag;

            // Pre-calculate squared components for rotation matrix formula
            float xx = x * x; float yy = y * y; float zz = z * z;
            // Pre-calculate XY, XZ, YZ products for rotation matrix formula
            float xy = x * y; float xz = x * z; float yz = y * z;
            // Pre-calculate WX, WY, WZ products for rotation matrix formula
            float wx = w * x; float wy = w * y; float wz = w * z;

            // Convert quaternion to rotation matrix using standard formula
            // Row 0, Column 0: 1 - 2(yy + zz)
            float r00 = 1f - 2f * (yy + zz);
            // Row 0, Column 1: 2(xy - wz)
            float r01 = 2f * (xy - wz);
            // Row 0, Column 2: 2(xz + wy)
            float r02 = 2f * (xz + wy);

            // Row 1, Column 0: 2(xy + wz)
            float r10 = 2f * (xy + wz);
            // Row 1, Column 1: 1 - 2(xx + zz)
            float r11 = 1f - 2f * (xx + zz);
            // Row 1, Column 2: 2(yz - wx)
            float r12 = 2f * (yz - wx);

            // Row 2, Column 0: 2(xz - wy)
            float r20 = 2f * (xz - wy);
            // Row 2, Column 1: 2(yz + wx)
            float r21 = 2f * (yz + wx);
            // Row 2, Column 2: 1 - 2(xx + yy)
            float r22 = 1f - 2f * (xx + yy);

            // Create and return the final 4x4 transformation matrix
            return new ManualMatrix
            {
                // Row 0: rotation components r00, r01, r02 and X translation
                m00 = r00, m01 = r01, m02 = r02, m03 = position.x,
                // Row 1: rotation components r10, r11, r12 and Y translation
                m10 = r10, m11 = r11, m12 = r12, m13 = position.y,
                // Row 2: rotation components r20, r21, r22 and Z translation
                m20 = r20, m21 = r21, m22 = r22, m23 = position.z,
                // Row 3: homogeneous coordinates [0 0 0 1]
                m30 = 0,   m31 = 0,   m32 = 0,   m33 = 1
            };
        }

        /// <summary>
        /// Invert a rigid TR matrix (assumes upper 3x3 is rotation and orthonormal).
        /// </summary>
        public ManualMatrix InverseRigid()
        {
            // Start with identity matrix to build the inverse
            ManualMatrix inv = Identity;
            // Transpose the rotation part: swap m00 with m00 (no change), m01 with m10, m02 with m20
            inv.m00 = m00; inv.m01 = m10; inv.m02 = m20;
            // Continue transposing: m10 with m01, m11 with m11 (no change), m12 with m21
            inv.m10 = m01; inv.m11 = m11; inv.m12 = m21;
            // Finish transposing: m20 with m02, m21 with m12, m22 with m22 (no change)
            inv.m20 = m02; inv.m21 = m12; inv.m22 = m22;
            // Extract the translation vector from the original matrix
            Vector3 t = new Vector3(m03, m13, m23);
            // Calculate inverse translation: -R^T * t using the transposed rotation part
            Vector3 it = MultiplyRotationOnly(inv, -t);
            // Set the inverse translation components in the result matrix
            inv.m03 = it.x; inv.m13 = it.y; inv.m23 = it.z;
            // Set the bottom row to standard homogeneous coordinates [0 0 0 1]
            inv.m30 = 0; inv.m31 = 0; inv.m32 = 0; inv.m33 = 1;
            // Return the inverted rigid transformation matrix
            return inv;
        }

        // Helper method to multiply only the rotation part (3x3 upper-left) of a matrix with a vector
        private static Vector3 MultiplyRotationOnly(ManualMatrix m, Vector3 v)
        {
            // Create new vector with transformed components
            return new Vector3(
                // X component: dot product of matrix row 0 with vector
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                // Y component: dot product of matrix row 1 with vector
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                // Z component: dot product of matrix row 2 with vector
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
            );
        }

        // Transform a point (position) by applying both rotation and translation
        public Vector3 MultiplyPoint(Vector3 p)
        {
            // Create new vector with transformed point
            return new Vector3(
                // X component: row 0 dot product with point + X translation
                m00 * p.x + m01 * p.y + m02 * p.z + m03,
                // Y component: row 1 dot product with point + Y translation
                m10 * p.x + m11 * p.y + m12 * p.z + m13,
                // Z component: row 2 dot product with point + Z translation
                m20 * p.x + m21 * p.y + m22 * p.z + m23
            );
        }

        // Transform a vector (direction) by applying only rotation, no translation
        public Vector3 MultiplyVector(Vector3 v)
        {
            // Create new vector with transformed direction
            return new Vector3(
                // X component: row 0 dot product with vector (no translation)
                m00 * v.x + m01 * v.y + m02 * v.z,
                // Y component: row 1 dot product with vector (no translation)
                m10 * v.x + m11 * v.y + m12 * v.z,
                // Z component: row 2 dot product with vector (no translation)
                m20 * v.x + m21 * v.y + m22 * v.z
            );
        }
    }
}

