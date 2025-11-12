namespace Rayen.attempt2
{
    using UnityEngine;

    /// <summary>
    /// Custom quaternion implementation for rotation without Unity's built-in quaternions.
    /// Quaternions prevent gimbal lock and are more stable than Euler angles.
    /// </summary>
    public struct CustomQuaternion
    {
        public float w, x, y, z;

        public CustomQuaternion(float w, float x, float y, float z)
        {
            this.w = w;
            this.x = x;
            this.y = y;
            this.z = z;
        }

        // Identity quaternion (no rotation)
        public static CustomQuaternion Identity => new CustomQuaternion(1, 0, 0, 0);

        // Magnitude of quaternion
        public float Magnitude()
        {
            return Mathf.Sqrt(w * w + x * x + y * y + z * z);
        }

        // Normalize quaternion
        public CustomQuaternion Normalized()
        {
            float mag = Magnitude();
            if (mag < 0.0001f) return Identity;
            return new CustomQuaternion(w / mag, x / mag, y / mag, z / mag);
        }

        // Multiply two quaternions (q1 * q2)
        public static CustomQuaternion Multiply(CustomQuaternion q1, CustomQuaternion q2)
        {
            return new CustomQuaternion(
                q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
                q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
                q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
                q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
            );
        }

        // Create quaternion from angular velocity and timestep
        // This is the key for integrating rotation from angular momentum
        public static CustomQuaternion FromAngularVelocity(Vector3 omega, float dt)
        {
            float angle = omega.magnitude * dt;
            if (angle < 0.0001f) return Identity;

            Vector3 axis = omega.normalized;
            float halfAngle = angle * 0.5f;
            float sinHalf = Mathf.Sin(halfAngle);

            return new CustomQuaternion(
                Mathf.Cos(halfAngle),
                axis.x * sinHalf,
                axis.y * sinHalf,
                axis.z * sinHalf
            );
        }

        // Convert quaternion to rotation matrix
        public Matrix4x4 ToRotationMatrix()
        {
            // Normalize first
            CustomQuaternion q = this.Normalized();
            
            float xx = q.x * q.x;
            float yy = q.y * q.y;
            float zz = q.z * q.z;
            float xy = q.x * q.y;
            float xz = q.x * q.z;
            float yz = q.y * q.z;
            float wx = q.w * q.x;
            float wy = q.w * q.y;
            float wz = q.w * q.z;

            Matrix4x4 m = Matrix4x4.identity;
            
            m[0, 0] = 1 - 2 * (yy + zz);
            m[0, 1] = 2 * (xy - wz);
            m[0, 2] = 2 * (xz + wy);
            
            m[1, 0] = 2 * (xy + wz);
            m[1, 1] = 1 - 2 * (xx + zz);
            m[1, 2] = 2 * (yz - wx);
            
            m[2, 0] = 2 * (xz - wy);
            m[2, 1] = 2 * (yz + wx);
            m[2, 2] = 1 - 2 * (xx + yy);

            return m;
        }

        // Create quaternion from rotation matrix (useful for initialization)
        public static CustomQuaternion FromRotationMatrix(Matrix4x4 m)
        {
            float trace = m[0, 0] + m[1, 1] + m[2, 2];
            
            if (trace > 0)
            {
                float s = Mathf.Sqrt(trace + 1.0f) * 2;
                return new CustomQuaternion(
                    0.25f * s,
                    (m[2, 1] - m[1, 2]) / s,
                    (m[0, 2] - m[2, 0]) / s,
                    (m[1, 0] - m[0, 1]) / s
                );
            }
            else if (m[0, 0] > m[1, 1] && m[0, 0] > m[2, 2])
            {
                float s = Mathf.Sqrt(1.0f + m[0, 0] - m[1, 1] - m[2, 2]) * 2;
                return new CustomQuaternion(
                    (m[2, 1] - m[1, 2]) / s,
                    0.25f * s,
                    (m[0, 1] + m[1, 0]) / s,
                    (m[0, 2] + m[2, 0]) / s
                );
            }
            else if (m[1, 1] > m[2, 2])
            {
                float s = Mathf.Sqrt(1.0f + m[1, 1] - m[0, 0] - m[2, 2]) * 2;
                return new CustomQuaternion(
                    (m[0, 2] - m[2, 0]) / s,
                    (m[0, 1] + m[1, 0]) / s,
                    0.25f * s,
                    (m[1, 2] + m[2, 1]) / s
                );
            }
            else
            {
                float s = Mathf.Sqrt(1.0f + m[2, 2] - m[0, 0] - m[1, 1]) * 2;
                return new CustomQuaternion(
                    (m[1, 0] - m[0, 1]) / s,
                    (m[0, 2] + m[2, 0]) / s,
                    (m[1, 2] + m[2, 1]) / s,
                    0.25f * s
                );
            }
        }

        public override string ToString()
        {
            return $"CustomQuaternion({w:F3}, {x:F3}, {y:F3}, {z:F3})";
        }
    }
}
