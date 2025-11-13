using UnityEngine;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Utility class for rotation implementations - Euler and Quaternion methods
    /// Provides different rotation integration approaches for educational purposes
    /// </summary>
    public static class RotationHelper
    {
        #region Euler Rotation
        /// <summary>
        /// Integrates rotation using Euler angles (simple but prone to gimbal lock)
        /// </summary>
        public static Vector3 IntegrateEulerRotation(Vector3 eulerAngles, Vector3 angularVelocity, float deltaTime)
        {
            // Convert angular velocity from radians to degrees
            Vector3 angularDegrees = angularVelocity * Mathf.Rad2Deg;
            return eulerAngles + angularDegrees * deltaTime;
        }

        /// <summary>
        /// Converts Euler angles to a rotation matrix
        /// </summary>
        public static Matrix4x4 EulerToMatrix(Vector3 eulerAngles)
        {
            float x = eulerAngles.x * Mathf.Deg2Rad;
            float y = eulerAngles.y * Mathf.Deg2Rad;
            float z = eulerAngles.z * Mathf.Deg2Rad;

            float cx = Mathf.Cos(x);
            float sx = Mathf.Sin(x);
            float cy = Mathf.Cos(y);
            float sy = Mathf.Sin(y);
            float cz = Mathf.Cos(z);
            float sz = Mathf.Sin(z);

            Matrix4x4 m = Matrix4x4.identity;
            m.m00 = cy * cz;
            m.m01 = -cy * sz;
            m.m02 = sy;
            m.m10 = cx * sz + sx * sy * cz;
            m.m11 = cx * cz - sx * sy * sz;
            m.m12 = -sx * cy;
            m.m20 = sx * sz - cx * sy * cz;
            m.m21 = sx * cz + cx * sy * sz;
            m.m22 = cx * cy;

            return m;
        }

        /// <summary>
        /// Converts a rotation matrix to Euler angles
        /// </summary>
        public static Vector3 MatrixToEuler(Matrix4x4 m)
        {
            Vector3 euler = Vector3.zero;

            euler.x = Mathf.Atan2(-m.m12, m.m22);
            euler.y = Mathf.Asin(m.m02);
            euler.z = Mathf.Atan2(-m.m01, m.m00);

            return euler * Mathf.Rad2Deg;
        }
        #endregion

        #region Quaternion Rotation
        /// <summary>
        /// Creates a quaternion from axis and angle
        /// </summary>
        public static Quaternion AxisAngleToQuaternion(Vector3 axis, float angle)
        {
            axis.Normalize();
            float halfAngle = angle * 0.5f * Mathf.Deg2Rad;
            float s = Mathf.Sin(halfAngle);
            
            return new Quaternion(
                axis.x * s,
                axis.y * s,
                axis.z * s,
                Mathf.Cos(halfAngle)
            );
        }

        /// <summary>
        /// Extracts axis and angle from a quaternion
        /// </summary>
        public static void QuaternionToAxisAngle(Quaternion q, out Vector3 axis, out float angle)
        {
            angle = 2.0f * Mathf.Acos(q.w) * Mathf.Rad2Deg;
            float s = Mathf.Sqrt(1.0f - q.w * q.w);
            
            if (s < PhysicsConstants.EPSILON)
            {
                axis = Vector3.up;
            }
            else
            {
                axis = new Vector3(q.x / s, q.y / s, q.z / s);
            }
        }

        /// <summary>
        /// Spherical linear interpolation between two quaternions
        /// </summary>
        public static Quaternion Slerp(Quaternion a, Quaternion b, float t)
        {
            // Ensure shortest path
            float dot = Quaternion.Dot(a, b);
            if (dot < 0.0f)
            {
                b = new Quaternion(-b.x, -b.y, -b.z, -b.w);
                dot = -dot;
            }

            if (dot > 0.9995f)
            {
                // Use linear interpolation for very close quaternions
                return Quaternion.Lerp(a, b, t);
            }

            float theta = Mathf.Acos(dot);
            float sinTheta = Mathf.Sin(theta);
            float wa = Mathf.Sin((1.0f - t) * theta) / sinTheta;
            float wb = Mathf.Sin(t * theta) / sinTheta;

            return new Quaternion(
                wa * a.x + wb * b.x,
                wa * a.y + wb * b.y,
                wa * a.z + wb * b.z,
                wa * a.w + wb * b.w
            );
        }
        #endregion

        #region Rotation Utilities
        /// <summary>
        /// Applies a rotation to a vector
        /// </summary>
        public static Vector3 RotateVector(Vector3 vector, Quaternion rotation)
        {
            return rotation * vector;
        }

        /// <summary>
        /// Gets the shortest rotation from one direction to another
        /// </summary>
        public static Quaternion FromToRotation(Vector3 from, Vector3 to)
        {
            from.Normalize();
            to.Normalize();

            float dot = Vector3.Dot(from, to);

            if (dot >= 1.0f)
            {
                return Quaternion.identity;
            }
            else if (dot <= -1.0f)
            {
                Vector3 perpAxis = Vector3.Cross(Vector3.up, from);
                if (perpAxis.sqrMagnitude < PhysicsConstants.EPSILON_SMALL)
                    perpAxis = Vector3.Cross(Vector3.right, from);
                perpAxis.Normalize();
                return AxisAngleToQuaternion(perpAxis, 180f);
            }

            Vector3 axis = Vector3.Cross(from, to);
            float angle = Mathf.Acos(dot) * Mathf.Rad2Deg;
            return AxisAngleToQuaternion(axis, angle);
        }

        /// <summary>
        /// Decomposes a quaternion into swing and twist components
        /// Useful for joint constraints
        /// </summary>
        public static void DecomposeSwingTwist(Quaternion rotation, Vector3 twistAxis, out Quaternion swing, out Quaternion twist)
        {
            twistAxis.Normalize();
            
            Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);
            Vector3 projection = Vector3.Project(rotationAxis, twistAxis);

            twist = new Quaternion(projection.x, projection.y, projection.z, rotation.w).normalized;
            swing = rotation * Quaternion.Inverse(twist);
        }
        #endregion
    }
}
