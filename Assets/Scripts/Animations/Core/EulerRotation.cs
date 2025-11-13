// Import Unity's math and vector utilities
using UnityEngine;

// Namespace for core physics simulation utilities
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
            // Convert angular velocity from radians per second to degrees per second
            Vector3 angularDegrees = angularVelocity * Mathf.Rad2Deg;
            // Add the change in angle over this time step to current Euler angles
            return eulerAngles + angularDegrees * deltaTime;
        }

        /// <summary>
        /// Converts Euler angles to a rotation matrix
        /// </summary>
        public static Matrix4x4 EulerToMatrix(Vector3 eulerAngles)
        {
            // Convert X Euler angle from degrees to radians
            float x = eulerAngles.x * Mathf.Deg2Rad;
            // Convert Y Euler angle from degrees to radians
            float y = eulerAngles.y * Mathf.Deg2Rad;
            // Convert Z Euler angle from degrees to radians
            float z = eulerAngles.z * Mathf.Deg2Rad;

            // Calculate cosine of X rotation
            float cx = Mathf.Cos(x);
            // Calculate sine of X rotation
            float sx = Mathf.Sin(x);
            // Calculate cosine of Y rotation
            float cy = Mathf.Cos(y);
            // Calculate sine of Y rotation
            float sy = Mathf.Sin(y);
            // Calculate cosine of Z rotation
            float cz = Mathf.Cos(z);
            // Calculate sine of Z rotation
            float sz = Mathf.Sin(z);

            // Start with identity matrix
            Matrix4x4 m = Matrix4x4.identity;
            // Build rotation matrix from Euler angles using ZYX rotation order
            // Row 0, Column 0: cos(y) * cos(z)
            m.m00 = cy * cz;
            // Row 0, Column 1: -cos(y) * sin(z)
            m.m01 = -cy * sz;
            // Row 0, Column 2: sin(y)
            m.m02 = sy;
            // Row 1, Column 0: cos(x) * sin(z) + sin(x) * sin(y) * cos(z)
            m.m10 = cx * sz + sx * sy * cz;
            // Row 1, Column 1: cos(x) * cos(z) - sin(x) * sin(y) * sin(z)
            m.m11 = cx * cz - sx * sy * sz;
            // Row 1, Column 2: -sin(x) * cos(y)
            m.m12 = -sx * cy;
            // Row 2, Column 0: sin(x) * sin(z) - cos(x) * sin(y) * cos(z)
            m.m20 = sx * sz - cx * sy * cz;
            // Row 2, Column 1: sin(x) * cos(z) + cos(x) * sin(y) * sin(z)
            m.m21 = sx * cz + cx * sy * sz;
            // Row 2, Column 2: cos(x) * cos(y)
            m.m22 = cx * cy;

            // Return the constructed rotation matrix
            return m;
        }

        /// <summary>
        /// Converts a rotation matrix to Euler angles
        /// </summary>
        public static Vector3 MatrixToEuler(Matrix4x4 m)
        {
            // Create vector to store extracted Euler angles
            Vector3 euler = Vector3.zero;

            // Extract X rotation using arctangent of m12 and m22
            euler.x = Mathf.Atan2(-m.m12, m.m22);
            // Extract Y rotation using arcsine of m02
            euler.y = Mathf.Asin(m.m02);
            // Extract Z rotation using arctangent of m01 and m00
            euler.z = Mathf.Atan2(-m.m01, m.m00);

            // Convert from radians to degrees and return
            return euler * Mathf.Rad2Deg;
        }
        #endregion

        #region Quaternion Rotation
        /// <summary>
        /// Creates a quaternion from axis and angle
        /// </summary>
        public static Quaternion AxisAngleToQuaternion(Vector3 axis, float angle)
        {
            // Normalize the rotation axis to unit length
            axis.Normalize();
            // Calculate half the rotation angle in radians
            float halfAngle = angle * 0.5f * Mathf.Deg2Rad;
            // Calculate sine of half angle for quaternion components
            float s = Mathf.Sin(halfAngle);
            
            // Construct and return quaternion: q = [x*sin(θ/2), y*sin(θ/2), z*sin(θ/2), cos(θ/2)]
            return new Quaternion(
                axis.x * s, // X component of quaternion
                axis.y * s, // Y component of quaternion
                axis.z * s, // Z component of quaternion
                Mathf.Cos(halfAngle) // W component of quaternion
            );
        }

        /// <summary>
        /// Extracts axis and angle from a quaternion
        /// </summary>
        public static void QuaternionToAxisAngle(Quaternion q, out Vector3 axis, out float angle)
        {
            // Calculate rotation angle from W component: θ = 2 * arccos(w)
            angle = 2.0f * Mathf.Acos(q.w) * Mathf.Rad2Deg;
            // Calculate the scale factor for the axis: s = sqrt(1 - w²)
            float s = Mathf.Sqrt(1.0f - q.w * q.w);
            
            // Check if rotation is very small (near zero angle)
            if (s < PhysicsConstants.EPSILON)
            {
                // Default to up axis when rotation is negligible
                axis = Vector3.up;
            }
            else
            {
                // Extract normalized axis by dividing xyz components by scale factor
                axis = new Vector3(q.x / s, q.y / s, q.z / s);
            }
        }

        /// <summary>
        /// Spherical linear interpolation between two quaternions
        /// </summary>
        public static Quaternion Slerp(Quaternion a, Quaternion b, float t)
        {
            // Calculate dot product to find angle between quaternions
            float dot = Quaternion.Dot(a, b);
            // Ensure shortest path by negating if dot product is negative
            if (dot < 0.0f)
            {
                // Negate quaternion b to take shorter path
                b = new Quaternion(-b.x, -b.y, -b.z, -b.w);
                // Update dot product with negated quaternion
                dot = -dot;
            }

            // Check if quaternions are very close (nearly parallel)
            if (dot > 0.9995f)
            {
                // Use faster linear interpolation for very close quaternions
                return Quaternion.Lerp(a, b, t);
            }

            // Calculate angle between quaternions using arccosine of dot product
            float theta = Mathf.Acos(dot);
            // Calculate sine of angle for normalization
            float sinTheta = Mathf.Sin(theta);
            // Calculate weight for quaternion a: sin((1-t)*θ) / sin(θ)
            float wa = Mathf.Sin((1.0f - t) * theta) / sinTheta;
            // Calculate weight for quaternion b: sin(t*θ) / sin(θ)
            float wb = Mathf.Sin(t * theta) / sinTheta;

            // Construct interpolated quaternion by blending a and b with calculated weights
            return new Quaternion(
                wa * a.x + wb * b.x, // Interpolated X component
                wa * a.y + wb * b.y, // Interpolated Y component
                wa * a.z + wb * b.z, // Interpolated Z component
                wa * a.w + wb * b.w  // Interpolated W component
            );
        }
        #endregion

        #region Rotation Utilities
        /// <summary>
        /// Applies a rotation to a vector
        /// </summary>
        public static Vector3 RotateVector(Vector3 vector, Quaternion rotation)
        {
            // Use quaternion multiplication to rotate the vector
            return rotation * vector;
        }

        /// <summary>
        /// Gets the shortest rotation from one direction to another
        /// </summary>
        public static Quaternion FromToRotation(Vector3 from, Vector3 to)
        {
            // Normalize the source vector to unit length
            from.Normalize();
            // Normalize the target vector to unit length
            to.Normalize();

            // Calculate dot product between normalized vectors (cosine of angle)
            float dot = Vector3.Dot(from, to);

            // Check if vectors are already aligned (parallel, same direction)
            if (dot >= 1.0f)
            {
                // No rotation needed, return identity quaternion
                return Quaternion.identity;
            }
            // Check if vectors are opposite (anti-parallel, 180° apart)
            else if (dot <= -1.0f)
            {
                // Find perpendicular axis by crossing with up vector
                Vector3 perpAxis = Vector3.Cross(Vector3.up, from);
                // If cross product is near zero, try right vector instead
                if (perpAxis.sqrMagnitude < PhysicsConstants.EPSILON_SMALL)
                    perpAxis = Vector3.Cross(Vector3.right, from);
                // Normalize the perpendicular axis
                perpAxis.Normalize();
                // Return 180 degree rotation around perpendicular axis
                return AxisAngleToQuaternion(perpAxis, 180f);
            }

            // Calculate rotation axis using cross product (perpendicular to both vectors)
            Vector3 axis = Vector3.Cross(from, to);
            // Calculate rotation angle using arccosine of dot product, convert to degrees
            float angle = Mathf.Acos(dot) * Mathf.Rad2Deg;
            // Create and return quaternion from axis and angle
            return AxisAngleToQuaternion(axis, angle);
        }

        /// <summary>
        /// Decomposes a quaternion into swing and twist components
        /// Useful for joint constraints
        /// </summary>
        public static void DecomposeSwingTwist(Quaternion rotation, Vector3 twistAxis, out Quaternion swing, out Quaternion twist)
        {
            // Normalize the twist axis to unit length
            twistAxis.Normalize();
            
            // Extract rotation axis from quaternion (xyz components)
            Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);
            // Project rotation axis onto twist axis to get twist component
            Vector3 projection = Vector3.Project(rotationAxis, twistAxis);

            // Construct twist quaternion from projected axis and w component, then normalize
            twist = new Quaternion(projection.x, projection.y, projection.z, rotation.w).normalized;
            // Calculate swing by removing twist from original rotation: swing = rotation * twist^-1
            swing = rotation * Quaternion.Inverse(twist);
        }
        #endregion
    }
}
