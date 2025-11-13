using UnityEngine;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Utility class for common mathematical operations used in physics simulations
    /// Provides optimized and reusable math functions for matrices, quaternions, and physics calculations
    /// </summary>
    public static class MathUtils
    {
        #region Matrix Operations
        /// <summary>
        /// Inverts a 3x3 matrix embedded in a Matrix4x4
        /// Used for inertia tensor calculations
        /// </summary>
        public static Matrix4x4 InvertMatrix3x3(Matrix4x4 m)
        {
            float det = m.m00 * (m.m11 * m.m22 - m.m12 * m.m21)
                      - m.m01 * (m.m10 * m.m22 - m.m12 * m.m20)
                      + m.m02 * (m.m10 * m.m21 - m.m11 * m.m20);
            
            if (Mathf.Abs(det) < PhysicsConstants.EPSILON)
                det = PhysicsConstants.EPSILON;
            
            float invDet = 1.0f / det;
            
            Matrix4x4 inv = Matrix4x4.zero;
            inv.m00 = (m.m11 * m.m22 - m.m12 * m.m21) * invDet;
            inv.m01 = (m.m02 * m.m21 - m.m01 * m.m22) * invDet;
            inv.m02 = (m.m01 * m.m12 - m.m02 * m.m11) * invDet;
            inv.m10 = (m.m12 * m.m20 - m.m10 * m.m22) * invDet;
            inv.m11 = (m.m00 * m.m22 - m.m02 * m.m20) * invDet;
            inv.m12 = (m.m02 * m.m10 - m.m00 * m.m12) * invDet;
            inv.m20 = (m.m10 * m.m21 - m.m11 * m.m20) * invDet;
            inv.m21 = (m.m01 * m.m20 - m.m00 * m.m21) * invDet;
            inv.m22 = (m.m00 * m.m11 - m.m01 * m.m10) * invDet;
            inv.m33 = 1f;
            
            return inv;
        }

        /// <summary>
        /// Multiplies two 3x3 matrices (stored in Matrix4x4)
        /// </summary>
        public static Matrix4x4 MultiplyMatrix3x3(Matrix4x4 a, Matrix4x4 b)
        {
            Matrix4x4 result = Matrix4x4.zero;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    result[i, j] = a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j];
                }
            }
            result.m33 = 1f;
            return result;
        }

        /// <summary>
        /// Transposes a 3x3 matrix (stored in Matrix4x4)
        /// </summary>
        public static Matrix4x4 TransposeMatrix3x3(Matrix4x4 m)
        {
            Matrix4x4 result = Matrix4x4.zero;
            result.m00 = m.m00; result.m01 = m.m10; result.m02 = m.m20;
            result.m10 = m.m01; result.m11 = m.m11; result.m12 = m.m21;
            result.m20 = m.m02; result.m21 = m.m12; result.m22 = m.m22;
            result.m33 = 1f;
            return result;
        }

        /// <summary>
        /// Multiplies a Matrix4x4 by a Vector3 (treating it as a 3D vector)
        /// </summary>
        public static Vector3 MultiplyMatrixVector3(Matrix4x4 m, Vector3 v)
        {
            return new Vector3(
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
            );
        }

        /// <summary>
        /// Creates a skew-symmetric matrix from a vector (for cross product operations)
        /// </summary>
        public static Matrix4x4 SkewSymmetric(Vector3 v)
        {
            Matrix4x4 m = Matrix4x4.zero;
            m.m01 = -v.z;
            m.m02 = v.y;
            m.m10 = v.z;
            m.m12 = -v.x;
            m.m20 = -v.y;
            m.m21 = v.x;
            m.m33 = 1f;
            return m;
        }
        #endregion

        #region Quaternion Operations
        /// <summary>
        /// Multiplies two quaternions (x, y, z, w format)
        /// </summary>
        public static Vector4 MultiplyQuaternion(Vector4 q1, Vector4 q2)
        {
            float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
            float y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
            float z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
            float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
            return new Vector4(x, y, z, w);
        }

        /// <summary>
        /// Normalizes a quaternion
        /// </summary>
        public static Vector4 NormalizeQuaternion(Vector4 q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (mag < PhysicsConstants.EPSILON)
                return new Vector4(0, 0, 0, 1);
            return q / mag;
        }

        /// <summary>
        /// Converts a quaternion to a rotation matrix
        /// </summary>
        public static Matrix4x4 QuaternionToMatrix(Vector4 q)
        {
            float x = q.x, y = q.y, z = q.z, w = q.w;
            float xx = x * x, yy = y * y, zz = z * z;
            float xy = x * y, xz = x * z, yz = y * z;
            float wx = w * x, wy = w * y, wz = w * z;

            Matrix4x4 m = Matrix4x4.identity;
            m.m00 = 1f - 2f * (yy + zz);
            m.m01 = 2f * (xy - wz);
            m.m02 = 2f * (xz + wy);
            m.m10 = 2f * (xy + wz);
            m.m11 = 1f - 2f * (xx + zz);
            m.m12 = 2f * (yz - wx);
            m.m20 = 2f * (xz - wy);
            m.m21 = 2f * (yz + wx);
            m.m22 = 1f - 2f * (xx + yy);
            return m;
        }

        /// <summary>
        /// Converts a quaternion to a rotation matrix (Unity Quaternion version)
        /// </summary>
        public static Matrix4x4 QuaternionToMatrix(Quaternion q)
        {
            return QuaternionToMatrix(new Vector4(q.x, q.y, q.z, q.w));
        }
        #endregion

        #region Inertia Tensor Calculations
        /// <summary>
        /// Calculates the inertia tensor for a box shape
        /// </summary>
        public static Matrix4x4 CalculateBoxInertiaTensor(float mass, Vector3 size)
        {
            Matrix4x4 tensor = Matrix4x4.zero;
            float m = mass / 12.0f;
            tensor.m00 = m * (size.y * size.y + size.z * size.z);
            tensor.m11 = m * (size.x * size.x + size.z * size.z);
            tensor.m22 = m * (size.x * size.x + size.y * size.y);
            tensor.m33 = 1f;
            return tensor;
        }

        /// <summary>
        /// Calculates the inertia tensor for a sphere
        /// </summary>
        public static Matrix4x4 CalculateSphereInertiaTensor(float mass, float radius)
        {
            float I = (2.0f / 5.0f) * mass * radius * radius;
            Matrix4x4 tensor = Matrix4x4.zero;
            tensor.m00 = I;
            tensor.m11 = I;
            tensor.m22 = I;
            tensor.m33 = 1f;
            return tensor;
        }

        /// <summary>
        /// Calculates the inertia tensor for a cylinder
        /// </summary>
        public static Matrix4x4 CalculateCylinderInertiaTensor(float mass, float radius, float height)
        {
            Matrix4x4 tensor = Matrix4x4.zero;
            tensor.m00 = (1.0f / 12.0f) * mass * (3f * radius * radius + height * height);
            tensor.m11 = (1.0f / 2.0f) * mass * radius * radius;
            tensor.m22 = (1.0f / 12.0f) * mass * (3f * radius * radius + height * height);
            tensor.m33 = 1f;
            return tensor;
        }

        /// <summary>
        /// Transforms an inertia tensor to world space
        /// </summary>
        public static Matrix4x4 TransformInertiaTensor(Matrix4x4 localTensor, Matrix4x4 rotationMatrix)
        {
            return MultiplyMatrix3x3(
                MultiplyMatrix3x3(rotationMatrix, localTensor),
                TransposeMatrix3x3(rotationMatrix)
            );
        }
        #endregion

        #region Vector Operations
        /// <summary>
        /// Projects a box onto an axis (for collision detection)
        /// </summary>
        public static float ProjectBoxOntoAxis(Vector3 halfExtents, Vector3[] axes, Vector3 axis)
        {
            return Mathf.Abs(Vector3.Dot(axes[0], axis)) * halfExtents.x +
                   Mathf.Abs(Vector3.Dot(axes[1], axis)) * halfExtents.y +
                   Mathf.Abs(Vector3.Dot(axes[2], axis)) * halfExtents.z;
        }

        /// <summary>
        /// Dot product without calling Vector3.Dot from gameplay code
        /// </summary>
        public static float Dot(Vector3 a, Vector3 b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        /// <summary>
        /// Linear interpolation without calling Vector3.Lerp from gameplay code
        /// </summary>
        public static Vector3 Lerp(Vector3 a, Vector3 b, float t)
        {
            float ct = Mathf.Clamp01(t);
            return new Vector3(
                a.x + (b.x - a.x) * ct,
                a.y + (b.y - a.y) * ct,
                a.z + (b.z - a.z) * ct
            );
        }

        /// <summary>
        /// Returns squared magnitude of a vector
        /// </summary>
        public static float SqrMagnitude(Vector3 v)
        {
            return v.x * v.x + v.y * v.y + v.z * v.z;
        }

        /// <summary>
        /// Returns magnitude of a vector (sqrt of squared magnitude)
        /// </summary>
        public static float Magnitude(Vector3 v)
        {
            return Mathf.Sqrt(SqrMagnitude(v));
        }

        /// <summary>
        /// Returns squared distance between two vectors
        /// </summary>
        public static float SqrDistance(Vector3 a, Vector3 b)
        {
            return SqrMagnitude(new Vector3(a.x - b.x, a.y - b.y, a.z - b.z));
        }

        /// <summary>
        /// Returns distance between two vectors
        /// </summary>
        public static float Distance(Vector3 a, Vector3 b)
        {
            return Mathf.Sqrt(SqrDistance(a, b));
        }
        
        /// <summary>
        /// Clamps the magnitude of a vector
        /// </summary>
        public static Vector3 ClampMagnitude(Vector3 vector, float maxMagnitude)
        {
            float sqrMag = vector.sqrMagnitude;
            if (sqrMag > maxMagnitude * maxMagnitude)
            {
                float mag = Mathf.Sqrt(sqrMag);
                return vector * (maxMagnitude / mag);
            }
            return vector;
        }

        /// <summary>
        /// Performs component-wise multiplication of two vectors
        /// </summary>
        public static Vector3 ComponentMultiply(Vector3 a, Vector3 b)
        {
            return new Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
        }

        /// <summary>
        /// Performs component-wise division of two vectors
        /// </summary>
        public static Vector3 ComponentDivide(Vector3 a, Vector3 b)
        {
            return new Vector3(
                Mathf.Abs(b.x) > PhysicsConstants.EPSILON ? a.x / b.x : 0f,
                Mathf.Abs(b.y) > PhysicsConstants.EPSILON ? a.y / b.y : 0f,
                Mathf.Abs(b.z) > PhysicsConstants.EPSILON ? a.z / b.z : 0f
            );
        }

        /// <summary>
        /// Checks if a vector is approximately zero
        /// </summary>
        public static bool IsNearZero(Vector3 v, float epsilon = PhysicsConstants.EPSILON_SMALL)
        {
            return v.sqrMagnitude < epsilon * epsilon;
        }

        /// <summary>
        /// Safely normalizes a vector (returns zero if magnitude is too small)
        /// </summary>
        public static Vector3 SafeNormalize(Vector3 v, Vector3 fallback = default)
        {
            float sqrMag = v.sqrMagnitude;
            if (sqrMag < PhysicsConstants.EPSILON_SMALL * PhysicsConstants.EPSILON_SMALL)
                return fallback;
            return v / Mathf.Sqrt(sqrMag);
        }
        #endregion

        #region Interpolation
        /// <summary>
        /// Performs smooth damping on a value
        /// </summary>
        public static float SmoothDamp(float current, float target, ref float velocity, float smoothTime, float maxSpeed, float deltaTime)
        {
            smoothTime = Mathf.Max(0.0001f, smoothTime);
            float omega = 2f / smoothTime;
            float x = omega * deltaTime;
            float exp = 1f / (1f + x + 0.48f * x * x + 0.235f * x * x * x);
            float change = current - target;
            float originalTo = target;
            float maxChange = maxSpeed * smoothTime;
            change = Mathf.Clamp(change, -maxChange, maxChange);
            target = current - change;
            float temp = (velocity + omega * change) * deltaTime;
            velocity = (velocity - omega * temp) * exp;
            float output = target + (change + temp) * exp;
            if (originalTo - current > 0.0f == output > originalTo)
            {
                output = originalTo;
                velocity = (output - originalTo) / deltaTime;
            }
            return output;
        }

        /// <summary>
        /// Performs exponential decay interpolation
        /// </summary>
        public static float ExponentialDecay(float current, float target, float decay, float deltaTime)
        {
            return Mathf.Lerp(current, target, 1f - Mathf.Exp(-decay * deltaTime));
        }

        /// <summary>
        /// Performs exponential decay interpolation for vectors
        /// </summary>
        public static Vector3 ExponentialDecayVector(Vector3 current, Vector3 target, float decay, float deltaTime)
        {
            return Vector3.Lerp(current, target, 1f - Mathf.Exp(-decay * deltaTime));
        }
        #endregion

        #region Angle and Rotation Helpers
        /// <summary>
        /// Converts degrees to radians
        /// </summary>
        public static float DegToRad(float degrees)
        {
            return degrees * Mathf.Deg2Rad;
        }

        /// <summary>
        /// Converts radians to degrees
        /// </summary>
        public static float RadToDeg(float radians)
        {
            return radians * Mathf.Rad2Deg;
        }

        /// <summary>
        /// Wraps an angle to [-180, 180] range
        /// </summary>
        public static float WrapAngle(float angle)
        {
            angle = angle % 360f;
            if (angle > 180f)
                angle -= 360f;
            else if (angle < -180f)
                angle += 360f;
            return angle;
        }

        /// <summary>
        /// Calculates the shortest angle difference between two angles
        /// </summary>
        public static float DeltaAngle(float current, float target)
        {
            float delta = WrapAngle(target - current);
            return delta;
        }
        #endregion
    }
}
