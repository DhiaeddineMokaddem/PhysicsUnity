﻿// Import Unity's mathematical utilities
using UnityEngine;

// Namespace for core physics simulation utilities
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
            // Calculate determinant using the rule of Sarrus for 3x3 matrices
            // det = m00(m11*m22 - m12*m21) - m01(m10*m22 - m12*m20) + m02(m10*m21 - m11*m20)
            float det = m.m00 * (m.m11 * m.m22 - m.m12 * m.m21)
                      - m.m01 * (m.m10 * m.m22 - m.m12 * m.m20)
                      + m.m02 * (m.m10 * m.m21 - m.m11 * m.m20);
            
            // Check if determinant is too close to zero (matrix is singular/non-invertible)
            if (Mathf.Abs(det) < PhysicsConstants.EPSILON)
                det = PhysicsConstants.EPSILON; // Use small epsilon to avoid division by zero
            
            // Calculate inverse determinant for use in all elements
            float invDet = 1.0f / det;
            
            // Create zero matrix to store result
            Matrix4x4 inv = Matrix4x4.zero;
            // Calculate inverse matrix elements using cofactor method
            // Row 0, Column 0: (m11*m22 - m12*m21) / det
            inv.m00 = (m.m11 * m.m22 - m.m12 * m.m21) * invDet;
            // Row 0, Column 1: (m02*m21 - m01*m22) / det
            inv.m01 = (m.m02 * m.m21 - m.m01 * m.m22) * invDet;
            // Row 0, Column 2: (m01*m12 - m02*m11) / det
            inv.m02 = (m.m01 * m.m12 - m.m02 * m.m11) * invDet;
            // Row 1, Column 0: (m12*m20 - m10*m22) / det
            inv.m10 = (m.m12 * m.m20 - m.m10 * m.m22) * invDet;
            // Row 1, Column 1: (m00*m22 - m02*m20) / det
            inv.m11 = (m.m00 * m.m22 - m.m02 * m.m20) * invDet;
            // Row 1, Column 2: (m02*m10 - m00*m12) / det
            inv.m12 = (m.m02 * m.m10 - m.m00 * m.m12) * invDet;
            // Row 2, Column 0: (m10*m21 - m11*m20) / det
            inv.m20 = (m.m10 * m.m21 - m.m11 * m.m20) * invDet;
            // Row 2, Column 1: (m01*m20 - m00*m21) / det
            inv.m21 = (m.m01 * m.m20 - m.m00 * m.m21) * invDet;
            // Row 2, Column 2: (m00*m11 - m01*m10) / det
            inv.m22 = (m.m00 * m.m11 - m.m01 * m.m10) * invDet;
            // Set bottom-right element to 1 for homogeneous coordinates
            inv.m33 = 1f;
            
            // Return the inverted matrix
            return inv;
        }

        /// <summary>
        /// Multiplies two 3x3 matrices (stored in Matrix4x4)
        /// </summary>
        public static Matrix4x4 MultiplyMatrix3x3(Matrix4x4 a, Matrix4x4 b)
        {
            // Create zero matrix to store result
            Matrix4x4 result = Matrix4x4.zero;
            // Iterate through rows of first matrix
            for (int i = 0; i < 3; i++)
            {
                // Iterate through columns of second matrix
                for (int j = 0; j < 3; j++)
                {
                    // Calculate dot product of row i from matrix a with column j from matrix b
                    // result[i,j] = sum of a[i,k] * b[k,j] for k = 0 to 2
                    result[i, j] = a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j];
                }
            }
            // Set bottom-right element to 1 for homogeneous coordinates
            result.m33 = 1f;
            // Return the multiplied matrix
            return result;
        }

        /// <summary>
        /// Transposes a 3x3 matrix (stored in Matrix4x4)
        /// </summary>
        public static Matrix4x4 TransposeMatrix3x3(Matrix4x4 m)
        {
            // Create zero matrix to store result
            Matrix4x4 result = Matrix4x4.zero;
            // Transpose by swapping rows and columns
            // Row 0 becomes Column 0: m00 stays, m01 becomes m10, m02 becomes m20
            result.m00 = m.m00; result.m01 = m.m10; result.m02 = m.m20;
            // Row 1 becomes Column 1: m10 becomes m01, m11 stays, m12 becomes m21
            result.m10 = m.m01; result.m11 = m.m11; result.m12 = m.m21;
            // Row 2 becomes Column 2: m20 becomes m02, m21 becomes m12, m22 stays
            result.m20 = m.m02; result.m21 = m.m12; result.m22 = m.m22;
            // Set bottom-right element to 1 for homogeneous coordinates
            result.m33 = 1f;
            // Return the transposed matrix
            return result;
        }

        /// <summary>
        /// Multiplies a Matrix4x4 by a Vector3 (treating it as a 3D vector)
        /// </summary>
        public static Vector3 MultiplyMatrixVector3(Matrix4x4 m, Vector3 v)
        {
            // Transform vector using matrix multiplication
            return new Vector3(
                // X component: dot product of matrix row 0 with vector
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                // Y component: dot product of matrix row 1 with vector
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                // Z component: dot product of matrix row 2 with vector
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
            );
        }

        /// <summary>
        /// Creates a skew-symmetric matrix from a vector (for cross product operations)
        /// Used to represent cross products as matrix multiplications: [v]× * w = v × w
        /// </summary>
        public static Matrix4x4 SkewSymmetric(Vector3 v)
        {
            // Create zero matrix
            Matrix4x4 m = Matrix4x4.zero;
            // Skew-symmetric matrix has the property: M^T = -M
            // Row 0: [0, -vz, vy]
            m.m01 = -v.z;  // Element (0,1) = -z component
            m.m02 = v.y;   // Element (0,2) = y component
            // Row 1: [vz, 0, -vx]
            m.m10 = v.z;   // Element (1,0) = z component
            m.m12 = -v.x;  // Element (1,2) = -x component
            // Row 2: [-vy, vx, 0]
            m.m20 = -v.y;  // Element (2,0) = -y component
            m.m21 = v.x;   // Element (2,1) = x component
            // Set bottom-right element to 1 for homogeneous coordinates
            m.m33 = 1f;
            // Return skew-symmetric matrix
            return m;
        }
        #endregion

        #region Quaternion Operations
        /// <summary>
        /// Multiplies two quaternions (x, y, z, w format)
        /// </summary>
        public static Vector4 MultiplyQuaternion(Vector4 q1, Vector4 q2)
        {
            // Quaternion multiplication formula: q1 * q2
            // x component: w1*x2 + x1*w2 + y1*z2 - z1*y2
            float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
            // y component: w1*y2 - x1*z2 + y1*w2 + z1*x2
            float y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
            // z component: w1*z2 + x1*y2 - y1*x2 + z1*w2
            float z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
            // w component: w1*w2 - x1*x2 - y1*y2 - z1*z2
            float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
            // Return the multiplied quaternion as Vector4
            return new Vector4(x, y, z, w);
        }

        /// <summary>
        /// Normalizes a quaternion
        /// </summary>
        public static Vector4 NormalizeQuaternion(Vector4 q)
        {
            // Calculate magnitude: sqrt(x² + y² + z² + w²)
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            // Check if magnitude is too small (quaternion is nearly zero)
            if (mag < PhysicsConstants.EPSILON)
                return new Vector4(0, 0, 0, 1); // Return identity quaternion
            // Divide all components by magnitude to normalize
            return q / mag;
        }

        /// <summary>
        /// Converts a quaternion to a rotation matrix
        /// </summary>
        public static Matrix4x4 QuaternionToMatrix(Vector4 q)
        {
            // Extract quaternion components
            float x = q.x, y = q.y, z = q.z, w = q.w;
            // Pre-calculate squared components for matrix formula
            float xx = x * x, yy = y * y, zz = z * z;
            // Pre-calculate product components for matrix formula
            float xy = x * y, xz = x * z, yz = y * z;
            // Pre-calculate w products for matrix formula
            float wx = w * x, wy = w * y, wz = w * z;

            // Create identity matrix to build upon
            Matrix4x4 m = Matrix4x4.identity;
            // Convert quaternion to 3x3 rotation matrix using standard formula
            // Row 0 of rotation matrix
            m.m00 = 1f - 2f * (yy + zz);  // 1 - 2(y² + z²)
            m.m01 = 2f * (xy - wz);        // 2(xy - wz)
            m.m02 = 2f * (xz + wy);        // 2(xz + wy)
            // Row 1 of rotation matrix
            m.m10 = 2f * (xy + wz);        // 2(xy + wz)
            m.m11 = 1f - 2f * (xx + zz);  // 1 - 2(x² + z²)
            m.m12 = 2f * (yz - wx);        // 2(yz - wx)
            // Row 2 of rotation matrix
            m.m20 = 2f * (xz - wy);        // 2(xz - wy)
            m.m21 = 2f * (yz + wx);        // 2(yz + wx)
            m.m22 = 1f - 2f * (xx + yy);  // 1 - 2(x² + y²)
            // Return the rotation matrix
            return m;
        }

        /// <summary>
        /// Converts a quaternion to a rotation matrix (Unity Quaternion version)
        /// </summary>
        public static Matrix4x4 QuaternionToMatrix(Quaternion q)
        {
            // Convert Unity Quaternion to Vector4 and call the Vector4 version
            return QuaternionToMatrix(new Vector4(q.x, q.y, q.z, q.w));
        }
        #endregion

        #region Inertia Tensor Calculations
        /// <summary>
        /// Calculates the inertia tensor for a box shape
        /// Moment of inertia determines how difficult it is to rotate an object
        /// </summary>
        public static Matrix4x4 CalculateBoxInertiaTensor(float mass, Vector3 size)
        {
            // Create zero matrix to store inertia tensor
            Matrix4x4 tensor = Matrix4x4.zero;
            // Pre-calculate mass factor: m/12 for box inertia formula
            float m = mass / 12.0f;
            // Diagonal element (0,0): Ixx = (m/12) * (height² + depth²)
            // Resistance to rotation around X-axis depends on Y and Z dimensions
            tensor.m00 = m * (size.y * size.y + size.z * size.z);
            // Diagonal element (1,1): Iyy = (m/12) * (width² + depth²)
            // Resistance to rotation around Y-axis depends on X and Z dimensions
            tensor.m11 = m * (size.x * size.x + size.z * size.z);
            // Diagonal element (2,2): Izz = (m/12) * (width² + height²)
            // Resistance to rotation around Z-axis depends on X and Y dimensions
            tensor.m22 = m * (size.x * size.x + size.y * size.y);
            // Set bottom-right element to 1 for homogeneous coordinates
            tensor.m33 = 1f;
            // Return the inertia tensor
            return tensor;
        }

        /// <summary>
        /// Calculates the inertia tensor for a sphere
        /// Sphere has same moment of inertia about all axes (isotropy)
        /// </summary>
        public static Matrix4x4 CalculateSphereInertiaTensor(float mass, float radius)
        {
            // Calculate moment of inertia for sphere: I = (2/5) * m * r²
            // This is same for all three principal axes due to spherical symmetry
            float I = (2.0f / 5.0f) * mass * radius * radius;
            // Create zero matrix to store inertia tensor
            Matrix4x4 tensor = Matrix4x4.zero;
            // Set all three diagonal elements to same value (isotropic)
            tensor.m00 = I;  // Ixx - moment of inertia around X-axis
            tensor.m11 = I;  // Iyy - moment of inertia around Y-axis
            tensor.m22 = I;  // Izz - moment of inertia around Z-axis
            // Set bottom-right element to 1 for homogeneous coordinates
            tensor.m33 = 1f;
            // Return the inertia tensor
            return tensor;
        }

        /// <summary>
        /// Calculates the inertia tensor for a cylinder (axis along Y)
        /// </summary>
        public static Matrix4x4 CalculateCylinderInertiaTensor(float mass, float radius, float height)
        {
            // Create zero matrix to store inertia tensor
            Matrix4x4 tensor = Matrix4x4.zero;
            // Ixx = Izz = (1/12) * m * (3r² + h²) - perpendicular to cylinder axis
            // Rotation around X or Z axis (perpendicular to cylinder)
            tensor.m00 = (1.0f / 12.0f) * mass * (3f * radius * radius + height * height);
            // Iyy = (1/2) * m * r² - around cylinder axis
            // Rotation around Y axis (cylinder's central axis) only depends on radius
            tensor.m11 = (1.0f / 2.0f) * mass * radius * radius;
            // Izz = Ixx (cylinder is symmetric around Y-axis)
            tensor.m22 = (1.0f / 12.0f) * mass * (3f * radius * radius + height * height);
            // Set bottom-right element to 1 for homogeneous coordinates
            tensor.m33 = 1f;
            // Return the inertia tensor
            return tensor;
        }

        /// <summary>
        /// Transforms an inertia tensor from local space to world space
        /// Formula: I_world = R * I_local * R^T
        /// </summary>
        public static Matrix4x4 TransformInertiaTensor(Matrix4x4 localTensor, Matrix4x4 rotationMatrix)
        {
            // Transform inertia tensor to world space using: I_world = R * I_local * R^T
            // First multiply: temp = rotationMatrix * localTensor
            // Then multiply: result = temp * transpose(rotationMatrix)
            return MultiplyMatrix3x3(
                MultiplyMatrix3x3(rotationMatrix, localTensor),  // R * I_local
                TransposeMatrix3x3(rotationMatrix)               // * R^T
            );
        }
        #endregion

        #region Vector Operations
        /// <summary>
        /// Projects a box onto an axis (for collision detection)
        /// Used in Separating Axis Theorem (SAT) for oriented bounding box collision
        /// </summary>
        public static float ProjectBoxOntoAxis(Vector3 halfExtents, Vector3[] axes, Vector3 axis)
        {
            // Calculate projection radius: sum of absolute dot products of each axis with test axis, scaled by half-extents
            // This gives the "radius" of the box when projected onto the given axis
            return Mathf.Abs(Vector3.Dot(axes[0], axis)) * halfExtents.x +  // Contribution from X-axis
                   Mathf.Abs(Vector3.Dot(axes[1], axis)) * halfExtents.y +  // Contribution from Y-axis
                   Mathf.Abs(Vector3.Dot(axes[2], axis)) * halfExtents.z;   // Contribution from Z-axis
        }

        /// <summary>
        /// Dot product without calling Vector3.Dot from gameplay code
        /// Manual implementation for educational purposes
        /// </summary>
        public static float Dot(Vector3 a, Vector3 b)
        {
            // Dot product: a·b = ax*bx + ay*by + az*bz
            // Result is scalar representing projection of a onto b (or vice versa)
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        /// <summary>
        /// Linear interpolation without calling Vector3.Lerp from gameplay code
        /// Returns a point t% of the way from a to b
        /// </summary>
        public static Vector3 Lerp(Vector3 a, Vector3 b, float t)
        {
            // Clamp t to [0,1] range to ensure interpolation stays between a and b
            float ct = Mathf.Clamp01(t);
            // Linear interpolation formula: result = a + (b - a) * t
            return new Vector3(
                a.x + (b.x - a.x) * ct,  // Interpolate X component
                a.y + (b.y - a.y) * ct,  // Interpolate Y component
                a.z + (b.z - a.z) * ct   // Interpolate Z component
            );
        }

        /// <summary>
        /// Returns squared magnitude of a vector
        /// Faster than Magnitude() as it avoids the square root operation
        /// </summary>
        public static float SqrMagnitude(Vector3 v)
        {
            // Squared magnitude: |v|² = vx² + vy² + vz²
            // Use for distance comparisons to avoid expensive sqrt
            return v.x * v.x + v.y * v.y + v.z * v.z;
        }

        /// <summary>
        /// Returns magnitude of a vector (sqrt of squared magnitude)
        /// The length of the vector
        /// </summary>
        public static float Magnitude(Vector3 v)
        {
            // Magnitude: |v| = sqrt(vx² + vy² + vz²)
            // This is the Euclidean length of the vector
            return Mathf.Sqrt(SqrMagnitude(v));
        }

        /// <summary>
        /// Returns squared distance between two vectors
        /// Faster than Distance() for comparison purposes
        /// </summary>
        public static float SqrDistance(Vector3 a, Vector3 b)
        {
            // Squared distance: |b-a|² = (bx-ax)² + (by-ay)² + (bz-az)²
            // Create difference vector and return its squared magnitude
            return SqrMagnitude(new Vector3(a.x - b.x, a.y - b.y, a.z - b.z));
        }

        /// <summary>
        /// Returns distance between two vectors
        /// The Euclidean distance in 3D space
        /// </summary>
        public static float Distance(Vector3 a, Vector3 b)
        {
            // Distance: |b-a| = sqrt((bx-ax)² + (by-ay)² + (bz-az)²)
            // Calculate squared distance then take square root
            return Mathf.Sqrt(SqrDistance(a, b));
        }
        
        /// <summary>
        /// Clamps the magnitude of a vector to a maximum length
        /// Used to limit speeds or forces
        /// </summary>
        public static Vector3 ClampMagnitude(Vector3 vector, float maxMagnitude)
        {
            // Get squared magnitude for comparison (avoids sqrt)
            float sqrMag = vector.sqrMagnitude;
            // Check if vector exceeds maximum magnitude
            if (sqrMag > maxMagnitude * maxMagnitude)
            {
                // Vector is too long - need to scale it down
                // Calculate actual magnitude
                float mag = Mathf.Sqrt(sqrMag);
                // Scale vector to have exactly maxMagnitude length
                // Formula: v_clamped = v * (maxMag / |v|)
                return vector * (maxMagnitude / mag);
            }
            // Vector is within limit - return unchanged
            return vector;
        }

        /// <summary>
        /// Performs component-wise multiplication of two vectors
        /// Also known as Hadamard product or element-wise multiplication
        /// </summary>
        public static Vector3 ComponentMultiply(Vector3 a, Vector3 b)
        {
            // Multiply each component separately: (ax*bx, ay*by, az*bz)
            // This is different from dot product or cross product
            return new Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
        }

        /// <summary>
        /// Performs component-wise division of two vectors
        /// Includes safety check to avoid division by zero
        /// </summary>
        public static Vector3 ComponentDivide(Vector3 a, Vector3 b)
        {
            // Divide each component separately with zero-check
            return new Vector3(
                // X component: Check if divisor is not zero, otherwise return 0
                Mathf.Abs(b.x) > PhysicsConstants.EPSILON ? a.x / b.x : 0f,
                // Y component: Check if divisor is not zero, otherwise return 0
                Mathf.Abs(b.y) > PhysicsConstants.EPSILON ? a.y / b.y : 0f,
                // Z component: Check if divisor is not zero, otherwise return 0
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
