// Import Unity's Vector3, Quaternion, and Matrix4x4 types
using UnityEngine;

// Namespace for core physics simulation utilities
namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Utility class for transformation operations
    /// Provides pure math implementations for position, rotation, and scale operations
    /// NO Unity Transform used - all calculations done manually for physics education
    /// </summary>
    public static class TransformUtils
    {
        #region Coordinate Transformations
        /// <summary>
        /// Transforms a point from local space to world space
        /// Formula: worldPoint = position + rotation * (scale * localPoint)
        /// Order: Scale → Rotate → Translate (SRT)
        /// </summary>
        public static Vector3 TransformPoint(Vector3 localPoint, Vector3 position, Quaternion rotation, Vector3 scale)
        {
            // Step 1: Scale the local point (component-wise multiplication)
            Vector3 scaled = Vector3.Scale(localPoint, scale);
            // Step 2: Rotate the scaled point using quaternion multiplication
            Vector3 rotated = rotation * scaled;
            // Step 3: Translate by adding the world position
            return position + rotated;
        }

        /// <summary>
        /// Transforms a point from local space to world space using a matrix
        /// Alternative version using matrix multiplication instead of quaternion
        /// </summary>
        public static Vector3 TransformPointMatrix(Vector3 localPoint, Vector3 position, Matrix4x4 rotationMatrix, Vector3 scale)
        {
            // Step 1: Scale the local point
            Vector3 scaled = Vector3.Scale(localPoint, scale);
            // Step 2: Rotate using matrix-vector multiplication
            Vector3 rotated = MathUtils.MultiplyMatrixVector3(rotationMatrix, scaled);
            // Step 3: Translate by adding world position
            return position + rotated;
        }

        /// <summary>
        /// Transforms a point from world space to local space
        /// Formula: localPoint = scale^-1 * (rotation^-1 * (worldPoint - position))
        /// Order: Inverse of SRT = T^-1 → R^-1 → S^-1
        /// </summary>
        public static Vector3 InverseTransformPoint(Vector3 worldPoint, Vector3 position, Quaternion rotation, Vector3 scale)
        {
            // Step 1: Un-translate by subtracting world position
            Vector3 translated = worldPoint - position;
            // Step 2: Un-rotate using inverse quaternion
            Vector3 rotated = Quaternion.Inverse(rotation) * translated;
            // Step 3: Un-scale by component-wise division
            return MathUtils.ComponentDivide(rotated, scale);
        }

        /// <summary>
        /// Transforms a direction from local space to world space (ignores position and scale)
        /// Directions don't have position and aren't affected by scale
        /// Formula: worldDirection = rotation * localDirection
        /// </summary>
        public static Vector3 TransformDirection(Vector3 localDirection, Quaternion rotation)
        {
            // Only apply rotation to direction vectors
            return rotation * localDirection;
        }

        /// <summary>
        /// Transforms a direction from world space to local space (ignores position and scale)
        /// Formula: localDirection = rotation^-1 * worldDirection
        /// </summary>
        public static Vector3 InverseTransformDirection(Vector3 worldDirection, Quaternion rotation)
        {
            // Apply inverse rotation to world direction
            return Quaternion.Inverse(rotation) * worldDirection;
        }
        #endregion

        #region Axis Extraction
        /// <summary>
        /// Gets the right axis (X-axis) from a rotation quaternion
        /// Returns the direction this rotation considers "right" (positive X)
        /// </summary>
        public static Vector3 GetRight(Quaternion rotation)
        {
            // Transform the unit X vector (1,0,0) by the rotation
            return rotation * Vector3.right;
        }

        /// <summary>
        /// Gets the up axis (Y-axis) from a rotation quaternion
        /// Returns the direction this rotation considers "up" (positive Y)
        /// </summary>
        public static Vector3 GetUp(Quaternion rotation)
        {
            // Transform the unit Y vector (0,1,0) by the rotation
            return rotation * Vector3.up;
        }

        /// <summary>
        /// Gets the forward axis (Z-axis) from a rotation quaternion
        /// Returns the direction this rotation considers "forward" (positive Z)
        /// </summary>
        public static Vector3 GetForward(Quaternion rotation)
        {
            // Transform the unit Z vector (0,0,1) by the rotation
            return rotation * Vector3.forward;
        }

        /// <summary>
        /// Gets the right axis from a rotation matrix
        /// Extracts the first column of the 3x3 rotation part
        /// </summary>
        public static Vector3 GetRightFromMatrix(Matrix4x4 matrix)
        {
            // First column represents the transformed X-axis (right)
            return new Vector3(matrix.m00, matrix.m10, matrix.m20).normalized;
        }

        /// <summary>
        /// Gets the up axis from a rotation matrix
        /// Extracts the second column of the 3x3 rotation part
        /// </summary>
        public static Vector3 GetUpFromMatrix(Matrix4x4 matrix)
        {
            // Second column represents the transformed Y-axis (up)
            return new Vector3(matrix.m01, matrix.m11, matrix.m21).normalized;
        }

        /// <summary>
        /// Gets the forward axis from a rotation matrix
        /// Extracts the third column of the 3x3 rotation part
        /// </summary>
        public static Vector3 GetForwardFromMatrix(Matrix4x4 matrix)
        {
            // Third column represents the transformed Z-axis (forward)
            return new Vector3(matrix.m02, matrix.m12, matrix.m22).normalized;
        }
        #endregion

        #region Velocity Calculations
        /// <summary>
        /// Calculates velocity at a point on a rotating rigid body
        /// Formula: v = v_center + ω × r
        /// Where r is the vector from center to point, and ω is angular velocity
        /// </summary>
        public static Vector3 GetVelocityAtPoint(Vector3 centerVelocity, Vector3 angularVelocity, Vector3 point, Vector3 centerPosition)
        {
            // Calculate lever arm from center of mass to point
            Vector3 r = point - centerPosition;
            // Linear velocity at point = center velocity + (angular velocity × lever arm)
            // Cross product gives the tangential velocity component due to rotation
            return centerVelocity + Vector3.Cross(angularVelocity, r);
        }

        /// <summary>
        /// Calculates the relative velocity between two points on potentially different bodies
        /// Used in collision detection to find the speed at which objects are approaching/separating
        /// </summary>
        public static Vector3 GetRelativeVelocityAtPoint(
            Vector3 vel1, Vector3 angularVel1, Vector3 pos1, Vector3 contactPoint1,  // Body 1 state
            Vector3 vel2, Vector3 angularVel2, Vector3 pos2, Vector3 contactPoint2)  // Body 2 state
        {
            // Calculate absolute velocity at contact point on body 1
            Vector3 v1 = GetVelocityAtPoint(vel1, angularVel1, contactPoint1, pos1);
            // Calculate absolute velocity at contact point on body 2
            Vector3 v2 = GetVelocityAtPoint(vel2, angularVel2, contactPoint2, pos2);
            // Relative velocity = velocity of body 2 relative to body 1
            return v2 - v1;
        }
        #endregion

        #region Impulse Application
        /// <summary>
        /// Applies an impulse at a point on a rigid body
        /// Updates both linear and angular velocity
        /// Formula:
        ///   Δv = J / m (linear)
        ///   Δω = I^-1 * (r × J) (angular)
        /// </summary>
        public static void ApplyImpulseAtPoint(
            ref Vector3 velocity,           // Linear velocity to update (by reference)
            ref Vector3 angularVelocity,    // Angular velocity to update (by reference)
            Vector3 impulse,                // Impulse vector to apply (force * time)
            Vector3 point,                  // Point of application in world space
            Vector3 centerPosition,         // Center of mass position
            float inverseMass,              // 1 / mass (precomputed for efficiency)
            Matrix4x4 inverseInertiaTensor) // I^-1 in world space
        {
            // Linear velocity change: Δv = J / m = J * (1/m)
            velocity += impulse * inverseMass;

            // Calculate lever arm from center of mass to application point
            Vector3 r = point - centerPosition;
            // Calculate angular impulse: L = r × J (cross product gives torque impulse)
            Vector3 angularImpulse = Vector3.Cross(r, impulse);
            // Angular velocity change: Δω = I^-1 * L
            angularVelocity += MathUtils.MultiplyMatrixVector3(inverseInertiaTensor, angularImpulse);
        }

        /// <summary>
        /// Applies an impulse to two bodies at a contact point
        /// Uses Newton's third law: forces are equal and opposite
        /// </summary>
        public static void ApplyImpulseToBodies(
            ref Vector3 vel1, ref Vector3 angularVel1, Vector3 pos1, float invMass1, Matrix4x4 invInertia1,  // Body 1
            ref Vector3 vel2, ref Vector3 angularVel2, Vector3 pos2, float invMass2, Matrix4x4 invInertia2,  // Body 2
            Vector3 impulse,      // Impulse to apply to body 2 (negative for body 1)
            Vector3 contactPoint) // Contact point in world space
        {
            // Apply negative impulse to body 1 (action-reaction pair)
            ApplyImpulseAtPoint(ref vel1, ref angularVel1, -impulse, contactPoint, pos1, invMass1, invInertia1);
            // Apply positive impulse to body 2
            ApplyImpulseAtPoint(ref vel2, ref angularVel2, impulse, contactPoint, pos2, invMass2, invInertia2);
        }
        #endregion

        #region Distance and Closest Point Calculations
        /// <summary>
        /// Calculates the closest point on a line segment to a point
        /// </summary>
        public static Vector3 ClosestPointOnLineSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
        {
            Vector3 line = lineEnd - lineStart;
            float lineLength = line.magnitude;
            
            if (lineLength < PhysicsConstants.EPSILON)
                return lineStart;

            Vector3 lineDir = line / lineLength;
            Vector3 toPoint = point - lineStart;
            float projection = Vector3.Dot(toPoint, lineDir);

            projection = Mathf.Clamp(projection, 0f, lineLength);
            return lineStart + lineDir * projection;
        }

        /// <summary>
        /// Calculates the closest point on an infinite line to a point
        /// </summary>
        public static Vector3 ClosestPointOnLine(Vector3 point, Vector3 linePoint, Vector3 lineDirection)
        {
            lineDirection.Normalize();
            Vector3 toPoint = point - linePoint;
            float projection = Vector3.Dot(toPoint, lineDirection);
            return linePoint + lineDirection * projection;
        }

        /// <summary>
        /// Calculates the closest point on a plane to a point
        /// </summary>
        public static Vector3 ClosestPointOnPlane(Vector3 point, Vector3 planePoint, Vector3 planeNormal)
        {
            planeNormal.Normalize();
            float distance = Vector3.Dot(point - planePoint, planeNormal);
            return point - planeNormal * distance;
        }

        /// <summary>
        /// Calculates distance from a point to a line segment
        /// </summary>
        public static float DistanceToLineSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
        {
            Vector3 closest = ClosestPointOnLineSegment(point, lineStart, lineEnd);
            return (point - closest).magnitude;
        }
        #endregion

        #region Bounding Volume Calculations
        /// <summary>
        /// Calculates the axis-aligned bounding box (AABB) for a set of points
        /// </summary>
        public static void CalculateAABB(Vector3[] points, out Vector3 min, out Vector3 max)
        {
            if (points == null || points.Length == 0)
            {
                min = max = Vector3.zero;
                return;
            }

            min = max = points[0];
            for (int i = 1; i < points.Length; i++)
            {
                min = Vector3.Min(min, points[i]);
                max = Vector3.Max(max, points[i]);
            }
        }

        /// <summary>
        /// Expands an AABB to include a point
        /// </summary>
        public static void ExpandAABB(ref Vector3 min, ref Vector3 max, Vector3 point)
        {
            min = Vector3.Min(min, point);
            max = Vector3.Max(max, point);
        }

        /// <summary>
        /// Gets the 8 corners of an oriented bounding box
        /// </summary>
        public static Vector3[] GetOBBCorners(Vector3 center, Vector3 halfExtents, Quaternion rotation)
        {
            Vector3[] corners = new Vector3[8];
            int index = 0;
            
            for (int x = -1; x <= 1; x += 2)
            {
                for (int y = -1; y <= 1; y += 2)
                {
                    for (int z = -1; z <= 1; z += 2)
                    {
                        Vector3 localCorner = new Vector3(
                            x * halfExtents.x,
                            y * halfExtents.y,
                            z * halfExtents.z
                        );
                        corners[index++] = center + rotation * localCorner;
                    }
                }
            }
            
            return corners;
        }

        /// <summary>
        /// Calculates the center and half-extents of an AABB
        /// </summary>
        public static void GetAABBCenterAndExtents(Vector3 min, Vector3 max, out Vector3 center, out Vector3 halfExtents)
        {
            center = (min + max) * 0.5f;
            halfExtents = (max - min) * 0.5f;
        }
        #endregion

        #region Rotation Utilities
        /// <summary>
        /// Creates a rotation that looks at a target
        /// </summary>
        public static Quaternion LookRotation(Vector3 forward, Vector3 up)
        {
            forward = MathUtils.SafeNormalize(forward, Vector3.forward);
            up = MathUtils.SafeNormalize(up, Vector3.up);
            
            Vector3 right = Vector3.Cross(up, forward).normalized;
            up = Vector3.Cross(forward, right);
            
            Matrix4x4 m = Matrix4x4.identity;
            m.SetColumn(0, new Vector4(right.x, right.y, right.z, 0));
            m.SetColumn(1, new Vector4(up.x, up.y, up.z, 0));
            m.SetColumn(2, new Vector4(forward.x, forward.y, forward.z, 0));
            
            return MatrixToQuaternion(m);
        }

        /// <summary>
        /// Converts a rotation matrix to a quaternion
        /// </summary>
        public static Quaternion MatrixToQuaternion(Matrix4x4 m)
        {
            Quaternion q = new Quaternion();
            q.w = Mathf.Sqrt(Mathf.Max(0, 1 + m.m00 + m.m11 + m.m22)) / 2;
            q.x = Mathf.Sqrt(Mathf.Max(0, 1 + m.m00 - m.m11 - m.m22)) / 2;
            q.y = Mathf.Sqrt(Mathf.Max(0, 1 - m.m00 + m.m11 - m.m22)) / 2;
            q.z = Mathf.Sqrt(Mathf.Max(0, 1 - m.m00 - m.m11 + m.m22)) / 2;
            q.x *= Mathf.Sign(q.x * (m.m21 - m.m12));
            q.y *= Mathf.Sign(q.y * (m.m02 - m.m20));
            q.z *= Mathf.Sign(q.z * (m.m10 - m.m01));
            return q;
        }

        /// <summary>
        /// Calculates the angular velocity required to rotate from one orientation to another
        /// </summary>
        public static Vector3 CalculateAngularVelocity(Quaternion from, Quaternion to, float deltaTime)
        {
            if (deltaTime < PhysicsConstants.EPSILON)
                return Vector3.zero;

            Quaternion deltaRotation = to * Quaternion.Inverse(from);
            
            // Convert to axis-angle
            deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);
            
            // Wrap angle to [-180, 180]
            angle = MathUtils.WrapAngle(angle);
            
            // Calculate angular velocity
            return axis * (angle * Mathf.Deg2Rad / deltaTime);
        }
        #endregion

        #region Interpolation
        /// <summary>
        /// Performs smooth interpolation between two positions
        /// </summary>
        public static Vector3 SmoothMove(Vector3 current, Vector3 target, ref Vector3 velocity, float smoothTime, float deltaTime)
        {
            float omega = 2f / smoothTime;
            float x = omega * deltaTime;
            float exp = 1f / (1f + x + 0.48f * x * x + 0.235f * x * x * x);
            
            Vector3 change = current - target;
            Vector3 temp = (velocity + omega * change) * deltaTime;
            velocity = (velocity - omega * temp) * exp;
            
            return target + (change + temp) * exp;
        }

        /// <summary>
        /// Performs smooth rotation interpolation
        /// </summary>
        public static Quaternion SmoothRotate(Quaternion current, Quaternion target, ref Vector3 angularVelocity, float smoothTime, float deltaTime)
        {
            Quaternion delta = target * Quaternion.Inverse(current);
            delta.ToAngleAxis(out float angle, out Vector3 axis);
            angle = MathUtils.WrapAngle(angle);
            
            float targetAngularSpeed = angle * Mathf.Deg2Rad / smoothTime;
            Vector3 targetAngularVelocity = axis * targetAngularSpeed;
            
            angularVelocity = Vector3.Lerp(angularVelocity, targetAngularVelocity, deltaTime / smoothTime);
            
            return IntegrationUtils.IntegrateRotationQuaternion(current, angularVelocity, deltaTime);
        }
        #endregion
    }
}
