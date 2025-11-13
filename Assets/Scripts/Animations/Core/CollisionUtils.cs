using UnityEngine;

namespace PhysicsUnity.Core
{
    /// <summary>
    /// Shared collision detection utilities and data structures
    /// Provides reusable collision algorithms for various primitive shapes
    /// </summary>
    public static class CollisionUtils
    {
        #region Collision Data Structures
        /// <summary>
        /// Contains information about a collision contact
        /// </summary>
        public struct ContactInfo
        {
            public Vector3 point;
            public Vector3 normal;
            public float penetration;
            public bool hasCollision;

            public ContactInfo(Vector3 point, Vector3 normal, float penetration)
            {
                this.point = point;
                this.normal = normal;
                this.penetration = penetration;
                this.hasCollision = true;
            }

            public static ContactInfo NoCollision => new ContactInfo { hasCollision = false };
        }
        #endregion

        #region Sphere Collision
        /// <summary>
        /// Checks collision between two spheres
        /// </summary>
        public static bool CheckSphereSphereCollision(
            Vector3 centerA, float radiusA,
            Vector3 centerB, float radiusB,
            out Vector3 normal, out float penetration)
        {
            Vector3 delta = centerB - centerA;
            float distSq = delta.sqrMagnitude;
            float radiusSum = radiusA + radiusB;
            
            if (distSq < radiusSum * radiusSum)
            {
                float dist = Mathf.Sqrt(distSq);
                if (dist < PhysicsConstants.EPSILON)
                {
                    normal = Vector3.up;
                    penetration = radiusSum;
                }
                else
                {
                    normal = delta / dist;
                    penetration = radiusSum - dist;
                }
                return true;
            }
            
            normal = Vector3.zero;
            penetration = 0f;
            return false;
        }

        /// <summary>
        /// Checks collision between a sphere and a plane
        /// </summary>
        public static bool CheckSpherePlaneCollision(
            Vector3 sphereCenter, float sphereRadius,
            Vector3 planeNormal, float planeDistance,
            out Vector3 contactPoint, out float penetration)
        {
            float dist = Vector3.Dot(sphereCenter, planeNormal) - planeDistance;
            
            if (dist < sphereRadius)
            {
                penetration = sphereRadius - dist;
                contactPoint = sphereCenter - planeNormal * dist;
                return true;
            }
            
            contactPoint = Vector3.zero;
            penetration = 0f;
            return false;
        }

        /// <summary>
        /// Checks collision between a sphere and an AABB
        /// </summary>
        public static bool CheckSphereAABBCollision(
            Vector3 sphereCenter, float sphereRadius,
            Vector3 aabbMin, Vector3 aabbMax,
            out ContactInfo contact)
        {
            Vector3 closestPoint = new Vector3(
                Mathf.Clamp(sphereCenter.x, aabbMin.x, aabbMax.x),
                Mathf.Clamp(sphereCenter.y, aabbMin.y, aabbMax.y),
                Mathf.Clamp(sphereCenter.z, aabbMin.z, aabbMax.z)
            );

            Vector3 delta = sphereCenter - closestPoint;
            float distSq = delta.sqrMagnitude;

            if (distSq < sphereRadius * sphereRadius)
            {
                float dist = Mathf.Sqrt(distSq);
                Vector3 normal = dist > PhysicsConstants.EPSILON ? delta / dist : Vector3.up;
                float penetration = sphereRadius - dist;
                contact = new ContactInfo(closestPoint, normal, penetration);
                return true;
            }

            contact = ContactInfo.NoCollision;
            return false;
        }
        #endregion

        #region AABB Collision
        /// <summary>
        /// Checks collision between two axis-aligned bounding boxes
        /// </summary>
        public static bool CheckAABBCollision(
            Vector3 minA, Vector3 maxA,
            Vector3 minB, Vector3 maxB)
        {
            return minA.x <= maxB.x && maxA.x >= minB.x &&
                   minA.y <= maxB.y && maxA.y >= minB.y &&
                   minA.z <= maxB.z && maxA.z >= minB.z;
        }

        /// <summary>
        /// Checks collision between two AABBs with detailed contact info
        /// </summary>
        public static bool CheckAABBCollisionDetailed(
            Vector3 minA, Vector3 maxA,
            Vector3 minB, Vector3 maxB,
            out ContactInfo contact)
        {
            if (!CheckAABBCollision(minA, maxA, minB, maxB))
            {
                contact = ContactInfo.NoCollision;
                return false;
            }

            // Calculate overlap on each axis
            float overlapX = Mathf.Min(maxA.x - minB.x, maxB.x - minA.x);
            float overlapY = Mathf.Min(maxA.y - minB.y, maxB.y - minA.y);
            float overlapZ = Mathf.Min(maxA.z - minB.z, maxB.z - minA.z);

            // Find the axis of least penetration
            Vector3 normal;
            float penetration;

            if (overlapX < overlapY && overlapX < overlapZ)
            {
                normal = ((minA.x + maxA.x) < (minB.x + maxB.x)) ? Vector3.right : Vector3.left;
                penetration = overlapX;
            }
            else if (overlapY < overlapZ)
            {
                normal = ((minA.y + maxA.y) < (minB.y + maxB.y)) ? Vector3.up : Vector3.down;
                penetration = overlapY;
            }
            else
            {
                normal = ((minA.z + maxA.z) < (minB.z + maxB.z)) ? Vector3.forward : Vector3.back;
                penetration = overlapZ;
            }

            Vector3 centerA = (minA + maxA) * 0.5f;
            Vector3 centerB = (minB + maxB) * 0.5f;
            Vector3 contactPoint = (centerA + centerB) * 0.5f;

            contact = new ContactInfo(contactPoint, normal, penetration);
            return true;
        }

        /// <summary>
        /// Gets AABB bounds for a sphere
        /// </summary>
        public static void GetSphereAABB(Vector3 center, float radius, out Vector3 min, out Vector3 max)
        {
            Vector3 offset = new Vector3(radius, radius, radius);
            min = center - offset;
            max = center + offset;
        }

        /// <summary>
        /// Gets AABB bounds for a transformed box
        /// </summary>
        public static void GetBoxAABB(Vector3 center, Vector3 halfSize, Quaternion rotation, out Vector3 min, out Vector3 max)
        {
            Vector3[] corners = TransformUtils.GetOBBCorners(center, halfSize, rotation);
            TransformUtils.CalculateAABB(corners, out min, out max);
        }
        #endregion

        #region OBB (Oriented Bounding Box) Collision
        /// <summary>
        /// Checks collision between two oriented bounding boxes using SAT (Separating Axis Theorem)
        /// </summary>
        public static bool CheckOBBCollision(
            Vector3 centerA, Vector3 halfExtentsA, Quaternion rotationA,
            Vector3 centerB, Vector3 halfExtentsB, Quaternion rotationB)
        {
            // Get axes for both boxes
            Vector3[] axesA = new Vector3[]
            {
                TransformUtils.GetRight(rotationA),
                TransformUtils.GetUp(rotationA),
                TransformUtils.GetForward(rotationA)
            };

            Vector3[] axesB = new Vector3[]
            {
                TransformUtils.GetRight(rotationB),
                TransformUtils.GetUp(rotationB),
                TransformUtils.GetForward(rotationB)
            };

            Vector3 t = centerB - centerA;

            // Test axes of box A
            for (int i = 0; i < 3; i++)
            {
                if (!TestAxis(axesA[i], halfExtentsA, axesB, halfExtentsB, t))
                    return false;
            }

            // Test axes of box B
            for (int i = 0; i < 3; i++)
            {
                if (!TestAxis(axesB[i], halfExtentsA, axesA, halfExtentsB, t))
                    return false;
            }

            // Test cross products of axes
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Vector3 axis = Vector3.Cross(axesA[i], axesB[j]);
                    if (axis.sqrMagnitude < PhysicsConstants.EPSILON_SMALL)
                        continue;

                    axis.Normalize();
                    if (!TestAxis(axis, halfExtentsA, axesA, halfExtentsB, t))
                        return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Helper method to test a separating axis for OBB collision
        /// </summary>
        private static bool TestAxis(Vector3 axis, Vector3 halfExtentsA, Vector3[] axesA, Vector3 halfExtentsB, Vector3 t)
        {
            float ra = halfExtentsA.x * Mathf.Abs(Vector3.Dot(axesA[0], axis)) +
                      halfExtentsA.y * Mathf.Abs(Vector3.Dot(axesA[1], axis)) +
                      halfExtentsA.z * Mathf.Abs(Vector3.Dot(axesA[2], axis));

            float rb = halfExtentsB.x * Mathf.Abs(Vector3.Dot(axesA[0], axis)) +
                      halfExtentsB.y * Mathf.Abs(Vector3.Dot(axesA[1], axis)) +
                      halfExtentsB.z * Mathf.Abs(Vector3.Dot(axesA[2], axis));

            float distance = Mathf.Abs(Vector3.Dot(t, axis));

            return distance <= ra + rb;
        }
        #endregion

        #region Ray Collision
        /// <summary>
        /// Performs a raycast against a sphere
        /// </summary>
        public static bool RaycastSphere(
            Vector3 rayOrigin, Vector3 rayDirection,
            Vector3 sphereCenter, float sphereRadius,
            out float hitDistance)
        {
            Vector3 m = rayOrigin - sphereCenter;
            float b = Vector3.Dot(m, rayDirection);
            float c = Vector3.Dot(m, m) - sphereRadius * sphereRadius;

            if (c > 0f && b > 0f)
            {
                hitDistance = 0f;
                return false;
            }

            float discriminant = b * b - c;
            if (discriminant < 0f)
            {
                hitDistance = 0f;
                return false;
            }

            hitDistance = -b - Mathf.Sqrt(discriminant);
            if (hitDistance < 0f)
                hitDistance = 0f;

            return true;
        }

        /// <summary>
        /// Performs a raycast against an AABB
        /// </summary>
        public static bool RaycastAABB(
            Vector3 rayOrigin, Vector3 rayDirection,
            Vector3 aabbMin, Vector3 aabbMax,
            out float hitDistance)
        {
            float tMin = 0f;
            float tMax = float.MaxValue;

            for (int i = 0; i < 3; i++)
            {
                float origin = i == 0 ? rayOrigin.x : (i == 1 ? rayOrigin.y : rayOrigin.z);
                float dir = i == 0 ? rayDirection.x : (i == 1 ? rayDirection.y : rayDirection.z);
                float min = i == 0 ? aabbMin.x : (i == 1 ? aabbMin.y : aabbMin.z);
                float max = i == 0 ? aabbMax.x : (i == 1 ? aabbMax.y : aabbMax.z);

                if (Mathf.Abs(dir) < PhysicsConstants.EPSILON)
                {
                    if (origin < min || origin > max)
                    {
                        hitDistance = 0f;
                        return false;
                    }
                }
                else
                {
                    float t1 = (min - origin) / dir;
                    float t2 = (max - origin) / dir;

                    if (t1 > t2)
                    {
                        float temp = t1;
                        t1 = t2;
                        t2 = temp;
                    }

                    tMin = Mathf.Max(tMin, t1);
                    tMax = Mathf.Min(tMax, t2);

                    if (tMin > tMax)
                    {
                        hitDistance = 0f;
                        return false;
                    }
                }
            }

            hitDistance = tMin;
            return true;
        }

        /// <summary>
        /// Performs a raycast against a plane
        /// </summary>
        public static bool RaycastPlane(
            Vector3 rayOrigin, Vector3 rayDirection,
            Vector3 planeNormal, float planeDistance,
            out float hitDistance)
        {
            float denom = Vector3.Dot(rayDirection, planeNormal);

            if (Mathf.Abs(denom) < PhysicsConstants.EPSILON)
            {
                hitDistance = 0f;
                return false;
            }

            hitDistance = (planeDistance - Vector3.Dot(rayOrigin, planeNormal)) / denom;
            return hitDistance >= 0f;
        }
        #endregion

        #region Plane Utilities
        /// <summary>
        /// Creates a plane from a point and normal
        /// </summary>
        public static void CreatePlane(Vector3 point, Vector3 normal, out Vector3 planeNormal, out float planeDistance)
        {
            planeNormal = normal.normalized;
            planeDistance = Vector3.Dot(point, planeNormal);
        }

        /// <summary>
        /// Calculates the signed distance from a point to a plane
        /// </summary>
        public static float DistanceToPlane(Vector3 point, Vector3 planeNormal, float planeDistance)
        {
            return Vector3.Dot(point, planeNormal) - planeDistance;
        }

        /// <summary>
        /// Projects a point onto a plane
        /// </summary>
        public static Vector3 ProjectPointOnPlane(Vector3 point, Vector3 planeNormal, float planeDistance)
        {
            float distance = DistanceToPlane(point, planeNormal, planeDistance);
            return point - planeNormal * distance;
        }
        #endregion

        #region Collision Response Helpers
        /// <summary>
        /// Calculates the impulse needed to resolve a collision
        /// </summary>
        public static Vector3 CalculateCollisionImpulse(
            Vector3 relativeVelocity,
            Vector3 normal,
            float restitution,
            float invMassA,
            float invMassB,
            Vector3 contactPointA,
            Vector3 contactPointB,
            Vector3 centerA,
            Vector3 centerB,
            Matrix4x4 invInertiaA,
            Matrix4x4 invInertiaB)
        {
            Vector3 rA = contactPointA - centerA;
            Vector3 rB = contactPointB - centerB;

            Vector3 angularA = Vector3.Cross(MathUtils.MultiplyMatrixVector3(invInertiaA, Vector3.Cross(rA, normal)), rA);
            Vector3 angularB = Vector3.Cross(MathUtils.MultiplyMatrixVector3(invInertiaB, Vector3.Cross(rB, normal)), rB);

            float denominator = invMassA + invMassB + Vector3.Dot(angularA + angularB, normal);

            if (denominator < PhysicsConstants.EPSILON)
                return Vector3.zero;

            float numerator = -(1f + restitution) * Vector3.Dot(relativeVelocity, normal);
            float impulseMagnitude = numerator / denominator;

            return normal * impulseMagnitude;
        }

        /// <summary>
        /// Calculates friction impulse
        /// </summary>
        public static Vector3 CalculateFrictionImpulse(
            Vector3 relativeVelocity,
            Vector3 normal,
            Vector3 normalImpulse,
            float friction)
        {
            Vector3 tangent = relativeVelocity - normal * Vector3.Dot(relativeVelocity, normal);
            float tangentMagnitude = tangent.magnitude;

            if (tangentMagnitude < PhysicsConstants.EPSILON)
                return Vector3.zero;

            tangent /= tangentMagnitude;

            float frictionMagnitude = Mathf.Min(friction * normalImpulse.magnitude, tangentMagnitude);
            return -tangent * frictionMagnitude;
        }

        /// <summary>
        /// Combines friction coefficients from two materials (geometric mean)
        /// </summary>
        public static float CombineFriction(float friction1, float friction2)
        {
            return Mathf.Sqrt(friction1 * friction2);
        }

        /// <summary>
        /// Checks collision between an object and the ground plane
        /// </summary>
        public static bool CheckGroundCollision(Vector3 position, float halfHeight, float groundLevel, out Vector3 contactPoint, out float penetration)
        {
            float bottomY = position.y - halfHeight;
            
            if (bottomY <= groundLevel)
            {
                penetration = groundLevel - bottomY;
                contactPoint = new Vector3(position.x, groundLevel, position.z);
                return true;
            }
            
            contactPoint = Vector3.zero;
            penetration = 0f;
            return false;
        }

        /// <summary>
        /// Checks if two 1D intervals overlap (for AABB collision detection)
        /// </summary>
        public static bool IntervalsOverlap(float min1, float max1, float min2, float max2)
        {
            return min1 <= max2 && max1 >= min2;
        }
        #endregion

        #region Sphere vs OBB
        /// <summary>
        /// Sphere vs OBB collision test (returns contact info). OBB defined by center, halfExtents, rotation.
        /// </summary>
        public static bool CheckSphereOBB(
            Vector3 sphereCenter, float sphereRadius,
            Vector3 obbCenter, Vector3 obbHalfExtents, Quaternion obbRotation,
            out ContactInfo contact)
        {
            // Transform sphere center into OBB local space
            Vector3 local = TransformUtils.InverseTransformPoint(sphereCenter, obbCenter, obbRotation, Vector3.one);
            // Closest point in local box
            Vector3 clamped = new Vector3(
                Mathf.Clamp(local.x, -obbHalfExtents.x, obbHalfExtents.x),
                Mathf.Clamp(local.y, -obbHalfExtents.y, obbHalfExtents.y),
                Mathf.Clamp(local.z, -obbHalfExtents.z, obbHalfExtents.z)
            );
            // Back to world
            Vector3 closestWorld = TransformUtils.TransformPoint(clamped, obbCenter, obbRotation, Vector3.one);
            Vector3 delta = sphereCenter - closestWorld;
            float distSq = delta.sqrMagnitude;
            float rSq = sphereRadius * sphereRadius;
            if (distSq <= rSq)
            {
                float dist = Mathf.Sqrt(Mathf.Max(distSq, PhysicsConstants.EPSILON));
                Vector3 n = dist > PhysicsConstants.EPSILON ? delta / dist : Vector3.up;
                float penetration = sphereRadius - dist;
                contact = new ContactInfo(closestWorld, n, penetration);
                return true;
            }
            contact = ContactInfo.NoCollision;
            return false;
        }
        #endregion
    }
}
