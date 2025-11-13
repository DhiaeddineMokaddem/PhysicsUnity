namespace Rayen.attempt2
{
    using UnityEngine;

    /// <summary>
    /// Oriented Bounding Box collision detection using Separating Axis Theorem (SAT).
    /// This handles collisions between rotated boxes correctly.
    /// </summary>
    public static class OBBCollision
    {
        public struct CollisionInfo
        {
            public bool hasCollision;
            public Vector3 normal;        // Collision normal (from A to B)
            public float penetration;     // Penetration depth
            public Vector3 contactPoint;  // Point of contact in world space
        }

        // Main SAT collision test for two OBBs
        public static CollisionInfo TestOBB(CustomRigidBody3D a, CustomRigidBody3D b)
        {
            CollisionInfo info = new CollisionInfo();
            info.hasCollision = false;
            info.penetration = float.MaxValue;

            // Get the 15 axes to test (SAT)
            Vector3[] axes = GetSATAxes(a, b);

            Vector3 bestAxis = Vector3.zero;
            float minPenetration = float.MaxValue;

            // Test each axis
            foreach (Vector3 axis in axes)
            {
                if (axis.sqrMagnitude < 0.0001f) continue; // Skip degenerate axes

                Vector3 normalizedAxis = axis.normalized;

                // Project both OBBs onto this axis
                float[] projA = ProjectOBB(a, normalizedAxis);
                float[] projB = ProjectOBB(b, normalizedAxis);

                float overlapMin = Mathf.Max(projA[0], projB[0]);
                float overlapMax = Mathf.Min(projA[1], projB[1]);
                float overlap = overlapMax - overlapMin;

                // If no overlap on this axis, boxes don't collide
                if (overlap < 0)
                {
                    return info; // No collision
                }

                // Track the minimum penetration axis
                if (overlap < minPenetration)
                {
                    minPenetration = overlap;
                    bestAxis = normalizedAxis;

                    // Make sure normal points from A to B
                    Vector3 centerDiff = b.Position - a.Position;
                    if (Vector3.Dot(centerDiff, bestAxis) < 0)
                    {
                        bestAxis = -bestAxis;
                    }
                }
            }

            // If we get here, all axes had overlap - collision detected!
            info.hasCollision = true;
            info.normal = bestAxis;
            info.penetration = minPenetration;
            info.contactPoint = GetContactPoint(a, b, bestAxis);

            return info;
        }

        // Get all 15 SAT axes: 6 face normals + 9 edge cross products
        private static Vector3[] GetSATAxes(CustomRigidBody3D a, CustomRigidBody3D b)
        {
            Vector3[] axes = new Vector3[15];

            // Get face normals from rotation matrices
            // A's face normals (columns of rotation matrix)
            axes[0] = new Vector3(a.R[0, 0], a.R[1, 0], a.R[2, 0]); // X axis
            axes[1] = new Vector3(a.R[0, 1], a.R[1, 1], a.R[2, 1]); // Y axis
            axes[2] = new Vector3(a.R[0, 2], a.R[1, 2], a.R[2, 2]); // Z axis

            // B's face normals
            axes[3] = new Vector3(b.R[0, 0], b.R[1, 0], b.R[2, 0]);
            axes[4] = new Vector3(b.R[0, 1], b.R[1, 1], b.R[2, 1]);
            axes[5] = new Vector3(b.R[0, 2], b.R[1, 2], b.R[2, 2]);

            // Edge-edge cross products (9 combinations)
            int index = 6;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    axes[index] = Vector3.Cross(axes[i], axes[3 + j]);
                    index++;
                }
            }

            return axes;
        }

        // Project OBB onto an axis, returns [min, max]
        private static float[] ProjectOBB(CustomRigidBody3D rb, Vector3 axis)
        {
            float min = float.MaxValue;
            float max = float.MinValue;

            // Project all 8 vertices
            for (int i = 0; i < rb.Vertices.Length; i++)
            {
                Vector3 worldVertex = Math3D.MultiplyMatrixVector3(rb.R, rb.Vertices[i]) + rb.Position;
                float projection = Vector3.Dot(worldVertex, axis);
                
                if (projection < min) min = projection;
                if (projection > max) max = projection;
            }

            return new float[] { min, max };
        }

        // Find approximate contact point (midpoint of closest vertices)
        private static Vector3 GetContactPoint(CustomRigidBody3D a, CustomRigidBody3D b, Vector3 normal)
        {
            // Find closest vertex from A to B's surface
            Vector3 closestOnA = Vector3.zero;
            float maxProjection = float.MinValue;

            for (int i = 0; i < a.Vertices.Length; i++)
            {
                Vector3 worldVertex = Math3D.MultiplyMatrixVector3(a.R, a.Vertices[i]) + a.Position;
                float projection = Vector3.Dot(worldVertex, normal);
                
                if (projection > maxProjection)
                {
                    maxProjection = projection;
                    closestOnA = worldVertex;
                }
            }

            // Find closest vertex from B to A's surface
            Vector3 closestOnB = Vector3.zero;
            float minProjection = float.MaxValue;

            for (int i = 0; i < b.Vertices.Length; i++)
            {
                Vector3 worldVertex = Math3D.MultiplyMatrixVector3(b.R, b.Vertices[i]) + b.Position;
                float projection = Vector3.Dot(worldVertex, normal);
                
                if (projection < minProjection)
                {
                    minProjection = projection;
                    closestOnB = worldVertex;
                }
            }

            // Contact point is midpoint
            return (closestOnA + closestOnB) * 0.5f;
        }

        // Compute accurate AABB for a rotated box (for broad phase)
        public static void GetAABB(CustomRigidBody3D rb, out Vector3 min, out Vector3 max)
        {
            min = Vector3.one * float.MaxValue;
            max = Vector3.one * float.MinValue;

            for (int i = 0; i < rb.Vertices.Length; i++)
            {
                Vector3 worldVertex = Math3D.MultiplyMatrixVector3(rb.R, rb.Vertices[i]) + rb.Position;
                
                min.x = Mathf.Min(min.x, worldVertex.x);
                min.y = Mathf.Min(min.y, worldVertex.y);
                min.z = Mathf.Min(min.z, worldVertex.z);
                
                max.x = Mathf.Max(max.x, worldVertex.x);
                max.y = Mathf.Max(max.y, worldVertex.y);
                max.z = Mathf.Max(max.z, worldVertex.z);
            }
        }

        // Fast AABB overlap test (broad phase)
        public static bool AABBOverlap(Vector3 minA, Vector3 maxA, Vector3 minB, Vector3 maxB)
        {
            return (minA.x <= maxB.x && maxA.x >= minB.x) &&
                   (minA.y <= maxB.y && maxA.y >= minB.y) &&
                   (minA.z <= maxB.z && maxA.z >= minB.z);
        }
    }
}
