using UnityEngine;

namespace PhysicsUnity.Core
{
    /// <summary>
    /// Utility class for debug visualization in the editor
    /// Provides methods to draw physics objects, forces, and collision information
    /// </summary>
    public static class DebugDrawUtils
    {
        #region Basic Shapes
        /// <summary>
        /// Draws a wire sphere in the scene view
        /// </summary>
        public static void DrawWireSphere(Vector3 center, float radius, Color color, int segments = 16)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = color;

            // Draw circles on XY, XZ, and YZ planes
            DrawCircle(center, radius, Vector3.forward, segments);
            DrawCircle(center, radius, Vector3.right, segments);
            DrawCircle(center, radius, Vector3.up, segments);

            Gizmos.color = oldColor;
        }

        /// <summary>
        /// Draws a circle
        /// </summary>
        public static void DrawCircle(Vector3 center, float radius, Vector3 normal, int segments = 32)
        {
            Vector3 forward = Vector3.Slerp(normal, -normal, 0.5f);
            Vector3 right = Vector3.Cross(normal, forward).normalized * radius;
            forward = Vector3.Cross(right, normal).normalized * radius;

            Vector3 prevPoint = center + forward;
            float angleStep = 360f / segments;

            for (int i = 1; i <= segments; i++)
            {
                float angle = i * angleStep;
                Vector3 nextPoint = center + 
                    Quaternion.AngleAxis(angle, normal) * forward;
                Gizmos.DrawLine(prevPoint, nextPoint);
                prevPoint = nextPoint;
            }
        }

        /// <summary>
        /// Draws a wire cube with rotation
        /// </summary>
        public static void DrawWireCube(Vector3 center, Vector3 size, Quaternion rotation, Color color)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = color;

            Vector3 halfSize = size * 0.5f;
            Vector3[] corners = new Vector3[8];
            int idx = 0;

            for (int x = -1; x <= 1; x += 2)
            {
                for (int y = -1; y <= 1; y += 2)
                {
                    for (int z = -1; z <= 1; z += 2)
                    {
                        corners[idx++] = center + rotation * new Vector3(
                            x * halfSize.x,
                            y * halfSize.y,
                            z * halfSize.z
                        );
                    }
                }
            }

            // Draw edges
            Gizmos.DrawLine(corners[0], corners[1]);
            Gizmos.DrawLine(corners[0], corners[2]);
            Gizmos.DrawLine(corners[0], corners[4]);
            Gizmos.DrawLine(corners[1], corners[3]);
            Gizmos.DrawLine(corners[1], corners[5]);
            Gizmos.DrawLine(corners[2], corners[3]);
            Gizmos.DrawLine(corners[2], corners[6]);
            Gizmos.DrawLine(corners[3], corners[7]);
            Gizmos.DrawLine(corners[4], corners[5]);
            Gizmos.DrawLine(corners[4], corners[6]);
            Gizmos.DrawLine(corners[5], corners[7]);
            Gizmos.DrawLine(corners[6], corners[7]);

            Gizmos.color = oldColor;
        }

        /// <summary>
        /// Draws an AABB (axis-aligned bounding box)
        /// </summary>
        public static void DrawAABB(Vector3 min, Vector3 max, Color color)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = color;

            Vector3 center = (min + max) * 0.5f;
            Vector3 size = max - min;
            Gizmos.DrawWireCube(center, size);

            Gizmos.color = oldColor;
        }
        #endregion

        #region Arrows and Vectors
        /// <summary>
        /// Draws an arrow representing a vector
        /// </summary>
        public static void DrawArrow(Vector3 start, Vector3 direction, Color color, float arrowHeadLength = 0.25f, float arrowHeadAngle = 20f)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = color;

            Vector3 end = start + direction;
            Gizmos.DrawLine(start, end);

            if (direction.sqrMagnitude > PhysicsConstants.EPSILON_SMALL)
            {
                Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * Vector3.forward;
                Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * Vector3.forward;
                
                Gizmos.DrawLine(end, end + right * arrowHeadLength);
                Gizmos.DrawLine(end, end + left * arrowHeadLength);
            }

            Gizmos.color = oldColor;
        }

        /// <summary>
        /// Draws a force vector
        /// </summary>
        public static void DrawForce(Vector3 position, Vector3 force, Color color, float scale = 1f)
        {
            DrawArrow(position, force * scale, color);
        }

        /// <summary>
        /// Draws velocity vector
        /// </summary>
        public static void DrawVelocity(Vector3 position, Vector3 velocity, Color color)
        {
            DrawArrow(position, velocity, color);
        }
        #endregion

        #region Contact Points
        /// <summary>
        /// Draws a contact point with normal
        /// </summary>
        public static void DrawContactPoint(Vector3 point, Vector3 normal, float penetration, Color pointColor, Color normalColor)
        {
            Color oldColor = Gizmos.color;

            // Draw contact point
            Gizmos.color = pointColor;
            Gizmos.DrawSphere(point, 0.05f);

            // Draw normal
            Gizmos.color = normalColor;
            DrawArrow(point, normal * 0.5f, normalColor);

            // Draw penetration depth
            if (penetration > PhysicsConstants.EPSILON)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(point, point - normal * penetration);
            }

            Gizmos.color = oldColor;
        }

        /// <summary>
        /// Draws collision information
        /// </summary>
        public static void DrawCollisionInfo(CollisionUtils.ContactInfo contact)
        {
            if (contact.hasCollision)
            {
                DrawContactPoint(contact.point, contact.normal, contact.penetration, Color.yellow, Color.cyan);
            }
        }
        #endregion

        #region Coordinate Frames
        /// <summary>
        /// Draws a coordinate frame (X=Red, Y=Green, Z=Blue)
        /// </summary>
        public static void DrawCoordinateFrame(Vector3 position, Quaternion rotation, float scale = 1f)
        {
            Vector3 right = rotation * Vector3.right * scale;
            Vector3 up = rotation * Vector3.up * scale;
            Vector3 forward = rotation * Vector3.forward * scale;

            Gizmos.color = Color.red;
            Gizmos.DrawLine(position, position + right);
            
            Gizmos.color = Color.green;
            Gizmos.DrawLine(position, position + up);
            
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(position, position + forward);
        }

        /// <summary>
        /// Draws axes with labels
        /// </summary>
        public static void DrawAxes(Vector3 position, Quaternion rotation, float length = 1f)
        {
            DrawArrow(position, rotation * Vector3.right * length, Color.red);
            DrawArrow(position, rotation * Vector3.up * length, Color.green);
            DrawArrow(position, rotation * Vector3.forward * length, Color.blue);
        }
        #endregion

        #region Physics Visualization
        /// <summary>
        /// Draws trajectory prediction
        /// </summary>
        public static void DrawTrajectory(Vector3 startPosition, Vector3 velocity, float gravity, int steps = 20, float timeStep = 0.1f, Color color = default)
        {
            if (color == default)
                color = Color.yellow;

            Color oldColor = Gizmos.color;
            Gizmos.color = color;

            Vector3 pos = startPosition;
            Vector3 vel = velocity;
            Vector3 gravityVec = new Vector3(0, -gravity, 0);

            for (int i = 0; i < steps; i++)
            {
                Vector3 nextPos = pos + vel * timeStep;
                vel += gravityVec * timeStep;

                Gizmos.DrawLine(pos, nextPos);
                pos = nextPos;
            }

            Gizmos.color = oldColor;
        }

        /// <summary>
        /// Draws a spring connection
        /// </summary>
        public static void DrawSpring(Vector3 start, Vector3 end, float coilRadius = 0.1f, int coils = 5, Color color = default)
        {
            if (color == default)
                color = Color.yellow;

            Color oldColor = Gizmos.color;
            Gizmos.color = color;

            Vector3 direction = end - start;
            float length = direction.magnitude;
            
            if (length < PhysicsConstants.EPSILON)
                return;

            direction /= length;
            Vector3 perpendicular = Vector3.Cross(direction, Vector3.up);
            if (perpendicular.sqrMagnitude < PhysicsConstants.EPSILON_SMALL)
                perpendicular = Vector3.Cross(direction, Vector3.right);
            perpendicular.Normalize();

            int segments = coils * 8;
            Vector3 prevPoint = start;

            for (int i = 0; i <= segments; i++)
            {
                float t = i / (float)segments;
                float angle = t * coils * Mathf.PI * 2f;
                
                Vector3 offset = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, direction) * perpendicular * coilRadius;
                Vector3 nextPoint = start + direction * (t * length) + offset;

                Gizmos.DrawLine(prevPoint, nextPoint);
                prevPoint = nextPoint;
            }

            Gizmos.color = oldColor;
        }

        /// <summary>
        /// Draws a constraint between two objects
        /// </summary>
        public static void DrawConstraint(Vector3 point1, Vector3 point2, Color color, bool drawEndPoints = true)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = color;

            Gizmos.DrawLine(point1, point2);

            if (drawEndPoints)
            {
                Gizmos.DrawSphere(point1, 0.05f);
                Gizmos.DrawSphere(point2, 0.05f);
            }

            Gizmos.color = oldColor;
        }
        #endregion

        #region Labels and Text
        /// <summary>
        /// Draws text in 3D space (requires GUI)
        /// </summary>
        public static void DrawLabel(Vector3 position, string text, Color color)
        {
            #if UNITY_EDITOR
            UnityEditor.Handles.color = color;
            UnityEditor.Handles.Label(position, text);
            #endif
        }

        /// <summary>
        /// Draws physics information label
        /// </summary>
        public static void DrawPhysicsInfo(Vector3 position, string name, Vector3 velocity, Vector3 angularVelocity, float mass)
        {
            #if UNITY_EDITOR
            string info = $"{name}\n" +
                         $"Mass: {mass:F2}\n" +
                         $"Vel: {velocity.magnitude:F2}\n" +
                         $"AngVel: {angularVelocity.magnitude:F2}";
            DrawLabel(position, info, Color.white);
            #endif
        }
        #endregion

        #region Grid and Reference
        /// <summary>
        /// Draws a grid on a plane
        /// </summary>
        public static void DrawGrid(Vector3 center, Vector3 normal, float size, int divisions, Color color)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = color;

            Quaternion rotation = Quaternion.LookRotation(normal);
            Vector3 right = rotation * Vector3.right;
            Vector3 forward = rotation * Vector3.forward;

            float step = size / divisions;
            float halfSize = size * 0.5f;

            for (int i = 0; i <= divisions; i++)
            {
                float offset = -halfSize + i * step;
                
                Vector3 start1 = center + right * offset + forward * halfSize;
                Vector3 end1 = center + right * offset - forward * halfSize;
                Gizmos.DrawLine(start1, end1);

                Vector3 start2 = center + forward * offset + right * halfSize;
                Vector3 end2 = center + forward * offset - right * halfSize;
                Gizmos.DrawLine(start2, end2);
            }

            Gizmos.color = oldColor;
        }

        /// <summary>
        /// Draws a ground plane reference
        /// </summary>
        public static void DrawGroundPlane(Vector3 center, float size = 10f, Color color = default)
        {
            if (color == default)
                color = new Color(0.5f, 0.5f, 0.5f, 0.5f);

            DrawGrid(center, Vector3.up, size, 10, color);
        }
        #endregion
    }
}
