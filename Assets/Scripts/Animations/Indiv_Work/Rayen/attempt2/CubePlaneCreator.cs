namespace Rayen.attempt2
{
    using System.Collections.Generic;
    using UnityEngine;

    public class CubePlaneCreator : MonoBehaviour
    {
        public float width = 3;
        public float height = 3;
        public float spacing = 0.05f;

        public float cubeHeight = 0.05f;
        public float cubeWidth = 0.5f;
        public float cubeDepth = 0.5f;

        public float heightOffset = 3.0f;
        public Material cubeMaterial;

        [Header("Realistic Physics Parameters")]
        public float restitution = 0.1f;
        public float friction = 0.6f;
        public float linearDamping = 0.05f;
        public float angularDamping = 0.1f;

        [Header("Sleep System (Prevents micro-movements)")]
        public float sleepLinearVelocity = 0.02f;
        public float sleepAngularVelocity = 0.02f;
        public float sleepTimeThreshold = 0.2f;
        public float groundDampingMultiplier = 5.0f;

        [Header("Stability Improvements")]
        public float groundContactThreshold = 0.02f;
        public bool preventEdgeBalancing = true;
        public float stabilityThreshold = 0.15f;

        [Header("Cube-to-Cube Collision")]
        public bool enableCubeCubeCollision = true;
        public float cubeRestitution = 0.1f;

        private List<CustomRigidBody3D> customRigidBodies = new List<CustomRigidBody3D>();
        private List<GameObject> cubeGameObjects = new List<GameObject>();
        private List<float> sleepTimers = new List<float>();
        private List<bool> isSleeping = new List<bool>();
        public CustomSceneObjects sceneObjects;

        void Start()
        {
            CreateCubePlane();
        }

        void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;

            for (int i = 0; i < customRigidBodies.Count; i++)
            {
                var rb = customRigidBodies[i];

                // Skip sleeping bodies
                if (isSleeping[i])
                {
                    if (rb.Velocity.magnitude > sleepLinearVelocity * 2 ||
                        rb.GetAngularVelocity().magnitude > sleepAngularVelocity * 2)
                    {
                        isSleeping[i] = false;
                        sleepTimers[i] = 0f;
                    }
                    else
                    {
                        continue; // Stay asleep
                    }
                }

                // Apply gravity
                Vector3 gravityForce = Vector3.down * rb.Mass * rb.Gravity;
                rb.ApplyForce(gravityForce, rb.Position);

                // Check ground contact
                bool onGround = IsOnGround(rb);
                int numGroundContacts = CountGroundContacts(rb);

                // Apply extra damping on ground
                float currentLinearDamping = linearDamping;
                float currentAngularDamping = angularDamping;

                if (onGround)
                {
                    currentLinearDamping *= groundDampingMultiplier;
                    currentAngularDamping *= groundDampingMultiplier;

                    // Heavily damp small residual motion
                    if (rb.Velocity.magnitude < 0.05f) rb.Velocity *= 0.01f;
                    if (rb.GetAngularVelocity().magnitude < 0.05f) rb.L *= 0.01f;

                    // Snap to ground to avoid micro-bounces
                    rb.Position.y = sceneObjects.planeY + cubeHeight * 0.5f;

                    // Prevent edge balancing
                    if (preventEdgeBalancing && numGroundContacts > 0)
                    {
                        float contactRatio = numGroundContacts / 4.0f;
                        if (contactRatio < stabilityThreshold)
                        {
                            Vector3 tipDirection = GetTipDirection(rb);
                            Vector3 torque = tipDirection * rb.Mass * 2.0f;
                            rb.L += torque * dt;
                        }
                    }
                }

                // Apply damping forces
                Vector3 dampingForce = -currentLinearDamping * rb.Velocity * rb.Mass;
                rb.ApplyForce(dampingForce, rb.Position);

                Vector3 omega = rb.GetAngularVelocity();
                float angularDampingFactor = Mathf.Pow(1.0f - currentAngularDamping, dt);
                rb.L *= angularDampingFactor;

                // Update physics
                rb.UpdatePhysics();

                // Handle collisions
                HandlePlaneCollision(rb, dt);
                HandleSphereCollision(rb, dt);

                // Update sleep state
                UpdateSleepState(rb, i, onGround, dt);

                // Update visuals
                cubeGameObjects[i].transform.position = rb.Position;
                MeshFilter mf = cubeGameObjects[i].GetComponent<MeshFilter>();
                Mesh mesh = mf.mesh;
                mesh.vertices = rb.GetTransformedVertices();
                mesh.RecalculateNormals();
            }

            // Cube-to-cube collisions
            if (enableCubeCubeCollision)
            {
                HandleCubeCubeCollisions();
            }
        }

        void UpdateSleepState(CustomRigidBody3D rb, int index, bool onGround, float dt)
        {
            Vector3 omega = rb.GetAngularVelocity();
            float linearSpeed = rb.Velocity.magnitude;
            float angularSpeed = omega.magnitude;

            bool isBelowThreshold = linearSpeed < sleepLinearVelocity &&
                                    angularSpeed < sleepAngularVelocity;

            if (isBelowThreshold && onGround)
            {
                sleepTimers[index] += dt;

                if (sleepTimers[index] >= sleepTimeThreshold)
                {
                    isSleeping[index] = true;
                    rb.Velocity = Vector3.zero;
                    rb.L = Vector3.zero;
                }
            }
            else
            {
                sleepTimers[index] = 0f;
            }
        }

        bool IsOnGround(CustomRigidBody3D rb)
        {
            if (sceneObjects == null) return false;
            float lowestY = rb.GetLowestPointY();
            return Mathf.Abs(lowestY - sceneObjects.planeY) < groundContactThreshold;
        }

        int CountGroundContacts(CustomRigidBody3D rb)
        {
            if (sceneObjects == null) return 0;
            int contacts = 0;
            for (int i = 0; i < rb.Vertices.Length; i++)
            {
                Vector3 worldVertex = Math3D.MultiplyMatrixVector3(rb.R, rb.Vertices[i]) + rb.Position;
                if (Mathf.Abs(worldVertex.y - sceneObjects.planeY) < groundContactThreshold)
                    contacts++;
            }
            return contacts;
        }

        Vector3 GetTipDirection(CustomRigidBody3D rb)
        {
            Vector3 contactCenter = Vector3.zero;
            int contactCount = 0;

            for (int i = 0; i < rb.Vertices.Length; i++)
            {
                Vector3 worldVertex = Math3D.MultiplyMatrixVector3(rb.R, rb.Vertices[i]) + rb.Position;
                if (Mathf.Abs(worldVertex.y - sceneObjects.planeY) < groundContactThreshold)
                {
                    contactCenter += new Vector3(worldVertex.x, 0, worldVertex.z);
                    contactCount++;
                }
            }

            if (contactCount == 0) return Vector3.zero;
            contactCenter /= contactCount;
            Vector3 comXZ = new Vector3(rb.Position.x, 0, rb.Position.z);
            return (comXZ - contactCenter).normalized;
        }

        void HandleCubeCubeCollisions()
        {
            for (int i = 0; i < customRigidBodies.Count; i++)
            {
                // Skip sleeping bodies
                if (isSleeping[i]) continue;
                
                for (int j = i + 1; j < customRigidBodies.Count; j++)
                {
                    // Skip if both sleeping
                    if (isSleeping[i] && isSleeping[j]) continue;
                    
                    var rbA = customRigidBodies[i];
                    var rbB = customRigidBodies[j];
                    
                    // AABB collision detection
                    Vector3 minA = rbA.Position - new Vector3(rbA.A, rbA.B, rbA.C) * 0.5f;
                    Vector3 maxA = rbA.Position + new Vector3(rbA.A, rbA.B, rbA.C) * 0.5f;
                    Vector3 minB = rbB.Position - new Vector3(rbB.A, rbB.B, rbB.C) * 0.5f;
                    Vector3 maxB = rbB.Position + new Vector3(rbB.A, rbB.B, rbB.C) * 0.5f;
                    
                    bool overlap = (minA.x <= maxB.x && maxA.x >= minB.x) &&
                                   (minA.y <= maxB.y && maxA.y >= minB.y) &&
                                   (minA.z <= maxB.z && maxA.z >= minB.z);
                    
                    if (overlap)
                    {
                        ResolveCubeCubeCollision(rbA, rbB);
                        // Wake up both bodies
                        isSleeping[i] = false;
                        isSleeping[j] = false;
                        sleepTimers[i] = 0f;
                        sleepTimers[j] = 0f;
                    }
                }
            }
        }

        void ResolveCubeCubeCollision(CustomRigidBody3D a, CustomRigidBody3D b)
        {
            Vector3 delta = b.Position - a.Position;
            Vector3 overlap = Vector3.zero;
            overlap.x = (a.A + b.A) * 0.5f - Mathf.Abs(delta.x);
            overlap.y = (a.B + b.B) * 0.5f - Mathf.Abs(delta.y);
            overlap.z = (a.C + b.C) * 0.5f - Mathf.Abs(delta.z);
            
            float minOverlap = Mathf.Min(overlap.x, Mathf.Min(overlap.y, overlap.z));
            Vector3 normal = Vector3.zero;
            
            if (minOverlap == overlap.x)
                normal.x = Mathf.Sign(delta.x);
            else if (minOverlap == overlap.y)
                normal.y = Mathf.Sign(delta.y);
            else
                normal.z = Mathf.Sign(delta.z);
            
            float penetration = minOverlap;
            
            // Separate with mass ratio
            float totalMass = a.Mass + b.Mass;
            a.Position -= normal * penetration * (b.Mass / totalMass);
            b.Position += normal * penetration * (a.Mass / totalMass);
            
            // Calculate relative velocity
            float vrel = Vector3.Dot(b.Velocity - a.Velocity, normal);
            
            if (vrel < 0)
            {
                // Realistic impulse with friction
                float impulse = -(1 + cubeRestitution) * vrel / (1 / a.Mass + 1 / b.Mass);
                Vector3 impulseVec = impulse * normal;
                
                a.Velocity -= impulseVec / a.Mass;
                b.Velocity += impulseVec / b.Mass;
                
                // Minimal angular momentum (prevent excessive rotation)
                Vector3 contactPoint = (a.Position + b.Position) * 0.5f;
                Vector3 rA = contactPoint - a.Position;
                Vector3 rB = contactPoint - b.Position;
                
                // Reduced angular effect
                a.L -= Vector3.Cross(rA, impulseVec) * 0.2f;
                b.L += Vector3.Cross(rB, impulseVec) * 0.2f;
            }
        }

        void HandlePlaneCollision(CustomRigidBody3D rb, float dt)
        {
            if (sceneObjects == null) return;
            
            float lowestY = rb.GetLowestPointY();
            
            if (lowestY < sceneObjects.planeY)
            {
                // Find all vertices near ground for multi-contact
                List<Vector3> contactPointsLocal = new List<Vector3>();
                
                for (int i = 0; i < rb.Vertices.Length; i++)
                {
                    Vector3 worldVertex = Math3D.MultiplyMatrixVector3(rb.R, rb.Vertices[i]) + rb.Position;
                    if (worldVertex.y < sceneObjects.planeY + 0.01f) // Slightly above for contact
                    {
                        contactPointsLocal.Add(rb.Vertices[i]);
                    }
                }
                
                float penetration = sceneObjects.planeY - lowestY;
                rb.Position.y += penetration;
                
                // Use average contact point for more stability
                Vector3 avgContactLocal = Vector3.zero;
                foreach (var cp in contactPointsLocal)
                {
                    avgContactLocal += cp;
                }
                if (contactPointsLocal.Count > 0)
                {
                    avgContactLocal /= contactPointsLocal.Count;
                }
                
                Vector3 contactPoint = Math3D.MultiplyMatrixVector3(rb.R, avgContactLocal) + rb.Position;
                contactPoint.y = sceneObjects.planeY;
                
                Vector3 r = contactPoint - rb.Position;
                Matrix4x4 IInv = rb.R * rb.IbodyInv * rb.R.transpose;
                Vector3 omega = Math3D.MultiplyMatrixVector3(IInv, rb.L);
                Vector3 velocityAtContact = rb.Velocity + Vector3.Cross(omega, r);
                
                Vector3 normal = Vector3.up;
                float vn = Vector3.Dot(velocityAtContact, normal);
                
                if (vn < 0)
                {
                    Vector3 vt = velocityAtContact - vn * normal;
                    
                    Vector3 rCrossN = Vector3.Cross(r, normal);
                    Vector3 rCrossNOverI = Math3D.MultiplyMatrixVector3(IInv, rCrossN);
                    float effectiveMass = 1.0f / rb.Mass + Vector3.Dot(rCrossN, rCrossNOverI);
                    
                    float jn = -(1 + restitution) * vn / effectiveMass;
                    
                    Vector3 impulseN = jn * normal;
                    rb.Velocity += impulseN / rb.Mass;
                    rb.L += Vector3.Cross(r, impulseN);
                    
                    // Friction
                    if (vt.magnitude > 0.001f)
                    {
                        Vector3 tangent = vt.normalized;
                        float vtMag = vt.magnitude;
                        
                        Vector3 rCrossT = Vector3.Cross(r, tangent);
                        Vector3 rCrossTOverI = Math3D.MultiplyMatrixVector3(IInv, rCrossT);
                        float effectiveMassTangent = 1.0f / rb.Mass + Vector3.Dot(rCrossT, rCrossTOverI);
                        
                        float jt = -vtMag / effectiveMassTangent;
                        jt = Mathf.Clamp(jt, -friction * jn, friction * jn);
                        
                        Vector3 impulseT = jt * tangent;
                        rb.Velocity += impulseT / rb.Mass;
                        rb.L += Vector3.Cross(r, impulseT);
                    }
                }
            }
        }

        void HandleSphereCollision(CustomRigidBody3D rb, float dt)
        {
            if (sceneObjects == null) return;
            
            Vector3 closestPoint = Vector3.zero;
            float minDist = float.MaxValue;
            Vector3 closestVertexLocal = Vector3.zero;
            
            for (int i = 0; i < rb.Vertices.Length; i++)
            {
                Vector3 worldVertex = Math3D.MultiplyMatrixVector3(rb.R, rb.Vertices[i]) + rb.Position;
                float dist = (worldVertex - sceneObjects.spherePosition).magnitude;
                if (dist < minDist)
                {
                    minDist = dist;
                    closestPoint = worldVertex;
                    closestVertexLocal = rb.Vertices[i];
                }
            }
            
            if (minDist < sceneObjects.sphereRadius)
            {
                Vector3 normal = (rb.Position - sceneObjects.spherePosition).normalized;
                float penetration = sceneObjects.sphereRadius - minDist;
                
                rb.Position += normal * penetration;
                
                Vector3 contactPoint = closestPoint;
                Vector3 r = contactPoint - rb.Position;
                
                Matrix4x4 IInv = rb.R * rb.IbodyInv * rb.R.transpose;
                Vector3 omega = Math3D.MultiplyMatrixVector3(IInv, rb.L);
                Vector3 velocityAtContact = rb.Velocity + Vector3.Cross(omega, r);
                
                float vn = Vector3.Dot(velocityAtContact, normal);
                
                if (vn < 0)
                {
                    Vector3 vt = velocityAtContact - vn * normal;
                    
                    Vector3 rCrossN = Vector3.Cross(r, normal);
                    Vector3 rCrossNOverI = Math3D.MultiplyMatrixVector3(IInv, rCrossN);
                    float effectiveMass = 1.0f / rb.Mass + Vector3.Dot(rCrossN, rCrossNOverI);
                    
                    float jn = -(1 + restitution) * vn / effectiveMass;
                    Vector3 impulseN = jn * normal;
                    
                    rb.Velocity += impulseN / rb.Mass;
                    rb.L += Vector3.Cross(r, impulseN);
                    
                    if (vt.magnitude > 0.001f)
                    {
                        Vector3 tangent = vt.normalized;
                        float vtMag = vt.magnitude;
                        
                        Vector3 rCrossT = Vector3.Cross(r, tangent);
                        Vector3 rCrossTOverI = Math3D.MultiplyMatrixVector3(IInv, rCrossT);
                        float effectiveMassTangent = 1.0f / rb.Mass + Vector3.Dot(rCrossT, rCrossTOverI);
                        
                        float jt = -vtMag / effectiveMassTangent;
                        jt = Mathf.Clamp(jt, -friction * jn, friction * jn);
                        
                        Vector3 impulseT = jt * tangent;
                        rb.Velocity += impulseT / rb.Mass;
                        rb.L += Vector3.Cross(r, impulseT);
                    }
                }
            }
        }

        void CreateCubePlane()
        {
            int numCubesX = Mathf.FloorToInt(width / (cubeWidth + spacing));
            int numCubesZ = Mathf.FloorToInt(height / (cubeDepth + spacing));

            for (int i = 0; i < numCubesX; i++)
            {
                for (int j = 0; j < numCubesZ; j++)
                {
                    float x = i * (cubeWidth + spacing);
                    float z = j * (cubeDepth + spacing);
                    Vector3 position = new Vector3(x, heightOffset, z);

                    CubeObject4 cube = new CubeObject4(cubeWidth, cubeHeight, cubeDepth, Color.green);
                    CustomRigidBody3D crb = new CustomRigidBody3D(cubeWidth, cubeHeight, cubeDepth, 1f, position, Color.green, cube.vertices, cube.triangles);
                    customRigidBodies.Add(crb);

                    GameObject cubeGO = new GameObject("Cube_" + i + "_" + j);
                    cubeGO.transform.position = position;
                    MeshFilter mf = cubeGO.AddComponent<MeshFilter>();
                    MeshRenderer mr = cubeGO.AddComponent<MeshRenderer>();
                    mr.material = cubeMaterial;
                    mr.material.color = Color.green;

                    Mesh mesh = new Mesh();
                    mesh.vertices = cube.vertices;
                    mesh.triangles = cube.triangles;
                    mesh.RecalculateNormals();
                    mf.mesh = mesh;

                    cubeGameObjects.Add(cubeGO);
                    sleepTimers.Add(0f);
                    isSleeping.Add(false);
                }
            }
        }
    }
}