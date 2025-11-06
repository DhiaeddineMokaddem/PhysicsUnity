namespace Rayen.attempt2
{
    using UnityEngine;

    public class CustomRigidBody3D
    {
        public float A, B, C, Mass;
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 P; // Accumulated linear momentum change
        public Vector3 L; // Angular momentum
        public Matrix4x4 R; // Rotation matrix
        public Matrix4x4 Ibody; // Body-space inertia tensor
        public Matrix4x4 IbodyInv; // Inverse body-space inertia tensor
        public Color Color;
        public float Gravity = 9.81f;
        public float Damping = 0.5f;
        
        public Vector3[] Vertices;
        public int[] Triangles;
        
        private Vector3 accumulatedForce; // Accumulated forces this frame
        private Vector3 accumulatedTorque; // Accumulated torques this frame

        public CustomRigidBody3D(float a, float b, float c, float mass, Vector3 position, Color color, Vector3[] vertices, int[] triangles)
        {
            A = a;
            B = b;
            C = c;
            Mass = mass;
            Position = position;
            Color = color;
            Vertices = vertices;
            Triangles = triangles;
            Velocity = Vector3.zero;
            P = Vector3.zero;
            L = Vector3.zero;
            R = Matrix4x4.identity;
            accumulatedForce = Vector3.zero;
            accumulatedTorque = Vector3.zero;

            // Inertia tensor for a box
            float ixx = (1f / 12f) * mass * (b * b + c * c);
            float iyy = (1f / 12f) * mass * (a * a + c * c);
            float izz = (1f / 12f) * mass * (a * a + b * b);

            Ibody = Matrix4x4.zero;
            Ibody[0, 0] = ixx; 
            Ibody[1, 1] = iyy; 
            Ibody[2, 2] = izz; 
            Ibody[3, 3] = 1;
            
            IbodyInv = Matrix4x4.zero;
            IbodyInv[0, 0] = 1 / ixx; 
            IbodyInv[1, 1] = 1 / iyy; 
            IbodyInv[2, 2] = 1 / izz; 
            IbodyInv[3, 3] = 1;
        }

        public void ApplyForce(Vector3 f, Vector3 point)
        {
            accumulatedForce += f;
            Vector3 r = point - Position;
            accumulatedTorque += Vector3.Cross(r, f);
        }

        public void UpdatePhysics()
        {
            float dt = Time.fixedDeltaTime;
            
            // Integrate linear motion from accumulated forces
            Vector3 acceleration = accumulatedForce / Mass;
            Velocity += acceleration * dt;
            Position += Velocity * dt;
            
            // Integrate angular motion from accumulated torques
            L += accumulatedTorque * dt;
            
            // Calculate world-space inertia tensor inverse
            Matrix4x4 IInv = R * IbodyInv * R.transpose;
            
            // Calculate angular velocity from angular momentum
            Vector3 omega = Math3D.MultiplyMatrixVector3(IInv, L);
            
            // Build skew-symmetric matrix for omega
            Matrix4x4 omegaMat = Matrix4x4.zero;
            omegaMat[0, 1] = -omega.z; 
            omegaMat[0, 2] = omega.y;
            omegaMat[1, 0] = omega.z; 
            omegaMat[1, 2] = -omega.x;
            omegaMat[2, 0] = -omega.y; 
            omegaMat[2, 1] = omega.x;

            // Update rotation matrix: R' = R + (omega* × R) * dt
            Matrix4x4 rupdate = Math3D.Add(Matrix4x4.identity, Math3D.MultiplyScalar(omegaMat, dt));
            R = Math3D.MultiplyMatrix4x4(rupdate, R);
            
            // Orthonormalize to prevent numerical drift
            R = Math3D.GramSchmidt(R);
            
            // Clear accumulated forces for next frame
            accumulatedForce = Vector3.zero;
            accumulatedTorque = Vector3.zero;
        }

        public Vector3[] GetTransformedVertices()
        {
            Vector3[] transformedVertices = new Vector3[Vertices.Length];
            for (int i = 0; i < Vertices.Length; i++)
            {
                // Only apply rotation - GameObject transform handles position
                transformedVertices[i] = Math3D.MultiplyMatrixVector3(R, Vertices[i]);
            }
            return transformedVertices;
        }
        
        // Get the lowest point of the rotated cube (for collision detection)
        public float GetLowestPointY()
        {
            float lowestY = float.MaxValue;
            for (int i = 0; i < Vertices.Length; i++)
            {
                Vector3 worldVertex = Math3D.MultiplyMatrixVector3(R, Vertices[i]) + Position;
                if (worldVertex.y < lowestY)
                    lowestY = worldVertex.y;
            }
            return lowestY;
        }
        
        // Get angular velocity (useful for debugging)
        public Vector3 GetAngularVelocity()
        {
            Matrix4x4 IInv = R * IbodyInv * R.transpose;
            return Math3D.MultiplyMatrixVector3(IInv, L);
        }
    }
}