namespace Rayen.attempt2
{
    using UnityEngine;

    public class CustomRigidBody3D
    {
        public float A, B, C, Mass;
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 P;
        public Vector3 L;
        public Matrix4x4 R;
        public Matrix4x4 Ibody;
        public Matrix4x4 IbodyInv;
        public float DT = 0.02f;
        public Vector3[] Vertices;
        public int[] Triangles;
        public Color Color;

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

            float ixx = (1f / 12f) * mass * (b * b + c * c);
            float iyy = (1f / 12f) * mass * (a * a + c * c);
            float izz = (1f / 12f) * mass * (a * a + b * b);

            Ibody = Matrix4x4.zero;
            Ibody[0, 0] = ixx; Ibody[1, 1] = iyy; Ibody[2, 2] = izz; Ibody[3, 3] = 1;
            IbodyInv = Matrix4x4.zero;
            IbodyInv[0, 0] = 1 / ixx; IbodyInv[1, 1] = 1 / iyy; IbodyInv[2, 2] = 1 / izz; IbodyInv[3, 3] = 1;
        }

        public void ApplyForce(Vector3 f, Vector3 point)
        {
            P += f * DT;
            Vector3 r = point - Position;
            L += Vector3.Cross(r, f) * DT;
        }

        public void UpdatePhysics()
        {
            Velocity = P / Mass;
            Vector3 omega = Math3D.MultiplyMatrixVector3(R * IbodyInv * R.transpose, L);
            Position += Velocity * DT;

            // Omega matrix
            Matrix4x4 omegaMat = Matrix4x4.zero;
            omegaMat[0, 1] = -omega.z; omegaMat[0, 2] = omega.y;
            omegaMat[1, 0] = omega.z; omegaMat[1, 2] = -omega.x;
            omegaMat[2, 0] = -omega.y; omegaMat[2, 1] = omega.x;

            Matrix4x4 rupdate = Math3D.Add(Matrix4x4.identity, Math3D.MultiplyScalar(omegaMat, DT));
            R = Math3D.MultiplyMatrix4x4(rupdate, R);
            R = Math3D.GramSchmidt(R);
        }

        public Vector3[] GetTransformedVertices()
        {
            Vector3[] transformedVertices = new Vector3[Vertices.Length];
            for (int i = 0; i < Vertices.Length; i++)
                transformedVertices[i] = Math3D.MultiplyMatrixVector3(R, Vertices[i]);
            return transformedVertices;
        }
    }
}
