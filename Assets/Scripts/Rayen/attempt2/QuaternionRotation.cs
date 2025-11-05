using UnityEngine;

public class QuaternionRotation
{
    public struct MyQuaternion
    {
        public float w, x, y, z;
        public MyQuaternion(float w, float x, float y, float z)
        {
            this.w = w; this.x = x; this.y = y; this.z = z;
        }
    }

    public static MyQuaternion FromAxisAngle(Vector3 axis, float angleDeg)
    {
        float angle = angleDeg * Mathf.Deg2Rad / 2f;
        axis.Normalize();
        return new MyQuaternion(
            Mathf.Cos(angle),
            axis.x * Mathf.Sin(angle),
            axis.y * Mathf.Sin(angle),
            axis.z * Mathf.Sin(angle)
        );
    }

    public static MyQuaternion Multiply(MyQuaternion q1, MyQuaternion q2)
    {
        return new MyQuaternion(
            q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
            q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        );
    }

    public static float[,] ToMatrix(MyQuaternion q)
    {
        float[,] M = new float[4, 4];
        float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

        M[0, 0] = 1 - 2 * (yy + zz); M[0, 1] = 2 * (xy - wz); M[0, 2] = 2 * (xz + wy); M[0, 3] = 0;
        M[1, 0] = 2 * (xy + wz); M[1, 1] = 1 - 2 * (xx + zz); M[1, 2] = 2 * (yz - wx); M[1, 3] = 0;
        M[2, 0] = 2 * (xz - wy); M[2, 1] = 2 * (yz + wx); M[2, 2] = 1 - 2 * (xx + yy); M[2, 3] = 0;
        M[3, 0] = 0; M[3, 1] = 0; M[3, 2] = 0; M[3, 3] = 1;

        return M;
    }
}
