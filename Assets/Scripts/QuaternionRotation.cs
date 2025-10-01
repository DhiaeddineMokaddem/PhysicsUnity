using UnityEngine;

// Handles rotation using quaternions (pure math, no Unity Quaternion)
// Supports both local and global rotation modes
public class QuaternionRotation : MonoBehaviour
{
    // Axis of rotation (normalized)
    public Vector3 axis = Vector3.up;
    // Rotation speed (degrees/sec, magnitude of rotationSpeedXYZ)
    public float angleSpeed;
    // Accumulated rotation quaternion (x, y, z, w)
    private Vector4 accumulatedQuat = new Vector4(0, 0, 0, 1); // Identity quaternion
    // Accumulated transformation matrix for rotation
    public Matrix4x4 accumulatedTransform = Matrix4x4.identity;
    // If true, apply local rotation; else global
    public bool isLocal = true;

    // Converts a quaternion (x, y, z, w) to a rotation matrix
    private Matrix4x4 QuaternionToMatrix(Vector4 q)
    {
        float x = q.x, y = q.y, z = q.z, w = q.w;
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;
        Matrix4x4 m = Matrix4x4.identity;
        m.m00 = 1 - 2 * (yy + zz);
        m.m01 = 2 * (xy - wz);
        m.m02 = 2 * (xz + wy);
        m.m10 = 2 * (xy + wz);
        m.m11 = 1 - 2 * (xx + zz);
        m.m12 = 2 * (yz - wx);
        m.m20 = 2 * (xz - wy);
        m.m21 = 2 * (yz + wx);
        m.m22 = 1 - 2 * (xx + yy);
        return m;
    }

    // Multiplies two quaternions
    private Vector4 MultiplyQuat(Vector4 q1, Vector4 q2)
    {
        float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
        float y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
        float z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
        float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        return new Vector4(x, y, z, w);
    }

    // Normalizes a quaternion
    private Vector4 NormalizeQuat(Vector4 q)
    {
        float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (mag < 1e-8f) return new Vector4(0, 0, 0, 1);
        return q / mag;
    }

    // Builds a quaternion from axis and angle (radians)
    private Vector4 AxisAngleQuat(Vector3 axis, float angle)
    {
        Vector3 normAxis = axis.normalized;
        float half = angle / 2f;
        float sinHalf = Mathf.Sin(half);
        return new Vector4(normAxis.x * sinHalf, normAxis.y * sinHalf, normAxis.z * sinHalf, Mathf.Cos(half));
    }

    // SLERP between two quaternions using pure math
    // q0, q1: quaternions
    // t: interpolation parameter (0 to 1)
    private Vector4 SlerpQuat(Vector4 q0, Vector4 q1, float t)
    {
        float dot = q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w;
        // If dot < 0, negate one quaternion to take shortest path
        if (dot < 0f) {
            q1 = -q1;
            dot = -dot;
        }
        dot = Mathf.Clamp(dot, -1f, 1f);
        float theta = Mathf.Acos(dot);
        if (theta < 1e-6f) return q0;
        float sinTheta = Mathf.Sin(theta);
        float w0 = Mathf.Sin((1 - t) * theta) / sinTheta;
        float w1 = Mathf.Sin(t * theta) / sinTheta;
        return NormalizeQuat(new Vector4(
            w0 * q0.x + w1 * q1.x,
            w0 * q0.y + w1 * q1.y,
            w0 * q0.z + w1 * q1.z,
            w0 * q0.w + w1 * q1.w
        ));
    }

    // Rotates a 3D point using quaternion: P' = q · P · q⁻¹
    // P is (0, x, y, z), q is (x, y, z, w)
    // Returns rotated Vector3
    public Vector3 RotatePointByQuaternion(Vector3 point, Vector4 quat)
    {
        // Convert point to pure quaternion (0, x, y, z)
        Vector4 p = new Vector4(point.x, point.y, point.z, 0f);
        // Compute q⁻¹ (conjugate)
        Vector4 qConj = new Vector4(-quat.x, -quat.y, -quat.z, quat.w);
        // Compute q · p
        Vector4 qp = MultiplyQuat(quat, p);
        // Compute (q · p) · q⁻¹
        Vector4 result = MultiplyQuat(qp, qConj);
        // Return vector part (x, y, z)
        return new Vector3(result.x, result.y, result.z);
    }

    // Applies continuous rotation using axis-angle quaternion (pure math)
    // Each frame, rotates by a small angle around the specified axis
    public void ApplyRotation(float t)
    {
        float dtFrame = Time.fixedDeltaTime;
        // Build incremental quaternion for this frame
        Vector4 dq = AxisAngleQuat(axis, Mathf.Deg2Rad * angleSpeed * dtFrame);
        // Update accumulated quaternion (local/global)
        if (isLocal)
            accumulatedQuat = MultiplyQuat(accumulatedQuat, dq);
        else
            accumulatedQuat = MultiplyQuat(dq, accumulatedQuat);
        accumulatedQuat = NormalizeQuat(accumulatedQuat);
        // Build rotation matrix for compatibility (not used for mesh anymore)
        accumulatedTransform = QuaternionToMatrix(accumulatedQuat);
    }

    // Rotates all mesh vertices using pure quaternion math
    // Call this from your mesh script instead of using a matrix
    public Vector3[] RotateVertices(Vector3[] vertices)
    {
        Vector3[] rotated = new Vector3[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            rotated[i] = RotatePointByQuaternion(vertices[i], accumulatedQuat);
        return rotated;
    }

}
