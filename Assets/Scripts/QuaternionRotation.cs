using UnityEngine;

// Handles rotation using quaternions (axis-angle math)
// Supports both local and global rotation modes
public class QuaternionRotation : MonoBehaviour
{
    // Axis of rotation (normalized)
    public Vector3 axis = Vector3.up;
    // Rotation speed (degrees/sec, magnitude of rotationSpeedXYZ)
    public float angleSpeed;
    // Accumulated transformation matrix for rotation
    public Matrix4x4 accumulatedTransform = Matrix4x4.identity;
    // If true, apply local rotation; else global
    public bool isLocal = true;

    // Applies continuous rotation using axis-angle quaternion
    // Each frame, rotates by a small angle around the specified axis
    public void ApplyRotation(float t)
    {
        float dtFrame = Time.fixedDeltaTime;
        // Calculate the incremental rotation angle for this frame (in radians)
        float angleRad = Mathf.Deg2Rad * angleSpeed * dtFrame;
        // Normalize axis to ensure valid quaternion
        Vector3 normAxis = axis.normalized;
        float halfAngle = angleRad / 2f;
        float sinHalf = Mathf.Sin(halfAngle);
        // Build quaternion from axis and angle
        Quaternion q = new Quaternion(normAxis.x * sinHalf, normAxis.y * sinHalf, normAxis.z * sinHalf, Mathf.Cos(halfAngle));
        // Apply rotation: local (post-multiply) or global (pre-multiply)
        if (isLocal)
            accumulatedTransform = accumulatedTransform * Matrix4x4.Rotate(q);
        else
            accumulatedTransform = Matrix4x4.Rotate(q) * accumulatedTransform;
    }
}
