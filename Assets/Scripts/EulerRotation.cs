using UnityEngine;

// Handles rotation using Euler angles
// Provides local and global rotation methods using pure math
public class EulerRotation : MonoBehaviour
{
    // Rotation speed for each axis (degrees/sec)
    // X: pitch, Y: yaw, Z: roll
    public Vector3 rotationSpeedXYZ = Vector3.zero;
    // Accumulated transformation matrix for rotation
    public Matrix4x4 accumulatedTransform = Matrix4x4.identity;

    // Applies local rotation (rotation is post-multiplied)
    // This means the new rotation is applied relative to the object's current orientation
    public void ApplyLocalRotation()
    {
        float dtFrame = Time.fixedDeltaTime;
        // Convert rotation speeds from degrees/sec to radians for this frame
        float alpha = Mathf.Deg2Rad * rotationSpeedXYZ.y * dtFrame; // Yaw (Y axis)
        float beta = Mathf.Deg2Rad * rotationSpeedXYZ.x * dtFrame;  // Pitch (X axis)
        float gamma = Mathf.Deg2Rad * rotationSpeedXYZ.z * dtFrame; // Roll (Z axis)

        // Construct rotation matrices for each axis
        // Rx: rotation around X (pitch)
        Matrix4x4 Rx = new Matrix4x4(
            new Vector4(1, 0, 0, 0),
            new Vector4(0, Mathf.Cos(beta), -Mathf.Sin(beta), 0),
            new Vector4(0, Mathf.Sin(beta), Mathf.Cos(beta), 0),
            new Vector4(0, 0, 0, 1)
        );
        // Ry: rotation around Y (yaw)
        Matrix4x4 Ry = new Matrix4x4(
            new Vector4(Mathf.Cos(alpha), 0, Mathf.Sin(alpha), 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(-Mathf.Sin(alpha), 0, Mathf.Cos(alpha), 0),
            new Vector4(0, 0, 0, 1)
        );
        // Rz: rotation around Z (roll)
        Matrix4x4 Rz = new Matrix4x4(
            new Vector4(Mathf.Cos(gamma), -Mathf.Sin(gamma), 0, 0),
            new Vector4(Mathf.Sin(gamma), Mathf.Cos(gamma), 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );
        // Update accumulated transform (local rotation)
        // Order: Yaw, then Pitch, then Roll
        accumulatedTransform = accumulatedTransform * (Ry * Rx * Rz);
    }

    // Applies global rotation (rotation is pre-multiplied)
    // This means the new rotation is applied in world space, before the object's current orientation
    public void ApplyGlobalRotation()
    {
        float dtFrame = Time.fixedDeltaTime;
        // Convert rotation speeds from degrees/sec to radians for this frame
        float alpha = Mathf.Deg2Rad * rotationSpeedXYZ.y * dtFrame; // Yaw (Y axis)
        float beta = Mathf.Deg2Rad * rotationSpeedXYZ.x * dtFrame;  // Pitch (X axis)
        float gamma = Mathf.Deg2Rad * rotationSpeedXYZ.z * dtFrame; // Roll (Z axis)

        // Construct rotation matrices for each axis
        // Rx: rotation around X (pitch)
        Matrix4x4 Rx = new Matrix4x4(
            new Vector4(1, 0, 0, 0),
            new Vector4(0, Mathf.Cos(beta), -Mathf.Sin(beta), 0),
            new Vector4(0, Mathf.Sin(beta), Mathf.Cos(beta), 0),
            new Vector4(0, 0, 0, 1)
        );
        // Ry: rotation around Y (yaw)
        Matrix4x4 Ry = new Matrix4x4(
            new Vector4(Mathf.Cos(alpha), 0, Mathf.Sin(alpha), 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(-Mathf.Sin(alpha), 0, Mathf.Cos(alpha), 0),
            new Vector4(0, 0, 0, 1)
        );
        // Rz: rotation around Z (roll)
        Matrix4x4 Rz = new Matrix4x4(
            new Vector4(Mathf.Cos(gamma), -Mathf.Sin(gamma), 0, 0),
            new Vector4(Mathf.Sin(gamma), Mathf.Cos(gamma), 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );
        // Update accumulated transform (global rotation)
        // Order: Yaw, then Pitch, then Roll
        accumulatedTransform = (Ry * Rx * Rz) * accumulatedTransform;
    }
}
