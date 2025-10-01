using UnityEngine;

// Main controller for the physics-based cube
// Handles free fall, rotation (Euler or Quaternion), and mesh updates
[RequireComponent(typeof(MatrixCubeMesh))]
public class MatrixCube : MonoBehaviour
{
    // --- Physics settings ---
    [Header("Free fall settings")]
    [SerializeField] float gravity = 9.81f; // Gravity strength (m/s^2)
    [SerializeField] float drag = 0.1f;     // Drag coefficient
    [SerializeField] float dt = 0.02f;      // Fixed timestep for Euler integration

    // Set initial position in editor
    [Header("Initial position")]
    public Vector3 startPosition = Vector3.zero;

    // Internal state for velocity and position
    [SerializeField] private Vector3 velocity; // Current velocity of the cube
    [SerializeField] private Vector3 position; // Current position of the cube

    // --- Rotation settings ---
    [Header("Rotation speeds (degrees/sec)")]
    [SerializeField] Vector3 rotationSpeedXYZ = Vector3.zero; // Euler rotation speeds (X, Y, Z)
    [Header("Transform type")]
    [SerializeField] bool isLocal = true; // If true, use local rotation; else global
    [Header("Rotation mode")]
    [SerializeField] bool useQuaternion = false; // If true, use quaternion rotation; else Euler

    // --- Script references ---
    [Header("script references")]
    [SerializeField] private MatrixCubeMesh meshScript; // Handles mesh rendering
    [SerializeField] private EulerRotation eulerScript; // Handles Euler rotation
    [SerializeField] private QuaternionRotation quatScript; // Handles Quaternion rotation

    private bool landed = false; // True when cube hits the ground

    // Called once at the start
    // Initializes velocity, position, and fetches required script references
    void Start()
    {
        velocity = Vector3.zero;
        position = startPosition; // Set initial position from editor
        // Auto-fetch references if not assigned in inspector
        if (meshScript == null)
            meshScript = GetComponent<MatrixCubeMesh>();
        if (eulerScript == null)
            eulerScript = GetComponent<EulerRotation>();
        if (quatScript == null)
            quatScript = GetComponent<QuaternionRotation>();
    }

    // Called every physics frame
    // Handles physics, rotation, and mesh update
    void FixedUpdate()
    {
        // --- Physics: Free fall with drag ---
        // Only update position/velocity if not landed
        if (!landed)
        {
            // Calculate acceleration due to gravity and drag
            Vector3 acceleration = Vector3.down * gravity - drag * velocity;
            velocity += acceleration * dt; // Update velocity
            position += velocity * dt;     // Update position
            // --- Stop cube at ground (y=0) ---
            if (position.y <= 0f)
            {
                position.y = 0f;
                velocity = Vector3.zero;
                landed = true; // Mark as landed
            }
        }

        // --- Build translation matrix from position ---
        // This matrix moves the cube to its current position
        Matrix4x4 T = Matrix4x4.identity;
        T.m03 = position.x;
        T.m13 = position.y;
        T.m23 = position.z;

        // --- Update rotation parameters from inspector ---
        // Euler: rotationSpeedXYZ is used directly
        if (eulerScript != null)
            eulerScript.rotationSpeedXYZ = rotationSpeedXYZ;
        // Quaternion: axis is direction, angleSpeed is magnitude
        if (quatScript != null)
        {
            // Use all axes for quaternion rotation
            quatScript.axis = rotationSpeedXYZ.normalized;
            quatScript.angleSpeed = rotationSpeedXYZ.magnitude;
        }

        // --- Rotation handling ---
        // Combine translation and rotation into final transform
        Matrix4x4 finalTransform = T;
        // Always apply rotation, even when landed
        if (useQuaternion && quatScript != null)
        {
            // Apply quaternion rotation (continuous axis-angle)
            float t = Time.time;
            quatScript.ApplyRotation(t);
            finalTransform *= quatScript.accumulatedTransform;
        }
        else if (eulerScript != null)
        {
            // Apply Euler rotation (local/global)
            if (isLocal)
                eulerScript.ApplyLocalRotation();
            else
                eulerScript.ApplyGlobalRotation();
            finalTransform *= eulerScript.accumulatedTransform;
        }
        // --- Update mesh with new transform ---
        // This visually updates the cube in the scene
        if (meshScript != null)
            meshScript.UpdateMesh(finalTransform);
    }
}
