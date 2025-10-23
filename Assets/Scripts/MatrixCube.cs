using UnityEngine;

[RequireComponent(typeof(MatrixCubeMesh))]
public class MatrixCube : MonoBehaviour
{
    [Header("Free fall settings")]
    [SerializeField] float gravity = 9.81f;
    [SerializeField] float drag = 0.1f;
    [SerializeField] float dt = 0.02f;

    [Header("Initial position")]
    public Vector3 startPosition = Vector3.zero;

    [SerializeField] private Vector3 velocity;
    [SerializeField] private Vector3 position;

    [Header("Rotation speeds (degrees/sec)")]
    [SerializeField] Vector3 rotationSpeedXYZ = Vector3.zero;
    [Header("Transform type")]
    [SerializeField] bool isLocal = true;
    [Header("Rotation mode")]
    [SerializeField] bool useQuaternion = false;

    [Header("Script references")]
    [SerializeField] private MatrixCubeMesh meshScript;
    [SerializeField] private EulerRotation eulerScript;
    [SerializeField] private QuaternionRotation quatScript;

    [Header("Ground detection")]
    [SerializeField] private GameObject groundPlane;

    private bool landed = false;
    public float e = 0.5f;
    public float epsilon = 1e-4f; // tolerance for bisection

    void Start()
    {
        velocity = Vector3.zero;
        position = startPosition;

        if (meshScript == null)
            meshScript = GetComponent<MatrixCubeMesh>();
        if (eulerScript == null)
            eulerScript = GetComponent<EulerRotation>();
        if (quatScript == null)
            quatScript = GetComponent<QuaternionRotation>();
    }
    void FixedUpdate()
    {
        if (!landed)
        {
            // Compute acceleration (could later include more forces)
            Vector3 acceleration = Vector3.down * gravity - drag * velocity;

            // Predict new velocity and position
            Vector3 newVelocity = velocity + acceleration * dt;
            Vector3 newPosition = position + velocity * dt;

            // --- Plane collision detection using dichotomy (bisection) ---
            if (groundPlane != null)
            {
                Vector3 planeNormal = groundPlane.transform.up.normalized;
                Vector3 planePoint = groundPlane.transform.position;

                float f_a = Vector3.Dot(position - planePoint, planeNormal);
                float f_b = Vector3.Dot(newPosition - planePoint, planeNormal);

                // Check if the segment crosses the plane
                if (f_a * f_b < 0f)
                {
                    landed = true;

                    // Use dichotomy to find t* where f(t*) = 0
                    float a = 0f;
                    float b = dt;
                    float mid = 0f;

                    while (Mathf.Abs(b - a) > epsilon)
                    {
                        mid = (a + b) * 0.5f;
                        Vector3 testPos = position + velocity * mid + 0.5f * acceleration * mid * mid;
                        float f_mid = Vector3.Dot(testPos - planePoint, planeNormal);

                        if (f_mid * f_a < 0f)
                            b = mid;
                        else
                            a = mid;
                    }

                    float tImpact = (a + b) * 0.5f;

                    // Compute exact impact position and velocity at tImpact
                    Vector3 impactPos = position + velocity * tImpact + 0.5f * acceleration * tImpact * tImpact;
                    Vector3 impactVel = velocity + acceleration * tImpact;

                    // Reflect velocity across plane (bounce)
                    float vDotN = Vector3.Dot(impactVel, planeNormal);
                    impactVel = impactVel - (1 + e) * vDotN * planeNormal;

                    // Continue motion for remaining time after impact
                    float tRemain = dt - tImpact;
                    newVelocity = impactVel + acceleration * tRemain;
                    newPosition = impactPos + impactVel * tRemain + 0.5f * acceleration * tRemain * tRemain;
                }
            }

            // Commit new state
            velocity = newVelocity;
            position = newPosition;
        }

        // --- Build translation matrix ---
        Matrix4x4 T = Matrix4x4.identity;
        T.m03 = position.x;
        T.m13 = position.y;
        T.m23 = position.z;

        // --- Rotation updates ---
        if (eulerScript != null)
            eulerScript.rotationSpeedXYZ = rotationSpeedXYZ;
        if (quatScript != null)
        {
            quatScript.axis = rotationSpeedXYZ.normalized;
            quatScript.angleSpeed = rotationSpeedXYZ.magnitude;
        }

        Matrix4x4 finalTransform = T;
        if (useQuaternion && quatScript != null)
        {
            float t = Time.time;
            quatScript.ApplyRotation(t);
            finalTransform *= quatScript.accumulatedTransform;
        }
        else if (eulerScript != null)
        {
            if (isLocal)
                eulerScript.ApplyLocalRotation();
            else
                eulerScript.ApplyGlobalRotation();
            finalTransform *= eulerScript.accumulatedTransform;
        }

        if (meshScript != null)
            meshScript.UpdateMesh(finalTransform);
    }
}
