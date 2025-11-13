using UnityEngine;
using PhysicsSimulation.Core;

/// <summary>
/// Matrix-based cube with free fall physics and rotation
/// Refactored to use shared utilities from PhysicsUnity.Core
/// </summary>
[RequireComponent(typeof(MatrixCubeMesh))]
public class MatrixCube : MonoBehaviour
{
    /*#region Inspector Properties
    [Header("Free fall settings")]
    [SerializeField] float gravity = PhysicsConstants.GRAVITY;
    [SerializeField] float drag = 0.1f;
    [SerializeField] float dt = PhysicsConstants.DEFAULT_TIME_STEP;

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
    [SerializeField] private EulerRotationc eulerScript;
    [SerializeField] private QuaternionRotation quatScript;

    [Header("Ground detection")]
    [SerializeField] private GameObject groundPlane;

    public float e = PhysicsConstants.DEFAULT_RESTITUTION;
    public float epsilon = PhysicsConstants.EPSILON_SMALL;
    #endregion

    #region Private Fields
    private bool landed = false;
    #endregion

    #region Initialization
    void Start()
    {
        velocity = Vector3.zero;
        position = startPosition;

        if (meshScript == null) meshScript = GetComponent<MatrixCubeMesh>();
        if (eulerScript == null) eulerScript = GetComponent<EulerRotationc>();
        if (quatScript == null) quatScript = GetComponent<QuaternionRotation>();
    }
    #endregion

    #region Physics Update
    void FixedUpdate()
    {
        if (!landed)
        {
            Vector3 acceleration = PhysicsConstants.GRAVITY_VECTOR - drag * velocity;
            Vector3 newVelocity = IntegrationUtils.IntegrateVelocityEuler(velocity, acceleration, dt);
            Vector3 newPosition = IntegrationUtils.IntegratePositionEuler(position, velocity, dt);

            if (groundPlane != null)
            {
                Vector3 planeNormal = groundPlane.transform.up.normalized;
                Vector3 planePoint = groundPlane.transform.position;

                float f_a = Vector3.Dot(position - planePoint, planeNormal);
                float f_b = Vector3.Dot(newPosition - planePoint, planeNormal);

                if (f_a * f_b < 0f)
                {
                    landed = true;

                    float a = 0f;
                    float b = dt;
                    float mid = 0f;

                    while (Mathf.Abs(b - a) > epsilon)
                    {
                        mid = (a + b) * 0.5f;
                        Vector3 testPos = position + velocity * mid + 0.5f * acceleration * mid * mid;
                        float f_mid = Vector3.Dot(testPos - planePoint, planeNormal);

                        if (f_mid * f_a < 0f) b = mid;
                        else a = mid;
                    }

                    float tImpact = (a + b) * 0.5f;

                    Vector3 impactPos = position + velocity * tImpact + 0.5f * acceleration * tImpact * tImpact;
                    Vector3 impactVel = velocity + acceleration * tImpact;

                    float vDotN = Vector3.Dot(impactVel, planeNormal);
                    impactVel = impactVel - (1 + e) * vDotN * planeNormal;

                    float tRemain = dt - tImpact;
                    newVelocity = impactVel + acceleration * tRemain;
                    newPosition = impactPos + impactVel * tRemain + 0.5f * acceleration * tRemain * tRemain;
                }
            }

            velocity = newVelocity;
            position = newPosition;
        }

        Matrix4x4 T = Matrix4x4.identity;
        T.m03 = position.x;
        T.m13 = position.y;
        T.m23 = position.z;

        if (eulerScript != null) eulerScript.rotationSpeedXYZ = rotationSpeedXYZ;
        if (quatScript != null)
        {
            quatScript.axis = rotationSpeedXYZ.normalized;
            quatScript.angleSpeed = rotationSpeedXYZ.magnitude;
        }

        Matrix4x4 finalTransform = T;
        if (useQuaternion && quatScript != null)
        {
            quatScript.ApplyRotation(Time.time);
            finalTransform *= quatScript.accumulatedTransform;
        }
        else if (eulerScript != null)
        {
            if (isLocal) eulerScript.ApplyLocalRotation();
            else eulerScript.ApplyGlobalRotation();
            finalTransform *= eulerScript.accumulatedTransform;
        }

        if (meshScript != null) meshScript.UpdateMesh(finalTransform);
    }
    #endregion
    */
}
