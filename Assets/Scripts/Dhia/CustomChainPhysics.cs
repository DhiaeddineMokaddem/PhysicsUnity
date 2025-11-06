using UnityEngine;

public class CustomChainPhysics : MonoBehaviour
{
    [Header("Chain Settings")]
    [SerializeField] private int linkCount = 10;
    [SerializeField] private float linkLength = 0.5f;
    [SerializeField] private float linkRadius = 0.1f;
    [SerializeField] private Transform anchor; // Object to follow

    [Header("Physics Settings")]
    [SerializeField] private float gravity = 9.81f;
    [SerializeField] private float damping = 0.98f;
    [SerializeField] private int constraintIterations = 5;
    [SerializeField] private float stiffness = 0.8f;

    // Custom data structures
    private Vector3[] positions;
    private Vector3[] prevPositions;
    private Vector3[] velocities;
    private Quaternion[] rotations;
    private float[] masses;

    private Vector3 anchorPos;
    private Vector3 prevAnchorPos;

    void Start()
    {
        InitializeChain();
    }

    void InitializeChain()
    {
        positions = new Vector3[linkCount];
        prevPositions = new Vector3[linkCount];
        velocities = new Vector3[linkCount];
        rotations = new Quaternion[linkCount];
        masses = new float[linkCount];

        // Initialize anchor position
        anchorPos = anchor != null ? anchor.position : transform.position;
        prevAnchorPos = anchorPos;

        // Create chain hanging down from anchor
        for (int i = 0; i < linkCount; i++)
        {
            positions[i] = anchorPos + Vector3.down * linkLength * i;
            prevPositions[i] = positions[i];
            velocities[i] = Vector3.zero;
            rotations[i] = Quaternion.identity;
            masses[i] = 1f;
        }
    }

    void Update()
    {
        float dt = Time.deltaTime;

        // Update anchor position
        if (anchor != null)
        {
            prevAnchorPos = anchorPos;
            anchorPos = anchor.position;
        }

        // Verlet integration
        VerletIntegration(dt);

        // Apply constraints multiple times for stability
        for (int iter = 0; iter < constraintIterations; iter++)
        {
            ApplyConstraints();
        }

        // Update rotations based on link directions
        UpdateRotations();
    }

    void VerletIntegration(float dt)
    {
        for (int i = 0; i < linkCount; i++)
        {
            Vector3 temp = positions[i];

            // Verlet integration: x(t+dt) = 2*x(t) - x(t-dt) + a*dt^2
            Vector3 acceleration = Vector3.down * gravity;

            // Add velocity from anchor movement if first link
            if (i == 0)
            {
                Vector3 anchorVel = (anchorPos - prevAnchorPos) / dt;
                acceleration += anchorVel / dt;
            }

            positions[i] = positions[i] + (positions[i] - prevPositions[i]) * damping + acceleration * dt * dt;
            prevPositions[i] = temp;

            // Update velocity for reference
            velocities[i] = (positions[i] - prevPositions[i]) / dt;
        }
    }

    void ApplyConstraints()
    {
        // First link is attached to anchor
        positions[0] = Vector3.Lerp(positions[0], anchorPos, stiffness);

        // Distance constraints between links
        for (int i = 0; i < linkCount - 1; i++)
        {
            Vector3 delta = positions[i + 1] - positions[i];
            float currentDist = delta.magnitude;

            if (currentDist > 0.0001f)
            {
                float diff = (currentDist - linkLength) / currentDist;
                Vector3 correction = delta * diff * 0.5f;

                // Apply correction based on masses
                float totalMass = masses[i] + masses[i + 1];
                float ratio1 = masses[i + 1] / totalMass;
                float ratio2 = masses[i] / totalMass;

                // Don't move first link as much (it's anchored)
                if (i == 0)
                {
                    positions[i + 1] -= correction * 2f;
                }
                else
                {
                    positions[i] += correction * ratio1;
                    positions[i + 1] -= correction * ratio2;
                }
            }
        }
    }

    void UpdateRotations()
    {
        for (int i = 0; i < linkCount - 1; i++)
        {
            Vector3 direction = (positions[i + 1] - positions[i]).normalized;

            if (direction.magnitude > 0.001f)
            {
                // Calculate rotation to point towards next link
                rotations[i] = Quaternion.LookRotation(direction, Vector3.up);
            }
        }

        // Last link keeps orientation of previous link
        if (linkCount > 1)
        {
            rotations[linkCount - 1] = rotations[linkCount - 2];
        }
    }

    void OnDrawGizmos()
    {
        if (positions == null || positions.Length == 0)
            return;

        // Draw links
        for (int i = 0; i < linkCount; i++)
        {
            // Draw sphere at joint
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(positions[i], linkRadius * 0.5f);

            // Draw cylinder representing link
            if (i < linkCount - 1)
            {
                Gizmos.color = Color.cyan;
                DrawCylinder(positions[i], positions[i + 1], linkRadius);
            }

            // Draw rotation axes
            if (rotations != null && i < rotations.Length)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(positions[i], positions[i] + rotations[i] * Vector3.right * linkLength * 0.3f);

                Gizmos.color = Color.green;
                Gizmos.DrawLine(positions[i], positions[i] + rotations[i] * Vector3.up * linkLength * 0.3f);

                Gizmos.color = Color.blue;
                Gizmos.DrawLine(positions[i], positions[i] + rotations[i] * Vector3.forward * linkLength * 0.3f);
            }
        }

        // Draw anchor
        if (anchor != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(anchorPos, linkRadius);
        }
    }

    void DrawCylinder(Vector3 start, Vector3 end, float radius)
    {
        Vector3 dir = end - start;
        int segments = 8;
        Vector3 perpendicular = Vector3.Cross(dir.normalized, Vector3.up);

        if (perpendicular.magnitude < 0.1f)
            perpendicular = Vector3.Cross(dir.normalized, Vector3.right);

        perpendicular = perpendicular.normalized * radius;

        for (int i = 0; i < segments; i++)
        {
            float angle1 = (float)i / segments * Mathf.PI * 2f;
            float angle2 = (float)(i + 1) / segments * Mathf.PI * 2f;

            Quaternion rot1 = Quaternion.AngleAxis(angle1 * Mathf.Rad2Deg, dir.normalized);
            Quaternion rot2 = Quaternion.AngleAxis(angle2 * Mathf.Rad2Deg, dir.normalized);

            Vector3 p1 = start + rot1 * perpendicular;
            Vector3 p2 = start + rot2 * perpendicular;
            Vector3 p3 = end + rot1 * perpendicular;
            Vector3 p4 = end + rot2 * perpendicular;

            // Draw lines forming cylinder
            Gizmos.DrawLine(p1, p2);
            Gizmos.DrawLine(p3, p4);
            Gizmos.DrawLine(p1, p3);
        }
    }

    // Public methods to get custom position/rotation
    public Vector3 GetLinkPosition(int index)
    {
        if (index >= 0 && index < linkCount)
            return positions[index];
        return Vector3.zero;
    }

    public Quaternion GetLinkRotation(int index)
    {
        if (index >= 0 && index < linkCount)
            return rotations[index];
        return Quaternion.identity;
    }

    public int GetLinkCount()
    {
        return linkCount;
    }
}