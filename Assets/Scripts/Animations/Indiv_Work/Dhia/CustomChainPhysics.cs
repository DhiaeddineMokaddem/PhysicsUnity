using UnityEngine;

public class CustomChainPhysics : MonoBehaviour
{
    [Header("Chain Settings")]
    [SerializeField] private int linkCount = 10;
    [SerializeField] private float linkLength = 0.5f;
    [SerializeField] private float linkRadius = 0.1f;
    [SerializeField] private GameObject chainLinkPrefab; // 3D model for chain link

    [Header("Anchor Settings")]
    [SerializeField] private Transform anchorStart; // First anchor
    [SerializeField] private Transform anchorEnd; // Second anchor
    [SerializeField] private float anchorStartStrength = 100f; // Breaking force threshold
    [SerializeField] private float anchorEndStrength = 100f;
    [SerializeField] private float maxStretchDistance = 15f; // Max distance before break

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

    // 3D model instances
    private GameObject[] linkObjects;

    private Vector3 anchorStartPos;
    private Vector3 prevAnchorStartPos;
    private Vector3 anchorEndPos;
    private Vector3 prevAnchorEndPos;

    // Anchor states
    private bool startAnchorActive = true;
    private bool endAnchorActive = true;
    private float currentTension = 0f;

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
        linkObjects = new GameObject[linkCount];

        // Initialize anchor positions
        anchorStartPos = anchorStart != null ? anchorStart.position : transform.position;
        prevAnchorStartPos = anchorStartPos;

        anchorEndPos = anchorEnd != null ? anchorEnd.position : transform.position + Vector3.right * linkLength * linkCount;
        prevAnchorEndPos = anchorEndPos;

        // Create chain between anchors
        for (int i = 0; i < linkCount; i++)
        {
            float t = (float)i / (linkCount - 1);
            positions[i] = Vector3.Lerp(anchorStartPos, anchorEndPos, t);
            prevPositions[i] = positions[i];
            velocities[i] = Vector3.zero;
            rotations[i] = Quaternion.identity;
            masses[i] = 1f;

            // Instantiate 3D model if prefab is assigned
            if (chainLinkPrefab != null)
            {
                linkObjects[i] = Instantiate(chainLinkPrefab, positions[i], rotations[i], transform);
                linkObjects[i].name = "ChainLink_" + i;
            }
        }

        startAnchorActive = true;
        endAnchorActive = true;
    }

    void Update()
    {
        float dt = Time.deltaTime;

        // Update anchor positions
        if (anchorStart != null && startAnchorActive)
        {
            prevAnchorStartPos = anchorStartPos;
            anchorStartPos = anchorStart.position;
        }

        if (anchorEnd != null && endAnchorActive)
        {
            prevAnchorEndPos = anchorEndPos;
            anchorEndPos = anchorEnd.position;
        }

        // Check for breaking condition
        CheckAnchorBreaking();

        // Verlet integration
        VerletIntegration(dt);

        // Apply constraints multiple times for stability
        for (int iter = 0; iter < constraintIterations; iter++)
        {
            ApplyConstraints();
        }

        // Update rotations based on link directions
        UpdateRotations();

        // Update 3D model positions and rotations
        UpdateLinkModels();
    }

    void CheckAnchorBreaking()
    {
        if (!startAnchorActive && !endAnchorActive)
            return;

        // Calculate chain length
        float chainLength = linkLength * (linkCount - 1);

        // Calculate distance between anchors
        float anchorDistance = Vector3.Distance(anchorStartPos, anchorEndPos);

        // Calculate tension (how much stretched beyond natural length)
        float stretch = anchorDistance - chainLength;
        currentTension = Mathf.Max(0, stretch);

        // Check if stretched too far
        if (stretch > maxStretchDistance)
        {
            // Break the weaker anchor
            if (startAnchorActive && endAnchorActive)
            {
                if (anchorStartStrength <= anchorEndStrength)
                {
                    BreakAnchor(true);
                }
                else
                {
                    BreakAnchor(false);
                }
            }
        }

        // Also check based on force/strength
        if (startAnchorActive && currentTension > anchorStartStrength)
        {
            BreakAnchor(true);
        }
        else if (endAnchorActive && currentTension > anchorEndStrength)
        {
            BreakAnchor(false);
        }
    }

    void BreakAnchor(bool breakStart)
    {
        if (breakStart)
        {
            startAnchorActive = false;
            Debug.Log("Start anchor broke! Tension: " + currentTension);
        }
        else
        {
            endAnchorActive = false;
            Debug.Log("End anchor broke! Tension: " + currentTension);
        }
    }

    void VerletIntegration(float dt)
    {
        for (int i = 0; i < linkCount; i++)
        {
            // Skip anchored links
            if ((i == 0 && startAnchorActive) || (i == linkCount - 1 && endAnchorActive))
                continue;

            Vector3 temp = positions[i];

            // Verlet integration: x(t+dt) = 2*x(t) - x(t-dt) + a*dt^2
            Vector3 acceleration = Vector3.down * gravity;

            // Add velocity from anchor movement for first/last links
            if (i == 0 && startAnchorActive)
            {
                Vector3 anchorVel = (anchorStartPos - prevAnchorStartPos) / dt;
                acceleration += anchorVel / dt;
            }
            else if (i == linkCount - 1 && endAnchorActive)
            {
                Vector3 anchorVel = (anchorEndPos - prevAnchorEndPos) / dt;
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
        // First link attached to start anchor
        if (startAnchorActive)
        {
            positions[0] = Vector3.Lerp(positions[0], anchorStartPos, stiffness);
        }

        // Last link attached to end anchor
        if (endAnchorActive)
        {
            positions[linkCount - 1] = Vector3.Lerp(positions[linkCount - 1], anchorEndPos, stiffness);
        }

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

                // Don't move anchored links
                bool link1Anchored = (i == 0 && startAnchorActive);
                bool link2Anchored = (i + 1 == linkCount - 1 && endAnchorActive);

                if (link1Anchored && !link2Anchored)
                {
                    positions[i + 1] -= correction * 2f;
                }
                else if (link2Anchored && !link1Anchored)
                {
                    positions[i] += correction * 2f;
                }
                else if (!link1Anchored && !link2Anchored)
                {
                    positions[i] += correction * ratio1;
                    positions[i + 1] -= correction * ratio2;
                }
                // If both anchored, don't move either (shouldn't happen normally)
            }
        }
    }

    void UpdateRotations()
    {
        for (int i = 0; i < linkCount; i++)
        {
            if (i < linkCount - 1)
            {
                Vector3 direction = (positions[i + 1] - positions[i]).normalized;

                if (direction.magnitude > 0.001f)
                {
                    // Calculate rotation to point towards next link
                    Quaternion baseRotation = Quaternion.LookRotation(direction, Vector3.up);

                    // Alternate 90° rotation for each link to prevent intersection
                    if (i % 2 == 0)
                    {
                        rotations[i] = baseRotation;
                    }
                    else
                    {
                        rotations[i] = baseRotation * Quaternion.Euler(0, 0, 90);
                    }
                }
            }
            else
            {
                // Last link keeps orientation of previous link but rotated 90°
                if (linkCount > 1)
                {
                    Vector3 direction = (positions[i] - positions[i - 1]).normalized;
                    if (direction.magnitude > 0.001f)
                    {
                        Quaternion baseRotation = Quaternion.LookRotation(direction, Vector3.up);
                        rotations[i] = baseRotation * Quaternion.Euler(0, 0, 90);
                    }
                }
            }
        }
    }

    void UpdateLinkModels()
    {
        if (linkObjects == null)
            return;

        for (int i = 0; i < linkCount; i++)
        {
            if (linkObjects[i] != null)
            {
                // Hide first and last links (they're inside anchors)
                bool shouldRender = i != 0 && i != linkCount - 1;
                linkObjects[i].SetActive(shouldRender);

                // Update the GameObject's Transform using our custom position/rotation
                if (shouldRender)
                {
                    linkObjects[i].transform.position = positions[i];
                    linkObjects[i].transform.rotation = rotations[i];
                }
            }
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

            // Draw cylinder representing link - color based on tension
            if (i < linkCount - 1)
            {
                float tensionRatio = Mathf.Clamp01(currentTension / maxStretchDistance);
                Gizmos.color = Color.Lerp(Color.cyan, Color.red, tensionRatio);
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

        // Draw start anchor
        if (anchorStart != null)
        {
            Gizmos.color = startAnchorActive ? Color.green : Color.gray;
            Gizmos.DrawWireSphere(anchorStartPos, linkRadius * 1.5f);
            Gizmos.DrawWireCube(anchorStartPos, Vector3.one * linkRadius);
        }

        // Draw end anchor
        if (anchorEnd != null)
        {
            Gizmos.color = endAnchorActive ? Color.green : Color.gray;
            Gizmos.DrawWireSphere(anchorEndPos, linkRadius * 1.5f);
            Gizmos.DrawWireCube(anchorEndPos, Vector3.one * linkRadius);
        }

        // Draw tension indicator
        if (startAnchorActive && endAnchorActive && currentTension > 0)
        {
            Vector3 midPoint = (anchorStartPos + anchorEndPos) * 0.5f;
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(anchorStartPos, anchorEndPos);
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

    // Public methods
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

    public float GetCurrentTension()
    {
        return currentTension;
    }

    public bool IsStartAnchorActive()
    {
        return startAnchorActive;
    }

    public bool IsEndAnchorActive()
    {
        return endAnchorActive;
    }

    // Manual reattach methods
    public void ReattachStartAnchor()
    {
        startAnchorActive = true;
        if (anchorStart != null)
        {
            anchorStartPos = anchorStart.position;
            prevAnchorStartPos = anchorStartPos;
        }
    }

    public void ReattachEndAnchor()
    {
        endAnchorActive = true;
        if (anchorEnd != null)
        {
            anchorEndPos = anchorEnd.position;
            prevAnchorEndPos = anchorEndPos;
        }
    }

    void OnDestroy()
    {
        // Clean up instantiated objects
        if (linkObjects != null)
        {
            for (int i = 0; i < linkObjects.Length; i++)
            {
                if (linkObjects[i] != null)
                {
                    Destroy(linkObjects[i]);
                }
            }
        }
    }
}