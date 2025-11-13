using UnityEngine;

//Main controller
public class CustomChainPhysics : MonoBehaviour
{
    [Header("Chain Settings")]
    [SerializeField] private int linkCount = 10;
    [SerializeField] private float linkLength = 0.5f;
    [SerializeField] private float linkRadius = 0.1f;
    [SerializeField] private GameObject chainLinkPrefab;

    [Header("Anchor Settings")]
    [SerializeField] public Vector3 anchorStart;
    [SerializeField] public Vector3 anchorEnd;
    [SerializeField] private float anchorStartStrength = 100f;
    [SerializeField] private float anchorEndStrength = 100f;
    [SerializeField] private float maxStretchDistance = 15f;

    [Header("Physics Settings")]
    [SerializeField] private float gravity = 9.81f;
    [SerializeField] private float damping = 0.98f;
    [SerializeField] private int constraintIterations = 5;
    [SerializeField] private float stiffness = 0.8f;
    [SerializeField][Range(0f, 2f)] private float breakReactionAlpha = 1f;

    // Modules
    private ChainLink[] links;
    private ChainPhysics physics;
    private ChainAnchor startAnchor;
    private ChainAnchor endAnchor;
    private ChainBreaking breaking;
    private BreakReaction breakReaction;
    private ChainVisualizer visualizer;

    void Start()
    {
        InitializeChain();
    }

    void InitializeChain()
    {
        // Initialize modules
        physics = new ChainPhysics(gravity, damping, linkLength, constraintIterations, stiffness);
        startAnchor = new ChainAnchor(this, true, anchorStartStrength);
        endAnchor = new ChainAnchor(this, false, anchorEndStrength);
        breaking = new ChainBreaking(maxStretchDistance);
        breakReaction = new BreakReaction(breakReactionAlpha);
        visualizer = new ChainVisualizer(linkRadius, linkLength, chainLinkPrefab, transform);

        // Initialize links
        links = new ChainLink[linkCount];
        for (int i = 0; i < linkCount; i++)
        {
            float t = (float)i / (linkCount - 1);
            Vector3 pos = Vector3.Lerp(startAnchor.position, endAnchor.position, t);
            links[i] = new ChainLink(pos, 1f);
        }

        // Initialize visuals
        visualizer.InitializeVisuals(links);
    }

    void Update()
    {
        float dt = Time.deltaTime;

        // Update anchors
        startAnchor.UpdatePosition();
        endAnchor.UpdatePosition();

        // Update break reaction alpha
        breakReaction.alpha = breakReactionAlpha;

        // Check for breaking
        breaking.CheckAndBreak(links, startAnchor, endAnchor, breakReaction, linkLength);

        // Physics simulation
        physics.VerletIntegration(links, dt, startAnchor, endAnchor, breakReaction);

        for (int i = 0; i < physics.GetConstraintIterations(); i++)
        {
            physics.ApplyConstraints(links, startAnchor, endAnchor);
        }

        physics.UpdateRotations(links);

        // Update visuals
        visualizer.UpdateVisuals(links);

        // Update break state
        breakReaction.UpdateBreakState();
    }

    void OnDrawGizmos()
    {
        if (links != null)
        {
            visualizer.DrawGizmos(links, startAnchor, endAnchor, breaking.GetCurrentTension(), maxStretchDistance);
        }
    }

    void OnDestroy()
    {
        visualizer.Cleanup(links);
    }

    // Public API
    public Vector3 GetLinkPosition(int index)
    {
        if (index >= 0 && index < linkCount && links != null)
            return links[index].position;
        return Vector3.zero;
    }

    public Quaternion GetLinkRotation(int index)
    {
        if (index >= 0 && index < linkCount && links != null)
            return links[index].rotation;
        return Quaternion.identity;
    }

    public int GetLinkCount()
    {
        return linkCount;
    }

    public float GetCurrentTension()
    {
        return breaking.GetCurrentTension();
    }

    public bool IsStartAnchorActive()
    {
        return startAnchor.isActive;
    }

    public bool IsEndAnchorActive()
    {
        return endAnchor.isActive;
    }

    public void ReattachStartAnchor()
    {
        startAnchor.Reattach();
    }

    public void ReattachEndAnchor()
    {
        endAnchor.Reattach();
    }
}