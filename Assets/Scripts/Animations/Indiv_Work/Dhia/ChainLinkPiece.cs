using UnityEngine;

/// <summary>
/// A self-contained chain link simulation that follows another link above it.
/// Uses custom spring physics (no Rigidbody, no Unity Joints).
/// Each link maintains a rest distance from its "parent" link.
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ChainLinkPiece : MonoBehaviour
{
    [Header("Link Connection")]
    [Tooltip("The link above this one (can be null for the first link).")]
    public ChainLinkPiece linkAbove;

    [Tooltip("Distance this link tries to maintain from the link above.")]
    public float restLength = 0.3f;

    [Header("Physics Settings")]
    [Tooltip("How stiff the spring is. Higher = less stretchy.")]
    public float stiffness = 150f;
    [Tooltip("Damping factor (0 = none, 1 = full stop).")]
    [Range(0f, 1f)] public float damping = 0.98f;
    [Tooltip("Mass of this link.")]
    public float mass = 0.5f;
    [Tooltip("Gravity applied to this link.")]
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);

    [Header("Breaking")]
    [Tooltip("If stretched beyond this factor × restLength, connection breaks.")]
    public float breakStretchFactor = 1.6f;
    [Tooltip("If true, the link will visually drop when broken.")]
    public bool fallWhenBroken = true;

    [Header("Visuals")]
    [Tooltip("Visual width of the chain segment.")]
    public float visualWidth = 0.05f;
    [Tooltip("Length of this visual segment (for mesh generation).")]
    public float visualLength = 0.3f;


    [Header("Change Only In Run Time !")]
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 force;
    public bool isBroken = false;
    public Mesh mesh;

    void Start()
    {
        position = transform.position;
        velocity = Vector3.zero;
        force = Vector3.zero;

        mesh = new Mesh();
        mesh.name = "ChainLinkPiece Mesh";
        GetComponent<MeshFilter>().mesh = mesh;

        BuildMesh();
    }

    void BuildMesh()
    {
        // Build a simple quad to represent the link
        Vector3[] verts = new Vector3[4];
        float halfWidth = visualWidth * 0.5f;
        float halfLength = visualLength * 0.5f;

        verts[0] = new Vector3(-halfWidth, -halfLength, 0);
        verts[1] = new Vector3(halfWidth, -halfLength, 0);
        verts[2] = new Vector3(-halfWidth, halfLength, 0);
        verts[3] = new Vector3(halfWidth, halfLength, 0);

        int[] tris = { 0, 2, 1, 2, 3, 1 };

        mesh.vertices = verts;
        mesh.triangles = tris;
        mesh.RecalculateNormals();
    }

    void Update()
    {
        if (linkAbove != null)
        {
            Simulate(Time.deltaTime);
        }
        UpdateMeshTransform();
    }

    void Simulate(float dt)
    {
        if (isBroken)
        {
            // Still fall with gravity if broken
            velocity += gravity * dt;
            position += velocity * dt;
            return;
        }

        // Reset forces
        force = Vector3.zero;

        // Gravity
        force += gravity * mass;

        if (linkAbove != null && !linkAbove.isBroken)
        {
            Vector3 delta = position - linkAbove.position;
            float length = delta.magnitude;
            if (length > Mathf.Epsilon)
            {
                Vector3 dir = delta / length;
                float displacement = length - restLength;

                // Check for breaking
                if (length > restLength * breakStretchFactor)
                {
                    //isBroken = true;
                    if (!fallWhenBroken)
                    {
                        velocity = Vector3.zero;
                    }
                    Debug.Log($"[{name}] Link broke from {linkAbove.name}");
                    return;
                }

                // Hooke's law spring
                Vector3 springForce = -stiffness * displacement * dir;
                force += springForce;
            }
        }

        // Integrate motion
        Vector3 accel = force / mass;
        velocity = (velocity + accel * dt) * damping;
        position += velocity * dt;
    }

    void UpdateMeshTransform()
    {
        transform.position = position;

        // Orient mesh towards the link above if still connected
        if (linkAbove != null && !isBroken)
        {
            Vector3 dir = linkAbove.position - position;
            if (dir.sqrMagnitude > 0.0001f)
                transform.rotation = Quaternion.LookRotation(Vector3.forward, dir.normalized);
        }
        else
        {
            // Just fall naturally
            transform.rotation = Quaternion.identity;
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = isBroken ? Color.red : Color.yellow;
        Gizmos.DrawSphere(Application.isPlaying ? position : transform.position, 0.02f);

        if (linkAbove != null)
        {
            Vector3 from = Application.isPlaying ? position : transform.position;
            Vector3 to = Application.isPlaying ? linkAbove.position : linkAbove.transform.position;
            Gizmos.DrawLine(from, to);
        }
    }
}
