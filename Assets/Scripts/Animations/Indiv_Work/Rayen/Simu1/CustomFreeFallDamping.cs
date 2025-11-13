using UnityEngine;

public class CustomFreeFallDamping : MonoBehaviour
{
    [Header("Physical parameters")]
    public Vector3 velocity = Vector3.zero;
    public float gravity = 9.81f;
    [Tooltip("Coefficient de frottement visqueux (kg/s). Plus grand = freinage plus fort.")]
    public float damping = 0.5f;
    [Tooltip("Si vrai, dt = Time.fixedDeltaTime (recommandé). Sinon utilisez 'customDt'.")]
    public bool useFixedDeltaTime = true;
    [Tooltip("Utilisé seulement si useFixedDeltaTime = false")]
    public float customDt = 0.002f;

    [Header("Initial / Visual")]
    public Vector3 startPosition = new Vector3(0f, 5f, 0f);

    // Internals
    private Vector3 position;
    private MeshFilter meshFilter;
    private Vector3[] originalVertices;

    void Start()
    {
        position = startPosition;
        velocity = Vector3.zero;
        meshFilter = GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            originalVertices = meshFilter.mesh.vertices;
        }
        ApplyTranslation(position);
    }

    void FixedUpdate()
    {
        float dt = useFixedDeltaTime ? Time.fixedDeltaTime : customDt;
        Vector3 acceleration = Vector3.down * gravity - damping * velocity;
        velocity += acceleration * dt;
        position += velocity * dt;
        if (position.y <= 0f)
        {
            position.y = 0f;
            velocity = Vector3.zero;
        }
        ApplyTranslation(position);
    }

    void ApplyTranslation(Vector3 t)
    {
        if (meshFilter == null || originalVertices == null) return;
        Vector3[] movedVertices = new Vector3[originalVertices.Length];
        for (int i = 0; i < originalVertices.Length; i++)
        {
            movedVertices[i] = originalVertices[i] + t;
        }
        meshFilter.mesh.vertices = movedVertices;
        meshFilter.mesh.RecalculateNormals();
    }
}
