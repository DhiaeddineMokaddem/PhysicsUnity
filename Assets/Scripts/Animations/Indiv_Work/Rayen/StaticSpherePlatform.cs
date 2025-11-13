// filepath: c:\Users\Rayen\Documents\GitHub\PhysicsUnity\Assets\Scripts\Animations\Indiv_Work\aziz\StaticSpherePlatform.cs
using UnityEngine;
using PhysicsSimulation.Indiv_Work.Aziz;

/// <summary>
/// Static sphere that interacts with custom RigidBody3D cubes using pure math (no Unity physics/Transform for simulation).
/// It detects and resolves collisions against all RigidBody3D each FixedUpdate, but the sphere itself never moves.
/// A child render sphere is created for visuals if none exists. Colliders are removed to avoid Unity physics.
/// </summary>
public class StaticSpherePlatform : MonoBehaviour
{
    [Header("Sphere Properties")]
    public float radius = 1.5f;
    public Vector3 center = new Vector3(0f, 1.5f, 0f);
    public Material sphereMaterial;
    public Color gizmoColor = new Color(1f, 0.3f, 0.3f, 1f);
    public bool showGizmos = true;

    [Header("Collision Settings")]
    public float localElasticity = 1.0f; // Multiplies PhysicsManagerRayen.globalElasticity

    // Manual position storage for simulation (Transform only for rendering)
    [HideInInspector] public Vector3 position;

    private CollisionDetectorRayen _CollisionDetectorRayen;
    private PhysicsManagerRayen _PhysicsManagerRayen;
    private GameObject _renderSphere;

    void Awake()
    {
        // Initialize manual state and visuals
        position = center;
        EnsureRenderSphere();
    }

    void Start()
    {
        // Find or create a PhysicsManagerRayen so ground/cube-cube are processed
        _PhysicsManagerRayen = FindFirstObjectByType<PhysicsManagerRayen>();
        if (_PhysicsManagerRayen == null)
        {
            var go = new GameObject("PhysicsManagerRayen");
            _PhysicsManagerRayen = go.AddComponent<PhysicsManagerRayen>();
        }

        // Local collision detector for sphere-cube tests
        _CollisionDetectorRayen = gameObject.GetComponent<CollisionDetectorRayen>();
        if (_CollisionDetectorRayen == null)
            _CollisionDetectorRayen = gameObject.AddComponent<CollisionDetectorRayen>();

        // Sync visuals
        UpdateRenderTransform();
    }

    void FixedUpdate()
    {
        if (_PhysicsManagerRayen != null && _PhysicsManagerRayen.pauseSimulation) return;

        // Iterate all custom rigid bodies and collide with this immovable sphere
        var bodies = FindObjectsByType<RigidBody3D>(FindObjectsSortMode.None);
        float elasticity = (_PhysicsManagerRayen != null ? _PhysicsManagerRayen.globalElasticity : 1f) * Mathf.Max(0f, localElasticity);

        for (int i = 0; i < bodies.Length; i++)
        {
            var body = bodies[i];
            if (body == null || body.isKinematic) continue;

            CollisionInfo col;
            if (_CollisionDetectorRayen.TryDetectSphereCubeCollision(position, radius, body, out col))
            {
                // bodyA is the static sphere (null), bodyB is the cube
                _CollisionDetectorRayen.ResolveCollision(col, elasticity);
            }
        }

        // Keep the render-only transform synced
        UpdateRenderTransform();
    }

    private void EnsureRenderSphere()
    {
        // Try to find an existing child sphere for visuals
        Transform child = transform.Find("RenderSphere");
        if (child != null)
        {
            _renderSphere = child.gameObject;
        }
        else
        {
            // Create a render-only sphere
            _renderSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            _renderSphere.name = "RenderSphere";
            _renderSphere.transform.SetParent(transform, worldPositionStays: false);
            // Remove collider to avoid Unity physics
            var col = _renderSphere.GetComponent<Collider>();
            if (col != null) DestroyImmediate(col);
        }

        // Apply material if provided
        var mr = _renderSphere.GetComponent<Renderer>();
        if (mr != null)
        {
            if (sphereMaterial != null) mr.material = sphereMaterial;
            else mr.material.color = gizmoColor;
        }

        // Scale by radius
        _renderSphere.transform.localScale = Vector3.one * (radius * 2f);
    }

    private void UpdateRenderTransform()
    {
        // Render-only: place the sphere at the manual center
        transform.position = position;
        // Maintain scale in case radius changed
        if (_renderSphere != null)
        {
            _renderSphere.transform.localScale = Vector3.one * (radius * 2f);
        }
    }

    void OnValidate()
    {
        // Keep position tied to center in editor
        position = center;
        if (_renderSphere != null)
        {
            _renderSphere.transform.localScale = Vector3.one * (radius * 2f);
        }
    }

    void OnDrawGizmos()
    {
        if (!showGizmos) return;
        Vector3 drawPos = Application.isPlaying ? position : center;
        Gizmos.color = gizmoColor;
        Gizmos.DrawWireSphere(drawPos, radius);
    }
}
