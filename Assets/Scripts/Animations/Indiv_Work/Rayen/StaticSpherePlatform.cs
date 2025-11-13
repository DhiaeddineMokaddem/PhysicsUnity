// filepath: c:\Users\Rayen\Documents\GitHub\PhysicsUnity\Assets\Scripts\Animations\Indiv_Work\aziz\StaticSpherePlatform.cs
using UnityEngine;
using PhysicsUnity.Indiv_Work.Aziz;

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
    public float localElasticity = 1.0f; // Multiplies PhysicsManager.globalElasticity

    // Manual position storage for simulation (Transform only for rendering)
    [HideInInspector] public Vector3 position;

    private CollisionDetector _collisionDetector;
    private PhysicsManager _physicsManager;
    private GameObject _renderSphere;

    void Awake()
    {
        // Initialize manual state and visuals
        position = center;
        EnsureRenderSphere();
    }

    void Start()
    {
        // Find or create a PhysicsManager so ground/cube-cube are processed
        _physicsManager = FindFirstObjectByType<PhysicsManager>();
        if (_physicsManager == null)
        {
            var go = new GameObject("PhysicsManager");
            _physicsManager = go.AddComponent<PhysicsManager>();
        }

        // Local collision detector for sphere-cube tests
        _collisionDetector = gameObject.GetComponent<CollisionDetector>();
        if (_collisionDetector == null)
            _collisionDetector = gameObject.AddComponent<CollisionDetector>();

        // Sync visuals
        UpdateRenderTransform();
    }

    void FixedUpdate()
    {
        if (_physicsManager != null && _physicsManager.pauseSimulation) return;

        // Iterate all custom rigid bodies and collide with this immovable sphere
        var bodies = FindObjectsByType<RigidBody3D>(FindObjectsSortMode.None);
        float elasticity = (_physicsManager != null ? _physicsManager.globalElasticity : 1f) * Mathf.Max(0f, localElasticity);

        for (int i = 0; i < bodies.Length; i++)
        {
            var body = bodies[i];
            if (body == null || body.isKinematic) continue;

            CollisionInfo col;
            if (_collisionDetector.TryDetectSphereCubeCollision(position, radius, body, out col))
            {
                // bodyA is the static sphere (null), bodyB is the cube
                _collisionDetector.ResolveCollision(col, elasticity);
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
