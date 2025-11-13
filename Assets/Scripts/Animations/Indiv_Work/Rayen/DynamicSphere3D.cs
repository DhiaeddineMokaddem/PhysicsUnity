// filepath: c:\Users\Rayen\Documents\GitHub\PhysicsUnity\Assets\Scripts\Animations\Indiv_Work\aziz\DynamicSphere3D.cs
using UnityEngine;
using PhysicsSimulation.Core;

/// <summary>
/// Dynamic custom-physics sphere updated by PhysicsManager substeps.
/// Stores manual position/velocity and uses VisualRenderer only for rendering.
/// </summary>
public class DynamicSphere3D : MonoBehaviour
{
    [Header("Physical Properties")]
    public float radius = 2.0f;
    public float mass = 5.0f;
    public float restitution = PhysicsConstants.DEFAULT_RESTITUTION;
    public float friction = PhysicsConstants.DEFAULT_FRICTION;
    public float linearDamping = PhysicsConstants.DEFAULT_LINEAR_DAMPING;
    public bool isKinematic = false;
    public bool useGravity = true;

    [Header("State")]
    public Vector3 velocity = Vector3.zero;

    // Manual transform state
    [HideInInspector] public Vector3 position;

    [Header("Visuals")]
    public Material sphereMaterial;
    public Color color = Color.red;

    private VisualRenderer visualRenderer;

    void Awake()
    {
        visualRenderer = GetComponent<VisualRenderer>();
        if (visualRenderer == null)
        {
            visualRenderer = gameObject.AddComponent<VisualRenderer>();
        }

        position = visualRenderer.GetPosition();
        
        // Ensure we only use renderers visually
        var col = GetComponent<Collider>();
        if (col != null) DestroyImmediate(col);
        var mr = GetComponent<Renderer>();
        if (mr != null)
        {
            if (sphereMaterial != null) mr.material = sphereMaterial;
            else mr.material.color = color;
        }
        
        visualRenderer.UpdateScale(Vector3.one * (radius * 2f));
    }

    public void IntegratePhysics(float deltaTime)
    {
        if (isKinematic) return;

        if (useGravity)
        {
            velocity += PhysicsConstants.GRAVITY_VECTOR * deltaTime;
        }

        // Damping
        velocity = IntegrationUtils.ApplyDamping(velocity, linearDamping, deltaTime);

        // Integrate position
        position = IntegrationUtils.IntegratePositionEuler(position, velocity, deltaTime);

        UpdateVisualTransform();
    }

    public void AddImpulse(Vector3 impulse)
    {
        if (isKinematic) return;
        velocity += impulse / mass;
    }

    public Vector3 GetVelocityAtPoint(Vector3 point)
    {
        return velocity; // no rotation
    }

    public void UpdateVisualTransform()
    {
        if (visualRenderer != null)
        {
            visualRenderer.UpdatePosition(position);
            visualRenderer.UpdateScale(Vector3.one * (radius * 2f));
        }
    }
}

