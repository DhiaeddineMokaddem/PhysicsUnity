using UnityEngine;
using Rayen.attempt2;

public class CustomRigidBodyForceApplier : MonoBehaviour
{
    public CustomRigidBody3D rigidBody;
    public float gravity = 9.81f;
    public float damping = 0.5f;
    public bool useFixedDeltaTime = true;
    public float customDt = 0.02f;

    void FixedUpdate()
    {
        if (rigidBody == null) return;
        float dt = useFixedDeltaTime ? Time.fixedDeltaTime : customDt;
        rigidBody.ClearForces();
        // Gravity force
        Vector3 gravityForce = Vector3.down * rigidBody.Mass * gravity;
        rigidBody.ApplyForce(gravityForce, rigidBody.Position);
        // Damping force
        Vector3 dampingForce = -damping * rigidBody.Velocity;
        rigidBody.ApplyForce(dampingForce, rigidBody.Position);
    }
}

