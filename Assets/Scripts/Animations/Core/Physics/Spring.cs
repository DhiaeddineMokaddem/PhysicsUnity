using UnityEngine;

namespace PhysicsSimulation.Core.Physics
{
    /// <summary>
    /// Lightweight soft-body point (Verlet-style integrator, pure data).
    /// </summary>
    public class SoftBodyPoint
    {
        public Vector3 position;
        public Vector3 previousPosition;
        public Vector3 force;
        public float mass;

        public SoftBodyPoint(Vector3 pos, float m)
        {
            position = pos;
            previousPosition = pos;
            force = Vector3.zero;
            mass = Mathf.Max(m, PhysicsConstants.EPSILON);
        }

        public void AddForce(Vector3 f) => force += f;

        public void Integrate(float dt, float damping)
        {
            Vector3 velocity = position - previousPosition;
            Vector3 newPos = position + velocity * (1f - damping * dt) + (force / mass) * dt * dt;
            previousPosition = position;
            position = newPos;
            force = Vector3.zero;
        }
    }

    /// <summary>
    /// Position-based spring constraint for soft-body simulation.
    /// </summary>
    public class Spring
    {
        public SoftBodyPoint a, b;
        public float restLength;
        public float stiffness;

        public Spring(SoftBodyPoint p1, SoftBodyPoint p2, float k)
        {
            a = p1;
            b = p2;
            restLength = (p1.position - p2.position).magnitude;
            stiffness = k;
        }

        public void Apply()
        {
            Vector3 delta = b.position - a.position;
            float dist = delta.magnitude;
            if (dist <= 1e-6f) return;
            float diff = (dist - restLength) / dist;
            Vector3 correction = delta * 0.5f * stiffness * 0.01f * diff;
            a.position += correction;
            b.position -= correction;
        }
    }
}

