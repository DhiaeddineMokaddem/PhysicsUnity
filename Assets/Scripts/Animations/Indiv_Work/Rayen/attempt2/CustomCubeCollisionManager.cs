using System.Collections.Generic;
using UnityEngine;
using Rayen.attempt2;

public class CustomCubeCollisionManager : MonoBehaviour
{
    public List<CustomRigidBody3D> rigidBodies = new List<CustomRigidBody3D>();
    public float restitution = 0.5f; // Bounciness

    void FixedUpdate()
    {
        // 1. Build intervals for sweep and prune (X axis)
        List<IntervalData> intervals = new List<IntervalData>();
        for (int i = 0; i < rigidBodies.Count; i++)
        {
            var rb = rigidBodies[i];
            Vector3 min = rb.Position - new Vector3(rb.A, rb.B, rb.C) * 0.5f;
            Vector3 max = rb.Position + new Vector3(rb.A, rb.B, rb.C) * 0.5f;
            intervals.Add(new IntervalData { index = i, start = min.x, end = max.x, min = min, max = max });
        }
        // 2. Sort intervals by start
        intervals.Sort((a, b) => a.start.CompareTo(b.start));
        // 3. Sweep and prune
        for (int i = 0; i < intervals.Count; i++)
        {
            for (int j = i + 1; j < intervals.Count && intervals[j].start <= intervals[i].end; j++)
            {
                // Narrow phase: check full AABB overlap
                if (AABBOverlap(intervals[i], intervals[j]))
                {
                    ResolveCollision(rigidBodies[intervals[i].index], rigidBodies[intervals[j].index]);
                }
            }
        }
    }

    bool AABBOverlap(IntervalData a, IntervalData b)
    {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
               (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
               (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    void ResolveCollision(CustomRigidBody3D a, CustomRigidBody3D b)
    {
        // Find the direction to separate
        Vector3 delta = b.Position - a.Position;
        Vector3 overlap = Vector3.zero;
        overlap.x = (a.A + b.A) * 0.5f - Mathf.Abs(delta.x);
        overlap.y = (a.B + b.B) * 0.5f - Mathf.Abs(delta.y);
        overlap.z = (a.C + b.C) * 0.5f - Mathf.Abs(delta.z);
        // Find minimum overlap axis
        float minOverlap = Mathf.Min(overlap.x, Mathf.Min(overlap.y, overlap.z));
        Vector3 separation = Vector3.zero;
        if (minOverlap == overlap.x)
            separation.x = Mathf.Sign(delta.x) * minOverlap;
        else if (minOverlap == overlap.y)
            separation.y = Mathf.Sign(delta.y) * minOverlap;
        else
            separation.z = Mathf.Sign(delta.z) * minOverlap;
        // Separate cubes
        a.Position -= separation * 0.5f;
        b.Position += separation * 0.5f;
        // Simple velocity response (elastic collision)
        Vector3 normal = separation.normalized;
        float vrel = Vector3.Dot(b.Velocity - a.Velocity, normal);
        if (vrel < 0)
        {
            float impulse = -(1 + restitution) * vrel / (1 / a.Mass + 1 / b.Mass);
            Vector3 impulseVec = impulse * normal;
            a.Velocity -= impulseVec / a.Mass;
            b.Velocity += impulseVec / b.Mass;
        }
    }

    struct IntervalData
    {
        public int index;
        public float start, end;
        public Vector3 min, max;
    }
}

