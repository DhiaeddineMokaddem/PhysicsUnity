//Handles physics simulation
using UnityEngine;

public class ChainPhysics
{
    private float gravity;
    private float damping;
    private float linkLength;
    private int constraintIterations;
    private float stiffness;

    public ChainPhysics(float grav, float damp, float length, int iterations, float stiff)
    {
        gravity = grav;
        damping = damp;
        linkLength = length;
        constraintIterations = iterations;
        stiffness = stiff;
    }

    public void VerletIntegration(ChainLink[] links, float dt, ChainAnchor startAnchor, ChainAnchor endAnchor, BreakReaction breakReaction)
    {
        for (int i = 0; i < links.Length; i++)
        {
            // Skip anchored links
            if ((i == 0 && startAnchor.isActive) || (i == links.Length - 1 && endAnchor.isActive))
                continue;

            Vector3 temp = links[i].position;

            // Verlet integration: x(t+dt) = 2*x(t) - x(t-dt) + a*dt^2
            Vector3 acceleration = Vector3.down * gravity;

            // Add velocity from anchor movement for first/last links
            if (i == 0 && startAnchor.isActive)
            {
                Vector3 anchorVel = (startAnchor.position - startAnchor.prevPosition) / dt;
                acceleration += anchorVel / dt;
            }
            else if (i == links.Length - 1 && endAnchor.isActive)
            {
                Vector3 anchorVel = (endAnchor.position - endAnchor.prevPosition) / dt;
                acceleration += anchorVel / dt;
            }

            // Apply extra impulse right after break for exaggerated reaction
            if (breakReaction.justBroke && Time.time - breakReaction.breakTime < 0.5f)
            {
                float breakForce = breakReaction.tension * breakReaction.alpha * 10f;
                acceleration += breakReaction.impulseDirection * breakForce;
            }

            links[i].position = links[i].position + (links[i].position - links[i].prevPosition) * damping + acceleration * dt * dt;
            links[i].prevPosition = temp;

            // Update velocity for reference
            links[i].velocity = (links[i].position - links[i].prevPosition) / dt;
        }
    }

    public void ApplyConstraints(ChainLink[] links, ChainAnchor startAnchor, ChainAnchor endAnchor)
    {
        // First link attached to start anchor
        if (startAnchor.isActive)
        {
            links[0].position = Vector3.Lerp(links[0].position, startAnchor.position, stiffness);
        }

        // Last link attached to end anchor
        if (endAnchor.isActive)
        {
            links[links.Length - 1].position = Vector3.Lerp(links[links.Length - 1].position, endAnchor.position, stiffness);
        }

        // Distance constraints between links
        for (int i = 0; i < links.Length - 1; i++)
        {
            Vector3 delta = links[i + 1].position - links[i].position;
            float currentDist = delta.magnitude;

            if (currentDist > 0.0001f)
            {
                float diff = (currentDist - linkLength) / currentDist;
                Vector3 correction = delta * diff * 0.5f;

                // Apply correction based on masses
                float totalMass = links[i].mass + links[i + 1].mass;
                float ratio1 = links[i + 1].mass / totalMass;
                float ratio2 = links[i].mass / totalMass;

                // Don't move anchored links
                bool link1Anchored = (i == 0 && startAnchor.isActive);
                bool link2Anchored = (i + 1 == links.Length - 1 && endAnchor.isActive);

                if (link1Anchored && !link2Anchored)
                {
                    links[i + 1].position -= correction * 2f;
                }
                else if (link2Anchored && !link1Anchored)
                {
                    links[i].position += correction * 2f;
                }
                else if (!link1Anchored && !link2Anchored)
                {
                    links[i].position += correction * ratio1;
                    links[i + 1].position -= correction * ratio2;
                }
            }
        }
    }

    public void UpdateRotations(ChainLink[] links)
    {
        for (int i = 0; i < links.Length; i++)
        {
            if (i < links.Length - 1)
            {
                Vector3 direction = (links[i + 1].position - links[i].position).normalized;

                if (direction.magnitude > 0.001f)
                {
                    // Calculate rotation to point towards next link
                    Quaternion baseRotation = Quaternion.LookRotation(direction, Vector3.up);

                    // Alternate 90° rotation for each link to prevent intersection
                    if (i % 2 == 0)
                    {
                        links[i].rotation = baseRotation;
                    }
                    else
                    {
                        links[i].rotation = baseRotation * Quaternion.Euler(0, 0, 90);
                    }
                }
            }
            else
            {
                // Last link keeps orientation of previous link but rotated 90°
                if (links.Length > 1)
                {
                    Vector3 direction = (links[i].position - links[i - 1].position).normalized;
                    if (direction.magnitude > 0.001f)
                    {
                        Quaternion baseRotation = Quaternion.LookRotation(direction, Vector3.up);
                        links[i].rotation = baseRotation * Quaternion.Euler(0, 0, 90);
                    }
                }
            }
        }
    }

    public int GetConstraintIterations()
    {
        return constraintIterations;
    }
}