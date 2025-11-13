using UnityEngine;

//Manages breaking logic
public class ChainBreaking
{
    private float maxStretchDistance;
    private float currentTension;

    public ChainBreaking(float maxStretch)
    {
        maxStretchDistance = maxStretch;
        currentTension = 0f;
    }

    public float CheckAndBreak(ChainLink[] links, ChainAnchor startAnchor, ChainAnchor endAnchor, BreakReaction breakReaction, float linkLength)
    {
        if (!startAnchor.isActive && !endAnchor.isActive)
            return 0f;

        // Calculate natural chain length (sum of all link lengths)
        float naturalChainLength = linkLength * (links.Length - 1);

        // Calculate distance between anchors
        float anchorDistance = Vector3.Distance(startAnchor.position, endAnchor.position);

        // Calculate tension (how much stretched beyond natural length)
        float stretch = anchorDistance - naturalChainLength;
        currentTension = Mathf.Max(0, stretch);

        // Check if stretched too far
        if (stretch > maxStretchDistance)
        {
            // Break the weaker anchor
            if (startAnchor.isActive && endAnchor.isActive)
            {
                if (startAnchor.strength <= endAnchor.strength)
                {
                    BreakAnchor(true, links, startAnchor, endAnchor, breakReaction);
                }
                else
                {
                    BreakAnchor(false, links, startAnchor, endAnchor, breakReaction);
                }
            }
        }

        // Also check based on force/strength
        if (startAnchor.isActive && currentTension > startAnchor.strength)
        {
            BreakAnchor(true, links, startAnchor, endAnchor, breakReaction);
        }
        else if (endAnchor.isActive && currentTension > endAnchor.strength)
        {
            BreakAnchor(false, links, startAnchor, endAnchor, breakReaction);
        }

        return currentTension;
    }

    private void BreakAnchor(bool breakStart, ChainLink[] links, ChainAnchor startAnchor, ChainAnchor endAnchor, BreakReaction breakReaction)
    {
        if (breakStart)
        {
            startAnchor.isActive = false;
            Debug.Log("Start anchor broke! Tension: " + currentTension);

            // Calculate impulse direction and magnitude
            if (links.Length > 1)
            {
                breakReaction.impulseDirection = (links[1].position - links[0].position).normalized;
            }
        }
        else
        {
            endAnchor.isActive = false;
            Debug.Log("End anchor broke! Tension: " + currentTension);

            // Calculate impulse direction and magnitude
            if (links.Length > 1)
            {
                breakReaction.impulseDirection = (links[links.Length - 2].position - links[links.Length - 1].position).normalized;
            }
        }

        breakReaction.justBroke = true;
        breakReaction.breakTime = Time.time;
        breakReaction.tension = currentTension;

        // Apply initial break impulse to chain
        breakReaction.ApplyBreakImpulse(links, breakStart, currentTension);
    }

    public float GetCurrentTension()
    {
        return currentTension;
    }
}