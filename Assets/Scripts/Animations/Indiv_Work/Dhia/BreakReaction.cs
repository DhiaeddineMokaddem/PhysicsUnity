//Handles chain breaking physics
using UnityEngine;

public class BreakReaction
{
    public bool justBroke;
    public Vector3 impulseDirection;
    public float breakTime;
    public float tension;
    public float alpha;

    public BreakReaction(float reactionAlpha)
    {
        justBroke = false;
        impulseDirection = Vector3.zero;
        breakTime = 0f;
        tension = 0f;
        alpha = reactionAlpha;
    }

    public void ApplyBreakImpulse(ChainLink[] links, bool startBroke, float currentTension)
    {
        tension = currentTension;
        float impulseStrength = currentTension * alpha * 2f;

        if (startBroke)
        {
            // Apply impulse to first few links
            for (int i = 0; i < Mathf.Min(5, links.Length); i++)
            {
                float falloff = 1f - (i / 5f);
                Vector3 impulse = impulseDirection * impulseStrength * falloff;

                // Add some randomness for more chaotic effect
                impulse += new Vector3(
                    Random.Range(-1f, 1f),
                    Random.Range(-0.5f, 1f),
                    Random.Range(-1f, 1f)
                ) * alpha;

                links[i].prevPosition = links[i].position - impulse * Time.deltaTime;
            }
        }
        else
        {
            // Apply impulse to last few links
            for (int i = Mathf.Max(0, links.Length - 5); i < links.Length; i++)
            {
                float falloff = 1f - ((links.Length - 1 - i) / 5f);
                Vector3 impulse = impulseDirection * impulseStrength * falloff;

                // Add some randomness for more chaotic effect
                impulse += new Vector3(
                    Random.Range(-1f, 1f),
                    Random.Range(-0.5f, 1f),
                    Random.Range(-1f, 1f)
                ) * alpha;

                links[i].prevPosition = links[i].position - impulse * Time.deltaTime;
            }
        }
    }

    public void UpdateBreakState()
    {
        // Reset break flag after a short time
        if (justBroke && Time.time - breakTime > 0.5f)
        {
            justBroke = false;
        }
    }
}