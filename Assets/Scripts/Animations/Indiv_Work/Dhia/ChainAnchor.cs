using UnityEngine;

//Manages anchor points
public class ChainAnchor
{
    public CustomChainPhysics customChainPhysics;
    public bool isLinkedToStart;
    public Vector3 position;
    public Vector3 prevPosition;
    public float strength;
    public bool isActive;

    public ChainAnchor(CustomChainPhysics customChainPhysics, bool linkedStart, float anchorStrength)
    {
        strength = anchorStrength;
        isActive = true;
        isLinkedToStart = linkedStart;
        this.customChainPhysics = customChainPhysics;
        if (this.customChainPhysics != null)
        {
            position = anchorPosition();
            prevPosition = position;
        }
    }

    public void UpdatePosition()
    {
        if (anchorPosition() != null && isActive)
        {
            prevPosition = position;
            position = anchorPosition();
        }
    }

    public void Reattach()
    {
        isActive = true;
        if (anchorPosition() != null)
        {
            position = anchorPosition();
            prevPosition = position;
        }
    }
    public Vector3 anchorPosition()
    {
        if (isLinkedToStart)
        {
            return customChainPhysics.anchorStart;
        }
        else
        {
            return customChainPhysics.anchorEnd;
        }
    }
}