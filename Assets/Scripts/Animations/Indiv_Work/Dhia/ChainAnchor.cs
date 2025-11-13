//Manages anchor points
using UnityEngine;

public class ChainAnchor
{
    public Transform transform;
    public Vector3 position;
    public Vector3 prevPosition;
    public float strength;
    public bool isActive;

    public ChainAnchor(Transform anchorTransform, float anchorStrength)
    {
        transform = anchorTransform;
        strength = anchorStrength;
        isActive = true;

        if (transform != null)
        {
            position = transform.position;
            prevPosition = position;
        }
    }

    public void UpdatePosition()
    {
        if (transform != null && isActive)
        {
            prevPosition = position;
            position = transform.position;
        }
    }

    public void Reattach()
    {
        isActive = true;
        if (transform != null)
        {
            position = transform.position;
            prevPosition = position;
        }
    }
}