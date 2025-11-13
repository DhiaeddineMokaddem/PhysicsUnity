using UnityEngine;

//Data structure for a single chain link
public class ChainLink
{
    public Vector3 position;
    public Vector3 prevPosition;
    public Vector3 velocity;
    public Quaternion rotation;
    public float mass;
    public GameObject visualObject;

    public ChainLink(Vector3 pos, float linkMass = 1f)
    {
        position = pos;
        prevPosition = pos;
        velocity = Vector3.zero;
        rotation = Quaternion.identity;
        mass = linkMass;
        visualObject = null;
    }
}