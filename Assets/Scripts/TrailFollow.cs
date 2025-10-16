using System;
using UnityEngine;

// Follows the center of the cube by reading its mesh vertices
// Useful for visualizing the cube's trajectory
public class TrailFollow : MonoBehaviour
{
    public MatrixCube cube; // Reference to the main cube controller
    public MatrixCubeMeshDhiadeddineMokaddem meshScript; // Reference to the mesh script

    // Set trail position to cube's initial position before rendering
    // Ensures the trail starts at the correct location
    void Start()
    {
        if (cube != null)
            transform.position = cube.startPosition;
        else
            transform.position = GetCubeCenter(); // fallback if cube reference missing
    }

    // Calculates the center of the cube by averaging its transformed vertices
    // Used to follow the cube's movement
    public Vector3 GetCubeCenter()
    {
        Vector3[] vertices = meshScript.GetTransformedVertices();
        Vector3 center = Vector3.zero;
        foreach (Vector3 v in vertices)
        {
            center += v;
        }
        center /= vertices.Length;
        return center;
    }

    // Moves this object to follow the cube's center every frame
    void Update()
    {
        transform.position = GetCubeCenter();
    }
}
