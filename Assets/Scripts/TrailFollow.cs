using System;
using UnityEngine;

public class TrailFollow : MonoBehaviour
{
    public MatrixCube cube;

   //calculate the center of the cube
    public Vector3 GetCubeCenter()
    {
        Vector3[] vertices = cube.GetTransformedVertices();
        Vector3 center = Vector3.zero;
        foreach (Vector3 v in vertices)
        {
            center += v;
        }
        center /= vertices.Length;
        return center;
    }

    void Update()
    {
        // Move this object to follow the cube's center
        transform.position = GetCubeCenter();
    }
}
