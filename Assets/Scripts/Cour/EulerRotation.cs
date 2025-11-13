using UnityEngine;

public class EulerRotation
{
    public static float[,] RotationX(float angle)
    {
        float rad = angle * Mathf.Deg2Rad;
        return new float[,] {
            {1, 0, 0, 0},
            {0, Mathf.Cos(rad), -Mathf.Sin(rad), 0},
            {0, Mathf.Sin(rad),  Mathf.Cos(rad), 0},
            {0, 0, 0, 1}
        };
    }

    public static float[,] RotationY(float angle)
    {
        float rad = angle * Mathf.Deg2Rad;
        return new float[,] {
            {Mathf.Cos(rad), 0, Mathf.Sin(rad), 0},
            {0, 1, 0, 0},
            {-Mathf.Sin(rad), 0, Mathf.Cos(rad), 0},
            {0, 0, 0, 1}
        };
    }

    public static float[,] RotationZ(float angle)
    {
        float rad = angle * Mathf.Deg2Rad;
        return new float[,] {
            {Mathf.Cos(rad), -Mathf.Sin(rad), 0, 0},
            {Mathf.Sin(rad),  Mathf.Cos(rad), 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };
    }

    public static float[,] Multiply(float[,] A, float[,] B)
    {
        float[,] R = new float[4, 4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    R[i, j] += A[i, k] * B[k, j];
        return R;
    }
}
