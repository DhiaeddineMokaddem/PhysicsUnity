/// <summary>
/// Represents a connection between two particles.  The rest length is the
/// distance at which the spring is neither compressed nor stretched.
/// </summary>
public struct Spring
{
    public int indexA;
    public int indexB;
    public float restLength;

    public Spring(int a, int b, float rest)
    {
        indexA = a;
        indexB = b;
        restLength = rest;
    }
}
