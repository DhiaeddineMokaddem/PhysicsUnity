using System.Collections.Generic;
using UnityEngine;
using PhysicsSimulation.Core;

/// <summary>
/// Classe représentant une sphère avec toutes ses propriétés physiques
/// Utilise pour simuler les particules d'un gaz parfait
/// </summary>
[System.Serializable]
public class Sphere
{
    #region Properties
    public Vector3 position;
    public Vector3 velocity;
    public float radius;
    public float mass;
    public GameObject gameObject;
    #endregion

    public Sphere(Vector3 pos, Vector3 vel, float r, float m)
    {
        position = pos;
        velocity = vel;
        radius = r;
        mass = m;
    }

    #region AABB Properties
    public float MinX => position.x - radius;
    public float MaxX => position.x + radius;
    public float MinY => position.y - radius;
    public float MaxY => position.y + radius;
    public float MinZ => position.z - radius;
    public float MaxZ => position.z + radius;
    #endregion
}

/// <summary>
/// Classe pour les intervalles 1D utilisés dans l'algorithme Sweep and Prune
/// </summary>
[System.Serializable]
public class SphereInterval
{
    public Sphere sphere;
    public float min;
    public float max;

    public SphereInterval(Sphere s, float minVal, float maxVal)
    {
        sphere = s;
        min = minVal;
        max = maxVal;
    }

    public bool Overlaps(SphereInterval other)
    {
        return CollisionUtils.IntervalsOverlap(min, max, other.min, other.max);
    }
}

/// <summary>
/// Classe principale pour la simulation d'un gaz parfait dans une boîte cubique
/// Implémente l'algorithme Sweep and Prune pour optimiser la détection de collisions
/// Refactored to use shared utilities from PhysicsUnity.Core
/// </summary>
public class GasSimulation : MonoBehaviour
{
    #region Inspector Properties
    [Header("Paramètres de la Boîte")]
    [SerializeField] private float boxSize = 10f;

    [Header("Paramètres des Sphères")]
    [SerializeField] private int numberOfSpheres = 20;
    [SerializeField] private float sphereRadius = 0.3f;
    [SerializeField] private float sphereMass = 1f;
    [SerializeField] private float initialSpeedRange = 5f;

    [Header("Visualisation de la Boîte")]
    [SerializeField] private bool drawBox = true;
    [SerializeField] private Color boxColor = Color.cyan;
    [SerializeField] private float boxLineWidth = 0.05f;

    [Header("Matériaux")]
    [SerializeField] private Material sphereMaterial;
    #endregion

    #region Private Fields
    private List<Sphere> spheres = new List<Sphere>();
    private float halfBoxSize;
    private GameObject boxContainer;
    #endregion

    #region Initialization
    void Start()
    {
        halfBoxSize = boxSize / 2f;
        InitializeSpheres();
        CreateBoxVisualization();
    }

    void InitializeSpheres()
    {
        for (int i = 0; i < numberOfSpheres; i++)
        {
            Vector3 randomPos = new Vector3(
                Random.Range(-halfBoxSize + sphereRadius, halfBoxSize - sphereRadius),
                Random.Range(-halfBoxSize + sphereRadius, halfBoxSize - sphereRadius),
                Random.Range(-halfBoxSize + sphereRadius, halfBoxSize - sphereRadius)
            );

            Vector3 randomVel = new Vector3(
                Random.Range(-initialSpeedRange, initialSpeedRange),
                Random.Range(-initialSpeedRange, initialSpeedRange),
                Random.Range(-initialSpeedRange, initialSpeedRange)
            );

            Sphere sphere = new Sphere(randomPos, randomVel, sphereRadius, sphereMass);

            GameObject sphereObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphereObj.transform.position = randomPos;
            sphereObj.transform.localScale = Vector3.one * sphereRadius * 2f;
            sphereObj.name = "Sphere_" + i;

            Destroy(sphereObj.GetComponent<Collider>());

            if (sphereMaterial != null)
                sphereObj.GetComponent<Renderer>().material = sphereMaterial;
            else
                sphereObj.GetComponent<Renderer>().material.color = Random.ColorHSV();

            sphere.gameObject = sphereObj;
            spheres.Add(sphere);
        }
    }

    void CreateBoxVisualization()
    {
        if (!drawBox) return;

        boxContainer = new GameObject("BoxContainer");
        boxContainer.transform.parent = transform;

        float h = halfBoxSize;
        Vector3[] vertices = new Vector3[]
        {
            new Vector3(-h, -h, -h), new Vector3(h, -h, -h), new Vector3(h, -h, h), new Vector3(-h, -h, h),
            new Vector3(-h, h, -h), new Vector3(h, h, -h), new Vector3(h, h, h), new Vector3(-h, h, h)
        };

        int[,] edges = new int[,]
        {
            {0, 1}, {1, 2}, {2, 3}, {3, 0},
            {4, 5}, {5, 6}, {6, 7}, {7, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        };

        for (int i = 0; i < edges.GetLength(0); i++)
        {
            GameObject lineObj = new GameObject("Edge_" + i);
            lineObj.transform.parent = boxContainer.transform;
            LineRenderer lr = lineObj.AddComponent<LineRenderer>();
            lr.startWidth = boxLineWidth;
            lr.endWidth = boxLineWidth;
            lr.positionCount = 2;
            lr.SetPosition(0, vertices[edges[i, 0]]);
            lr.SetPosition(1, vertices[edges[i, 1]]);
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.startColor = boxColor;
            lr.endColor = boxColor;
        }
    }
    #endregion

    #region Physics Update
    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        UpdatePositions(dt);
        DetectAndResolveCollisions();
        HandleBoxCollisions();
        UpdateVisualPositions();
    }

    void UpdatePositions(float dt)
    {
        foreach (var sphere in spheres)
        {
            sphere.position = IntegrationUtils.IntegratePositionEuler(sphere.position, sphere.velocity, dt);
        }
    }

    void HandleBoxCollisions()
    {
        foreach (var sphere in spheres)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                if (sphere.position[axis] - sphere.radius < -halfBoxSize)
                {
                    sphere.position[axis] = -halfBoxSize + sphere.radius;
                    sphere.velocity[axis] = -sphere.velocity[axis];
                }
                else if (sphere.position[axis] + sphere.radius > halfBoxSize)
                {
                    sphere.position[axis] = halfBoxSize - sphere.radius;
                    sphere.velocity[axis] = -sphere.velocity[axis];
                }
            }
        }
    }

    void UpdateVisualPositions()
    {
        foreach (var sphere in spheres)
        {
            if (sphere.gameObject != null)
                sphere.gameObject.transform.position = sphere.position;
        }
    }
    #endregion

    #region Collision Detection
    void DetectAndResolveCollisions()
    {
        List<SphereInterval> intervalsX = new List<SphereInterval>();
        List<SphereInterval> intervalsY = new List<SphereInterval>();
        List<SphereInterval> intervalsZ = new List<SphereInterval>();

        foreach (var sphere in spheres)
        {
            intervalsX.Add(new SphereInterval(sphere, sphere.MinX, sphere.MaxX));
            intervalsY.Add(new SphereInterval(sphere, sphere.MinY, sphere.MaxY));
            intervalsZ.Add(new SphereInterval(sphere, sphere.MinZ, sphere.MaxZ));
        }

        intervalsX.Sort((a, b) => a.min.CompareTo(b.min));
        intervalsY.Sort((a, b) => a.min.CompareTo(b.min));
        intervalsZ.Sort((a, b) => a.min.CompareTo(b.min));

        HashSet<(Sphere, Sphere)> potentialCollisions = new HashSet<(Sphere, Sphere)>();

        for (int i = 0; i < intervalsX.Count; i++)
        {
            for (int j = i + 1; j < intervalsX.Count; j++)
            {
                if (intervalsX[j].min > intervalsX[i].max) break;

                if (intervalsX[i].Overlaps(intervalsX[j]))
                {
                    Sphere s1 = intervalsX[i].sphere;
                    Sphere s2 = intervalsX[j].sphere;
                    if (s1 != s2) potentialCollisions.Add((s1, s2));
                }
            }
        }

        foreach (var pair in potentialCollisions)
        {
            Vector3 normal;
            float penetration;
            if (CollisionUtils.CheckSphereSphereCollision(
                pair.Item1.position, pair.Item1.radius,
                pair.Item2.position, pair.Item2.radius,
                out normal, out penetration))
            {
                ResolveCollision(pair.Item1, pair.Item2, normal, penetration);
            }
        }
    }

    void ResolveCollision(Sphere s1, Sphere s2, Vector3 normal, float penetration)
    {
        float totalMass = s1.mass + s2.mass;
        float ratio1 = s2.mass / totalMass;
        float ratio2 = s1.mass / totalMass;

        s1.position -= normal * penetration * ratio1 * PhysicsConstants.SEPARATION_FACTOR;
        s2.position += normal * penetration * ratio2 * PhysicsConstants.SEPARATION_FACTOR;

        Vector3 relativeVelocity = s2.velocity - s1.velocity;
        float velocityAlongNormal = Vector3.Dot(relativeVelocity, normal);

        if (velocityAlongNormal < 0) return;

        float restitution = PhysicsConstants.DEFAULT_RESTITUTION;
        float impulseMagnitude = -(1 + restitution) * velocityAlongNormal;
        impulseMagnitude /= (1 / s1.mass + 1 / s2.mass);

        Vector3 impulse = normal * impulseMagnitude;
        s1.velocity -= impulse / s1.mass;
        s2.velocity += impulse / s2.mass;
    }
    #endregion

    #region Debug Visualization
    void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;
        
        foreach (var sphere in spheres)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(sphere.position, sphere.radius);
        }
    }
    #endregion
}