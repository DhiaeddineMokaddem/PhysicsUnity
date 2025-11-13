using UnityEngine;

public class CustomSceneObjects : MonoBehaviour
{
    [Header("Plane Settings")]
    public float planeY = 0f;
    public Vector2 planeSize = new Vector2(10f, 10f);
    public Material planeMaterial;

    [Header("Sphere Settings")]
    public Vector3 spherePosition = new Vector3(2f, 2f, 2f);
    public float sphereRadius = 1f;
    public Material sphereMaterial;

    [HideInInspector]
    public GameObject planeGO;
    [HideInInspector]
    public GameObject sphereGO;

    void Start()
    {
        // Create plane
        planeGO = GameObject.CreatePrimitive(PrimitiveType.Plane);
        planeGO.transform.position = new Vector3(0, planeY, 0);
        planeGO.transform.localScale = new Vector3(planeSize.x / 10f, 1, planeSize.y / 10f);
        if (planeMaterial != null)
            planeGO.GetComponent<Renderer>().material = planeMaterial;
        planeGO.name = "CustomPlane";
        // Remove collider if present (no Unity physics)
        var planeCollider = planeGO.GetComponent<Collider>();
        if (planeCollider) Destroy(planeCollider);

        // Create sphere
        sphereGO = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphereGO.transform.position = spherePosition;
        sphereGO.transform.localScale = Vector3.one * sphereRadius * 2f;
        if (sphereMaterial != null)
            sphereGO.GetComponent<Renderer>().material = sphereMaterial;
        sphereGO.name = "CustomSphere";
        // Remove collider if present (no Unity physics)
        var sphereCollider = sphereGO.GetComponent<Collider>();
        if (sphereCollider) Destroy(sphereCollider);
    }
}

