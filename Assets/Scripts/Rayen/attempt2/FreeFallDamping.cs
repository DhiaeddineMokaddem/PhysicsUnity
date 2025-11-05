using UnityEngine;

public class FreeFallDamping : MonoBehaviour
{
    [Header("Physical parameters")]
    public Vector3 velocity = Vector3.zero;
    public float gravity = 9.81f;

    [Tooltip("Coefficient de frottement visqueux (kg/s). Plus grand = freinage plus fort.")]
    public float damping = 0.5f;

    [Tooltip("Si vrai, dt = Time.fixedDeltaTime (recommandé). Sinon utilisez 'customDt'.")]
    public bool useFixedDeltaTime = true;
    [Tooltip("Utilisé seulement si useFixedDeltaTime = false")]
    public float customDt = 0.002f;

    [Header("Initial / Visual")]
    public Vector3 startPosition = new Vector3(0f, 5f, 0f);
    public Color cubeColor = Color.red;

    // Internals
    private Vector3 position;
    private CubeObject cubeObject;

    void Start()
    {
        // Initialisation
        position = startPosition;
        velocity = Vector3.zero;

        // Crée le cube via la classe utilitaire
        cubeObject = new CubeObject(position, cubeColor, "FreeFallCube");

        // Applique la position initiale
        ApplyTranslation(position);
    }

    void FixedUpdate()
    {
        float dt = useFixedDeltaTime ? Time.fixedDeltaTime : customDt;

        // Accélération : gravité + frottement
        Vector3 acceleration = Vector3.down * gravity - damping * velocity;

        // Intégration d’Euler
        velocity += acceleration * dt;
        position += velocity * dt;

        // Collision simple avec le plan y=0
        if (position.y <= 0f)
        {
            position.y = 0f;
            velocity = Vector3.zero;
        }

        // Applique translation par matrice
        ApplyTranslation(position);
    }

    void ApplyTranslation(Vector3 t)
    {
        float[,] T = new float[,]
        {
            {1f, 0f, 0f, t.x},
            {0f, 1f, 0f, t.y},
            {0f, 0f, 1f, t.z},
            {0f, 0f, 0f, 1f}
        };

        cubeObject.ApplyMatrix(T);
    }

    private void OnDestroy()
    {
        if (cubeObject != null) cubeObject.Destroy();
    }
}
