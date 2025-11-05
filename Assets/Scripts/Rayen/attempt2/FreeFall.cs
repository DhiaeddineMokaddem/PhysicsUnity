using UnityEngine;

public class FreeFall : MonoBehaviour
{
    [Header("Physical parameters")]
    public Vector3 velocity = Vector3.zero;
    public float gravity = 9.81f;

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

        // Créer le cube via la classe non-MonoBehaviour
        cubeObject = new CubeObject(position, cubeColor, "FreeFallCube");

        // Appliquer la position initiale
        ApplyTranslation(position);
    }

    void FixedUpdate()
    {
        float dt = useFixedDeltaTime ? Time.fixedDeltaTime : customDt;

        // Intégration d'Euler explicite
        velocity += Vector3.down * gravity * dt;
        position += velocity * dt;

        // Collision simple avec le plan y = 0 (arrêt sans restitution)
        if (position.y <= 0f)
        {
            position.y = 0f;
            velocity = Vector3.zero;
        }

        // Appliquer la translation via matrice 4x4 (format float[,])
        ApplyTranslation(position);
    }

    // Méthode utilitaire pour créer et appliquer la matrice de translation
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

    // Optionnel: au cas où on arrête la simulation, nettoyer
    private void OnDestroy()
    {
        if (cubeObject != null) cubeObject.Destroy();
    }
}
