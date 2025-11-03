using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Classe représentant une sphère avec toutes ses propriétés physiques
/// Utilisée pour simuler les particules d'un gaz parfait
/// </summary>
[System.Serializable]
public class Sphere
{
    // Propriétés physiques de la sphère
    public Vector3 position;    // Position dans l'espace 3D
    public Vector3 velocity;    // Vecteur vitesse (direction + magnitude)
    public float radius;        // Rayon de la sphère
    public float mass;          // Masse de la sphère
    public GameObject gameObject; // Référence au GameObject Unity pour la visualisation
    
    /// <summary>
    /// Constructeur de la sphère
    /// </summary>
    public Sphere(Vector3 pos, Vector3 vel, float r, float m)
    {
        position = pos;
        velocity = vel;
        radius = r;
        mass = m;
    }

    // Propriétés calculées pour l'algorithme Sweep and Prune
    // Ces valeurs représentent les limites de la boîte englobante (AABB) de la sphère
    public float MinX => position.x - radius;  // Limite gauche sur l'axe X
    public float MaxX => position.x + radius;  // Limite droite sur l'axe X
    public float MinY => position.y - radius;  // Limite basse sur l'axe Y
    public float MaxY => position.y + radius;  // Limite haute sur l'axe Y
    public float MinZ => position.z - radius;  // Limite arrière sur l'axe Z
    public float MaxZ => position.z + radius;  // Limite avant sur l'axe Z
}

/// <summary>
/// Classe pour les intervalles 1D utilisés dans l'algorithme Sweep and Prune
/// Un intervalle représente la projection d'une sphère sur un axe (X, Y ou Z)
/// </summary>
[System.Serializable]
public class SphereInterval
{
    public Sphere sphere;  // Référence à la sphère
    public float min;      // Point de début de l'intervalle (b_i dans le cours)
    public float max;      // Point de fin de l'intervalle (e_i dans le cours)
    
    /// <summary>
    /// Constructeur d'un intervalle
    /// </summary>
    public SphereInterval(Sphere s, float minVal, float maxVal)
    {
        sphere = s;
        min = minVal;
        max = maxVal;
    }
    
    /// <summary>
    /// Vérifie si deux intervalles se chevauchent
    /// Deux intervalles [b1,e1] et [b2,e2] se chevauchent si b1 <= e2 ET e1 >= b2
    /// </summary>
    public bool Overlaps(SphereInterval other)
    {
        return min <= other.max && max >= other.min;
    }
}

/// <summary>
/// Classe principale pour la simulation d'un gaz parfait dans une boîte cubique
/// Implémente l'algorithme Sweep and Prune pour optimiser la détection de collisions
/// Basée sur le modèle de collisions élastiques (conservation de l'énergie et de la quantité de mouvement)
/// </summary>
public class GasSimulation : MonoBehaviour
{
    [Header("Paramètres de la Boîte")]
    [SerializeField] private float boxSize = 10f;  // Taille du côté de la boîte cubique
    
    [Header("Paramètres des Sphères")]
    [SerializeField] private int numberOfSpheres = 20;       // Nombre de particules dans la simulation
    [SerializeField] private float sphereRadius = 0.3f;      // Rayon de chaque sphère
    [SerializeField] private float sphereMass = 1f;          // Masse de chaque sphère (supposée uniforme)
    [SerializeField] private float initialSpeedRange = 5f;   // Plage de vitesse initiale aléatoire
    
    [Header("Visualisation de la Boîte")]
    [SerializeField] private bool drawBox = true;            // Afficher ou non la boîte
    [SerializeField] private Color boxColor = Color.cyan;    // Couleur des arêtes de la boîte
    [SerializeField] private float boxLineWidth = 0.05f;     // Épaisseur des lignes de la boîte
    
    [Header("Matériaux")]
    [SerializeField] private Material sphereMaterial;        // Matériau optionnel pour les sphères
    
    private List<Sphere> spheres = new List<Sphere>();       // Liste de toutes les sphères
    private float halfBoxSize;                               // Demi-taille de la boîte (pour optimisation)
    private GameObject boxContainer;                         // Conteneur pour les lignes de la boîte

    /// <summary>
    /// Initialisation de la simulation au démarrage
    /// </summary>
    void Start()
    {
        halfBoxSize = boxSize / 2f;
        InitializeSpheres();
        CreateBoxVisualization();
    }

    /// <summary>
    /// Initialise toutes les sphères de la simulation
    /// - Positionne chaque sphère aléatoirement dans la boîte
    /// - Assigne une vitesse initiale aléatoire
    /// - Crée le GameObject visuel correspondant
    /// </summary>
    void InitializeSpheres()
    {
        for (int i = 0; i < numberOfSpheres; i++)
        {
            // Génération d'une position aléatoire dans la boîte
            // On s'assure que la sphère ne touche pas les parois au départ
            Vector3 randomPos = new Vector3(
                Random.Range(-halfBoxSize + sphereRadius, halfBoxSize - sphereRadius),
                Random.Range(-halfBoxSize + sphereRadius, halfBoxSize - sphereRadius),
                Random.Range(-halfBoxSize + sphereRadius, halfBoxSize - sphereRadius)
            );

            // Génération d'une vitesse initiale aléatoire dans toutes les directions
            // Cette vitesse simule l'agitation thermique des particules
            Vector3 randomVel = new Vector3(
                Random.Range(-initialSpeedRange, initialSpeedRange),
                Random.Range(-initialSpeedRange, initialSpeedRange),
                Random.Range(-initialSpeedRange, initialSpeedRange)
            );

            // Création de l'objet Sphere avec ses propriétés physiques
            Sphere sphere = new Sphere(randomPos, randomVel, sphereRadius, sphereMass);
            
            // Création du GameObject visuel (sphère primitive Unity)
            GameObject sphereObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphereObj.transform.position = randomPos;
            sphereObj.transform.localScale = Vector3.one * sphereRadius * 2f; // Multiplié par 2 car Unity utilise le diamètre
            sphereObj.name = "Sphere_" + i;
            
            // Désactivation du collider Unity par défaut
            // On gère les collisions manuellement pour plus de contrôle et de précision
            Destroy(sphereObj.GetComponent<Collider>());
            
            // Application du matériau ou d'une couleur aléatoire
            if (sphereMaterial != null)
            {
                sphereObj.GetComponent<Renderer>().material = sphereMaterial;
            }
            else
            {
                // Si aucun matériau n'est fourni, on assigne une couleur aléatoire
                sphereObj.GetComponent<Renderer>().material.color = Random.ColorHSV();
            }
            
            sphere.gameObject = sphereObj;
            spheres.Add(sphere);
        }
    }

    /// <summary>
    /// Crée la visualisation de la boîte cubique avec toutes ses arêtes
    /// Une boîte a 12 arêtes : 4 pour chaque face (haut, bas, et 4 verticales)
    /// </summary>
    void CreateBoxVisualization()
    {
        if (!drawBox) return;

        // Création d'un conteneur pour organiser les lignes
        boxContainer = new GameObject("BoxContainer");
        boxContainer.transform.parent = transform;

        float h = halfBoxSize;

        // Définition des 8 sommets du cube
        Vector3[] vertices = new Vector3[]
        {
            new Vector3(-h, -h, -h),  // 0: Bas-Arrière-Gauche
            new Vector3(h, -h, -h),   // 1: Bas-Arrière-Droite
            new Vector3(h, -h, h),    // 2: Bas-Avant-Droite
            new Vector3(-h, -h, h),   // 3: Bas-Avant-Gauche
            new Vector3(-h, h, -h),   // 4: Haut-Arrière-Gauche
            new Vector3(h, h, -h),    // 5: Haut-Arrière-Droite
            new Vector3(h, h, h),     // 6: Haut-Avant-Droite
            new Vector3(-h, h, h)     // 7: Haut-Avant-Gauche
        };

        // Définition des 12 arêtes du cube (chaque arête connecte 2 sommets)
        int[,] edges = new int[,]
        {
            // Arêtes de la face du bas
            {0, 1}, {1, 2}, {2, 3}, {3, 0},
            // Arêtes de la face du haut
            {4, 5}, {5, 6}, {6, 7}, {7, 4},
            // Arêtes verticales (connectant bas et haut)
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        };

        // Création d'une ligne pour chaque arête
        for (int i = 0; i < edges.GetLength(0); i++)
        {
            CreateEdgeLine(
                vertices[edges[i, 0]],  // Point de départ de l'arête
                vertices[edges[i, 1]],  // Point d'arrivée de l'arête
                i                       // Index pour nommer l'objet
            );
        }
    }

    /// <summary>
    /// Crée une ligne représentant une arête de la boîte
    /// </summary>
    /// <param name="start">Point de départ de la ligne</param>
    /// <param name="end">Point d'arrivée de la ligne</param>
    /// <param name="index">Index de l'arête (pour le nommage)</param>
    void CreateEdgeLine(Vector3 start, Vector3 end, int index)
    {
        // Création d'un GameObject pour cette arête
        GameObject edgeObject = new GameObject($"Edge_{index}");
        edgeObject.transform.parent = boxContainer.transform;

        // Ajout et configuration du LineRenderer
        LineRenderer lr = edgeObject.AddComponent<LineRenderer>();
        
        // Configuration du matériau et des couleurs
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.startColor = boxColor;
        lr.endColor = boxColor;
        
        // Configuration de l'épaisseur de la ligne
        lr.startWidth = boxLineWidth;
        lr.endWidth = boxLineWidth;
        
        // Définition des deux points de la ligne
        lr.positionCount = 2;
        lr.SetPosition(0, start);
        lr.SetPosition(1, end);
        
        // Désactivation de l'utilisation des coordonnées world space pour de meilleures performances
        lr.useWorldSpace = true;
    }

    /// <summary>
    /// Boucle principale de simulation appelée à intervalle fixe (physique)
    /// Pipeline de détection et résolution de collisions :
    /// 1. Mise à jour des positions (intégration d'Euler)
    /// 2. Détection des collisions avec Sweep and Prune
    /// 3. Résolution des collisions entre sphères (collisions élastiques)
    /// 4. Résolution des collisions avec les parois
    /// 5. Mise à jour de la visualisation
    /// </summary>
    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;  // Pas de temps fixe pour la simulation physique
        
        // ÉTAPE 1: Mise à jour des positions selon les équations de mouvement
        // p(t + Δt) = p(t) + v⃗ * Δt
        UpdatePositions(dt);
        
        // ÉTAPE 2: Phase Large (Broad Phase) - Détection rapide des collisions potentielles
        // Utilise l'algorithme Sweep and Prune pour réduire le nombre de tests
        List<(Sphere, Sphere)> collisionPairs = DetectCollisionsSAP();
        
        // ÉTAPE 3: Phase Précise (Narrow Phase) - Résolution des collisions entre sphères
        // Applique les formules de collision élastique
        HandleSphereCollisions(collisionPairs);
        
        // ÉTAPE 4: Gestion des collisions avec les parois de la boîte
        // Inversion de la vitesse perpendiculaire à la paroi
        HandleWallCollisions();
        
        // ÉTAPE 5: Synchronisation des GameObjects Unity avec les positions calculées
        UpdateVisualPositions();
    }

    /// <summary>
    /// Met à jour les positions de toutes les sphères selon leurs vitesses
    /// Utilise l'intégration d'Euler : p(t + Δt) = p(t) + v⃗ * Δt
    /// En l'absence de forces externes (gravité négligée), la vitesse reste constante
    /// </summary>
    /// <param name="dt">Pas de temps (Delta time)</param>
    void UpdatePositions(float dt)
    {
        foreach (var sphere in spheres)
        {
            // Équation de mouvement rectiligne uniforme
            sphere.position += sphere.velocity * dt;
        }
    }

    /// <summary>
    /// Implémentation de l'algorithme Sweep and Prune (SAP)
    /// Algorithme de détection de collision optimisé pour réduire le nombre de tests
    /// 
    /// PRINCIPE:
    /// 1. Projeter toutes les sphères sur l'axe X pour créer des intervalles [min, max]
    /// 2. Trier ces intervalles par ordre croissant de début (min)
    /// 3. Balayer l'axe en maintenant une liste d'intervalles actifs
    /// 4. Quand deux sphères se chevauchent sur X, vérifier Y et Z
    /// 5. Si chevauchement sur les 3 axes → collision potentielle
    /// 
    /// COMPLEXITÉ: O(n log n) au lieu de O(n²) pour la force brute
    /// </summary>
    /// <returns>Liste des paires de sphères en collision</returns>
    List<(Sphere, Sphere)> DetectCollisionsSAP()
    {
        List<(Sphere, Sphere)> potentialCollisions = new List<(Sphere, Sphere)>();
        
        // ÉTAPE 1: Créer les intervalles pour l'axe X
        // Chaque intervalle représente la projection de la sphère sur l'axe X
        List<SphereInterval> intervalsX = new List<SphereInterval>();
        foreach (var sphere in spheres)
        {
            intervalsX.Add(new SphereInterval(sphere, sphere.MinX, sphere.MaxX));
        }
        
        // ÉTAPE 2: Trier les intervalles par point de début (bi dans le cours)
        // Ceci permet de balayer l'axe de gauche à droite
        intervalsX.Sort((a, b) => a.min.CompareTo(b.min));
        
        // ÉTAPE 3: Balayage de l'axe X avec maintien d'une liste active
        // La liste active contient toutes les sphères dont l'intervalle n'est pas encore terminé
        List<Sphere> activeSpheresX = new List<Sphere>();
        
        for (int i = 0; i < intervalsX.Count; i++)
        {
            var currentInterval = intervalsX[i];
            
            // Retirer les sphères dont l'intervalle est terminé (ei < bi_current)
            // Ces sphères ne peuvent plus se chevaucher avec les suivantes
            activeSpheresX.RemoveAll(s => s.MaxX < currentInterval.min);
            
            // ÉTAPE 4: Tester les chevauchements avec toutes les sphères actives
            foreach (var activeSphere in activeSpheresX)
            {
                // Si chevauchement sur X, vérifier Y
                if (currentInterval.sphere.MinY <= activeSphere.MaxY && 
                    currentInterval.sphere.MaxY >= activeSphere.MinY)
                {
                    // Si chevauchement sur X et Y, vérifier Z
                    if (currentInterval.sphere.MinZ <= activeSphere.MaxZ && 
                        currentInterval.sphere.MaxZ >= activeSphere.MinZ)
                    {
                        // Les AABB (boîtes englobantes) se chevauchent sur les 3 axes
                        // Vérifier la distance réelle entre les centres des sphères
                        float distance = Vector3.Distance(currentInterval.sphere.position, activeSphere.position);
                        
                        // Collision si la distance est inférieure à la somme des rayons
                        if (distance <= currentInterval.sphere.radius + activeSphere.radius)
                        {
                            potentialCollisions.Add((currentInterval.sphere, activeSphere));
                        }
                    }
                }
            }
            
            // Ajouter la sphère courante à la liste active
            activeSpheresX.Add(currentInterval.sphere);
        }
        
        return potentialCollisions;
    }

    /// <summary>
    /// Gère les collisions élastiques entre les paires de sphères détectées
    /// 
    /// MODÈLE PHYSIQUE:
    /// - Conservation de la quantité de mouvement totale
    /// - Conservation de l'énergie cinétique totale
    /// - Pas de perte d'énergie (coefficient de restitution e = 1)
    /// 
    /// FORMULES (voir PDF Chapitre 4, page 42):
    /// 
    /// Vecteur normal de collision: n⃗ = (p⃗A - p⃗B) / ||p⃗A - p⃗B||
    /// 
    /// Nouvelles vitesses après collision:
    /// v⃗'A = v⃗A - (2mB / (mA + mB)) * ⟨v⃗A - v⃗B, p⃗A - p⃗B⟩ / ||p⃗A - p⃗B||² * (p⃗A - p⃗B)
    /// v⃗'B = v⃗B - (2mA / (mA + mB)) * ⟨v⃗B - v⃗A, p⃗B - p⃗A⟩ / ||p⃗B - p⃗A||² * (p⃗B - p⃗A)
    /// 
    /// où ⟨u⃗, v⃗⟩ représente le produit scalaire
    /// </summary>
    /// <param name="collisionPairs">Liste des paires de sphères en collision</param>
    void HandleSphereCollisions(List<(Sphere, Sphere)> collisionPairs)
    {
        foreach (var (sphereA, sphereB) in collisionPairs)
        {
            // CALCUL DU VECTEUR NORMAL DE COLLISION
            // n⃗ = direction de A vers B, normalisée
            Vector3 collisionNormal = (sphereA.position - sphereB.position).normalized;
            
            // CALCUL DE LA VITESSE RELATIVE
            // v⃗rel = v⃗A - v⃗B
            Vector3 relativeVelocity = sphereA.velocity - sphereB.velocity;
            
            // VITESSE LE LONG DE LA NORMALE DE COLLISION
            // Si positive, les sphères s'éloignent déjà → pas besoin de résoudre
            float velocityAlongNormal = Vector3.Dot(relativeVelocity, collisionNormal);
            
            // Optimisation: ne pas résoudre si les sphères s'éloignent déjà
            if (velocityAlongNormal > 0) continue;
            
            // CALCUL DES IMPULSIONS (changements de vitesse)
            float massSum = sphereA.mass + sphereB.mass;
            
            // Impulsion pour la sphère A
            // Formule: (2mB / (mA + mB)) * ⟨v⃗A - v⃗B, n⃗⟩ * n⃗
            Vector3 impulseA = (2f * sphereB.mass / massSum) * 
                               Vector3.Dot(sphereA.velocity - sphereB.velocity, collisionNormal) * 
                               collisionNormal;
            sphereA.velocity -= impulseA;
            
            // Impulsion pour la sphère B (direction opposée par conservation de la quantité de mouvement)
            // Formule: (2mA / (mA + mB)) * ⟨v⃗B - v⃗A, -n⃗⟩ * (-n⃗)
            Vector3 impulseB = (2f * sphereA.mass / massSum) * 
                               Vector3.Dot(sphereB.velocity - sphereA.velocity, -collisionNormal) * 
                               (-collisionNormal);
            sphereB.velocity -= impulseB;
            
            // CORRECTION DE L'INTERPÉNÉTRATION
            // Si les sphères se chevauchent, les séparer légèrement pour éviter un contact permanent
            float currentDistance = Vector3.Distance(sphereA.position, sphereB.position);
            float minDistance = sphereA.radius + sphereB.radius;
            float overlap = minDistance - currentDistance;
            
            if (overlap > 0)
            {
                // Séparer proportionnellement aux masses (plus léger bouge plus)
                Vector3 separation = collisionNormal * (overlap / 2f);
                sphereA.position += separation;
                sphereB.position -= separation;
            }
        }
    }

    /// <summary>
    /// Gère les collisions avec les parois de la boîte cubique
    /// 
    /// MODÈLE PHYSIQUE (voir PDF Chapitre 4, page 41):
    /// - Lorsqu'une sphère touche une paroi, sa vitesse perpendiculaire s'inverse
    /// - Formule: v⃗nouvelle = v⃗ - 2(v⃗ · n⃗)n⃗
    ///   où n⃗ est le vecteur normal à la paroi
    /// 
    /// CONDITIONS DE COLLISION:
    /// - Paroi X: px - r ≤ -L/2  ou  px + r ≥ L/2
    /// - Paroi Y: py - r ≤ -L/2  ou  py + r ≥ L/2
    /// - Paroi Z: pz - r ≤ -L/2  ou  pz + r ≥ L/2
    /// 
    /// où L est la taille de la boîte, r le rayon de la sphère
    /// </summary>
    void HandleWallCollisions()
    {
        foreach (var sphere in spheres)
        {
            // COLLISIONS AVEC LES PAROIS X (gauche et droite)
            // Paroi gauche (X = -halfBoxSize)
            if (sphere.position.x - sphere.radius <= -halfBoxSize)
            {
                // Repositionner la sphère juste à la limite
                sphere.position.x = -halfBoxSize + sphere.radius;
                // Inverser la composante X de la vitesse (rebond élastique)
                sphere.velocity.x = Mathf.Abs(sphere.velocity.x);
            }
            // Paroi droite (X = +halfBoxSize)
            else if (sphere.position.x + sphere.radius >= halfBoxSize)
            {
                sphere.position.x = halfBoxSize - sphere.radius;
                sphere.velocity.x = -Mathf.Abs(sphere.velocity.x);
            }
            
            // COLLISIONS AVEC LES PAROIS Y (bas et haut)
            // Paroi du bas (Y = -halfBoxSize)
            if (sphere.position.y - sphere.radius <= -halfBoxSize)
            {
                sphere.position.y = -halfBoxSize + sphere.radius;
                sphere.velocity.y = Mathf.Abs(sphere.velocity.y);
            }
            // Paroi du haut (Y = +halfBoxSize)
            else if (sphere.position.y + sphere.radius >= halfBoxSize)
            {
                sphere.position.y = halfBoxSize - sphere.radius;
                sphere.velocity.y = -Mathf.Abs(sphere.velocity.y);
            }
            
            // COLLISIONS AVEC LES PAROIS Z (arrière et avant)
            // Paroi arrière (Z = -halfBoxSize)
            if (sphere.position.z - sphere.radius <= -halfBoxSize)
            {
                sphere.position.z = -halfBoxSize + sphere.radius;
                sphere.velocity.z = Mathf.Abs(sphere.velocity.z);
            }
            // Paroi avant (Z = +halfBoxSize)
            else if (sphere.position.z + sphere.radius >= halfBoxSize)
            {
                sphere.position.z = halfBoxSize - sphere.radius;
                sphere.velocity.z = -Mathf.Abs(sphere.velocity.z);
            }
        }
    }

    /// <summary>
    /// Synchronise les positions des GameObjects Unity avec les positions calculées
    /// Cette méthode met à jour la visualisation après tous les calculs physiques
    /// </summary>
    void UpdateVisualPositions()
    {
        foreach (var sphere in spheres)
        {
            if (sphere.gameObject != null)
            {
                sphere.gameObject.transform.position = sphere.position;
            }
        }
    }

    /// <summary>
    /// Dessine les Gizmos dans l'éditeur Unity pour le débogage
    /// Affiche les boîtes englobantes (AABB) de chaque sphère
    /// </summary>
    void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;
        
        // Dessiner les boîtes englobantes des sphères en vert
        Gizmos.color = Color.green;
        foreach (var sphere in spheres)
        {
            Gizmos.DrawWireSphere(sphere.position, sphere.radius);
        }
    }
}