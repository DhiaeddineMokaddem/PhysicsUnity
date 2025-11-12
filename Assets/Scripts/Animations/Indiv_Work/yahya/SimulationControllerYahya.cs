using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Contrôle la simulation et l'interface utilisateur
/// MODIFIÉ: Adapté pour l'impact horizontal
/// </summary>
public class SimulationControllerYahya : MonoBehaviour
{
    [Header("Références")]
    public PhysicsManagerYahya physicsManager;
    public StructureBuilderYahya structureBuilder;
    public ImpactSphereYahya impactSphere;
    
    [Header("UI Elements (optionnels)")]
    public Text statsText;
    public Slider elasticitySlider;
    public Button resetButton;
    public Button launchSphereButton;
    public Button pauseButton;
    public Button switchSideButton;
    
    [Header("Contrôles Clavier")]
    public KeyCode resetKey = KeyCode.R;
    public KeyCode launchKey = KeyCode.Space;
    public KeyCode pauseKey = KeyCode.P;
    public KeyCode switchSideKey = KeyCode.S; // Nouveau: changer le côté de lancement
    public KeyCode increaseElasticityKey = KeyCode.Plus;
    public KeyCode decreaseElasticityKey = KeyCode.Minus;
    public KeyCode increaseImpactKey = KeyCode.I;
    public KeyCode decreaseImpactKey = KeyCode.K;
    public KeyCode increaseSpeedKey = KeyCode.UpArrow;
    public KeyCode decreaseSpeedKey = KeyCode.DownArrow;
    
    private bool isPaused = false;

    void Start()
    {
        // Trouver les composants si non assignés
        if (physicsManager == null)
            physicsManager = FindObjectOfType<PhysicsManagerYahya>();
        
        if (structureBuilder == null)
            structureBuilder = FindObjectOfType<StructureBuilderYahya>();
        
        if (impactSphere == null)
            impactSphere = FindObjectOfType<ImpactSphereYahya>();
        
        // Configuration UI
        SetupUI();
    }

    /// <summary>
    /// Configure les éléments UI
    /// </summary>
    void SetupUI()
    {
        if (elasticitySlider != null)
        {
            elasticitySlider.minValue = 0f;
            elasticitySlider.maxValue = 2f;
            elasticitySlider.value = physicsManager != null ? physicsManager.globalElasticity : 1f;
            elasticitySlider.onValueChanged.AddListener(OnElasticityChanged);
        }
        
        if (resetButton != null)
        {
            resetButton.onClick.AddListener(ResetSimulation);
        }
        
        if (launchSphereButton != null)
        {
            launchSphereButton.onClick.AddListener(LaunchSphere);
        }
        
        if (pauseButton != null)
        {
            pauseButton.onClick.AddListener(TogglePause);
        }
        
        if (switchSideButton != null)
        {
            switchSideButton.onClick.AddListener(SwitchLaunchSide);
        }
    }

    void Update()
    {
        // Contrôles clavier
        HandleKeyboardInput();
        
        // Mise à jour UI
        UpdateUI();
    }

    /// <summary>
    /// Gère les entrées clavier
    /// </summary>
    void HandleKeyboardInput()
    {
        if (Input.GetKeyDown(resetKey))
        {
            ResetSimulation();
        }
        
        if (Input.GetKeyDown(launchKey))
        {
            LaunchSphere();
        }
        
        if (Input.GetKeyDown(pauseKey))
        {
            TogglePause();
        }
        
        // NOUVEAU: Changer le côté de lancement
        if (Input.GetKeyDown(switchSideKey))
        {
            SwitchLaunchSide();
        }
        
        if (Input.GetKey(increaseElasticityKey))
        {
            AdjustElasticity(0.01f);
        }
        
        if (Input.GetKey(decreaseElasticityKey))
        {
            AdjustElasticity(-0.01f);
        }
        
        // Impact Multiplier
        if (Input.GetKey(increaseImpactKey))
        {
            AdjustImpactMultiplier(0.1f);
        }
        
        if (Input.GetKey(decreaseImpactKey))
        {
            AdjustImpactMultiplier(-0.1f);
        }
        
        // Contrôle de la vitesse de lancement
        if (Input.GetKey(increaseSpeedKey))
        {
            AdjustLaunchSpeed(1f);
        }
        
        if (Input.GetKey(decreaseSpeedKey))
        {
            AdjustLaunchSpeed(-1f);
        }
        
        // Build wall structure
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            structureBuilder?.BuildStructure();
        }
    }

    /// <summary>
    /// Met à jour l'interface utilisateur
    /// </summary>
    void UpdateUI()
    {
        if (statsText != null && physicsManager != null)
        {
            statsText.text = physicsManager.GetSimulationStats();
            
            if (impactSphere != null)
            {
                float currentSpeed = impactSphere.velocity.magnitude;
                float kineticEnergy = 0.5f * impactSphere.mass * currentSpeed * currentSpeed;
                float estimatedForce = kineticEnergy * impactSphere.impactMultiplier;
                
                statsText.text += $"\n\n=== Sphère d'Impact ===";
                statsText.text += $"\nVitesse: {currentSpeed:F1} m/s";
                statsText.text += $"\nVitesse de lancement: {impactSphere.launchSpeed:F1} m/s";
                statsText.text += $"\nCôté: {(impactSphere.launchFrom == ImpactSphereYahya.LaunchSide.Right ? "Droite→Gauche" : "Gauche→Droite")}";
                statsText.text += $"\nMultiplicateur: {impactSphere.impactMultiplier:F1}x";
                statsText.text += $"\nÉnergie cinétique: {kineticEnergy:F0} J";
                statsText.text += $"\nForce estimée: {estimatedForce:F0} N";
            }
            
            statsText.text += $"\n\n=== Contrôles ===";
            statsText.text += $"\n{resetKey}: Reset";
            statsText.text += $"\n{launchKey}: Launch Sphere";
            statsText.text += $"\n{pauseKey}: Pause";
            statsText.text += $"\n{switchSideKey}: Switch Side (→/←)";
            statsText.text += $"\n+/-: Élasticité";
            statsText.text += $"\nI/K: Multiplicateur d'impact";
            statsText.text += $"\n↑/↓: Vitesse de lancement";
            statsText.text += $"\n1: Build Wall";
        }
        
        if (pauseButton != null)
        {
            Text buttonText = pauseButton.GetComponentInChildren<Text>();
            if (buttonText != null)
            {
                buttonText.text = isPaused ? "Resume" : "Pause";
            }
        }
        
        if (switchSideButton != null && impactSphere != null)
        {
            Text buttonText = switchSideButton.GetComponentInChildren<Text>();
            if (buttonText != null)
            {
                buttonText.text = impactSphere.launchFrom == ImpactSphereYahya.LaunchSide.Right ? "→" : "←";
            }
        }
    }

    /// <summary>
    /// Réinitialise la simulation
    /// </summary>
    public void ResetSimulation()
    {
        Debug.Log("Réinitialisation de la simulation");
        
        if (physicsManager != null)
        {
            physicsManager.ResetSimulation();
        }
        
        if (impactSphere != null)
        {
            impactSphere.Reset();
        }
    }

    /// <summary>
    /// Lance la sphère d'impact horizontalement
    /// MODIFIÉ: Lance horizontalement vers le centre de la structure
    /// </summary>
    public void LaunchSphere()
    {
        if (impactSphere != null)
        {
            // Viser le centre de la structure horizontalement
            if (structureBuilder != null)
            {
                Vector3 structureCenter = structureBuilder.transform.position;
                // Garder la hauteur actuelle de la sphère
                structureCenter.y = impactSphere.launchHeight;
                impactSphere.LaunchTowards(structureCenter);
            }
            else
            {
                impactSphere.Launch();
            }
            
            string direction = impactSphere.launchFrom == ImpactSphereYahya.LaunchSide.Right ? "Droite→Gauche" : "Gauche→Droite";
            Debug.Log($"Sphère lancée {direction} à {impactSphere.launchSpeed:F1} m/s!");
        }
    }

    /// <summary>
    /// NOUVEAU: Change le côté de lancement de la sphère
    /// </summary>
    public void SwitchLaunchSide()
    {
        if (impactSphere != null)
        {
            impactSphere.launchFrom = impactSphere.launchFrom == ImpactSphereYahya.LaunchSide.Right 
                ? ImpactSphereYahya.LaunchSide.Left 
                : ImpactSphereYahya.LaunchSide.Right;
            
            // Reset la position de la sphère
            impactSphere.Reset();
            
            string newSide = impactSphere.launchFrom == ImpactSphereYahya.LaunchSide.Right ? "Droite→Gauche" : "Gauche→Droite";
            Debug.Log($"Côté de lancement changé: {newSide}");
        }
    }

    /// <summary>
    /// Bascule pause/lecture
    /// </summary>
    public void TogglePause()
    {
        isPaused = !isPaused;
        
        if (physicsManager != null)
        {
            physicsManager.pauseSimulation = isPaused;
        }
        
        Debug.Log(isPaused ? "Simulation en pause" : "Simulation en cours");
    }

    /// <summary>
    /// Modifie l'élasticité globale
    /// </summary>
    public void AdjustElasticity(float delta)
    {
        if (physicsManager != null)
        {
            physicsManager.globalElasticity = Mathf.Clamp(physicsManager.globalElasticity + delta, 0f, 2f);
            
            if (elasticitySlider != null)
            {
                elasticitySlider.value = physicsManager.globalElasticity;
            }
        }
    }
    
    /// <summary>
    /// Modifie le multiplicateur d'impact de la sphère
    /// </summary>
    public void AdjustImpactMultiplier(float delta)
    {
        if (impactSphere != null)
        {
            impactSphere.impactMultiplier = Mathf.Clamp(impactSphere.impactMultiplier + delta, 0.1f, 10f);
            Debug.Log($"Multiplicateur d'impact: {impactSphere.impactMultiplier:F1}x");
        }
    }
    
    /// <summary>
    /// Modifie la vitesse de lancement de la sphère
    /// </summary>
    public void AdjustLaunchSpeed(float delta)
    {
        if (impactSphere != null)
        {
            impactSphere.launchSpeed = Mathf.Clamp(impactSphere.launchSpeed + delta, 1f, 100f);
            Debug.Log($"Vitesse de lancement: {impactSphere.launchSpeed:F1} m/s");
        }
    }

    /// <summary>
    /// Callback pour le slider d'élasticité
    /// </summary>
    void OnElasticityChanged(float value)
    {
        if (physicsManager != null)
        {
            physicsManager.globalElasticity = value;
            Debug.Log($"Élasticité globale: {value:F2}");
        }
    }

    /// <summary>
    /// Applique une explosion à une position donnée
    /// </summary>
    public void ApplyExplosionAt(Vector3 position, float radius = 5f, float force = 1000f)
    {
        if (physicsManager != null)
        {
            physicsManager.ApplyExplosion(position, radius, force);
            physicsManager.BreakConstraintsInRadius(position, radius);
        }
    }

    /// <summary>
    /// Ralentit le temps (effet bullet-time)
    /// </summary>
    public void SetTimeScale(float scale)
    {
        Time.timeScale = Mathf.Clamp(scale, 0.1f, 2f);
        Debug.Log($"Time scale: {Time.timeScale}");
    }

    void OnGUI()
    {
        // Interface de debug simple
        if (statsText == null)
        {
            GUILayout.BeginArea(new Rect(10, 10, 350, 600));
            GUILayout.Box("Simulation de Corps Rigides - Impact Horizontal");
            
            if (physicsManager != null)
            {
                GUILayout.Label(physicsManager.GetSimulationStats());
            }
            
            GUILayout.Space(10);
            
            if (GUILayout.Button($"Reset ({resetKey})"))
            {
                ResetSimulation();
            }
            
            if (GUILayout.Button($"Launch Sphere ({launchKey})"))
            {
                LaunchSphere();
            }
            
            if (GUILayout.Button($"Pause/Resume ({pauseKey})"))
            {
                TogglePause();
            }
            
            // NOUVEAU: Bouton pour changer le côté
            if (impactSphere != null)
            {
                string sideText = impactSphere.launchFrom == ImpactSphereYahya.LaunchSide.Right ? "→ Droite→Gauche" : "← Gauche→Droite";
                if (GUILayout.Button($"Switch Side ({switchSideKey}): {sideText}"))
                {
                    SwitchLaunchSide();
                }
            }
            
            GUILayout.Space(10);
            
            if (GUILayout.Button("Build Wall (1)"))
            {
                structureBuilder?.BuildStructure();
            }
            

            GUILayout.Space(10);
            
            if (physicsManager != null)
            {
                GUILayout.Label($"Elasticity: {physicsManager.globalElasticity:F2} (+/-)");
                physicsManager.globalElasticity = GUILayout.HorizontalSlider(
                    physicsManager.globalElasticity, 0f, 2f
                );
            }
            
            GUILayout.Space(5);
            
            if (impactSphere != null)
            {
                GUILayout.Label($"Launch Speed: {impactSphere.launchSpeed:F1} m/s (↑/↓)");
                impactSphere.launchSpeed = GUILayout.HorizontalSlider(
                    impactSphere.launchSpeed, 1f, 100f
                );
                
                GUILayout.Space(5);
                
                GUILayout.Label($"Impact Multiplier: {impactSphere.impactMultiplier:F1}x (I/K)");
                impactSphere.impactMultiplier = GUILayout.HorizontalSlider(
                    impactSphere.impactMultiplier, 0.1f, 3f
                );
                
                GUILayout.Space(5);
                
                GUILayout.Label($"Max Impulse: {impactSphere.maxImpulsePerCollision:F0} N·s");
                impactSphere.maxImpulsePerCollision = GUILayout.HorizontalSlider(
                    impactSphere.maxImpulsePerCollision, 10f, 200f
                );
                
                GUILayout.Space(5);
                
                // Affichage de l'énergie estimée
                float ke = 0.5f * impactSphere.mass * impactSphere.launchSpeed * impactSphere.launchSpeed;
                float estForce = Mathf.Min(ke * impactSphere.impactMultiplier, 500f);
                GUILayout.Label($"Estimated Impact: {estForce:F0} N (max 500)");
            }
            
            GUILayout.EndArea();
        }
    }
}