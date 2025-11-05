using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Contrôleur de simulation avec support dimensions personnalisables
/// </summary>
public class UpdatedSimulationController : MonoBehaviour
{
    [Header("Références")]
    public AdaptedPhysicsManager physicsManager;
    public HorizontalPlateBuilder plateBuilder;
    public StaticImpactSphere staticSphere;
    
    [Header("UI (optionnel)")]
    public Text statsText;
    public Slider elasticitySlider;
    public Button resetButton;
    public Button rebuildButton;
    public Button pauseButton;
    
    [Header("Contrôles Clavier")]
    public KeyCode resetKey = KeyCode.R;
    public KeyCode pauseKey = KeyCode.P;
    public KeyCode rebuildKey = KeyCode.B;
    public KeyCode increaseElasticityKey = KeyCode.KeypadPlus;
    public KeyCode decreaseElasticityKey = KeyCode.KeypadMinus;
    
    private bool isPaused = false;

    void Start()
    {
        if (physicsManager == null)
            physicsManager = FindObjectOfType<AdaptedPhysicsManager>();
        
        if (plateBuilder == null)
            plateBuilder = FindObjectOfType<HorizontalPlateBuilder>();
        
        if (staticSphere == null)
            staticSphere = FindObjectOfType<StaticImpactSphere>();
        
        SetupUI();
    }

    void SetupUI()
    {
        if (elasticitySlider != null)
        {
            elasticitySlider.minValue = 0f;
            elasticitySlider.maxValue = 2f;
            elasticitySlider.value = physicsManager != null ? physicsManager.globalElasticity : 0.6f;
            elasticitySlider.onValueChanged.AddListener(OnElasticityChanged);
        }
        
        if (resetButton != null)
            resetButton.onClick.AddListener(ResetSimulation);
        
        if (rebuildButton != null)
            rebuildButton.onClick.AddListener(RebuildPlate);
        
        if (pauseButton != null)
            pauseButton.onClick.AddListener(TogglePause);
    }

    void Update()
    {
        HandleKeyboardInput();
        UpdateUI();
    }

    void HandleKeyboardInput()
    {
        if (Input.GetKeyDown(resetKey))
            ResetSimulation();
        
        if (Input.GetKeyDown(pauseKey))
            TogglePause();
        
        if (Input.GetKeyDown(rebuildKey))
            RebuildPlate();
        
        if (Input.GetKey(increaseElasticityKey))
            AdjustElasticity(0.01f);
        
        if (Input.GetKey(decreaseElasticityKey))
            AdjustElasticity(-0.01f);
        
        // Presets de dimensions (touches 1-3)
        if (Input.GetKeyDown(KeyCode.Alpha1) && plateBuilder != null)
        {
            plateBuilder.width = 5;
            plateBuilder.depth = 5;
            RebuildPlate();
        }
        
        if (Input.GetKeyDown(KeyCode.Alpha2) && plateBuilder != null)
        {
            plateBuilder.width = 10;
            plateBuilder.depth = 10;
            RebuildPlate();
        }
        
        if (Input.GetKeyDown(KeyCode.Alpha3) && plateBuilder != null)
        {
            plateBuilder.width = 15;
            plateBuilder.depth = 15;
            RebuildPlate();
        }
        
        // NOUVEAU: Presets épaisseur (touches 4-6)
        if (Input.GetKeyDown(KeyCode.Alpha4) && plateBuilder != null)
        {
            plateBuilder.ApplyThinPlatePreset(); // 1x0.1x1
        }
        
        if (Input.GetKeyDown(KeyCode.Alpha5) && plateBuilder != null)
        {
            plateBuilder.ApplyMediumPlatePreset(); // 1x0.5x1
        }
        
        if (Input.GetKeyDown(KeyCode.Alpha6) && plateBuilder != null)
        {
            plateBuilder.ApplyThickPlatePreset(); // 1x1x1
        }
    }

    void UpdateUI()
    {
        if (statsText != null && physicsManager != null)
        {
            statsText.text = "═══ SIMULATION PLAN HORIZONTAL ═══\n\n";
            statsText.text += physicsManager.GetSimulationStats();
            
            if (plateBuilder != null)
            {
                Vector3 dims = plateBuilder.GetPlateDimensions();
                statsText.text += $"\n\n═══ PLAN ═══\n";
                statsText.text += $"Grille: {plateBuilder.width}x{plateBuilder.depth}\n";
                statsText.text += $"Cube: {plateBuilder.cubeWidth:F2}x{plateBuilder.cubeHeight:F2}x{plateBuilder.cubeDepth:F2}m\n";
                statsText.text += $"Hauteur: {plateBuilder.suspensionHeight:F1}m\n";
                statsText.text += $"Taille totale: {dims.x:F1}x{dims.z:F1}m";
            }
            
            statsText.text += $"\n\n═══ CONTRÔLES ═══\n";
            statsText.text += $"{resetKey}: Reset\n";
            statsText.text += $"{pauseKey}: Pause\n";
            statsText.text += $"{rebuildKey}: Rebuild\n";
            statsText.text += $"+/-: Élasticité\n";
            statsText.text += $"1/2/3: Grilles (5x5, 10x10, 15x15)\n";
            statsText.text += $"4/5/6: Épaisseur (fine/moyenne/épaisse)";
        }
        
        if (pauseButton != null)
        {
            Text buttonText = pauseButton.GetComponentInChildren<Text>();
            if (buttonText != null)
                buttonText.text = isPaused ? "Resume" : "Pause";
        }
    }

    public void ResetSimulation()
    {
        Debug.Log("═══ RESET SIMULATION ═══");
        
        if (physicsManager != null)
            physicsManager.ResetSimulation();
    }

    public void RebuildPlate()
    {
        Debug.Log("═══ REBUILD PLATE ═══");
        
        if (plateBuilder != null)
        {
            plateBuilder.BuildHorizontalPlate();
            
            if (physicsManager != null)
                physicsManager.RegisterAllBodies();
        }
    }

    public void TogglePause()
    {
        isPaused = !isPaused;
        
        if (physicsManager != null)
            physicsManager.pauseSimulation = isPaused;
        
        Debug.Log(isPaused ? "⏸ PAUSE" : "▶ PLAY");
    }

    public void AdjustElasticity(float delta)
    {
        if (physicsManager != null)
        {
            physicsManager.globalElasticity = Mathf.Clamp(
                physicsManager.globalElasticity + delta, 0f, 2f
            );
            
            if (elasticitySlider != null)
                elasticitySlider.value = physicsManager.globalElasticity;
        }
    }

    void OnElasticityChanged(float value)
    {
        if (physicsManager != null)
        {
            physicsManager.globalElasticity = value;
            Debug.Log($"Élasticité α = {value:F2}");
        }
    }

    void OnGUI()
    {
        if (statsText == null)
        {
            GUILayout.BeginArea(new Rect(10, 10, 400, 700));
            
            GUILayout.Box("═══ SIMULATION PLAN HORIZONTAL ═══");
            
            if (physicsManager != null)
            {
                GUILayout.Label(physicsManager.GetSimulationStats());
            }
            
            GUILayout.Space(10);
            
            if (GUILayout.Button($"Reset ({resetKey})"))
                ResetSimulation();
            
            if (GUILayout.Button($"Rebuild Plate ({rebuildKey})"))
                RebuildPlate();
            
            if (GUILayout.Button($"Pause/Resume ({pauseKey})"))
                TogglePause();
            
            GUILayout.Space(10);
            
            if (plateBuilder != null)
            {
                GUILayout.Label("═══ DIMENSIONS GRILLE ═══");
                
                if (GUILayout.Button("5x5 (touche 1)"))
                {
                    plateBuilder.width = 5;
                    plateBuilder.depth = 5;
                    RebuildPlate();
                }
                
                if (GUILayout.Button("10x10 (touche 2)"))
                {
                    plateBuilder.width = 10;
                    plateBuilder.depth = 10;
                    RebuildPlate();
                }
                
                if (GUILayout.Button("15x15 (touche 3)"))
                {
                    plateBuilder.width = 15;
                    plateBuilder.depth = 15;
                    RebuildPlate();
                }
                
                GUILayout.Space(10);
                GUILayout.Label("═══ DIMENSIONS CUBES ═══");
                
                if (GUILayout.Button("Plaque FINE 1x0.1x1 (touche 4)"))
                {
                    plateBuilder.ApplyThinPlatePreset();
                }
                
                if (GUILayout.Button("Plaque MOYENNE 1x0.5x1 (touche 5)"))
                {
                    plateBuilder.ApplyMediumPlatePreset();
                }
                
                if (GUILayout.Button("Plaque ÉPAISSE 1x1x1 (touche 6)"))
                {
                    plateBuilder.ApplyThickPlatePreset();
                }
                
                GUILayout.Space(10);
                GUILayout.Label($"Largeur (X): {plateBuilder.cubeWidth:F2}m");
                plateBuilder.cubeWidth = GUILayout.HorizontalSlider(
                    plateBuilder.cubeWidth, 0.1f, 2f
                );
                
                GUILayout.Label($"Hauteur (Y): {plateBuilder.cubeHeight:F2}m");
                plateBuilder.cubeHeight = GUILayout.HorizontalSlider(
                    plateBuilder.cubeHeight, 0.05f, 2f
                );
                
                GUILayout.Label($"Profondeur (Z): {plateBuilder.cubeDepth:F2}m");
                plateBuilder.cubeDepth = GUILayout.HorizontalSlider(
                    plateBuilder.cubeDepth, 0.1f, 2f
                );
                
                GUILayout.Space(5);
                
                GUILayout.Label($"Hauteur suspension: {plateBuilder.suspensionHeight:F1}m");
                plateBuilder.suspensionHeight = GUILayout.HorizontalSlider(
                    plateBuilder.suspensionHeight, 2f, 10f
                );
            }
            
            GUILayout.Space(10);
            
            if (physicsManager != null)
            {
                GUILayout.Label($"Élasticité α: {physicsManager.globalElasticity:F2}");
                physicsManager.globalElasticity = GUILayout.HorizontalSlider(
                    physicsManager.globalElasticity, 0f, 2f
                );
            }
            
            if (staticSphere != null)
            {
                GUILayout.Space(5);
                GUILayout.Label($"Rayon rupture: {staticSphere.breakRadius:F1}m");
                staticSphere.breakRadius = GUILayout.HorizontalSlider(
                    staticSphere.breakRadius, 1f, 10f
                );
                
                GUILayout.Label($"Restitution sphère: {staticSphere.restitution:F2}");
                staticSphere.restitution = GUILayout.HorizontalSlider(
                    staticSphere.restitution, 0f, 1f
                );
            }
            
            GUILayout.EndArea();
        }
    }
}