using UnityEngine;

/// <summary>
/// Contrôleur d'interface utilisateur pour la simulation de poutre
/// </summary>
public class BeamUIController : MonoBehaviour
{
    [Header("Référence")]
    public BeamSimulation beamSimulation;
    
    [Header("Interface")]
    public bool showDebugUI = true;
    public KeyCode startKey = KeyCode.Space;
    public KeyCode resetKey = KeyCode.R;
    public KeyCode increaseAlphaKey = KeyCode.Plus;
    public KeyCode decreaseAlphaKey = KeyCode.Minus;
    
    private Rect windowRect = new Rect(10, 10, 320, 400);
    private float alphaSliderValue = 0.5f;
    private bool wasAlphaChanged = false;
    
    void Start()
    {
        if (beamSimulation == null)
        {
            beamSimulation = FindObjectOfType<BeamSimulation>();
        }
        
        if (beamSimulation != null)
        {
            alphaSliderValue = beamSimulation.alpha;
        }
    }
    
    void Update()
    {
        HandleKeyboardInput();
        
        // Synchroniser α avec le slider
        if (beamSimulation != null && wasAlphaChanged)
        {
            beamSimulation.alpha = alphaSliderValue;
            wasAlphaChanged = false;
        }
    }
    
    /// <summary>
    /// Gère les entrées clavier
    /// </summary>
    void HandleKeyboardInput()
    {
        if (beamSimulation == null) return;
        
        if (Input.GetKeyDown(startKey))
        {
            beamSimulation.StartSimulation();
        }
        
        if (Input.GetKeyDown(resetKey))
        {
            beamSimulation.ResetSimulation();
        }
        
        if (Input.GetKey(increaseAlphaKey) || Input.GetKey(KeyCode.Equals))
        {
            alphaSliderValue = Mathf.Clamp(alphaSliderValue + 0.01f, 0f, 2f);
            beamSimulation.alpha = alphaSliderValue;
        }
        
        if (Input.GetKey(decreaseAlphaKey) || Input.GetKey(KeyCode.Underscore))
        {
            alphaSliderValue = Mathf.Clamp(alphaSliderValue - 0.01f, 0f, 2f);
            beamSimulation.alpha = alphaSliderValue;
        }
    }
    
    void OnGUI()
    {
        if (!showDebugUI || beamSimulation == null) return;
        
        // Style personnalisé
        GUIStyle windowStyle = new GUIStyle(GUI.skin.window);
        windowStyle.fontSize = 12;
        
        windowRect = GUILayout.Window(0, windowRect, DrawWindow, "Simulation de Poutre Flexible", windowStyle);
    }
    
    /// <summary>
    /// Dessine la fenêtre d'interface
    /// </summary>
    void DrawWindow(int windowID)
    {
        GUILayout.BeginVertical();
        
        // === Section Informations ===
        GUILayout.Label("=== INFORMATIONS ===", GetHeaderStyle());
        
        GUILayout.Label($"État: {(beamSimulation != null ? "Prêt" : "Non initialisé")}");
        
        if (beamSimulation != null)
        {
            GUILayout.Label($"Longueur poutre: {beamSimulation.beamLength:F2} m");
            GUILayout.Label($"Masse segments: {beamSimulation.segmentMass:F2} kg");
            GUILayout.Label($"Masse projectile: {beamSimulation.projectileMass:F2} kg");
            GUILayout.Label($"Vitesse projectile: {beamSimulation.projectileSpeed:F1} m/s");
        }
        
        GUILayout.Space(10);
        
        // === Section Paramètre α ===
        GUILayout.Label("=== PARAMÈTRE α (ÉLASTICITÉ) ===", GetHeaderStyle());
        
        GUILayout.BeginHorizontal();
        GUILayout.Label($"α = {alphaSliderValue:F2}", GUILayout.Width(80));
        float newAlpha = GUILayout.HorizontalSlider(alphaSliderValue, 0f, 2f, GUILayout.Width(200));
        if (Mathf.Abs(newAlpha - alphaSliderValue) > 0.001f)
        {
            alphaSliderValue = newAlpha;
            wasAlphaChanged = true;
        }
        GUILayout.EndHorizontal();
        
        GUILayout.Label("0 = Rigide, 2 = Très flexible");
        
        GUILayout.Space(5);
        
        // Boutons rapides
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("α = 0"))
        {
            alphaSliderValue = 0f;
            wasAlphaChanged = true;
        }
        if (GUILayout.Button("α = 0.5"))
        {
            alphaSliderValue = 0.5f;
            wasAlphaChanged = true;
        }
        if (GUILayout.Button("α = 1"))
        {
            alphaSliderValue = 1f;
            wasAlphaChanged = true;
        }
        if (GUILayout.Button("α = 1.5"))
        {
            alphaSliderValue = 1.5f;
            wasAlphaChanged = true;
        }
        GUILayout.EndHorizontal();
        
        GUILayout.Space(10);
        
        // === Section Contrôles ===
        GUILayout.Label("=== CONTRÔLES ===", GetHeaderStyle());
        
        GUIStyle buttonStyle = new GUIStyle(GUI.skin.button);
        buttonStyle.fontSize = 14;
        buttonStyle.padding = new RectOffset(10, 10, 10, 10);
        
        if (GUILayout.Button($"▶ DÉMARRER ({startKey})", buttonStyle, GUILayout.Height(40)))
        {
            beamSimulation.StartSimulation();
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button($"⟲ RÉINITIALISER ({resetKey})", buttonStyle, GUILayout.Height(40)))
        {
            beamSimulation.ResetSimulation();
        }
        
        GUILayout.Space(10);
        
        // === Section Raccourcis clavier ===
        GUILayout.Label("=== RACCOURCIS CLAVIER ===", GetHeaderStyle());
        
        GUILayout.Label($"{startKey} : Démarrer la simulation");
        GUILayout.Label($"{resetKey} : Réinitialiser");
        GUILayout.Label($"+/- : Augmenter/Diminuer α");
        
        GUILayout.Space(10);
        
        // === Section Description ===
        GUILayout.Label("=== DESCRIPTION ===", GetHeaderStyle());
        
        GUILayout.Label("Une poutre flexible composée de deux", GetSmallTextStyle());
        GUILayout.Label("segments est fixée entre deux poteaux.", GetSmallTextStyle());
        GUILayout.Label("Un cube vert est lancé vers le centre.", GetSmallTextStyle());
        GUILayout.Label("", GetSmallTextStyle());
        GUILayout.Label("Le paramètre α contrôle la rigidité:", GetSmallTextStyle());
        GUILayout.Label("• α = 0 : Poutre très rigide", GetSmallTextStyle());
        GUILayout.Label("• α > 0 : Poutre flexible", GetSmallTextStyle());
        GUILayout.Label("• α > 1 : Fracture spectaculaire", GetSmallTextStyle());
        
        GUILayout.EndVertical();
        
        GUI.DragWindow();
    }
    
    /// <summary>
    /// Style pour les en-têtes
    /// </summary>
    GUIStyle GetHeaderStyle()
    {
        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontStyle = FontStyle.Bold;
        style.fontSize = 12;
        style.normal.textColor = Color.white;
        return style;
    }
    
    /// <summary>
    /// Style pour le petit texte
    /// </summary>
    GUIStyle GetSmallTextStyle()
    {
        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 10;
        style.wordWrap = true;
        return style;
    }
}