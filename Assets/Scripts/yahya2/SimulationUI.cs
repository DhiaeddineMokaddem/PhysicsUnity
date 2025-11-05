using UnityEngine;

/// <summary>
/// Interface utilisateur moderne pour la simulation
/// </summary>
public class SimulationUI : MonoBehaviour
{
    private BeamSimulationManager simManager;
    
    // Style UI
    private GUIStyle boxStyle;
    private GUIStyle buttonStyle;
    private GUIStyle labelStyle;
    private GUIStyle titleStyle;
    
    private bool stylesInitialized = false;

    void Start()
    {
        simManager = FindObjectOfType<BeamSimulationManager>();
        
        if (simManager == null)
        {
            Debug.LogError("BeamSimulationManager introuvable !");
        }
    }

    void InitializeStyles()
    {
        if (stylesInitialized) return;
        
        // Style de la boîte
        boxStyle = new GUIStyle(GUI.skin.box);
        boxStyle.normal.background = MakeTex(2, 2, new Color(0.1f, 0.1f, 0.1f, 0.9f));
        boxStyle.padding = new RectOffset(15, 15, 15, 15);
        
        // Style des boutons
        buttonStyle = new GUIStyle(GUI.skin.button);
        buttonStyle.normal.background = MakeTex(2, 2, new Color(0.2f, 0.5f, 0.8f, 1f));
        buttonStyle.hover.background = MakeTex(2, 2, new Color(0.3f, 0.6f, 0.9f, 1f));
        buttonStyle.active.background = MakeTex(2, 2, new Color(0.1f, 0.4f, 0.7f, 1f));
        buttonStyle.normal.textColor = Color.white;
        buttonStyle.hover.textColor = Color.white;
        buttonStyle.active.textColor = Color.white;
        buttonStyle.fontSize = 14;
        buttonStyle.fontStyle = FontStyle.Bold;
        buttonStyle.padding = new RectOffset(10, 10, 10, 10);
        
        // Style des labels
        labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.normal.textColor = Color.white;
        labelStyle.fontSize = 12;
        labelStyle.padding = new RectOffset(5, 5, 5, 5);
        
        // Style du titre
        titleStyle = new GUIStyle(GUI.skin.label);
        titleStyle.normal.textColor = new Color(0.3f, 0.8f, 1f);
        titleStyle.fontSize = 16;
        titleStyle.fontStyle = FontStyle.Bold;
        titleStyle.alignment = TextAnchor.MiddleCenter;
        titleStyle.padding = new RectOffset(5, 5, 10, 10);
        
        stylesInitialized = true;
    }

    void OnGUI()
    {
        InitializeStyles();
        
        if (simManager == null) return;
        
        DrawMainPanel();
        DrawInstructions();
    }

    void DrawMainPanel()
    {
        GUILayout.BeginArea(new Rect(20, 20, 320, 400));
        GUILayout.BeginVertical(boxStyle);
        
        // Titre
        GUILayout.Label("SIMULATION DE POUTRE", titleStyle);
        GUILayout.Space(10);
        
        // Bouton Start/Restart
        bool simStarted = GetSimulationState();
        
        if (!simStarted)
        {
            GUI.backgroundColor = new Color(0.2f, 0.8f, 0.2f);
            if (GUILayout.Button("▶ DÉMARRER LA SIMULATION", buttonStyle, GUILayout.Height(50)))
            {
                simManager.StartSimulation();
            }
            GUI.backgroundColor = Color.white;
        }
        else
        {
            GUILayout.Label("Simulation en cours...", labelStyle);
        }
        
        GUILayout.Space(10);
        
        // Bouton Reset
        GUI.backgroundColor = new Color(0.9f, 0.4f, 0.2f);
        if (GUILayout.Button("⟲ REDÉMARRER", buttonStyle, GUILayout.Height(40)))
        {
            simManager.RestartSimulation();
        }
        GUI.backgroundColor = Color.white;
        
        GUILayout.Space(20);
        
        // Paramètre d'élasticité
        GUILayout.Label("═══ PARAMÈTRES ═══", titleStyle);
        GUILayout.Space(10);
        
        GUILayout.Label($"Élasticité (α): {simManager.elasticity:F2}", labelStyle);
        GUILayout.Label("Plus α est élevé, plus la poutre est flexible", new GUIStyle(labelStyle) 
        { 
            fontSize = 10, 
            fontStyle = FontStyle.Italic,
            normal = { textColor = new Color(0.8f, 0.8f, 0.8f) }
        });
        
        float newElasticity = GUILayout.HorizontalSlider(simManager.elasticity, 0f, 2f, GUILayout.Height(20));
        if (Mathf.Abs(newElasticity - simManager.elasticity) > 0.001f)
        {
            simManager.elasticity = newElasticity;
        }
        
        GUILayout.Space(15);
        
        // Statistiques
        GUILayout.Label("═══ STATISTIQUES ═══", titleStyle);
        GUILayout.Space(5);
        
        // Info sur la vitesse du cube
        var cubeBody = FindCubeBody();
        if (cubeBody != null)
        {
            float speed = cubeBody.velocity.magnitude;
            GUILayout.Label($"Vitesse du cube: {speed:F1} m/s", labelStyle);
            
            float energy = 0.5f * cubeBody.mass * speed * speed;
            GUILayout.Label($"Énergie cinétique: {energy:F0} J", labelStyle);
        }
        
        GUILayout.Space(5);
        
        // Info sur les contraintes
        DrawJointInfo();
        
        GUILayout.EndVertical();
        GUILayout.EndArea();
    }

    void DrawInstructions()
    {
        GUILayout.BeginArea(new Rect(Screen.width - 270, 20, 250, 250));
        GUILayout.BeginVertical(boxStyle);
        
        GUILayout.Label("⌨ CONTRÔLES", titleStyle);
        GUILayout.Space(10);
        
        DrawControl("ESPACE", "Démarrer");
        DrawControl("R", "Redémarrer");
        DrawControl("Q / E", "Ajuster élasticité");
        DrawControl("ESC", "Quitter");
        
        GUILayout.Space(10);
        GUILayout.Label("═══════════════", new GUIStyle(labelStyle) { alignment = TextAnchor.MiddleCenter });
        GUILayout.Space(5);
        
        GUILayout.Label("💡 Le cube vert va percuter\nla jonction des deux segments\nde la poutre.", 
            new GUIStyle(labelStyle) 
            { 
                fontSize = 11,
                wordWrap = true,
                alignment = TextAnchor.MiddleCenter,
                normal = { textColor = new Color(1f, 1f, 0.6f) }
            });
        
        GUILayout.EndVertical();
        GUILayout.EndArea();
    }

    void DrawControl(string key, string action)
    {
        GUILayout.BeginHorizontal();
        
        GUILayout.Label(key, new GUIStyle(labelStyle) 
        { 
            fontSize = 11,
            fontStyle = FontStyle.Bold,
            normal = { textColor = new Color(0.3f, 0.8f, 1f) }
        });
        
        GUILayout.FlexibleSpace();
        
        GUILayout.Label(action, new GUIStyle(labelStyle) { fontSize = 11 });
        
        GUILayout.EndHorizontal();
    }

    void DrawJointInfo()
    {
        // Accéder aux informations des contraintes via reflection ou propriétés publiques
        var joints = GetJointsInfo();
        
        if (joints.centerJoint != null)
        {
            string status = joints.centerJointBroken ? "❌ CASSÉE" : 
                $"✓ Tension: {joints.centerJointTension:F0}N";
            
            Color color = joints.centerJointBroken ? Color.red : 
                Color.Lerp(Color.green, Color.yellow, joints.centerJointTension / 300f);
            
            GUILayout.Label($"Contrainte centrale: {status}", 
                new GUIStyle(labelStyle) { normal = { textColor = color } });
        }
        
        int brokenCount = joints.brokenPostJoints;
        int totalPostJoints = joints.totalPostJoints;
        
        if (brokenCount > 0)
        {
            GUILayout.Label($"Supports: {brokenCount}/{totalPostJoints} cassés", 
                new GUIStyle(labelStyle) { normal = { textColor = Color.red } });
        }
        else
        {
            GUILayout.Label($"Supports: {totalPostJoints}/{totalPostJoints} actifs", 
                new GUIStyle(labelStyle) { normal = { textColor = Color.green } });
        }
    }

    bool GetSimulationState()
    {
        // Utilise reflection pour accéder à simulationStarted (ou rendre le champ public)
        var field = typeof(BeamSimulationManager).GetField("simulationStarted", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        
        if (field != null)
        {
            return (bool)field.GetValue(simManager);
        }
        
        return false;
    }

    CustomRigidBody FindCubeBody()
    {
        var field = typeof(BeamSimulationManager).GetField("cubeBody", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        
        if (field != null)
        {
            return (CustomRigidBody)field.GetValue(simManager);
        }
        
        return null;
    }

    (CustomJoint centerJoint, bool centerJointBroken, float centerJointTension, 
     int brokenPostJoints, int totalPostJoints) GetJointsInfo()
    {
        var centerJointField = typeof(BeamSimulationManager).GetField("centerJoint", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        
        var postJointsField = typeof(BeamSimulationManager).GetField("postJoints", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        
        CustomJoint centerJoint = null;
        bool centerBroken = false;
        float centerTension = 0f;
        int brokenPost = 0;
        int totalPost = 0;
        
        if (centerJointField != null)
        {
            centerJoint = (CustomJoint)centerJointField.GetValue(simManager);
            if (centerJoint != null)
            {
                centerBroken = centerJoint.isBroken;
                centerTension = centerJoint.GetTension();
            }
        }
        
        if (postJointsField != null)
        {
            var postJoints = (System.Collections.Generic.List<CustomJoint>)postJointsField.GetValue(simManager);
            if (postJoints != null)
            {
                totalPost = postJoints.Count;
                foreach (var joint in postJoints)
                {
                    if (joint.isBroken) brokenPost++;
                }
            }
        }
        
        return (centerJoint, centerBroken, centerTension, brokenPost, totalPost);
    }

    Texture2D MakeTex(int width, int height, Color col)
    {
        Color[] pix = new Color[width * height];
        for (int i = 0; i < pix.Length; i++)
        {
            pix[i] = col;
        }
        
        Texture2D result = new Texture2D(width, height);
        result.SetPixels(pix);
        result.Apply();
        
        return result;
    }
}