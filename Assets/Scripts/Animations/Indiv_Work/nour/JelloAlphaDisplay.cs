using UnityEngine;
using TMPro;

/// <summary>
/// Displays the jello's alpha value in a TextMeshPro element
/// Shows current alpha, stiffness, restitution, and damping values
/// Pure display script - no physics calculations
/// </summary>
public class JelloAlphaDisplay : MonoBehaviour
{
    [Header("Target")]
    [Tooltip("The jello object to monitor")]
    public ControllableSoftJello jello;
    
    [Header("UI Elements")]
    [Tooltip("TextMeshPro component to display alpha value")]
    public TMP_Text displayText;
    
    [Header("Display Options")]
    [Tooltip("Show detailed info (stiffness, restitution, damping)")]
    public bool showDetailedInfo = true;
    
    [Tooltip("Number of decimal places to show")]
    [Range(0, 3)]
    public int decimalPlaces = 2;
    
    [Header("Text Formatting")]
    [Tooltip("Font size")]
    public int fontSize = 24;
    
    [Tooltip("Text color")]
    public Color textColor = Color.white;
    
    [Tooltip("Enable text shadow for better visibility")]
    public bool enableShadow = true;
    
    void Start()
    {
        // Try to find jello if not assigned
        if (jello == null)
        {
            jello = FindAnyObjectByType<ControllableSoftJello>();
        }
        
        // Try to get TMP_Text component if not assigned
        if (displayText == null)
        {
            displayText = GetComponent<TMP_Text>();
        }
        
        // Configure text appearance
        if (displayText != null)
        {
            displayText.fontSize = fontSize;
            displayText.color = textColor;
            
            // TextMeshPro doesn't need shadow component added - it has built-in outline/shadow
            // You can configure shadow in the TMP component directly in the inspector
        }
    }
    
    void Update()
    {
        if (jello == null || displayText == null) return;
        
        // Format the display string
        string format = "F" + decimalPlaces.ToString();
        
        if (showDetailedInfo)
        {
            // Detailed display with all parameters
            displayText.text = string.Format(
                "Alpha: {0}\n" +
                "Stiffness: {1}\n" +
                "Restitution: {2}\n" +
                "Damping: {3}",
                jello.alpha.ToString(format),
                jello.stiffness.ToString(format),
                jello.restitution.ToString(format),
                GetCurrentDamping().ToString(format)
            );
        }
        else
        {
            // Simple display - just alpha
            displayText.text = "Alpha: " + jello.alpha.ToString(format);
        }
    }
    
    /// <summary>
    /// Get current damping value (mirrors the calculation in ControllableSoftJello)
    /// </summary>
    private float GetCurrentDamping()
    {
        if (jello == null) return 0f;
        
        // Same calculation as in ControllableSoftJello.GetCurrentDamping()
        return Mathf.Lerp(jello.minDamping, jello.maxDamping, jello.alpha);
    }
}

