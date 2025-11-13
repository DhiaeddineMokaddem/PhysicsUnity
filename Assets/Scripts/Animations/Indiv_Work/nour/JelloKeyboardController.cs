using UnityEngine;
using PhysicsSimulation.Core; // for MathUtils

[DisallowMultipleComponent]
public class JelloKeyboardController : MonoBehaviour
{
    [Tooltip("Target jello to control. If null, tries to find on this GameObject.")]
    public ControllableSoftJello jello;

    [Header("Key Bindings (WASD)")]
    public KeyCode forwardKey = KeyCode.W; // forward
    public KeyCode backKey = KeyCode.S;    // backward
    public KeyCode leftKey = KeyCode.A;    // left
    public KeyCode rightKey = KeyCode.D;   // right
    public KeyCode jumpKey = KeyCode.Space;
    
    [Header("Alpha Control (Optional)")]
    [Tooltip("Enable alpha control with keys (Q/E to decrease/increase)")]
    public bool enableAlphaControl = true;
    public KeyCode decreaseAlphaKey = KeyCode.Q; // decrease alpha (more stiff/bouncy)
    public KeyCode increaseAlphaKey = KeyCode.E; // increase alpha (more soft/damped)
    [Tooltip("How much to change alpha per second when key is held")]
    public float alphaChangeRate = 0.5f;
    public float minAlpha = 0.4f;

    void Awake()
    {
        if (jello == null)
            jello = GetComponent<ControllableSoftJello>();
    }

    void Update()
    {
        if (jello == null) return;

        // Alpha control (if enabled)
        if (enableAlphaControl)
        {
            float alphaDelta = 0f;
            if (Input.GetKey(decreaseAlphaKey)) alphaDelta -= alphaChangeRate * Time.deltaTime;
            if (Input.GetKey(increaseAlphaKey)) alphaDelta += alphaChangeRate * Time.deltaTime;
            
            if (alphaDelta != 0f)
            {
                jello.alpha = Mathf.Clamp(jello.alpha + alphaDelta, minAlpha, 1f);
            }
        }

        // Build movement direction from keys (world XZ plane)
        Vector3 dir = Vector3.zero;
        if (Input.GetKey(leftKey))  dir.x -= 1f;
        if (Input.GetKey(rightKey)) dir.x += 1f;
        if (Input.GetKey(forwardKey)) dir.z += 1f;
        if (Input.GetKey(backKey))    dir.z -= 1f;

        // Normalize safely using Core
        dir = MathUtils.SafeNormalize(dir);

        // Set jello input direction (x,z) each frame
        var inDir = jello.inputDirection;
        inDir.x = dir.x;
        inDir.z = dir.z;

        // Jump (consumed by jello in FixedUpdate when grounded)
        if (Input.GetKeyDown(jumpKey))
            inDir.y = 1f;

        jello.inputDirection = inDir;
    }
}
