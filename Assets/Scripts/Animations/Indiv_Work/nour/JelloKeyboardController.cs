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

    void Awake()
    {
        if (jello == null)
            jello = GetComponent<ControllableSoftJello>();
    }

    void Update()
    {
        if (jello == null) return;

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
