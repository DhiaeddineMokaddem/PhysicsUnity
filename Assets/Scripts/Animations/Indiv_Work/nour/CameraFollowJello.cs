using UnityEngine;

/// <summary>
/// Camera follow script that tracks the jello's actual physics center of mass
/// Follows the moving mass points, not the static transform
/// Pure math implementation - no Unity physics
/// </summary>
public class CameraFollowJello : MonoBehaviour
{
    [Header("Follow Target")]
    public ControllableSoftJello jello;
    
    [Header("Camera Settings")]
    public Vector3 offset = new Vector3(0f, 5f, -10f);
    
    [Range(0.01f, 1f)]
    public float smoothSpeed = 0.125f;
    
    [Header("Optional Look At")]
    public bool lookAtTarget = true;
    
    public float lookAtHeightOffset;
    
    private bool hasWarnedAboutTarget;
    
    void Start()
    {
        // Try to auto-find the jello if not assigned
        if (jello == null)
        {
            jello = FindAnyObjectByType<ControllableSoftJello>();
            if (jello != null)
            {
                Debug.Log("[CameraFollowJello] Auto-found jello: " + jello.name);
            }
            else
            {
                Debug.LogWarning("[CameraFollowJello] No jello assigned and couldn't find ControllableSoftJello in scene!");
            }
        }
    }
    
    void LateUpdate()
    {
        if (jello == null)
        {
            if (!hasWarnedAboutTarget)
            {
                Debug.LogError("[CameraFollowJello] Camera cannot follow - no jello assigned! Please assign ControllableSoftJello to the 'Jello' field.");
                hasWarnedAboutTarget = true;
            }
            return;
        }
        
        // Calculate center of mass from all physics points
        Vector3 centerOfMass = CalculateCenterOfMass();
        
        // Calculate desired position (center of mass + offset)
        Vector3 targetPosition = centerOfMass + offset;
        
        // Smoothly move camera toward desired position using custom damping
        // This is manual interpolation, not Unity physics
        Vector3 currentPosition = transform.position;
        Vector3 delta = targetPosition - currentPosition;
        Vector3 smoothedDelta = delta * smoothSpeed;
        
        // Apply the movement
        transform.position = currentPosition + smoothedDelta;
        
        // Optional: Make camera look at center of mass
        if (lookAtTarget)
        {
            Vector3 lookAtPoint = centerOfMass + Vector3.up * lookAtHeightOffset;
            Vector3 direction = lookAtPoint - transform.position;
            
            // Manual rotation calculation (not using Unity physics)
            if (direction.sqrMagnitude > 0.001f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(direction);
                transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, smoothSpeed);
            }
        }
    }
    
    /// <summary>
    /// Calculate the center of mass of all physics points in the jello
    /// This is the actual position of the jello, not the static transform.position
    /// </summary>
    private Vector3 CalculateCenterOfMass()
    {
        Vector3 sum = Vector3.zero;
        int count = 0;
        
        // Get the 3D array of physics points
        int gridSize = jello.gridSize;
        
        // Sum all point positions
        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                for (int z = 0; z < gridSize; z++)
                {
                    // Access the physics point directly
                    var point = jello.GetPoint(x, y, z);
                    if (point != null)
                    {
                        sum += point.position;
                        count++;
                    }
                }
            }
        }
        
        // Return average position (center of mass)
        return count > 0 ? sum / count : jello.transform.position;
    }
}

