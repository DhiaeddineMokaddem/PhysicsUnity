// Import Unity's core mesh and transform functionality
using UnityEngine;

// Namespace for core physics simulation utilities
namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Manual mesh vertex renderer that transforms vertices directly without using Unity's Transform system.
    /// This is proper for physics subject projects where transform.position/rotation are not permitted.
    /// All transformations are done by manually updating mesh vertices using ManualMatrix.
    /// </summary>
    [RequireComponent(typeof(MeshFilter))] // Ensures this GameObject has a MeshFilter component
    public class VisualRenderer : MonoBehaviour
    {
        #region Private Fields
        // Reference to the mesh that will be modified each frame
        private Mesh mesh;
        // Array storing the original vertex positions (never modified)
        private Vector3[] originalVertices;
        // Array storing the transformed vertex positions (updated each frame)
        private Vector3[] transformedVertices;
        // Array storing the original normal vectors (never modified)
        private Vector3[] originalNormals;
        // Array storing the transformed normal vectors (updated each frame)
        private Vector3[] transformedNormals;
        
        // Current position in world space (manual physics position)
        private Vector3 currentPosition = Vector3.zero;
        // Current rotation as quaternion (manual physics rotation)
        private Quaternion currentRotation = Quaternion.identity;
        // Current scale vector (manual physics scale)
        private Vector3 currentScale = Vector3.one;
        
        // Flag indicating if mesh vertices need to be recalculated
        private bool isDirty = true;
        #endregion

        #region Unity Lifecycle
        // Called when the script instance is being loaded
        void Awake()
        {
            // Set up the mesh and copy original vertices for transformation
            InitializeMesh();
        }

        // Called after all Update functions (ensures physics has updated first)
        void LateUpdate()
        {
            // Only update mesh if something has changed
            if (isDirty)
            {
                // Transform all vertices using current position/rotation/scale
                UpdateMeshVertices();
                // Mark as clean until next change
                isDirty = false;
            }
        }
        #endregion

        #region Initialization
        // Initialize the mesh system and store original vertex data
        private void InitializeMesh()
        {
            // Get the MeshFilter component that holds the mesh
            MeshFilter meshFilter = GetComponent<MeshFilter>();
            // Verify MeshFilter exists
            if (meshFilter == null)
            {
                Debug.LogError("VisualRenderer requires a MeshFilter component!");
                return;
            }

            // Check if mesh exists
            if (meshFilter.sharedMesh != null)
            {
                // Create a copy of the mesh so we don't modify the original asset
                mesh = Instantiate(meshFilter.sharedMesh);
                // Assign the copy to the MeshFilter (replaces shared mesh)
                meshFilter.mesh = mesh;
                
                // Store original vertices from the mesh (will never be modified)
                originalVertices = mesh.vertices;
                // Create array to hold transformed vertices (same size as original)
                transformedVertices = new Vector3[originalVertices.Length];
                
                // Store original normals from the mesh (will never be modified)
                originalNormals = mesh.normals;
                // Create array to hold transformed normals (same size as original)
                transformedNormals = new Vector3[originalNormals.Length];
                
                // Read initial position from GameObject's transform (for initialization only)
                currentPosition = transform.position;
                // Read initial rotation from GameObject's transform (for initialization only)
                currentRotation = transform.rotation;
                // Read initial scale from GameObject's transform (for initialization only)
                currentScale = transform.localScale;
                
                // Reset GameObject transform to origin (we handle everything manually now)
                transform.position = Vector3.zero;
                // Reset GameObject rotation to identity (no rotation)
                transform.rotation = Quaternion.identity;
                // Reset GameObject scale to one (no scaling)
                transform.localScale = Vector3.one;
                
                // Perform initial vertex transformation with stored position/rotation
                UpdateMeshVertices();
            }
            else
            {
                Debug.LogError("MeshFilter has no mesh assigned!");
            }
        }
        #endregion

        #region Manual Transform Updates
        /// <summary>
        /// Update visual position by manually transforming mesh vertices
        /// </summary>
        public void UpdatePosition(Vector3 position)
        {
            // Store the new position
            currentPosition = position;
            // Mark mesh as needing update
            isDirty = true;
        }

        /// <summary>
        /// Update visual rotation by manually transforming mesh vertices
        /// </summary>
        public void UpdateRotation(Quaternion rotation)
        {
            // Store the new rotation
            currentRotation = rotation;
            // Mark mesh as needing update
            isDirty = true;
        }

        /// <summary>
        /// Update visual scale by manually scaling mesh vertices
        /// </summary>
        public void UpdateScale(Vector3 scale)
        {
            // Store the new scale
            currentScale = scale;
            // Mark mesh as needing update
            isDirty = true;
        }

        /// <summary>
        /// Update all transform properties at once
        /// </summary>
        public void UpdateTransform(Vector3 position, Quaternion rotation, Vector3 scale)
        {
            // Store the new position
            currentPosition = position;
            // Store the new rotation
            currentRotation = rotation;
            // Store the new scale
            currentScale = scale;
            // Mark mesh as needing update
            isDirty = true;
        }

        /// <summary>
        /// Update position and rotation (keeps existing scale)
        /// </summary>
        public void UpdateTransform(Vector3 position, Quaternion rotation)
        {
            // Store the new position
            currentPosition = position;
            // Store the new rotation
            currentRotation = rotation;
            // Mark mesh as needing update
            isDirty = true;
        }
        #endregion

        #region Getters
        /// <summary>
        /// Get current visual position
        /// </summary>
        public Vector3 GetPosition()
        {
            // Return the stored position
            return currentPosition;
        }

        /// <summary>
        /// Get current visual rotation
        /// </summary>
        public Quaternion GetRotation()
        {
            // Return the stored rotation
            return currentRotation;
        }

        /// <summary>
        /// Get current visual scale
        /// </summary>
        public Vector3 GetScale()
        {
            // Return the stored scale
            return currentScale;
        }
        #endregion

        #region Mesh Transformation
        /// <summary>
        /// Manually transform all mesh vertices using ManualMatrix
        /// NO UNITY TRANSFORM USED - Pure mathematical vertex transformation
        /// </summary>
        private void UpdateMeshVertices()
        {
            // Safety check: ensure mesh and vertices exist
            if (mesh == null || originalVertices == null) return;

            // Build transformation matrix manually from current position and rotation
            ManualMatrix matrix = ManualMatrix.TR(currentPosition, currentRotation);

            // Transform each vertex manually using the matrix
            for (int i = 0; i < originalVertices.Length; i++)
            {
                // First apply scale to the original vertex (component-wise multiplication)
                Vector3 scaledVertex = new Vector3(
                    originalVertices[i].x * currentScale.x, // Scale X component
                    originalVertices[i].y * currentScale.y, // Scale Y component
                    originalVertices[i].z * currentScale.z  // Scale Z component
                );
                
                // Then apply rotation and translation using manual matrix multiplication
                transformedVertices[i] = matrix.MultiplyPoint(scaledVertex);
            }

            // Transform normals (rotation only, no translation or scale)
            for (int i = 0; i < originalNormals.Length; i++)
            {
                // Apply rotation to normal using manual matrix multiplication
                // Normalize to ensure normal remains unit length after transformation
                transformedNormals[i] = matrix.MultiplyVector(originalNormals[i]).normalized;
            }

            // Update mesh with transformed vertices (this is what Unity renders)
            mesh.vertices = transformedVertices;
            // Update mesh with transformed normals (for lighting calculations)
            mesh.normals = transformedNormals;
            // Recalculate bounding box for culling and collision detection
            mesh.RecalculateBounds();
        }

        /// <summary>
        /// Force immediate mesh update (useful for initialization)
        /// </summary>
        public void ForceUpdate()
        {
            // Transform all vertices immediately (don't wait for LateUpdate)
            UpdateMeshVertices();
            // Mark as clean since we just updated
            isDirty = false;
        }
        #endregion
    }
}

