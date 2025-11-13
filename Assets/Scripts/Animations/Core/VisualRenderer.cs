using UnityEngine;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Manual mesh vertex renderer that transforms vertices directly without using Unity's Transform system.
    /// This is proper for physics subject projects where transform.position/rotation are not permitted.
    /// All transformations are done by manually updating mesh vertices using ManualMatrix.
    /// </summary>
    [RequireComponent(typeof(MeshFilter))]
    public class VisualRenderer : MonoBehaviour
    {
        #region Private Fields
        private Mesh mesh;
        private Vector3[] originalVertices;
        private Vector3[] transformedVertices;
        private Vector3[] originalNormals;
        private Vector3[] transformedNormals;
        
        private Vector3 currentPosition = Vector3.zero;
        private Quaternion currentRotation = Quaternion.identity;
        private Vector3 currentScale = Vector3.one;
        
        private bool isDirty = true;
        #endregion

        #region Unity Lifecycle
        void Awake()
        {
            InitializeMesh();
        }

        void LateUpdate()
        {
            if (isDirty)
            {
                UpdateMeshVertices();
                isDirty = false;
            }
        }
        #endregion

        #region Initialization
        private void InitializeMesh()
        {
            MeshFilter meshFilter = GetComponent<MeshFilter>();
            if (meshFilter == null)
            {
                Debug.LogError("VisualRenderer requires a MeshFilter component!");
                return;
            }

            // Create a copy of the mesh so we don't modify the original asset
            if (meshFilter.sharedMesh != null)
            {
                mesh = Instantiate(meshFilter.sharedMesh);
                meshFilter.mesh = mesh;
                
                // Store original vertices and normals
                originalVertices = mesh.vertices;
                transformedVertices = new Vector3[originalVertices.Length];
                
                originalNormals = mesh.normals;
                transformedNormals = new Vector3[originalNormals.Length];
                
                // Initialize position/rotation from GameObject's initial transform
                currentPosition = transform.position;
                currentRotation = transform.rotation;
                currentScale = transform.localScale;
                
                // Reset transform to origin since we'll handle everything manually
                transform.position = Vector3.zero;
                transform.rotation = Quaternion.identity;
                transform.localScale = Vector3.one;
                
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
            currentPosition = position;
            isDirty = true;
        }

        /// <summary>
        /// Update visual rotation by manually transforming mesh vertices
        /// </summary>
        public void UpdateRotation(Quaternion rotation)
        {
            currentRotation = rotation;
            isDirty = true;
        }

        /// <summary>
        /// Update visual scale by manually scaling mesh vertices
        /// </summary>
        public void UpdateScale(Vector3 scale)
        {
            currentScale = scale;
            isDirty = true;
        }

        /// <summary>
        /// Update all transform properties at once
        /// </summary>
        public void UpdateTransform(Vector3 position, Quaternion rotation, Vector3 scale)
        {
            currentPosition = position;
            currentRotation = rotation;
            currentScale = scale;
            isDirty = true;
        }

        /// <summary>
        /// Update position and rotation
        /// </summary>
        public void UpdateTransform(Vector3 position, Quaternion rotation)
        {
            currentPosition = position;
            currentRotation = rotation;
            isDirty = true;
        }
        #endregion

        #region Getters
        /// <summary>
        /// Get current visual position
        /// </summary>
        public Vector3 GetPosition()
        {
            return currentPosition;
        }

        /// <summary>
        /// Get current visual rotation
        /// </summary>
        public Quaternion GetRotation()
        {
            return currentRotation;
        }

        /// <summary>
        /// Get current visual scale
        /// </summary>
        public Vector3 GetScale()
        {
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
            if (mesh == null || originalVertices == null) return;

            // Build transformation matrix manually
            ManualMatrix matrix = ManualMatrix.TR(currentPosition, currentRotation);

            // Transform each vertex manually
            for (int i = 0; i < originalVertices.Length; i++)
            {
                // Apply scale first (local space)
                Vector3 scaledVertex = new Vector3(
                    originalVertices[i].x * currentScale.x,
                    originalVertices[i].y * currentScale.y,
                    originalVertices[i].z * currentScale.z
                );
                
                // Then apply rotation and translation using manual matrix
                transformedVertices[i] = matrix.MultiplyPoint(scaledVertex);
            }

            // Transform normals (rotation only, no translation)
            for (int i = 0; i < originalNormals.Length; i++)
            {
                transformedNormals[i] = matrix.MultiplyVector(originalNormals[i]).normalized;
            }

            // Update mesh with transformed vertices
            mesh.vertices = transformedVertices;
            mesh.normals = transformedNormals;
            mesh.RecalculateBounds();
        }

        /// <summary>
        /// Force immediate mesh update (useful for initialization)
        /// </summary>
        public void ForceUpdate()
        {
            UpdateMeshVertices();
            isDirty = false;
        }
        #endregion
    }
}

