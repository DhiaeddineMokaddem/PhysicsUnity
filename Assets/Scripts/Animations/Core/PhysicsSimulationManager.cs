using UnityEngine;
using System.Collections.Generic;

namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Centralized physics simulation manager
    /// Manages all physics bodies and handles the simulation loop
    /// </summary>
    public class PhysicsSimulationManager : MonoBehaviour
    {
        #region Singleton
        private static PhysicsSimulationManager instance;
        public static PhysicsSimulationManager Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = FindObjectOfType<PhysicsSimulationManager>();
                    if (instance == null)
                    {
                        GameObject go = new GameObject("PhysicsSimulationManager");
                        instance = go.AddComponent<PhysicsSimulationManager>();
                    }
                }
                return instance;
            }
        }
        #endregion

        #region Configuration
        [Header("Simulation Settings")]
        [SerializeField] private float timeStep = PhysicsConstants.DEFAULT_TIME_STEP;
        [SerializeField] private int substeps = PhysicsConstants.DEFAULT_SUBSTEPS;
        [SerializeField] private bool useFixedTimestep = true;
        
        [Header("Collision Settings")]
        [SerializeField] private int solverIterations = PhysicsConstants.DEFAULT_SOLVER_ITERATIONS;
        [SerializeField] private bool enableCollisions = true;
        
        [Header("Debug")]
        [SerializeField] private bool drawDebugInfo = false;
        [SerializeField] private bool showPerformanceStats = false;
        
        public float TimeStep => timeStep;
        public int Substeps => substeps;
        public int SolverIterations => solverIterations;
        #endregion

        #region Private Fields
        private List<PhysicsBodyBase> bodies = new List<PhysicsBodyBase>();
        private float accumulatedTime = 0f;
        
        // Performance tracking
        private int physicsStepsThisFrame = 0;
        private float lastFrameTime = 0f;
        #endregion

        #region Unity Lifecycle
        void Awake()
        {
            if (instance != null && instance != this)
            {
                Destroy(gameObject);
                return;
            }
            instance = this;
            DontDestroyOnLoad(gameObject);
        }

        void FixedUpdate()
        {
            if (useFixedTimestep)
            {
                SimulatePhysics(Time.fixedDeltaTime);
            }
        }

        void Update()
        {
            if (!useFixedTimestep)
            {
                accumulatedTime += Time.deltaTime;
                physicsStepsThisFrame = 0;
                
                while (accumulatedTime >= timeStep)
                {
                    SimulatePhysics(timeStep);
                    accumulatedTime -= timeStep;
                    physicsStepsThisFrame++;
                    
                    // Prevent spiral of death
                    if (physicsStepsThisFrame > 10)
                    {
                        accumulatedTime = 0f;
                        break;
                    }
                }
            }
        }

        void OnDrawGizmos()
        {
            if (drawDebugInfo)
            {
                DrawDebugVisualization();
            }
        }
        #endregion

        #region Body Management
        /// <summary>
        /// Register a physics body with the simulation
        /// </summary>
        public void RegisterBody(PhysicsBodyBase body)
        {
            if (!bodies.Contains(body))
            {
                bodies.Add(body);
            }
        }

        /// <summary>
        /// Unregister a physics body from the simulation
        /// </summary>
        public void UnregisterBody(PhysicsBodyBase body)
        {
            bodies.Remove(body);
        }

        /// <summary>
        /// Get all registered bodies
        /// </summary>
        public IReadOnlyList<PhysicsBodyBase> GetAllBodies()
        {
            return bodies.AsReadOnly();
        }

        /// <summary>
        /// Clear all registered bodies
        /// </summary>
        public void ClearAllBodies()
        {
            bodies.Clear();
        }
        #endregion

        #region Simulation
        /// <summary>
        /// Main physics simulation step
        /// </summary>
        private void SimulatePhysics(float dt)
        {
            float substepDt = dt / substeps;
            
            for (int i = 0; i < substeps; i++)
            {
                // Integrate all bodies
                foreach (var body in bodies)
                {
                    if (body != null && body.enabled)
                    {
                        body.IntegratePhysics(substepDt);
                    }
                }
                
                // Handle collisions if enabled
                if (enableCollisions)
                {
                    DetectAndResolveCollisions();
                }
            }
        }

        /// <summary>
        /// Detect and resolve collisions between all bodies
        /// </summary>
        private void DetectAndResolveCollisions()
        {
            // Broad phase: simple n^2 check (can be optimized with spatial partitioning)
            for (int i = 0; i < bodies.Count; i++)
            {
                for (int j = i + 1; j < bodies.Count; j++)
                {
                    if (bodies[i] != null && bodies[j] != null)
                    {
                        // Collision detection and response would go here
                        // This is a placeholder for the collision system
                    }
                }
            }
        }
        #endregion

        #region Debug Visualization
        private void DrawDebugVisualization()
        {
            if (showPerformanceStats)
            {
                #if UNITY_EDITOR
                Vector3 labelPos = Camera.main != null 
                    ? Camera.main.transform.position + Camera.main.transform.forward * 5f 
                    : Vector3.zero;
                    
                string stats = $"Physics Bodies: {bodies.Count}\n" +
                             $"Steps/Frame: {physicsStepsThisFrame}\n" +
                             $"Time Step: {timeStep:F4}s";
                
                DebugDrawUtils.DrawLabel(labelPos, stats, Color.white);
                #endif
            }
        }
        #endregion

        #region Public Utility Methods
        /// <summary>
        /// Pause physics simulation
        /// </summary>
        public void PauseSimulation()
        {
            enabled = false;
        }

        /// <summary>
        /// Resume physics simulation
        /// </summary>
        public void ResumeSimulation()
        {
            enabled = true;
            accumulatedTime = 0f;
        }

        /// <summary>
        /// Reset the entire simulation
        /// </summary>
        public void ResetSimulation()
        {
            accumulatedTime = 0f;
            physicsStepsThisFrame = 0;
        }
        #endregion
    }
}

