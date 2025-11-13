using UnityEngine;
using System.Collections.Generic;
using PhysicsSimulation.Core;

/// <summary>
/// ===================================================================================
/// CONTROLLABLE SOFT JELLO - DETAILED EXPLANATION
/// ===================================================================================
/// 
/// This class implements a soft-body physics simulation for a deformable "jello" cube
/// using PURE MATH (no Unity physics engine - Rigidbody, Collider, etc).
/// 
/// CORE CONCEPTS:
/// --------------
/// 1. MASS-SPRING SYSTEM: The jello is made of a 3D grid of mass points connected by springs
/// 2. VERLET INTEGRATION: Uses position and previous position to calculate velocity implicitly
/// 3. POSITION-BASED DYNAMICS: Springs apply position corrections instead of forces
/// 4. COLLISION DETECTION: Sphere vs OBB (oriented bounding box) math using TransformUtils
/// 5. MESH DEFORMATION: Visual mesh is deformed based on corner point positions
/// 
/// WHY THIS APPROACH:
/// ------------------
/// - Full control over physics behavior (no Unity black box)
/// - Stable soft-body simulation (position-based constraints are more stable than force-based)
/// - Pure math allows understanding exactly what's happening
/// - Works with custom collision system (SimpleOBB + CollisionUtils)
/// 
/// ===================================================================================
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ControllableSoftJello : MonoBehaviour
{
    // ===================================================================================
    // SOFT BODY CONFIGURATION
    // ===================================================================================
    
    [Header("Soft Body Settings")]
    
    /// <summary>
    /// Alpha control parameter (0-1).
    /// Controls the overall stiffness and bounciness of the jello:
    /// - Lower alpha (closer to 0) = SOFT and BOUNCY
    /// - Higher alpha (closer to 1) = STIFF and NOT BOUNCY (hard cube-like)
    /// Note: Values below 0.5 may cause collision instability.
    /// </summary>
    [Range(0f, 1f)]
    [Tooltip("0 = soft & bouncy, 1 = stiff & not bouncy (recommend min 0.5)")]
    public float alpha = 0.5f;
    
    /// <summary>
    /// Number of mass points per axis (e.g., 4 = 4x4x4 = 64 points total).
    /// Higher = more detail but slower performance.
    /// </summary>
    public int gridSize = 4;
    
    /// <summary>
    /// Distance between neighboring mass points in the grid (meters).
    /// Smaller = tighter lattice, larger = looser lattice.
    /// </summary>
    public float cellSize = 0.3f;
    
    /// <summary>
    /// Mass of each individual point (kg).
    /// Higher = heavier, less responsive to forces.
    /// </summary>
    public float pointMass = 0.1f;
    
    /// <summary>
    /// Maximum spring stiffness coefficient (when alpha = 1).
    /// Higher = stiffer jello (more shape retention), lower = squishier.
    /// </summary>
    public float maxStiffness = 50f;
    
    /// <summary>
    /// Minimum spring stiffness coefficient (when alpha = 0.5, the baseline).
    /// Lower values = softer jello.
    /// Must be high enough to prevent permanent deformation.
    /// </summary>
    public float minStiffness = 20f;
    
    /// <summary>
    /// Velocity damping coefficient (scaled by alpha).
    /// Higher damping removes more energy each step.
    /// </summary>
    [Header("Damping")]
    [Tooltip("Damping at alpha=0.5 (baseline)")]
    public float minDamping = 1.5f;
    [Tooltip("Damping at alpha=1 (stiff & not bouncy)")]
    public float maxDamping = 4.0f;
    
    /// <summary>
    /// Gravity acceleration (m/s²). Negative pulls down.
    /// Standard Earth gravity is -9.81 m/s².
    /// </summary>
    public float gravity = -9.81f;
    
    /// <summary>
    /// Maximum bounciness on collision (when alpha = 0).
    /// The actual restitution is calculated as: maxRestitution * (1 - alpha)
    /// 0 = no bounce (perfectly inelastic), 1 = perfect bounce (perfectly elastic).
    /// </summary>
    public float maxRestitution = 0.3f;
    
    /// <summary>
    /// Minimum bounciness on collision (when alpha = 1).
    /// Ensures some minimum bounce even at maximum alpha.
    /// </summary>
    public float minRestitution = 0.05f;
    
    /// <summary>
    /// Surface friction on collision (0-1).
    /// 0 = no friction (slippery), 1 = full friction (sticky).
    /// </summary>
    public float friction = 0.9f;
    
    /// <summary>
    /// Number of constraint solver iterations per physics step.
    /// Higher = more accurate spring constraints but slower.
    /// </summary>
    public int solverIterations = 8;
    
    /// <summary>
    /// Collision detection sphere radius around each mass point (meters).
    /// Larger = earlier collision detection (more forgiving), smaller = tighter fit.
    /// Must be tuned based on moveForce to avoid tunneling through objects.
    /// </summary>
    public float pointRadius = 0.15f;

    // ===================================================================================
    // MOVEMENT CONFIGURATION
    // ===================================================================================
    
    [Header("Movement Settings")]
    
    /// <summary>
    /// Force applied for horizontal movement (Newtons).
    /// Lower values (1-5) prevent tunneling through colliders at high speeds.
    /// </summary>
    public float moveForce = 1f;
    
    /// <summary>
    /// Impulse force applied for jumping (Newtons).
    /// Only applied when grounded.
    /// </summary>
    public float jumpForce = 80f;
    
    /// <summary>
    /// External input vector set by controller (e.g., JelloKeyboardController).
    /// x,z = movement direction (horizontal), y = jump trigger (1 = jump, 0 = no jump).
    /// </summary>
    [Tooltip("External input direction to move the jello (x,z). Set from your own input system.")]
    public Vector3 inputDirection = Vector3.zero;

    // ===================================================================================
    // VISUAL SETTINGS
    // ===================================================================================
    
    [Header("Visuals")]
    
    /// <summary>
    /// Material to apply to the jello mesh (optional).
    /// </summary>
    public Material jelloMaterial;
    
    /// <summary>
    /// Show debug visualization (springs, mass points) in Scene view.
    /// </summary>
    public bool drawDebug = true;
    
    /// <summary>
    /// Show collision detection spheres around each mass point.
    /// Useful for debugging why collisions are/aren't happening.
    /// </summary>
    public bool drawCollisionSpheres = true;

    // ===================================================================================
    // INTERNAL STATE (PRIVATE)
    // ===================================================================================
    
    /// <summary>
    /// 3D array of mass points forming the soft-body lattice.
    /// Each point has: position, previousPosition (for Verlet), force accumulator, mass.
    /// From Core/Physics/Spring.cs (SoftBodyPoint class).
    /// </summary>
    private PhysicsSimulation.Core.Physics.SoftBodyPoint[,,] points;
    
    /// <summary>
    /// List of springs connecting neighboring mass points.
    /// Each spring tries to maintain its rest length via position correction.
    /// From Core/Physics/Spring.cs (Spring class).
    /// </summary>
    private List<PhysicsSimulation.Core.Physics.Spring> springs;
    
    /// <summary>
    /// The visual mesh that gets deformed based on mass point positions.
    /// </summary>
    private Mesh mesh;
    
    /// <summary>
    /// Original (undeformed) vertex positions of the cube mesh.
    /// Used as template for trilinear interpolation.
    /// </summary>
    private Vector3[] baseVertices;
    
    /// <summary>
    /// Current deformed vertex positions (calculated each frame in LateUpdate).
    /// </summary>
    private Vector3[] deformedVertices;
    
    /// <summary>
    /// Whether the jello is touching a surface with upward-pointing normal.
    /// Used to gate jumping (can only jump when grounded).
    /// </summary>
    private bool grounded = false;

    // Backward-compatible dynamic properties (previously public fields)
    // External scripts that used jello.stiffness or jello.restitution will still work.
    public float stiffness { get { return GetCurrentStiffness(); } }
    public float restitution { get { return GetCurrentRestitution(); } }
    
    /// <summary>
    /// Get a physics point at the specified grid position
    /// Used by camera to calculate center of mass
    /// </summary>
    public PhysicsSimulation.Core.Physics.SoftBodyPoint GetPoint(int x, int y, int z)
    {
        if (points == null) return null;
        if (x < 0 || x >= gridSize || y < 0 || y >= gridSize || z < 0 || z >= gridSize)
            return null;
        return points[x, y, z];
    }

    // ===================================================================================
    // ALPHA-BASED PROPERTY CALCULATIONS
    // ===================================================================================
    
    /// <summary>
    /// Calculate current stiffness based on alpha value.
    /// Lower alpha = lower stiffness (softer)
    /// Higher alpha = higher stiffness (stiffer)
    /// </summary>
    private float GetCurrentStiffness()
    {
        // Low alpha -> min stiffness (soft); High alpha -> max stiffness (stiff)
        return Mathf.Lerp(minStiffness, maxStiffness, alpha);
    }
    
    /// <summary>
    /// Calculate current restitution (bounciness) based on alpha value.
    /// Lower alpha = higher restitution (more bouncy)
    /// Higher alpha = lower restitution (less bouncy, more damped)
    /// </summary>
    private float GetCurrentRestitution()
    {
        return Mathf.Lerp(maxRestitution, minRestitution, alpha);
    }

    /// <summary>
    /// Optional: scale collision sphere radius slightly when very soft to stabilize
    /// contact handling (earlier contact prevents deep penetration).
    /// </summary>
    private float GetCurrentPointRadius()
    {
        // Up to +15% radius when alpha=0 (soft), no change when alpha=1 (stiff)
        return pointRadius * Mathf.Lerp(1.15f, 1f, alpha);
    }

    /// <summary>
    /// Calculate current damping based on alpha value.
    /// Lower alpha = lower damping (more bouncy)
    /// Higher alpha = higher damping (less bouncy, more energy loss)
    /// </summary>
    private float GetCurrentDamping()
    {
        return Mathf.Lerp(minDamping, maxDamping, alpha);
    }

    // ===================================================================================
    // INITIALIZATION (CALLED ONCE ON START)
    // ===================================================================================
    
    void Start()
    {
        // 1. Create the mass-spring lattice
        InitSoftBody();
        
        // 2. Create the visual mesh
        CreateMesh();
        
        // 3. Ensure all collision objects (SimpleOBB) have valid transform matrices
        //    SimpleOBB components are scattered in the scene (floor, walls, etc.)
        //    and maintain a static registry (SimpleOBB.All)
        var all = SimpleOBB.All;
        for (int i = 0; i < all.Count; i++)
        {
            if (all[i] != null) all[i].RecomputeMatrices();
        }
    }

    /// <summary>
    /// ===================================================================================
    /// INITIALIZE SOFT BODY LATTICE
    /// ===================================================================================
    /// 
    /// Creates a 3D grid of mass points and connects them with springs.
    /// 
    /// ALGORITHM:
    /// ----------
    /// 1. Calculate starting corner position (centered on transform.position)
    /// 2. Create gridSize^3 mass points in a uniform 3D grid
    /// 3. For each point, create springs to all neighbors within 1.5 * cellSize
    ///    (includes structural springs along axes + shear/diagonal springs)
    /// 4. Avoid duplicate springs by checking if spring already exists
    /// 
    /// SPRING TYPES CREATED:
    /// ---------------------
    /// - Structural: along X, Y, Z axes (6 per interior point)
    /// - Shear: face diagonals (12 per interior point)
    /// - Bend: space diagonals (8 per interior point)
    /// Total: ~26 springs per interior point
    /// 
    /// WHY THIS MATTERS:
    /// -----------------
    /// More springs = more rigid/stable jello, but slower performance.
    /// The 1.5 * cellSize cutoff ensures we get structural + some shear springs
    /// without creating too many long-distance springs.
    /// 
    /// ===================================================================================
    /// </summary>
    void InitSoftBody()
    {
        // Allocate 3D array for mass points
        points = new PhysicsSimulation.Core.Physics.SoftBodyPoint[gridSize, gridSize, gridSize];
        springs = new List<PhysicsSimulation.Core.Physics.Spring>();
        
        // Calculate starting position (bottom-left-back corner of the grid)
        // Subtract half the total extent on each axis to center the grid
        Vector3 start = transform.position - Vector3.one * (gridSize - 1) * cellSize * 0.5f;
        
        // PASS 1: Create all mass points in a uniform grid
        for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
                for (int z = 0; z < gridSize; z++)
                {
                    // Position = start corner + offset based on grid indices
                    Vector3 pos = start + new Vector3(x, y, z) * cellSize;
                    points[x, y, z] = new PhysicsSimulation.Core.Physics.SoftBodyPoint(pos, pointMass);
                }
        
        // PASS 2: Create springs between neighboring points
        for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
                for (int z = 0; z < gridSize; z++)
                {
                    var p = points[x, y, z];
                    
                    // Check all 26 neighbors (3x3x3 cube minus center)
                    for (int i = -1; i <= 1; i++)
                        for (int j = -1; j <= 1; j++)
                            for (int k = -1; k <= 1; k++)
                            {
                                // Skip self
                                if (i == 0 && j == 0 && k == 0) continue;
                                
                                // Calculate neighbor indices
                                int nx = x + i, ny = y + j, nz = z + k;
                                
                                // Skip out-of-bounds neighbors
                                if (nx < 0 || ny < 0 || nz < 0 || 
                                    nx >= gridSize || ny >= gridSize || nz >= gridSize)
                                    continue;
                                
                                var np = points[nx, ny, nz];
                                
                                // Only create spring if neighbor is close enough
                                // (filters out some long diagonal springs)
                                float dist = MathUtils.Distance(p.position, np.position);
                                if (dist <= cellSize * 1.5f && !SpringExists(p, np))
                                {
                                    // Create spring with current distance as rest length
                                    // Use dynamic stiffness based on alpha
                                    springs.Add(new PhysicsSimulation.Core.Physics.Spring(p, np, GetCurrentStiffness()));
                                }
                            }
                }
    }

    /// <summary>
    /// Check if a spring already exists between two points (to avoid duplicates).
    /// Springs are bidirectional, so check both (a,b) and (b,a).
    /// </summary>
    bool SpringExists(PhysicsSimulation.Core.Physics.SoftBodyPoint a, PhysicsSimulation.Core.Physics.SoftBodyPoint b)
    {
        foreach (var s in springs)
            if ((s.a == a && s.b == b) || (s.a == b && s.b == a))
                return true;
        return false;
    }

    /// <summary>
    /// ===================================================================================
    /// CREATE VISUAL MESH
    /// ===================================================================================
    /// 
    /// Creates a cube mesh that will be deformed based on mass point positions.
    /// 
    /// KEY STEPS:
    /// ----------
    /// 1. Create new dynamic mesh (tells Unity it will change every frame)
    /// 2. Use MeshUtils to generate a basic cube mesh (avoids manual vertex/triangle creation)
    /// 3. Copy vertices and triangles to our mesh
    /// 4. Mark as dynamic and recalculate bounds (prevents culling bugs)
    /// 5. Store base vertices for later deformation
    /// 
    /// WHY MarkDynamic():
    /// ------------------
    /// Tells Unity this mesh will change frequently, so it can optimize
    /// GPU memory management (avoids re-uploading static mesh data).
    /// 
    /// WHY RecalculateBounds():
    /// -------------------------
    /// The mesh bounds determine frustum culling (when camera can't see it).
    /// Since we deform the mesh, bounds change every frame.
    /// Without this, the mesh disappears when camera gets close because
    /// Unity thinks it's outside view frustum (uses stale bounds).
    /// 
    /// ===================================================================================
    /// </summary>
    void CreateMesh()
    {
        // Create new mesh and mark it as dynamic (will be modified every frame)
        mesh = new Mesh();
        mesh.MarkDynamic();
        
        // Assign to MeshFilter component (required for rendering)
        GetComponent<MeshFilter>().mesh = mesh;
        
        // Apply material if provided
        if (jelloMaterial)
            GetComponent<MeshRenderer>().material = jelloMaterial;

        // Use Core MeshUtils to create a basic cube mesh
        // Size 2 because vertices range from -1 to +1 (total size = 2)
        Mesh cube = MeshUtils.CreateCubeMesh(new Vector3(2f, 2f, 2f));
        mesh.vertices = cube.vertices;
        mesh.triangles = cube.triangles;
        mesh.RecalculateNormals();  // For proper lighting
        mesh.RecalculateBounds();    // Set initial bounds

        // Store original vertices for deformation interpolation
        baseVertices = mesh.vertices;
        deformedVertices = new Vector3[baseVertices.Length];
    }

    // ===================================================================================
    // PHYSICS UPDATE (FIXED TIMESTEP)
    // ===================================================================================
    
    /// <summary>
    /// ===================================================================================
    /// MAIN PHYSICS LOOP
    /// ===================================================================================
    /// 
    /// Called at a fixed interval (typically 50 FPS / 0.02s) for stable physics.
    /// 
    /// EXECUTION ORDER (CRITICAL!):
    /// ----------------------------
    /// 1. Accumulate forces (gravity + movement + jump)
    /// 2. Integrate motion (Verlet: update positions based on velocity/forces)
    /// 3. Detect and resolve collisions (BEFORE springs!)
    /// 4. Apply spring constraints (position corrections)
    /// 5. Reset jump input
    /// 
    /// WHY COLLISIONS BEFORE SPRINGS:
    /// -------------------------------
    /// If springs run first, they pull points back into collision.
    /// Then collision detection sees penetration but springs undo the correction.
    /// By doing collision FIRST, springs work with already-corrected positions.
    /// This was the main bug that caused jello to fall through floor!
    /// 
    /// VERLET INTEGRATION EXPLAINED:
    /// ------------------------------
    /// Instead of storing velocity explicitly, Verlet uses:
    ///   velocity = (position - previousPosition) / dt
    /// 
    /// Integration step:
    ///   newPosition = position + velocity * (1 - damping*dt) + (force/mass) * dt²
    ///   previousPosition = position
    ///   position = newPosition
    /// 
    /// Benefits:
    /// - Stable for stiff springs
    /// - No velocity accumulation errors
    /// - Simple to implement
    /// 
    /// ===================================================================================
    /// </summary>
    void FixedUpdate()
    {
        // Clamp timestep to avoid spiral of death (if frame rate drops below 50 FPS)
        float dt = Mathf.Min(Time.fixedDeltaTime, 0.02f);

        // ===============================================================================
        // STEP 0: UPDATE SPRING PROPERTIES BASED ON ALPHA
        // ===============================================================================
        
        // Update stiffness of all springs to reflect current alpha value
        // This allows real-time control of jello softness
        float currentStiffness = GetCurrentStiffness();
        foreach (var spring in springs)
        {
            spring.stiffness = currentStiffness;
        }

        // ===============================================================================
        // STEP 1: ACCUMULATE FORCES
        // ===============================================================================
        
        // GRAVITY: Apply to all mass points
        // Force = mass * acceleration (F = ma)
        foreach (var p in points)
            p.AddForce(Vector3.up * gravity * p.mass);

        // MOVEMENT: Apply horizontal force based on input direction
        Vector3 moveDir = new Vector3(inputDirection.x, 0f, inputDirection.z);
        
        // Only apply if input is non-zero (avoid division by zero in normalize)
        if (MathUtils.SqrMagnitude(moveDir) > PhysicsConstants.EPSILON_SMALL * PhysicsConstants.EPSILON_SMALL)
        {
            // Normalize direction to unit vector (using Core MathUtils, not Unity's .normalized)
            Vector3 dir = MathUtils.SafeNormalize(moveDir);
            Vector3 moveForceVec = dir * moveForce;
            
            // Apply same force to all points (moves entire jello as a unit)
            foreach (var p in points)
                p.AddForce(moveForceVec);
        }

        // JUMP: Apply upward impulse if grounded and jump requested
        if (grounded && inputDirection.y > 0)
        {
            foreach (var p in points)
                p.AddForce(Vector3.up * jumpForce);
        }

        // ===============================================================================
        // STEP 2: INTEGRATE MOTION (VERLET)
        // ===============================================================================
        
        // Update each point's position based on accumulated forces
        // This is done in the SoftBodyPoint.Integrate() method:
        //   velocity = position - previousPosition
        //   newPos = position + velocity*(1-damping*dt) + (force/mass)*dt²
        //   previousPosition = position
        //   position = newPos
        //   force = 0
        float currentDamping = GetCurrentDamping();
        foreach (var p in points)
            p.Integrate(dt, currentDamping);

        // ===============================================================================
        // STEP 3: COLLISION DETECTION AND RESPONSE (BEFORE SPRINGS!)
        // ===============================================================================
        
        // Reset grounded flag (will be set to true if any point touches ground)
        grounded = false;
        
        // Get all collision objects from static registry
        // SimpleOBB components register themselves on Enable and unregister on Disable
        var colliders = SimpleOBB.All;
        
        if (colliders != null && colliders.Count > 0)
        {
            // Check each mass point against all colliders
            for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
            for (int z = 0; z < gridSize; z++)
            {
                PhysicsSimulation.Core.Physics.SoftBodyPoint mp = points[x, y, z];
                float pr = GetCurrentPointRadius();
                
                // Test against each collider (floor, walls, obstacles, etc.)
                for (int ci = 0; ci < colliders.Count; ci++)
                {
                    var obb = colliders[ci];
                    if (obb == null) continue;
                    
                    // SPHERE VS OBB COLLISION TEST
                    // -----------------------------
                    // Treats mass point as a sphere with radius pointRadius
                    // OBB = Oriented Bounding Box (rotated box collider)
                    // 
                    // Algorithm (in CollisionUtils.CheckSphereOBB):
                    // 1. Transform sphere center to OBB local space
                    // 2. Clamp to box bounds to find closest point on box
                    // 3. Transform closest point back to world space
                    // 4. Check if distance from sphere to closest point <= radius
                    // 5. If collision, return contact normal and penetration depth
                    CollisionUtils.ContactInfo contact;
                    
                    if (CollisionUtils.CheckSphereOBB(
                        mp.position, pr,
                        obb.transform.position, obb.halfExtents, obb.transform.rotation,
                        out contact))
                    {
                        // COLLISION RESPONSE
                        // ------------------
                        
                        // 1. POSITION CORRECTION: Push point out of collision
                        //    Move along contact normal by penetration distance
                        mp.position += contact.normal * contact.penetration;
                        
                        // Separation bias (skin) to reduce immediate re-penetration when soft
                        float skin = Mathf.Lerp(0.005f, 0.001f, alpha);
                        mp.position += contact.normal * skin;
                        
                        // 2. VELOCITY REFLECTION: Bounce off surface
                        //    Calculate current velocity from Verlet state
                        Vector3 vel = (mp.position - mp.previousPosition) / dt;
                        
                        // Project velocity onto contact normal
                        float vn = MathUtils.Dot(vel, contact.normal);
                        
                        // Only apply response if moving into surface (vn < 0)
                        if (vn < 0f)
                        {
                            // Remove normal component and apply restitution (bounciness)
                            // vel = vel - vn * normal * (1 + restitution)
                            // Use dynamic restitution based on alpha
                            float currentRestitution = GetCurrentRestitution();
                            vel -= vn * contact.normal * (1f + currentRestitution);
                            
                            // Apply friction to tangential component
                            // Separate velocity into tangent and normal components
                            Vector3 vt = vel - MathUtils.Dot(vel, contact.normal) * contact.normal;
                            vel = vt * friction + MathUtils.Dot(vel, contact.normal) * contact.normal;
                            
                            // Clamp any residual inward normal velocity to zero to avoid tunneling
                            float vn2 = MathUtils.Dot(vel, contact.normal);
                            if (vn2 < 0f)
                                vel -= vn2 * contact.normal;
                        }
                        
                        // 3. UPDATE VERLET STATE: Set previousPosition to reflect new velocity
                        //    previousPosition = position - velocity * dt
                        mp.previousPosition = mp.position - vel * dt;
                        
                        // 4. CHECK IF GROUNDED: Only count as grounded if normal points up
                        //    (prevents "grounding" on walls/ceilings)
                        grounded = grounded || contact.normal.y > 0.3f;
                        
                        // Break after first collision per point to avoid over-correction
                        break;
                    }
                }
            }
        }

        // ===============================================================================
        // STEP 4: APPLY SPRING CONSTRAINTS (AFTER COLLISIONS!)
        // ===============================================================================
        
        // Run multiple solver iterations for accuracy
        // Keep baseline iterations at alpha=0; slightly fewer when very stiff to save work
        int iterations = Mathf.Max(1, Mathf.RoundToInt(Mathf.Lerp(solverIterations, Mathf.Ceil(solverIterations * 0.75f), alpha)));
        for (int i = 0; i < iterations; i++)
        {
            foreach (var s in springs)
                s.Apply();
        }

        // ===============================================================================
        // STEP 5: RESET JUMP INPUT (SINGLE-PRESS BEHAVIOR)
        // ===============================================================================
        
        // Consume jump input so it only applies for one frame
        // JelloKeyboardController sets inputDirection.y = 1 on key down
        // We process it once here, then reset to 0
        inputDirection.y = 0f;
    }

    // ===================================================================================
    // VISUAL UPDATE (AFTER PHYSICS)
    // ===================================================================================
    
    /// <summary>
    /// ===================================================================================
    /// MESH DEFORMATION
    /// ===================================================================================
    /// 
    /// Called after physics update to deform the visual mesh based on mass point positions.
    /// 
    /// ALGORITHM (TRILINEAR INTERPOLATION):
    /// -------------------------------------
    /// 1. Extract the 8 corner points of the lattice (mass-spring system)
    /// 2. For each vertex in the base mesh:
    ///    a. Get its position in the original cube (-1 to +1 range)
    ///    b. Convert to normalized coordinates (0 to 1 range)
    ///    c. Use trilinear interpolation to blend between 8 corner positions
    ///    d. Result is the deformed vertex position
    /// 
    /// WHY TRILINEAR INTERPOLATION:
    /// ----------------------------
    /// - Fast: Only needs 8 corner points, not all grid points
    /// - Smooth: Creates smooth deformation across mesh surface
    /// - Stable: Mesh always stays "attached" to physics simulation
    /// 
    /// TRILINEAR INTERPOLATION MATH:
    /// ------------------------------
    /// Given 8 corner positions and interpolation factors (fx, fy, fz):
    /// 1. Lerp along X between pairs: (0,1), (3,2), (4,5), (7,6)
    /// 2. Lerp along Z between results: bottom face, top face
    /// 3. Lerp along Y between the two faces
    /// Result: smooth blend of all 8 corners
    /// 
    /// WHY RecalculateBounds():
    /// ------------------------
    /// After deformation, mesh bounds must be updated so Unity knows where it is.
    /// Without this, frustum culling uses old bounds → mesh disappears when camera
    /// gets close because Unity thinks it's off-screen.
    /// 
    /// ===================================================================================
    /// </summary>
    void LateUpdate()
    {
        // Extract the 8 corner points of the lattice
        // Corner indices: [x][y][z]
        //   Bottom face (y=0): [0,0,0], [max,0,0], [max,0,max], [0,0,max]
        //   Top face (y=max): [0,max,0], [max,max,0], [max,max,max], [0,max,max]
        Vector3[] corners =
        {
            points[0,0,0].position,                             // 0: bottom-left-back
            points[gridSize-1,0,0].position,                    // 1: bottom-right-back
            points[gridSize-1,0,gridSize-1].position,           // 2: bottom-right-front
            points[0,0,gridSize-1].position,                    // 3: bottom-left-front
            points[0,gridSize-1,0].position,                    // 4: top-left-back
            points[gridSize-1,gridSize-1,0].position,           // 5: top-right-back
            points[gridSize-1,gridSize-1,gridSize-1].position,  // 6: top-right-front
            points[0,gridSize-1,gridSize-1].position            // 7: top-left-front
        };

        // Deform each vertex using trilinear interpolation
        for (int i = 0; i < baseVertices.Length; i++)
        {
            // Get original vertex position from base mesh (-1 to +1 range)
            // Convert to normalized coordinates (0 to 1 range)
            // (baseVertices[i] ranges from -1 to +1, adding Vector3.one shifts to 0 to 2, multiply by 0.5 → 0 to 1)
            Vector3 local = (baseVertices[i] + Vector3.one) * 0.5f;
            float fx = local.x;  // interpolation factor along X axis
            float fy = local.y;  // interpolation factor along Y axis
            float fz = local.z;  // interpolation factor along Z axis

            // Trilinear interpolation:
            // 1st level: Lerp along X axis (4 times, for each pair along X)
            // 2nd level: Lerp along Z axis (2 times, bottom and top faces)
            // 3rd level: Lerp along Y axis (1 time, between bottom and top)
            Vector3 interp =
                MathUtils.Lerp(
                    // Bottom face (y=0)
                    MathUtils.Lerp(
                        MathUtils.Lerp(corners[0], corners[1], fx),  // back edge
                        MathUtils.Lerp(corners[3], corners[2], fx),  // front edge
                        fz),
                    // Top face (y=1)
                    MathUtils.Lerp(
                        MathUtils.Lerp(corners[4], corners[5], fx),  // back edge
                        MathUtils.Lerp(corners[7], corners[6], fx),  // front edge
                        fz),
                    fy);

            // Store deformed position (relative to transform.position for local space)
            deformedVertices[i] = interp - transform.position;
        }

        // Update mesh with deformed vertices
        mesh.vertices = deformedVertices;
        
        // Recalculate normals for proper lighting
        mesh.RecalculateNormals();
        
        // CRITICAL: Recalculate bounds to prevent culling bugs
        // Without this, mesh disappears when camera gets close
        // because Unity uses stale bounds for frustum culling
        mesh.RecalculateBounds();
    }

    // ===================================================================================
    // DEBUG VISUALIZATION
    // ===================================================================================
    
    /// <summary>
    /// ===================================================================================
    /// DEBUG GIZMOS
    /// ===================================================================================
    /// 
    /// Draws debug visualization in Scene view:
    /// - Yellow lines: springs (connections between mass points)
    /// - Cyan spheres: mass points (physics lattice points)
    /// - Red wire spheres: collision detection radii (if enabled)
    /// 
    /// This is ONLY visible in Scene view, not in Game view.
    /// Useful for understanding what the physics system is doing.
    /// 
    /// ===================================================================================
    /// </summary>
    void OnDrawGizmos()
    {
        // Only draw during play mode when debug is enabled
        if (!Application.isPlaying || !drawDebug || springs == null) return;

        // Draw springs as yellow lines
        Gizmos.color = Color.yellow;
        foreach (var s in springs)
            Gizmos.DrawLine(s.a.position, s.b.position);

        // Draw mass points as small cyan spheres
        Gizmos.color = Color.cyan;
        foreach (var p in points)
            Gizmos.DrawSphere(p.position, 0.02f);

        // Draw collision detection spheres as red wireframes
        if (drawCollisionSpheres)
        {
            Gizmos.color = Color.red;
            float pr = Application.isPlaying ? GetCurrentPointRadius() : pointRadius;
            foreach (var p in points)
                Gizmos.DrawWireSphere(p.position, pr);
        }
    }
}

// ===================================================================================
// END OF FILE
// ===================================================================================
//
// KEY TAKEAWAYS:
// --------------
// 1. Soft body = mass-spring system with Verlet integration
// 2. Collision BEFORE springs is critical (prevents fall-through)
// 3. Sphere vs OBB collision uses pure math (TransformUtils + CollisionUtils)
// 4. Mesh deformation uses trilinear interpolation of 8 corner points
// 5. RecalculateBounds() prevents mesh from disappearing near camera
// 6. No Unity physics (Rigidbody/Collider) - all pure math for full control
//
// ===================================================================================
