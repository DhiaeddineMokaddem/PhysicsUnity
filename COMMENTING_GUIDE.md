# Line-by-Line Comments Guide for Physics Scripts

## Overview
This guide shows the pattern for adding detailed line-by-line comments to all physics scripts in the Animations folder. Apply this pattern consistently across all files.

## General Commenting Principles

### 1. File Header Comments
```csharp
// Import statements - explain what each provides
using UnityEngine;           // Unity's core engine functionality
using PhysicsSimulation.Core; // Custom physics simulation utilities

// Namespace declaration - organize code by folder structure
namespace PhysicsSimulation.Indiv_Work.Aziz
{
    /// <summary>
    /// Class documentation - purpose and key features
    /// </summary>
    public class RigidBody3D : MonoBehaviour
    {
```

### 2. Field Comments
```csharp
// UI Inspector header for grouping related properties
[Header("Physical Properties")]
// Mass in kilograms - affects force response and momentum
public float mass = 1.0f;
// Linear velocity vector in meters per second
public Vector3 velocity = Vector3.zero;
// Angular velocity vector in radians per second
public Vector3 angularVelocity = Vector3.zero;
// Coefficient of restitution (0 = inelastic, 1 = perfectly elastic)
public float restitution = PhysicsConstants.DEFAULT_RESTITUTION;
```

### 3. Method Comments
```csharp
/// <summary>
/// Unity lifecycle method - called when script instance is being loaded
/// </summary>
void Awake()
{
    // Try to get existing VisualRenderer component from this GameObject
    visualRenderer = GetComponent<VisualRenderer>();
    // If no VisualRenderer found, add one dynamically
    if (visualRenderer == null)
    {
        visualRenderer = gameObject.AddComponent<VisualRenderer>();
    }
    
    // Check if this is first initialization (not manually set before)
    if (!isInitialized)
    {
        // Read initial position from visual renderer
        position = visualRenderer.GetPosition();
        // Read initial rotation from visual renderer
        rotation = visualRenderer.GetRotation();
        // Read initial scale from visual renderer
        scale = visualRenderer.GetScale();
        // Mark as initialized to prevent re-initialization
        isInitialized = true;
    }
    
    // Calculate moment of inertia tensor for this rigid body
    CalculateInertiaTensor();
}
```

### 4. Physics Calculation Comments
```csharp
public void IntegratePhysics(float deltaTime)
{
    // Skip physics if this body is kinematic (controlled externally)
    if (isKinematic) return;

    // Apply gravitational force if gravity is enabled
    if (useGravity)
    {
        // F = m * g where g is gravity acceleration
        force += mass * PhysicsConstants.GRAVITY_VECTOR;
    }

    // Calculate linear acceleration from force using Newton's second law: a = F/m
    Vector3 acceleration = IntegrationUtils.ForceToAcceleration(force, mass);
    // Integrate velocity using Euler integration: v = v + a*dt
    velocity = IntegrationUtils.IntegrateVelocityEuler(velocity, acceleration, deltaTime);
    // Apply damping to simulate air resistance: v = v * (1 - damping*dt)
    velocity = IntegrationUtils.ApplyDamping(velocity, linearDamping, deltaTime);

    // MANUAL POSITION UPDATE: Update position using velocity
    // p = p + v*dt (Euler integration)
    position = IntegrationUtils.IntegratePositionEuler(position, velocity, deltaTime);

    // Calculate world-space inverse inertia tensor for angular acceleration
    Matrix4x4 worldInertiaTensorInv = CalculateWorldInverseInertiaTensor();
    // Calculate angular acceleration from torque: α = I^-1 * τ
    Vector3 angularAcceleration = IntegrationUtils.TorqueToAngularAcceleration(torque, worldInertiaTensorInv);
    // Integrate angular velocity: ω = ω + α*dt
    angularVelocity = IntegrationUtils.IntegrateVelocityEuler(angularVelocity, angularAcceleration, deltaTime);
    // Apply angular damping to slow down rotation
    angularVelocity = IntegrationUtils.ApplyDamping(angularVelocity, angularDamping, deltaTime);

    // MANUAL ROTATION UPDATE: Update rotation quaternion using angular velocity
    // q = q + 0.5 * ω * q * dt (quaternion integration)
    rotation = IntegrationUtils.IntegrateRotationQuaternion(rotation, angularVelocity, deltaTime);

    // Update visual mesh vertices using manual transformation
    UpdateVisualTransform();

    // Clear accumulated forces and torques for next frame
    force = Vector3.zero;
    torque = Vector3.zero;
}
```

### 5. Mathematical Operation Comments
```csharp
// Transform a world-space point to local space
public Vector3 InverseTransformPoint(Vector3 worldPoint)
{
    // Subtract position to get relative position
    Vector3 relative = worldPoint - position;
    // Rotate by inverse rotation to get local coordinates
    return Quaternion.Inverse(rotation) * relative;
    // Then scale by inverse scale (component-wise division)
    // result.x /= scale.x; result.y /= scale.y; result.z /= scale.z;
}
```

### 6. Collision Detection Comments
```csharp
public bool DetectCollision(RigidBody3D other, out CollisionInfo collision)
{
    // Initialize collision info structure
    collision = new CollisionInfo();
    
    // Calculate vector from this body to other body
    Vector3 delta = other.position - position;
    // Calculate combined radius (sum for sphere collision)
    float combinedRadius = radius + other.radius;
    // Calculate squared distance (avoid expensive square root)
    float distanceSquared = delta.sqrMagnitude;
    
    // Check if distance is less than combined radius (collision detected)
    if (distanceSquared < combinedRadius * combinedRadius)
    {
        // Calculate actual distance using square root
        float distance = Mathf.Sqrt(distanceSquared);
        // Calculate collision normal (direction of separation)
        collision.normal = delta / distance; // Normalize
        // Calculate penetration depth (how much objects overlap)
        collision.penetrationDepth = combinedRadius - distance;
        // Calculate contact point (on surface between objects)
        collision.contactPoint = position + collision.normal * radius;
        // Store references to both colliding bodies
        collision.bodyA = this;
        collision.bodyB = other;
        // Return true to indicate collision detected
        return true;
    }
    
    // No collision detected
    return false;
}
```

### 7. Conditional Logic Comments
```csharp
// Check if body should respond to physics
if (!isKinematic)
{
    // Add force to accumulated force vector
    force += f;
    
    // Calculate lever arm from center of mass to point of application
    Vector3 r = point - position;
    // Calculate torque using cross product: τ = r × F
    torque += Vector3.Cross(r, f);
}
else
{
    // Kinematic bodies ignore forces
    return;
}
```

### 8. Loop Comments
```csharp
// Transform each vertex manually using the transformation matrix
for (int i = 0; i < originalVertices.Length; i++)
{
    // Apply scale to original vertex (component-wise multiplication)
    Vector3 scaledVertex = new Vector3(
        originalVertices[i].x * currentScale.x, // Scale X component
        originalVertices[i].y * currentScale.y, // Scale Y component
        originalVertices[i].z * currentScale.z  // Scale Z component
    );
    
    // Apply rotation and translation using manual matrix multiplication
    // This transforms from local space to world space
    transformedVertices[i] = matrix.MultiplyPoint(scaledVertex);
}
```

## Comment Patterns by Script Type

### Physics Body Scripts (RigidBody3D, DynamicSphere3D)
- Comment **each physics property** with units and meaning
- Explain **integration steps** (Euler, Verlet, RK4, etc.)
- Document **coordinate space** (local vs world)
- Clarify **mathematical formulas** used

### Collision Scripts (CollisionDetector, CollisionInfo)
- Explain **geometric tests** (sphere-sphere, box-box, etc.)
- Document **penetration depth** calculation
- Clarify **normal vector** direction
- Comment **contact point** determination

### Impact Scripts (ImpactSphere, EnergizedPlateFracture)
- Explain **force application** logic
- Document **energy transfer** calculations
- Clarify **breaking threshold** checks
- Comment **impulse calculations**

### Manager Scripts (PhysicsManager, SimulationController)
- Explain **fixed timestep** updates
- Document **substep iterations**
- Clarify **constraint solving** order
- Comment **collision response** flow

## Special Cases

### 1. Manual Transform Updates (NO Unity Transform!)
```csharp
// MANUAL POSITION: Update position using manual calculation
position += velocity * deltaTime;

// MANUAL ROTATION: Update rotation using manual quaternion integration
rotation = IntegrationUtils.IntegrateRotationQuaternion(rotation, angularVelocity, deltaTime);

// UPDATE VISUAL MESH: Transform mesh vertices using ManualMatrix
// This replaces Unity's transform.position and transform.rotation!
if (visualRenderer != null)
{
    visualRenderer.UpdateTransform(position, rotation, scale);
}
```

### 2. Gizmo Drawing
```csharp
void OnDrawGizmos()
{
    // Use manual position when game is playing, otherwise use transform position
    Vector3 drawPos = Application.isPlaying ? position : transform.position;
    
    // Set gizmo color
    Gizmos.color = Color.green;
    // Draw wire sphere at manual position
    Gizmos.DrawWireSphere(drawPos, radius);
    
    // Draw velocity vector
    Gizmos.color = Color.blue;
    // Arrow from position in direction of velocity
    Gizmos.DrawLine(drawPos, drawPos + velocity);
}
```

### 3. Quaternion Operations
```csharp
// Normalize quaternion to prevent drift
float mag = Mathf.Sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
q.x /= mag; // Normalize X component
q.y /= mag; // Normalize Y component
q.z /= mag; // Normalize Z component
q.w /= mag; // Normalize W component
```

## Files to Comment

### ✅ Completed (Fully Commented):
1. Core/ManualMatrix.cs
2. Core/VisualRenderer.cs
3. Core/QuaternionRotation.cs
4. Core/EulerRotation.cs

### 📋 To Comment (Apply patterns above):
5. **Aziz Folder:**
   - RigidBody3D.cs
   - ImpactSphere.cs
   - CollisionDetector.cs
   - PhysicsManager.cs
   - RigidConstraint.cs
   - SimulationController.cs
   - StructureBuilder.cs
   - EnergizedPlateFracture.cs

6. **Rayen Folder:**
   - ImpactSphereRayen.cs
   - DynamicSphere3D.cs
   - StaticSpherePlatform.cs
   - CollisionDetectorRayen.cs
   - PhysicsManagerRayen.cs
   - CubesOverSphereDemo.cs

7. **Yahya Folder:**
   - RigidBody3DYahya.cs
   - ImpactSphereYahya.cs
   - CollisionDetectorYahya.cs
   - PhysicsManagerYahya.cs
   - RigidConstraintYahya.cs
   - SimulationControllerYahya.cs
   - StructureBuilderYahya.cs
   - EnergizedPlateFractureYahya.cs

8. **Remaining Core:**
   - IntegrationUtils.cs
   - MathUtils.cs
   - TransformUtils.cs
   - CollisionUtils.cs
   - PhysicsConstants.cs
   - MeshUtils.cs
   - DebugDrawUtils.cs

## Key Reminders

1. **Every line should have a comment** explaining what it does
2. **Mathematical operations** need formula explanations
3. **Physics calculations** need units (m/s, kg, N, etc.)
4. **NO transform.position/rotation** in physics code - always comment manual calculations
5. **Coordinate space** should be clear (local/world/relative)
6. **Integration methods** should be named (Euler/Verlet/RK4)
7. **Performance notes** for expensive operations
8. **Safety checks** (null checks, division by zero, etc.)

## Example: Complete Method with Full Comments

```csharp
/// <summary>
/// Applies an impulse at a specific point on the rigid body
/// Impulse instantly changes velocity (J = Δp = m*Δv)
/// </summary>
/// <param name="impulse">Impulse vector in Newtons-seconds (N·s)</param>
/// <param name="point">World-space point where impulse is applied</param>
public void AddImpulseAtPoint(Vector3 impulse, Vector3 point)
{
    // Skip if body is kinematic (not affected by forces)
    if (isKinematic) return;
    
    // Calculate world-space inverse inertia tensor
    // Needed for angular response calculation
    Matrix4x4 worldInertiaTensorInv = CalculateWorldInverseInertiaTensor();
    
    // Apply impulse to linear and angular velocity using helper function
    // This updates both velocity and angularVelocity in one call
    TransformUtils.ApplyImpulseAtPoint(
        ref velocity,           // Linear velocity to update (passed by reference)
        ref angularVelocity,    // Angular velocity to update (passed by reference)
        impulse,                // Impulse vector (force * time)
        point,                  // Application point in world space
        position,               // Center of mass position
        1.0f / mass,            // Inverse mass (for linear response)
        worldInertiaTensorInv   // Inverse inertia tensor (for angular response)
    );
}
```

This guide provides the framework for commenting all remaining physics scripts in a consistent, educational manner!

