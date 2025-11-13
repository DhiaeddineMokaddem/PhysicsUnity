# PhysicsUnity - Custom Physics Simulation Framework

A comprehensive Unity-based physics simulation project implementing multiple physics systems **from scratch** without using Unity's built-in physics engine. All simulations use pure mathematical implementations of physics principles including custom integration methods, collision detection, and constraint solving.

---

## 🎯 Project Overview

This project demonstrates advanced physics programming through five distinct simulations, each showcasing different aspects of computational physics:

1. **Chain Physics** - Verlet integration with breaking mechanics
2. **Soft Body Jello** - 3D mass-spring lattice with deformable mesh
3. **Plate Fracture** - Rigid body dynamics with spring-based fracture
4. **Rigid Body Dynamics** - Basic OBB collision and stacking
5. **Wall Demolition** - High-speed impact with constraint breaking

### Core Philosophy

**Pure Mathematical Physics - No Unity Physics Engine**

- ❌ No `Rigidbody` components
- ❌ No Unity `Collider` components  
- ❌ No Unity physics engine calls
- ✅ Manual position/velocity integration
- ✅ Custom collision detection algorithms
- ✅ Manual transform matrix calculations
- ✅ Position-based and impulse-based dynamics

---

## 📁 Project Structure

```
PhysicsUnity/
├── Assets/Scripts/
│   └── Animations/
│       ├── Core/                    # Shared physics utilities
│       │   ├── Physics/             
│       │   │   └── Spring.cs        # Mass-spring system classes
│       │   ├── ManualMatrix.cs      # Custom 4x4 transform matrices
│       │   ├── IntegrationUtils.cs  # Euler, Verlet, Semi-implicit integrators
│       │   ├── CollisionUtils.cs    # Collision detection algorithms
│       │   ├── PhysicsConstants.cs  # Physical constants (gravity, damping, etc.)
│       │   ├── MathUtils.cs         # Matrix operations, inertia tensors
│       │   ├── TransformUtils.cs    # Coordinate space transformations
│       │   ├── VisualRenderer.cs    # Manual mesh rendering
│       │   └── ...
│       └── Indiv_Work/              # Individual simulations
│           ├── Dhia/                # Chain physics
│           │   ├── ChainLink.cs
│           │   ├── ChainPhysics.cs
│           │   ├── ChainVisualizer.cs
│           │   ├── ChainBreaking.cs
│           │   └── ChainAnchor.cs
│           ├── nour/                # Soft body jello
│           │   └── ControllableSoftJello.cs
│           ├── aziz/                # Plate fracture
│           │   ├── RigidBody3D.cs
│           │   └── EnergizedPlateFracture.cs
│           ├── Rayen/               # Rigid body dynamics
│           │   └── DynamicSphere3D.cs
│           └── yahya/               # Wall demolition
│               ├── RigidBody3DYahya.cs
│               ├── ImpactSphereYahya.cs
│               ├── StructureBuilderYahya.cs
│               ├── RigidConstraintYahya.cs
│               └── SimulationControllerYahya.cs
```

---

## 🔧 Core Components

### 1. Integration Methods (`IntegrationUtils.cs`)

Three numerical integration schemes for physics simulation:

#### Explicit Euler
```csharp
// x(t+dt) = x(t) + v(t)*dt
Vector3 newPos = IntegrationUtils.IntegratePositionEuler(position, velocity, deltaTime);
```
- **Use case:** Simple systems, quick prototyping
- **Stability:** Low (can explode with large dt)

#### Verlet Integration
```csharp
// x(t+dt) = 2*x(t) - x(t-dt) + a(t)*dt²
Vector3 newPos = IntegrationUtils.IntegratePositionVerlet(
    currentPosition, previousPosition, acceleration, deltaTime);
```
- **Use case:** Constraint-based systems (chains, cloth)
- **Stability:** High (energy-conserving, time-reversible)

#### Semi-Implicit Euler (Symplectic)
```csharp
// Update velocity first, THEN position using new velocity
IntegrationUtils.IntegrateSemiImplicitEuler(
    ref position, ref velocity, acceleration, deltaTime);
```
- **Use case:** Rigid body dynamics
- **Stability:** Medium-High (more stable than explicit Euler)

### 2. Collision Detection (`CollisionUtils.cs`)

Custom collision detection for primitive shapes:

- **Sphere-Sphere:** Distance test
- **Sphere-Plane:** Point-plane distance
- **Sphere-OBB:** Closest point on oriented bounding box
- **Sphere-AABB:** Axis-aligned bounding box test

```csharp
// Example: Sphere-sphere collision
bool hasCollision = CollisionUtils.CheckSphereSphereCollision(
    centerA, radiusA, centerB, radiusB, 
    out Vector3 normal, out float penetration);
```

### 3. Manual Transform System (`ManualMatrix.cs`)

Complete bypass of Unity's Transform component for full control:

```csharp
// Build transformation matrix from position and rotation
ManualMatrix transform = ManualMatrix.TR(position, rotation);

// Transform points manually
Vector3 worldPoint = transform.MultiplyPoint(localPoint);

// Invert for reverse transformation
ManualMatrix inverse = transform.InverseRigid();
```

### 4. Physics Constants (`PhysicsConstants.cs`)

Centralized physical constants:

```csharp
PhysicsConstants.GRAVITY = 9.81f;                    // m/s²
PhysicsConstants.GRAVITY_VECTOR = (0, -9.81, 0);   // World space
PhysicsConstants.DEFAULT_RESTITUTION = 0.5f;        // Bounciness
PhysicsConstants.DEFAULT_FRICTION = 0.3f;           // Surface friction
PhysicsConstants.EPSILON = 1e-8f;                   // Numerical precision
```

---

## 🎮 Simulations

### 1. Chain Physics (Dhia)

**Description:** A physically accurate chain with breaking mechanics using Verlet integration and position-based constraints.

**Key Features:**
- Verlet integration (no explicit velocity needed)
- Distance constraints maintain link spacing
- Tension-based breaking when stretched beyond threshold
- Custom rendering via `Graphics.DrawMesh` (no Transform setters)
- Anchor attachment with dynamic reattachment

**Physics Principles:**
- **Verlet Integration:** `x(t+dt) = 2*x(t) - x(t-dt) + a*dt²`
- **Position-Based Dynamics:** Directly correct positions to satisfy constraints
- **Constraint Iteration:** Multiple solver passes for stability

**Key Code:**
```csharp
// Verlet integration
Vector3 temp = link.position;
Vector3 acceleration = Vector3.down * gravity;
link.position = link.position + (link.position - link.prevPosition) * damping 
                + acceleration * dt * dt;
link.prevPosition = temp;

// Distance constraint
Vector3 delta = link2.position - link1.position;
float currentDist = delta.magnitude;
float diff = (currentDist - linkLength) / currentDist;
Vector3 correction = delta * diff * 0.5f;
link1.position += correction;
link2.position -= correction;
```

**Controls:**
- Chain automatically hangs between two anchors
- Breaking occurs when tension exceeds threshold
- Visual feedback shows tension via color (cyan → red)

---

### 2. Soft Body Jello (Nour)

**Description:** A deformable jello cube using a 3D mass-spring lattice with alpha-controlled stiffness.

**Key Features:**
- 3D lattice of mass points (e.g., 4×4×4 = 64 points)
- Structural, shear, and bend springs
- Alpha parameter controls stiffness/damping (0 = soft & bouncy, 1 = stiff & rigid)
- Trilinear interpolation for smooth mesh deformation
- Sphere-OBB collision detection
- User controllable with force-based movement

**Physics Principles:**
- **Hooke's Law:** F = -k·Δx (spring force proportional to extension)
- **Mass-Spring System:** Network of connected springs forms volumetric structure
- **Trilinear Interpolation:** Smooth mesh deformation from 8 corner points

**Key Code:**
```csharp
// Spring force application
Vector3 delta = pointB.position - pointA.position;
float extension = delta.magnitude - restLength;
Vector3 springForce = stiffness * extension * delta.normalized;

// Alpha-controlled properties
float currentStiffness = Mathf.Lerp(minStiffness, maxStiffness, alpha);
float currentDamping = Mathf.Lerp(minDamping, maxDamping, alpha);
float currentRestitution = Mathf.Lerp(maxRestitution, minRestitution, alpha);

// Trilinear interpolation for mesh deformation
Vector3 deformedVertex = 
    c000 * (1-u) * (1-v) * (1-w) +
    c100 * u * (1-v) * (1-w) +
    c010 * (1-u) * v * (1-w) +
    // ... (8 corners total)
    c111 * u * v * w;
```

**Controls:**
- **Arrow Keys / WASD:** Move jello horizontally
- **Space:** Jump
- **Alpha Slider:** Adjust stiffness (0-1)

---

### 3. Plate Fracture (Aziz)

**Description:** Rigid body fracture simulation using Baraff-style dynamics with spring-based constraints and energy conversion on break.

**Key Features:**
- Full 6DOF rigid body dynamics (linear + angular)
- Spring constraints between adjacent cubes
- Elastic potential energy converts to kinetic on fracture
- Impulse-based collision with effective mass calculation
- Exponential map for rotation integration

**Physics Principles:**
- **Rigid Body Equation:** F = ma, τ = Iα
- **Inertia Tensor:** I = (1/12)m(h² + d²) for box
- **Baraff Effective Mass:** Accounts for rotation in impulse calculation
- **Energy Conversion:** E_potential = ½kx² → E_kinetic = ½mv²

**Key Code:**
```csharp
// Apply spring force with torque
Vector3 rA_world = cubeA.rotation * attachmentPointLocal;
Vector3 posA = cubeA.position + rA_world;
Vector3 springForce = springK * extension * direction;

cubeA.velocity += (springForce / cubeA.mass) * dt;
cubeA.angularVelocity += I_inv * Cross(rA_world, springForce) * dt;

// Fracture: convert elastic energy to kinetic
float elasticEnergy = 0.5f * springK * extension * extension;
float impulseEnergy = elasticEnergy * fractureAlpha;
float impulseMag = Mathf.Sqrt(2f * impulseEnergy * mass);
```

**Controls:**
- Plate automatically fractures on collision with sphere
- Parameter tuning via inspector (spring stiffness, fracture alpha)

---

### 4. Rigid Body Dynamics (Rayen)

**Description:** Basic rigid body cubes dropping onto a sphere, demonstrating collision and stacking.

**Key Features:**
- Simple rigid body implementation with inertia tensors
- OBB (oriented bounding box) collision detection
- Impulse-based collision resolution
- Multiple bodies stacking and interacting

**Physics Principles:**
- **Newton's Law of Restitution:** j = -(1 + e)·v_n / (1/m_a + 1/m_b)
- **OBB Collision:** Separating Axis Theorem (SAT)
- **Stacking:** Multiple rigid bodies in contact

**Key Code:**
```csharp
// Rigid body integration
if (useGravity)
    velocity += PhysicsConstants.GRAVITY_VECTOR * deltaTime;

velocity *= (1f - linearDamping * deltaTime);
position += velocity * deltaTime;
rotation = IntegrationUtils.IntegrateRotationQuaternion(
    rotation, angularVelocity, deltaTime);

// Impulse resolution
float vn = Vector3.Dot(relativeVelocity, normal);
float j = -(1f + restitution) * vn / (1f/massA + 1f/massB);
Vector3 impulse = j * normal;
bodyA.velocity += impulse / bodyA.mass;
bodyB.velocity -= impulse / bodyB.mass;
```

**Controls:**
- Automatic simulation (cubes fall and stack)
- Can spawn additional cubes via code

---

### 5. Wall Demolition (Yahya)

**Description:** A fast-moving sphere (75 m/s) crashes into a wall of interconnected cubes, demonstrating high-velocity impact and structural destruction.

**Key Features:**
- Large wall structure (15×10×1 = 150 cubes)
- High-speed projectile (75 m/s = 270 km/h!)
- Breakable rigid constraints between cubes
- Radial constraint breaking propagation
- Interactive launch controls
- Real-time statistics display

**Physics Principles:**
- **Kinetic Energy:** E = ½mv² = ½(10)(75)² = 28,125 Joules
- **Radial Damage:** Constraints break in expanding sphere from impact
- **Constraint Breaking:** Force threshold exceeded → connection breaks
- **Energy Dissipation:** Small loss per collision prevents unrealistic bouncing

**Key Code:**
```csharp
// Launch sphere at high speed
public void Launch()
{
    Vector3 direction = launchFrom == LaunchSide.Right ? Vector3.left : Vector3.right;
    velocity = direction * launchSpeed;  // 75 m/s!
    float kineticEnergy = 0.5f * mass * launchSpeed * launchSpeed;
    Debug.Log($"Energy: {kineticEnergy:F0} J");  // 28,125 Joules!
}

// Break nearby constraints on impact
void BreakNearbyConstraints(Vector3 impactPoint, float radius)
{
    foreach (var constraint in constraints)
    {
        Vector3 midpoint = (constraint.bodyA.position + constraint.bodyB.position) * 0.5f;
        float dist = Vector3.Distance(impactPoint, midpoint);
        
        if (dist < radius && !constraint.isBroken)
        {
            constraint.Break();
            
            // Apply radial explosion force
            Vector3 explosionDir = (midpoint - impactPoint).normalized;
            float forceMag = (1f - dist/radius) * 100f;
            constraint.bodyA.AddImpulse(explosionDir * forceMag);
        }
    }
}
```

**Controls:**
- **Space:** Launch sphere
- **R:** Reset/rebuild wall
- **S:** Switch launch side (left ↔ right)
- **↑/↓:** Increase/decrease launch speed
- **I/K:** Adjust impact multiplier
- **+/-:** Adjust elasticity

**Statistics Display:**
- Current velocity and kinetic energy
- Launch speed and direction
- Impact multiplier
- Number of broken constraints

---

## 📊 Comparison Table

| Simulation | Integration | Collision | Key Physics | Topology | Control |
|------------|-------------|-----------|-------------|----------|---------|
| **Chain** | Verlet | None (constraints only) | Distance constraints, breaking | Linear chain | Automatic |
| **Jello** | Verlet | Sphere-OBB | Mass-spring lattice | 3D volumetric | Player controlled |
| **Fracture** | Semi-implicit | Sphere-impulse | Energy conversion | 5×4 plate | Automatic |
| **Rigid Body** | Semi-implicit | OBB-Sphere | Stacking | Independent boxes | Automatic |
| **Wall** | Semi-implicit | Sphere-OBB | High-speed impact | 15×10×1 wall | Interactive launch |

---

## 🚀 Getting Started

### Prerequisites
- Unity 2021.3 or later
- Basic understanding of C# and physics

### Setup
1. Clone the repository
2. Open the project in Unity
3. Navigate to simulation scenes in `Assets/Scenes/`
4. Press Play to run

### Running a Simulation

Each simulation has its own scene:
- `ChainPhysics.unity` - Chain simulation
- `JelloSimulation.unity` - Soft body jello
- `PlateFracture.unity` - Plate fracture
- `RigidBodyDemo.unity` - Rigid body stacking
- `WallDemolition.unity` - Wall demolition

---

## 🔬 Technical Highlights

### Pure Math Approach

All simulations avoid Unity's physics engine and implement everything from scratch:

```csharp
// NO: Using Unity physics
transform.position = newPos;  // ❌
rigidbody.AddForce(force);    // ❌

// YES: Manual math
position = position + velocity * deltaTime;  // ✅
velocity = velocity + (force / mass) * deltaTime;  // ✅
visualRenderer.UpdatePosition(position);  // ✅
```

### Numerical Stability Techniques

1. **Position-Based Dynamics:** More stable than force-based for constraints
2. **Damping:** Essential for all systems to prevent energy buildup
3. **Constraint Iteration:** Multiple solver passes (e.g., 8 iterations for jello)
4. **Impulse Clamping:** Prevent extreme forces from causing explosions
5. **Energy Dissipation:** Small loss per collision prevents unrealistic behavior

### Performance Optimizations

1. **Spatial Partitioning:** Used in gas simulation (Sweep and Prune)
2. **AABB Early Exit:** Quick rejection before detailed collision tests
3. **Caching:** Precompute inertia tensors, mesh data
4. **Constraint Culling:** Only check nearby constraints for breaking
5. **Fixed Timestep:** Physics update decoupled from frame rate

---

## 📚 Educational Value

This project demonstrates:

1. **Numerical Integration Methods** - Euler, Verlet, Semi-implicit
2. **Collision Detection Algorithms** - Sphere, OBB, AABB tests
3. **Constraint Solving** - Distance, spring, rigid constraints
4. **Rigid Body Dynamics** - Inertia tensors, torque, angular momentum
5. **Soft Body Physics** - Mass-spring systems, mesh deformation
6. **Fracture Mechanics** - Energy conversion, constraint breaking
7. **High-Velocity Physics** - Maintaining stability at extreme speeds
8. **Matrix Mathematics** - Transformations, rotations, inversions

### Key Physics Concepts

- **Newton's Laws:** F = ma, action-reaction, inertia
- **Conservation Laws:** Energy, momentum (linear and angular)
- **Hooke's Law:** F = -kx (springs)
- **Collision Response:** Restitution, friction, impulses
- **Rotational Dynamics:** Torque, inertia tensors, angular velocity

---

## 🛠️ Extending the Project

### Adding a New Simulation

1. Create folder in `Assets/Scripts/Animations/Indiv_Work/YourName/`
2. Use core utilities from `PhysicsSimulation.Core` namespace
3. Implement custom physics logic
4. Register with `PhysicsManager` if using centralized updates
5. Create scene and test

### Custom Integration Method

```csharp
public static class CustomIntegrator
{
    public static Vector3 IntegrateCustom(
        Vector3 position, 
        Vector3 velocity, 
        Vector3 acceleration,
        float deltaTime)
    {
        // Your custom integration formula here
        return newPosition;
    }
}
```

### Custom Collision Detection

```csharp
public static class CustomCollision
{
    public static bool CheckCustomShape(
        ShapeA a, 
        ShapeB b, 
        out ContactInfo contact)
    {
        // Your collision algorithm here
        return hasCollision;
    }
}
```

## 👥 Contributors

- **Dhia** - Chain Physics with Breaking Mechanics
- **Nour** - Controllable Soft Body Jello
- **Aziz** - Energized Plate Fracture
- **Rayen** - Rigid Body Dynamics
- **Yahya** - Wall Demolition with High-Speed Impact