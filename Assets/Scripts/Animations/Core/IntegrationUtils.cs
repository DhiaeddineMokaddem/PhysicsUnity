using UnityEngine;

// Namespace for core physics simulation utilities
namespace PhysicsSimulation.Core
{
    /// <summary>
    /// Utility class for physics integration methods
    /// Provides various numerical integration schemes for position, velocity, and rotation
    /// Integration = calculating future state from current state and rate of change
    /// </summary>
    public static class IntegrationUtils
    {
        #region Euler Integration
        /// <summary>
        /// Performs explicit Euler integration for position
        /// Formula: x(t+dt) = x(t) + v(t)*dt
        /// Simplest but least accurate integration method
        /// </summary>
        public static Vector3 IntegratePositionEuler(Vector3 position, Vector3 velocity, float deltaTime)
        {
            // New position = current position + (velocity * time step)
            // This assumes constant velocity over the time step
            return position + velocity * deltaTime;
        }

        /// <summary>
        /// Performs explicit Euler integration for velocity
        /// Formula: v(t+dt) = v(t) + a(t)*dt
        /// Updates velocity based on current acceleration
        /// </summary>
        public static Vector3 IntegrateVelocityEuler(Vector3 velocity, Vector3 acceleration, float deltaTime)
        {
            // New velocity = current velocity + (acceleration * time step)
            // This assumes constant acceleration over the time step
            return velocity + acceleration * deltaTime;
        }
        #endregion

        #region Verlet Integration
        /// <summary>
        /// Performs Verlet integration (position-based)
        /// Formula: x(t+dt) = 2*x(t) - x(t-dt) + a(t)*dt²
        /// More stable than Euler for oscillatory systems, doesn't need velocity
        /// </summary>
        public static Vector3 IntegratePositionVerlet(Vector3 currentPosition, Vector3 previousPosition, Vector3 acceleration, float deltaTime)
        {
            // Verlet formula: next = 2*current - previous + acceleration*dt²
            // This is derived from Taylor series expansion
            // Advantage: time-reversible and energy-conserving
            return 2f * currentPosition - previousPosition + acceleration * deltaTime * deltaTime;
        }

        /// <summary>
        /// Calculates velocity from Verlet positions
        /// Formula: v(t) = (x(t) - x(t-dt)) / dt
        /// Velocity is implicit in Verlet - must be calculated from positions
        /// </summary>
        public static Vector3 CalculateVerletVelocity(Vector3 currentPosition, Vector3 previousPosition, float deltaTime)
        {
            // Safety check: avoid division by zero
            if (deltaTime < PhysicsConstants.EPSILON)
                return Vector3.zero;
            // Velocity = (position change) / (time step)
            // This is the average velocity over the last time step
            return (currentPosition - previousPosition) / deltaTime;
        }
        #endregion

        #region Semi-Implicit Euler (Symplectic Euler)
        /// <summary>
        /// Performs semi-implicit Euler integration (velocity first, then position)
        /// Formula: v(t+dt) = v(t) + a(t)*dt, then x(t+dt) = x(t) + v(t+dt)*dt
        /// More stable than explicit Euler for physics simulations - energy doesn't explode
        /// </summary>
        public static void IntegrateSemiImplicitEuler(
            ref Vector3 position,      // Position to update (passed by reference)
            ref Vector3 velocity,      // Velocity to update (passed by reference)
            Vector3 acceleration,      // Current acceleration
            float deltaTime)           // Time step
        {
            // Step 1: Update velocity first using current acceleration
            velocity += acceleration * deltaTime;
            // Step 2: Update position using NEW velocity (this is the key difference from explicit Euler)
            // Using new velocity makes this method more stable and energy-preserving
            position += velocity * deltaTime;
        }
        #endregion

        #region Rotation Integration
        /// <summary>
        /// Integrates angular velocity using quaternions
        /// Quaternion integration avoids gimbal lock and is more stable than Euler angles
        /// </summary>
        public static Quaternion IntegrateRotationQuaternion(Quaternion rotation, Vector3 angularVelocity, float deltaTime)
        {
            // If angular velocity is negligible, no rotation occurs
            if (angularVelocity.sqrMagnitude < PhysicsConstants.EPSILON_SMALL)
                return rotation;
            
            // Convert angular velocity vector to quaternion form
            // Formula: q_omega = [ωx/2, ωy/2, ωz/2, 0]
            // The factor of 0.5 comes from quaternion derivative formula: dq/dt = 0.5 * ω * q
            Quaternion angularQuat = new Quaternion(
                angularVelocity.x * 0.5f,  // Half of X component of angular velocity
                angularVelocity.y * 0.5f,  // Half of Y component of angular velocity
                angularVelocity.z * 0.5f,  // Half of Z component of angular velocity
                0f                          // W component is zero for pure angular velocity
            );
            
            // Calculate quaternion derivative: dq/dt = ω_quat * q
            // This is the rate of change of the rotation quaternion
            Quaternion derivative = angularQuat * rotation;
            
            // Integrate using Euler method: q(t+dt) = q(t) + dq/dt * dt
            Quaternion newRotation = new Quaternion(
                rotation.x + derivative.x * deltaTime,  // Update X component
                rotation.y + derivative.y * deltaTime,  // Update Y component
                rotation.z + derivative.z * deltaTime,  // Update Z component
                rotation.w + derivative.w * deltaTime   // Update W component
            );
            
            // Normalize to maintain unit quaternion property (prevents drift)
            // Quaternions must have magnitude 1 to represent valid rotations
            return newRotation.normalized;
        }

        /// <summary>
        /// Integrates rotation using rotation matrix and angular velocity
        /// Alternative to quaternion integration using matrix representation
        /// </summary>
        public static Matrix4x4 IntegrateRotationMatrix(Matrix4x4 rotation, Vector3 angularVelocity, float deltaTime)
        {
            // If angular velocity is negligible, no rotation occurs
            if (angularVelocity.sqrMagnitude < PhysicsConstants.EPSILON_SMALL)
                return rotation;
            
            // Create skew-symmetric matrix from angular velocity
            // [ω]× is used to represent cross product as matrix multiplication
            Matrix4x4 angularMatrix = MathUtils.SkewSymmetric(angularVelocity);
            
            // Calculate matrix derivative: dR/dt = [ω]× * R
            // This is the rate of change of the rotation matrix
            Matrix4x4 derivative = MathUtils.MultiplyMatrix3x3(angularMatrix, rotation);
            
            // Integrate using Euler method: R(t+dt) = R(t) + dR/dt * dt
            Matrix4x4 result = Matrix4x4.zero;
            // Update only the 3x3 rotation part of the matrix
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    // result[i,j] = rotation[i,j] + derivative[i,j] * dt
                    result[i, j] = rotation[i, j] + derivative[i, j] * deltaTime;
                }
            }
            // Set homogeneous coordinate
            result.m33 = 1f;
            
            // Return updated rotation matrix
            return result;
        }
        #endregion

        #region RK4 Integration
        /// <summary>
        /// Performs Runge-Kutta 4th order integration for position and velocity
        /// Formula: 
        ///   k1v = a(t)
        ///   k1p = v(t)
        ///   k2v = a(t + dt/2, p(t) + k1p*dt/2, v(t) + k1v*dt/2)
        ///   k2p = v(t) + k1v*dt/2
        ///   k3v = a(t + dt/2, p(t) + k2p*dt/2, v(t) + k2v*dt/2)
        ///   k3p = v(t) + k2v*dt/2
        ///   k4v = a(t + dt, p(t) + k3p*dt, v(t) + k3v*dt)
        ///   k4p = v(t) + k3v*dt
        ///   p(t+dt) = p(t) + (k1p + 2*k2p + 2*k3p + k4p) * dt/6
        ///   v(t+dt) = v(t) + (k1v + 2*k2v + 2*k3v + k4v) * dt/6
        /// Accurate but computationally expensive - uses 4 function evaluations per step
        /// </summary>
        public static void IntegrateRK4(
            ref Vector3 position,
            ref Vector3 velocity,
            System.Func<Vector3, Vector3, Vector3> accelerationFunc,
            float deltaTime)
        {
            Vector3 p0 = position;
            Vector3 v0 = velocity;

            Vector3 k1v = accelerationFunc(p0, v0);
            Vector3 k1p = v0;

            Vector3 k2v = accelerationFunc(p0 + k1p * deltaTime * 0.5f, v0 + k1v * deltaTime * 0.5f);
            Vector3 k2p = v0 + k1v * deltaTime * 0.5f;

            Vector3 k3v = accelerationFunc(p0 + k2p * deltaTime * 0.5f, v0 + k2v * deltaTime * 0.5f);
            Vector3 k3p = v0 + k2v * deltaTime * 0.5f;

            Vector3 k4v = accelerationFunc(p0 + k3p * deltaTime, v0 + k3v * deltaTime);
            Vector3 k4p = v0 + k3v * deltaTime;

            velocity = v0 + (k1v + 2f * k2v + 2f * k3v + k4v) * deltaTime / 6f;
            position = p0 + (k1p + 2f * k2p + 2f * k3p + k4p) * deltaTime / 6f;
        }
        #endregion

        #region Force and Acceleration Conversions
        /// <summary>
        /// Converts force to acceleration using Newton's second law (F = ma)
        /// Formula: a = F / m
        /// </summary>
        public static Vector3 ForceToAcceleration(Vector3 force, float mass)
        {
            // Safety check: avoid division by zero
            if (mass < PhysicsConstants.EPSILON)
                return Vector3.zero;
            // Acceleration = Force / Mass
            // This converts the force vector to an acceleration vector
            return force / mass;
        }

        /// <summary>
        /// Converts acceleration to force using Newton's second law
        /// Formula: F = m * a
        /// </summary>
        public static Vector3 AccelerationToForce(Vector3 acceleration, float mass)
        {
            // Force = Mass * Acceleration
            // This converts the acceleration vector to a force vector
            return acceleration * mass;
        }

        /// <summary>
        /// Converts torque to angular acceleration
        /// Formula: α = I⁻¹ * τ
        /// </summary>
        public static Vector3 TorqueToAngularAcceleration(Vector3 torque, Matrix4x4 inverseInertiaTensor)
        {
            // Angular acceleration = Inverse inertia tensor * Torque
            // This converts the torque vector to an angular acceleration vector
            return MathUtils.MultiplyMatrixVector3(inverseInertiaTensor, torque);
        }

        /// <summary>
        /// Converts angular acceleration to torque
        /// Formula: τ = I * α
        /// </summary>
        public static Vector3 AngularAccelerationToTorque(Vector3 angularAcceleration, Matrix4x4 inertiaTensor)
        {
            // Torque = Inertia tensor * Angular acceleration
            // This converts the angular acceleration vector to a torque vector
            return MathUtils.MultiplyMatrixVector3(inertiaTensor, angularAcceleration);
        }
        #endregion

        #region Damping
        /// <summary>
        /// Applies damping to a velocity vector
        /// Formula: v' = v * (1 - damping * dt)
        /// Linear damping - velocity is reduced by a factor each timestep
        /// </summary>
        public static Vector3 ApplyDamping(Vector3 velocity, float damping, float deltaTime)
        {
            // Damping factor = clamp(0, 1, 1 - damping_coefficient * time_step)
            float dampingFactor = Mathf.Clamp01(1f - damping * deltaTime);
            // New velocity = current velocity * damping factor
            return velocity * dampingFactor;
        }

        /// <summary>
        /// Applies exponential damping to a velocity vector (more physically accurate)
        /// Formula: v' = v * exp(-damping * dt)
        /// Exponential decay - velocity decreases exponentially over time
        /// </summary>
        public static Vector3 ApplyExponentialDamping(Vector3 velocity, float damping, float deltaTime)
        {
            // New velocity = current velocity * exp(-damping_coefficient * time_step)
            // This models velocity decay as a continuous exponential function
            return velocity * Mathf.Exp(-damping * deltaTime);
        }
        #endregion

        #region Constraint Projection
        /// <summary>
        /// Projects velocity to satisfy a distance constraint between two points
        /// Formula: 
        ///   delta = pos2 - pos1
        ///   currentDistance = |delta|
        ///   error = currentDistance - targetDistance
        ///   correction = delta * (error / currentDistance) * stiffness
        ///   pos1 += correction * (mass2 / totalMass)
        ///   pos2 -= correction * (mass1 / totalMass)
        /// </summary>
        public static void ProjectDistanceConstraint(
            ref Vector3 pos1, ref Vector3 pos2,
            float targetDistance,
            float mass1, float mass2,
            float stiffness = 1f)
        {
            Vector3 delta = pos2 - pos1;
            float currentDistance = delta.magnitude;
            
            if (currentDistance < PhysicsConstants.EPSILON)
                return;

            float error = currentDistance - targetDistance;
            Vector3 correction = delta * (error / currentDistance) * stiffness;

            float totalMass = mass1 + mass2;
            if (totalMass < PhysicsConstants.EPSILON)
                return;

            float w1 = mass2 / totalMass;
            float w2 = mass1 / totalMass;

            pos1 += correction * w1;
            pos2 -= correction * w2;
        }

        /// <summary>
        /// Projects velocity to maintain a distance constraint (velocity-level)
        /// Formula: 
        ///   delta = pos2 - pos1
        ///   distance = |delta|
        ///   normal = delta / distance
        ///   relativeVel = vel2 - vel1
        ///   normalVel = relativeVel ⋅ normal
        ///   impulse = normal * normalVel
        ///   vel1 += impulse * (mass2 / totalMass)
        ///   vel2 -= impulse * (mass1 / totalMass)
        /// </summary>
        public static void ProjectVelocityConstraint(
            ref Vector3 vel1, ref Vector3 vel2,
            Vector3 pos1, Vector3 pos2,
            float mass1, float mass2)
        {
            Vector3 delta = pos2 - pos1;
            float distance = delta.magnitude;
            
            if (distance < PhysicsConstants.EPSILON)
                return;

            Vector3 normal = delta / distance;
            Vector3 relativeVel = vel2 - vel1;
            float normalVel = Vector3.Dot(relativeVel, normal);

            if (normalVel >= 0)
                return;

            float totalMass = mass1 + mass2;
            if (totalMass < PhysicsConstants.EPSILON)
                return;

            Vector3 impulse = normal * normalVel;
            vel1 += impulse * (mass2 / totalMass);
            vel2 -= impulse * (mass1 / totalMass);
        }
        #endregion

        #region Spring Forces
        /// <summary>
        /// Calculates spring force using Hooke's law
        /// Formula: F_spring = -k * x
        ///   where x = currentLength - restLength
        ///   and k = stiffness
        /// </summary>
        public static Vector3 CalculateSpringForce(
            Vector3 pos1, Vector3 pos2,
            float restLength,
            float stiffness,
            float damping,
            Vector3 vel1, Vector3 vel2)
        {
            Vector3 delta = pos2 - pos1;
            float currentLength = delta.magnitude;
            
            if (currentLength < PhysicsConstants.EPSILON)
                return Vector3.zero;

            Vector3 direction = delta / currentLength;
            float extension = currentLength - restLength;

            // Spring force (Hooke's law)
            Vector3 springForce = direction * (extension * stiffness);

            // Damping force
            Vector3 relativeVel = vel2 - vel1;
            float dampingVel = Vector3.Dot(relativeVel, direction);
            Vector3 dampingForce = direction * (dampingVel * damping);

            return springForce + dampingForce;
        }

        /// <summary>
        /// Calculates spring force with critical damping
        /// Formula: 
        ///   criticalDamping = 2 * sqrt(stiffness * mass)
        ///   F_spring = -k * x
        ///   where x = currentLength - restLength
        /// </summary>
        public static Vector3 CalculateCriticallyDampedSpring(
            Vector3 pos1, Vector3 pos2,
            float restLength,
            float stiffness,
            Vector3 vel1, Vector3 vel2,
            float mass)
        {
            // Critical damping value - ensures fastest return to equilibrium without overshoot
            float criticalDamping = 2f * Mathf.Sqrt(stiffness * mass);
            // Calculate spring force using critical damping
            return CalculateSpringForce(pos1, pos2, restLength, stiffness, criticalDamping, vel1, vel2);
        }
        #endregion

        #region Energy Calculations
        /// <summary>
        /// Calculates kinetic energy from velocity and mass
        /// Formula: KE = 0.5 * m * v²
        /// </summary>
        public static float CalculateKineticEnergy(Vector3 velocity, float mass)
        {
            return 0.5f * mass * velocity.sqrMagnitude;
        }

        /// <summary>
        /// Calculates rotational kinetic energy
        /// Formula: RE = 0.5 * ω ⋅ (I * ω)
        ///   where I is the inertia tensor
        /// </summary>
        public static float CalculateRotationalEnergy(Vector3 angularVelocity, Matrix4x4 inertiaTensor)
        {
            Vector3 L = MathUtils.MultiplyMatrixVector3(inertiaTensor, angularVelocity);
            return 0.5f * Vector3.Dot(angularVelocity, L);
        }

        /// <summary>
        /// Calculates gravitational potential energy
        /// Formula: PE = m * g * h
        ///   where h is the height (y position)
        /// </summary>
        public static float CalculatePotentialEnergy(Vector3 position, float mass, float gravity = PhysicsConstants.GRAVITY)
        {
            return mass * gravity * position.y;
        }
        #endregion
    }
}
