using UnityEngine;

namespace PhysicsUnity.Core
{
    /// <summary>
    /// Utility class for physics integration methods
    /// Provides various numerical integration schemes for position, velocity, and rotation
    /// </summary>
    public static class IntegrationUtils
    {
        #region Euler Integration
        /// <summary>
        /// Performs explicit Euler integration for position
        /// </summary>
        public static Vector3 IntegratePositionEuler(Vector3 position, Vector3 velocity, float deltaTime)
        {
            return position + velocity * deltaTime;
        }

        /// <summary>
        /// Performs explicit Euler integration for velocity
        /// </summary>
        public static Vector3 IntegrateVelocityEuler(Vector3 velocity, Vector3 acceleration, float deltaTime)
        {
            return velocity + acceleration * deltaTime;
        }
        #endregion

        #region Verlet Integration
        /// <summary>
        /// Performs Verlet integration (position-based)
        /// </summary>
        public static Vector3 IntegratePositionVerlet(Vector3 currentPosition, Vector3 previousPosition, Vector3 acceleration, float deltaTime)
        {
            return 2f * currentPosition - previousPosition + acceleration * deltaTime * deltaTime;
        }

        /// <summary>
        /// Calculates velocity from Verlet positions
        /// </summary>
        public static Vector3 CalculateVerletVelocity(Vector3 currentPosition, Vector3 previousPosition, float deltaTime)
        {
            if (deltaTime < PhysicsConstants.EPSILON)
                return Vector3.zero;
            return (currentPosition - previousPosition) / deltaTime;
        }
        #endregion

        #region Semi-Implicit Euler (Symplectic Euler)
        /// <summary>
        /// Performs semi-implicit Euler integration (velocity first, then position)
        /// More stable than explicit Euler for physics simulations
        /// </summary>
        public static void IntegrateSemiImplicitEuler(
            ref Vector3 position,
            ref Vector3 velocity,
            Vector3 acceleration,
            float deltaTime)
        {
            velocity += acceleration * deltaTime;
            position += velocity * deltaTime;
        }
        #endregion

        #region Rotation Integration
        /// <summary>
        /// Integrates angular velocity using quaternions
        /// </summary>
        public static Quaternion IntegrateRotationQuaternion(Quaternion rotation, Vector3 angularVelocity, float deltaTime)
        {
            if (angularVelocity.sqrMagnitude < PhysicsConstants.EPSILON_SMALL)
                return rotation;
            
            // Convert angular velocity to quaternion derivative
            Quaternion angularQuat = new Quaternion(
                angularVelocity.x * 0.5f,
                angularVelocity.y * 0.5f,
                angularVelocity.z * 0.5f,
                0f
            );
            
            // Integrate: q' = q + dt * (w * q) / 2
            Quaternion derivative = angularQuat * rotation;
            Quaternion newRotation = new Quaternion(
                rotation.x + derivative.x * deltaTime,
                rotation.y + derivative.y * deltaTime,
                rotation.z + derivative.z * deltaTime,
                rotation.w + derivative.w * deltaTime
            );
            
            return newRotation.normalized;
        }

        /// <summary>
        /// Integrates rotation using rotation matrix and angular velocity
        /// </summary>
        public static Matrix4x4 IntegrateRotationMatrix(Matrix4x4 rotation, Vector3 angularVelocity, float deltaTime)
        {
            if (angularVelocity.sqrMagnitude < PhysicsConstants.EPSILON_SMALL)
                return rotation;
            
            Matrix4x4 angularMatrix = MathUtils.SkewSymmetric(angularVelocity);
            Matrix4x4 derivative = MathUtils.MultiplyMatrix3x3(angularMatrix, rotation);
            
            Matrix4x4 result = Matrix4x4.zero;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    result[i, j] = rotation[i, j] + derivative[i, j] * deltaTime;
                }
            }
            result.m33 = 1f;
            
            return result;
        }
        #endregion

        #region RK4 Integration
        /// <summary>
        /// Performs Runge-Kutta 4th order integration for position and velocity
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
        /// </summary>
        public static Vector3 ForceToAcceleration(Vector3 force, float mass)
        {
            if (mass < PhysicsConstants.EPSILON)
                return Vector3.zero;
            return force / mass;
        }

        /// <summary>
        /// Converts acceleration to force using Newton's second law
        /// </summary>
        public static Vector3 AccelerationToForce(Vector3 acceleration, float mass)
        {
            return acceleration * mass;
        }

        /// <summary>
        /// Converts torque to angular acceleration
        /// </summary>
        public static Vector3 TorqueToAngularAcceleration(Vector3 torque, Matrix4x4 inverseInertiaTensor)
        {
            return MathUtils.MultiplyMatrixVector3(inverseInertiaTensor, torque);
        }

        /// <summary>
        /// Converts angular acceleration to torque
        /// </summary>
        public static Vector3 AngularAccelerationToTorque(Vector3 angularAcceleration, Matrix4x4 inertiaTensor)
        {
            return MathUtils.MultiplyMatrixVector3(inertiaTensor, angularAcceleration);
        }
        #endregion

        #region Damping
        /// <summary>
        /// Applies damping to a velocity vector
        /// </summary>
        public static Vector3 ApplyDamping(Vector3 velocity, float damping, float deltaTime)
        {
            float dampingFactor = Mathf.Clamp01(1f - damping * deltaTime);
            return velocity * dampingFactor;
        }

        /// <summary>
        /// Applies exponential damping to a velocity vector (more physically accurate)
        /// </summary>
        public static Vector3 ApplyExponentialDamping(Vector3 velocity, float damping, float deltaTime)
        {
            return velocity * Mathf.Exp(-damping * deltaTime);
        }
        #endregion

        #region Constraint Projection
        /// <summary>
        /// Projects velocity to satisfy a distance constraint between two points
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
        /// </summary>
        public static Vector3 CalculateCriticallyDampedSpring(
            Vector3 pos1, Vector3 pos2,
            float restLength,
            float stiffness,
            Vector3 vel1, Vector3 vel2,
            float mass)
        {
            float criticalDamping = 2f * Mathf.Sqrt(stiffness * mass);
            return CalculateSpringForce(pos1, pos2, restLength, stiffness, criticalDamping, vel1, vel2);
        }
        #endregion

        #region Energy Calculations
        /// <summary>
        /// Calculates kinetic energy from velocity and mass
        /// </summary>
        public static float CalculateKineticEnergy(Vector3 velocity, float mass)
        {
            return 0.5f * mass * velocity.sqrMagnitude;
        }

        /// <summary>
        /// Calculates rotational kinetic energy
        /// </summary>
        public static float CalculateRotationalEnergy(Vector3 angularVelocity, Matrix4x4 inertiaTensor)
        {
            Vector3 L = MathUtils.MultiplyMatrixVector3(inertiaTensor, angularVelocity);
            return 0.5f * Vector3.Dot(angularVelocity, L);
        }

        /// <summary>
        /// Calculates gravitational potential energy
        /// </summary>
        public static float CalculatePotentialEnergy(Vector3 position, float mass, float gravity = PhysicsConstants.GRAVITY)
        {
            return mass * gravity * position.y;
        }
        #endregion
    }
}
