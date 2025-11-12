using UnityEngine;
using System.Collections.Generic;
using PhysicsUnity.Core;
using PhysicsUnity.Indiv_Work.Aziz;

/// <summary>
/// Informations sur une collision
/// </summary>
public struct CollisionInfo
{
    public RigidBody3D bodyA;
    public RigidBody3D bodyB;
    public Vector3 contactPoint;
    public Vector3 contactNormal;
    public float penetrationDepth;
    
    public CollisionInfo(RigidBody3D a, RigidBody3D b, Vector3 point, Vector3 normal, float depth)
    {
        bodyA = a;
        bodyB = b;
        contactPoint = point;
        contactNormal = normal;
        penetrationDepth = depth;
    }
}

/// <summary>
/// Système de détection et résolution de collisions - VERSION PURE MATH
/// Refactored to use shared utilities from PhysicsUnity.Core
/// </summary>
public class CollisionDetector : MonoBehaviour
{
    #region Configuration
    [Header("Paramètres de Collision")]
    public float collisionTolerance = PhysicsConstants.COLLISION_TOLERANCE;
    public int solverIterations = PhysicsConstants.DEFAULT_SOLVER_ITERATIONS;
    #endregion
    
    #region Private Fields
    private List<CollisionInfo> collisions = new List<CollisionInfo>();
    #endregion

    #region Cube Collision Detection
    /// <summary>
    /// Détecte les collisions entre deux cubes (AABB orienté) - PURE MATH
    /// </summary>
    public bool DetectCubeCollision(RigidBody3D cubeA, RigidBody3D cubeB, out CollisionInfo collision)
    {
        collision = new CollisionInfo();
        
        // PURE MATH: Utiliser les positions et rotations stockées
        Vector3 centerA = cubeA.position;
        Vector3 centerB = cubeB.position;
        
        Vector3 delta = centerB - centerA;
        
        // PURE MATH: Axes calculés depuis les quaternions
        Vector3[] axesA = new Vector3[]
        {
            cubeA.GetRight(),
            cubeA.GetUp(),
            cubeA.GetForward()
        };
        
        Vector3[] axesB = new Vector3[]
        {
            cubeB.GetRight(),
            cubeB.GetUp(),
            cubeB.GetForward()
        };
        
        Vector3 halfSizeA = cubeA.size * 0.5f;
        Vector3 halfSizeB = cubeB.size * 0.5f;
        
        float minPenetration = float.MaxValue;
        Vector3 minAxis = Vector3.zero;
        
        // Tester les axes du cube A
        for (int i = 0; i < 3; i++)
        {
            Vector3 axis = axesA[i];
            float projectionA = halfSizeA[i];
            float projectionB = MathUtils.ProjectBoxOntoAxis(halfSizeB, axesB, axis);
            float distance = Mathf.Abs(Vector3.Dot(delta, axis));
            float penetration = projectionA + projectionB - distance;
            
            if (penetration < -collisionTolerance)
                return false;
                
            if (penetration < minPenetration)
            {
                minPenetration = penetration;
                minAxis = axis;
            }
        }
        
        // Tester les axes du cube B
        for (int i = 0; i < 3; i++)
        {
            Vector3 axis = axesB[i];
            float projectionA = MathUtils.ProjectBoxOntoAxis(halfSizeA, axesA, axis);
            float projectionB = halfSizeB[i];
            float distance = Mathf.Abs(Vector3.Dot(delta, axis));
            float penetration = projectionA + projectionB - distance;
            
            if (penetration < -collisionTolerance)
                return false;
                
            if (penetration < minPenetration)
            {
                minPenetration = penetration;
                minAxis = axis;
            }
        }
        
        // Tester les produits vectoriels des axes
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Vector3 axis = Vector3.Cross(axesA[i], axesB[j]);
                if (axis.sqrMagnitude < PhysicsConstants.EPSILON_SMALL) continue;
                axis.Normalize();
                
                float projectionA = MathUtils.ProjectBoxOntoAxis(halfSizeA, axesA, axis);
                float projectionB = MathUtils.ProjectBoxOntoAxis(halfSizeB, axesB, axis);
                float distance = Mathf.Abs(Vector3.Dot(delta, axis));
                float penetration = projectionA + projectionB - distance;
                
                if (penetration < -collisionTolerance)
                    return false;
                    
                if (penetration < minPenetration)
                {
                    minPenetration = penetration;
                    minAxis = axis;
                }
            }
        }
        
        // S'assurer que la normale pointe de A vers B
        if (Vector3.Dot(minAxis, delta) < 0)
            minAxis = -minAxis;
        
        Vector3 contactPoint = centerA + delta * 0.5f;
        
        collision = new CollisionInfo(cubeA, cubeB, contactPoint, minAxis, minPenetration);
        return true;
    }
    #endregion

    #region Sphere Collision Detection
    /// <summary>
    /// Détecte la collision entre une sphère et un cube - PURE MATH
    /// </summary>
    public bool DetectSphereCollision(Vector3 sphereCenter, float sphereRadius, RigidBody3D cube, out CollisionInfo collision)
    {
        collision = new CollisionInfo();
        
        // PURE MATH: Transformation manuelle
        Vector3 localPoint = cube.InverseTransformPoint(sphereCenter);
        Vector3 halfSize = cube.size * 0.5f;
        
        Vector3 closestLocal = new Vector3(
            Mathf.Clamp(localPoint.x, -halfSize.x, halfSize.x),
            Mathf.Clamp(localPoint.y, -halfSize.y, halfSize.y),
            Mathf.Clamp(localPoint.z, -halfSize.z, halfSize.z)
        );
        
        Vector3 closestWorld = cube.TransformPoint(closestLocal);
        Vector3 delta = sphereCenter - closestWorld;
        float distSq = delta.sqrMagnitude;
        
        if (distSq < sphereRadius * sphereRadius + collisionTolerance)
        {
            float dist = Mathf.Sqrt(distSq);
            Vector3 normal;
            float penetration;
            
            if (dist < PhysicsConstants.SEPARATION_THRESHOLD)
            {
                Vector3 cubeToSphere = sphereCenter - cube.position;
                if (cubeToSphere.sqrMagnitude < PhysicsConstants.SEPARATION_THRESHOLD)
                {
                    cubeToSphere = Vector3.up;
                }
                normal = cubeToSphere.normalized;
                penetration = sphereRadius;
            }
            else
            {
                normal = delta / dist;
                penetration = sphereRadius - dist;
            }
            
            collision = new CollisionInfo(null, cube, closestWorld, normal, penetration);
            return true;
        }
        
        return false;
    }
    #endregion

    #region Collision Resolution
    /// <summary>
    /// Résout une collision avec réponse impulsive
    /// </summary>
    public void ResolveCollision(CollisionInfo collision, float elasticity)
    {
        RigidBody3D bodyA = collision.bodyA;
        RigidBody3D bodyB = collision.bodyB;
        
        if (bodyA != null && bodyA.isKinematic && bodyB != null && bodyB.isKinematic)
            return;
        
        Vector3 normal = collision.contactNormal;
        Vector3 contactPoint = collision.contactPoint;
        
        ResolvePenetration(collision);
        
        Vector3 velA = bodyA != null ? bodyA.GetVelocityAtPoint(contactPoint) : Vector3.zero;
        Vector3 velB = bodyB != null ? bodyB.GetVelocityAtPoint(contactPoint) : Vector3.zero;
        Vector3 relativeVel = velB - velA;
        
        float velAlongNormal = Vector3.Dot(relativeVel, normal);
        
        if (velAlongNormal > -PhysicsConstants.MIN_COLLISION_VELOCITY) return;
        
        float restitution = CalculateCombinedRestitution(bodyA, bodyB, elasticity);
        float invMassA = CalculateInverseMass(bodyA);
        float invMassB = CalculateInverseMass(bodyB);
        
        float j = -(1 + restitution) * velAlongNormal;
        j /= invMassA + invMassB;
        
        Vector3 impulse = normal * j;
        
        if (bodyA != null && !bodyA.isKinematic)
        {
            bodyA.AddImpulseAtPoint(-impulse, contactPoint);
        }
        
        if (bodyB != null && !bodyB.isKinematic)
        {
            bodyB.AddImpulseAtPoint(impulse, contactPoint);
        }
        
        ApplyFriction(collision, impulse);
    }

    /// <summary>
    /// Résout la pénétration - PURE MATH
    /// </summary>
    private void ResolvePenetration(CollisionInfo collision)
    {
        if (collision.penetrationDepth <= PhysicsConstants.SEPARATION_THRESHOLD) return;
        
        RigidBody3D bodyA = collision.bodyA;
        RigidBody3D bodyB = collision.bodyB;
        
        float invMassA = CalculateInverseMass(bodyA);
        float invMassB = CalculateInverseMass(bodyB);
        float totalInvMass = invMassA + invMassB;
        
        if (totalInvMass <= 0) return;
        
        Vector3 correction = collision.contactNormal * (collision.penetrationDepth * PhysicsConstants.SEPARATION_FACTOR) / totalInvMass;
        
        // PURE MATH: Modifier directement la position stockée
        if (bodyA != null && !bodyA.isKinematic)
        {
            bodyA.position -= correction * invMassA;
            bodyA.UpdateVisualTransform();
        }
        
        if (bodyB != null && !bodyB.isKinematic)
        {
            bodyB.position += correction * invMassB;
            bodyB.UpdateVisualTransform();
        }
    }

    /// <summary>
    /// Applique la friction tangentielle
    /// </summary>
    private void ApplyFriction(CollisionInfo collision, Vector3 normalImpulse)
    {
        RigidBody3D bodyA = collision.bodyA;
        RigidBody3D bodyB = collision.bodyB;
        
        Vector3 velA = bodyA != null ? bodyA.GetVelocityAtPoint(collision.contactPoint) : Vector3.zero;
        Vector3 velB = bodyB != null ? bodyB.GetVelocityAtPoint(collision.contactPoint) : Vector3.zero;
        Vector3 relativeVel = velB - velA;
        
        Vector3 tangent = relativeVel - collision.contactNormal * Vector3.Dot(relativeVel, collision.contactNormal);
        
        if (tangent.sqrMagnitude < PhysicsConstants.EPSILON_SMALL) return;
        
        tangent.Normalize();
        
        float frictionCoeff = CalculateCombinedFriction(bodyA, bodyB);
        
        float frictionImpulse = -Vector3.Dot(relativeVel, tangent);
        frictionImpulse = Mathf.Clamp(frictionImpulse, -normalImpulse.magnitude * frictionCoeff, 
                                                        normalImpulse.magnitude * frictionCoeff);
        
        Vector3 frictionVector = tangent * frictionImpulse;
        
        if (bodyA != null && !bodyA.isKinematic)
        {
            bodyA.AddImpulseAtPoint(-frictionVector, collision.contactPoint);
        }
        
        if (bodyB != null && !bodyB.isKinematic)
        {
            bodyB.AddImpulseAtPoint(frictionVector, collision.contactPoint);
        }
    }
    #endregion

    #region Helper Methods
    /// <summary>
    /// Calcule la restitution combinée avec le facteur d'élasticité global
    /// </summary>
    private float CalculateCombinedRestitution(RigidBody3D bodyA, RigidBody3D bodyB, float elasticity)
    {
        float restitution = elasticity;
        if (bodyA != null && bodyB != null)
            restitution = (bodyA.restitution + bodyB.restitution) * 0.5f * elasticity;
        else if (bodyA != null)
            restitution = bodyA.restitution * elasticity;
        else if (bodyB != null)
            restitution = bodyB.restitution * elasticity;
        
        return restitution;
    }

    /// <summary>
    /// Calcule la friction combinée entre deux corps
    /// </summary>
    private float CalculateCombinedFriction(RigidBody3D bodyA, RigidBody3D bodyB)
    {
        float frictionCoeff = PhysicsConstants.DEFAULT_FRICTION;
        if (bodyA != null && bodyB != null)
            frictionCoeff = CollisionUtils.CombineFriction(bodyA.friction, bodyB.friction);
        else if (bodyA != null)
            frictionCoeff = bodyA.friction;
        else if (bodyB != null)
            frictionCoeff = bodyB.friction;
        
        return frictionCoeff;
    }

    /// <summary>
    /// Calcule la masse inverse d'un corps
    /// </summary>
    private float CalculateInverseMass(RigidBody3D body)
    {
        return body != null && !body.isKinematic ? 1.0f / body.mass : 0f;
    }
    #endregion
}