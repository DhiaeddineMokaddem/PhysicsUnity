using UnityEngine;
using System.Collections.Generic;

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
/// </summary>
public class CollisionDetector : MonoBehaviour
{
    [Header("Paramètres de Collision")]
    public float collisionTolerance = 0.01f;
    public int solverIterations = 4;
    
    private List<CollisionInfo> collisions = new List<CollisionInfo>();

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
            float projectionB = ProjectBoxOntoAxis(halfSizeB, axesB, axis);
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
            float projectionA = ProjectBoxOntoAxis(halfSizeA, axesA, axis);
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
                if (axis.sqrMagnitude < 0.0001f) continue;
                axis.Normalize();
                
                float projectionA = ProjectBoxOntoAxis(halfSizeA, axesA, axis);
                float projectionB = ProjectBoxOntoAxis(halfSizeB, axesB, axis);
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

    private float ProjectBoxOntoAxis(Vector3 halfSize, Vector3[] axes, Vector3 axis)
    {
        return halfSize.x * Mathf.Abs(Vector3.Dot(axes[0], axis)) +
               halfSize.y * Mathf.Abs(Vector3.Dot(axes[1], axis)) +
               halfSize.z * Mathf.Abs(Vector3.Dot(axes[2], axis));
    }

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
            
            if (dist < 0.001f)
            {
                Vector3 cubeToSphere = sphereCenter - cube.position;
                if (cubeToSphere.sqrMagnitude < 0.001f)
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
        
        if (velAlongNormal > -0.01f) return;
        
        float restitution = elasticity;
        if (bodyA != null && bodyB != null)
            restitution = (bodyA.restitution + bodyB.restitution) * 0.5f * elasticity;
        else if (bodyA != null)
            restitution = bodyA.restitution * elasticity;
        else if (bodyB != null)
            restitution = bodyB.restitution * elasticity;
        
        float massA = bodyA != null && !bodyA.isKinematic ? bodyA.mass : float.MaxValue;
        float massB = bodyB != null && !bodyB.isKinematic ? bodyB.mass : float.MaxValue;
        
        float invMassA = bodyA != null && !bodyA.isKinematic ? 1.0f / massA : 0f;
        float invMassB = bodyB != null && !bodyB.isKinematic ? 1.0f / massB : 0f;
        
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
        if (collision.penetrationDepth <= 0.001f) return;
        
        RigidBody3D bodyA = collision.bodyA;
        RigidBody3D bodyB = collision.bodyB;
        
        float invMassA = bodyA != null && !bodyA.isKinematic ? 1.0f / bodyA.mass : 0f;
        float invMassB = bodyB != null && !bodyB.isKinematic ? 1.0f / bodyB.mass : 0f;
        float totalInvMass = invMassA + invMassB;
        
        if (totalInvMass <= 0) return;
        
        float separationFactor = 1.02f;
        Vector3 correction = collision.contactNormal * (collision.penetrationDepth * separationFactor) / totalInvMass;
        
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

    private void ApplyFriction(CollisionInfo collision, Vector3 normalImpulse)
    {
        RigidBody3D bodyA = collision.bodyA;
        RigidBody3D bodyB = collision.bodyB;
        
        Vector3 velA = bodyA != null ? bodyA.GetVelocityAtPoint(collision.contactPoint) : Vector3.zero;
        Vector3 velB = bodyB != null ? bodyB.GetVelocityAtPoint(collision.contactPoint) : Vector3.zero;
        Vector3 relativeVel = velB - velA;
        
        Vector3 tangent = relativeVel - collision.contactNormal * Vector3.Dot(relativeVel, collision.contactNormal);
        
        if (tangent.sqrMagnitude < 0.0001f) return;
        
        tangent.Normalize();
        
        float frictionCoeff = 0.3f;
        if (bodyA != null && bodyB != null)
            frictionCoeff = (bodyA.friction + bodyB.friction) * 0.5f;
        else if (bodyA != null)
            frictionCoeff = bodyA.friction;
        else if (bodyB != null)
            frictionCoeff = bodyB.friction;
        
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
}