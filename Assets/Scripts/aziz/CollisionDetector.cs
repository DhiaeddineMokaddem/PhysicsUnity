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
/// Système de détection et résolution de collisions
/// </summary>
public class CollisionDetector : MonoBehaviour
{
    [Header("Paramètres de Collision")]
    public float collisionTolerance = 0.01f; // Augmenté pour réduire faux positifs
    public int solverIterations = 18;
    
    private List<CollisionInfo> collisions = new List<CollisionInfo>();

    /// <summary>
    /// Détecte les collisions entre deux cubes (AABB orienté)
    /// </summary>
    public bool DetectCubeCollision(RigidBody3D cubeA, RigidBody3D cubeB, out CollisionInfo collision)
    {
        collision = new CollisionInfo();
        
        // Utiliser le test des axes séparés (SAT) pour AABB orientés
        Vector3 centerA = cubeA.transform.position;
        Vector3 centerB = cubeB.transform.position;
        
        Vector3 delta = centerB - centerA;
        
        // Axes du cube A
        Vector3[] axesA = new Vector3[]
        {
            cubeA.transform.right,
            cubeA.transform.up,
            cubeA.transform.forward
        };
        
        // Axes du cube B
        Vector3[] axesB = new Vector3[]
        {
            cubeB.transform.right,
            cubeB.transform.up,
            cubeB.transform.forward
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
        
        // Tester les produits vectoriels des axes (9 axes supplémentaires)
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
        
        // Point de contact approximatif (au centre de la zone de chevauchement)
        Vector3 contactPoint = centerA + delta * 0.5f;
        
        collision = new CollisionInfo(cubeA, cubeB, contactPoint, minAxis, minPenetration);
        return true;
    }

    /// <summary>
    /// Projette une boîte orientée sur un axe
    /// </summary>
    private float ProjectBoxOntoAxis(Vector3 halfSize, Vector3[] axes, Vector3 axis)
    {
        return halfSize.x * Mathf.Abs(Vector3.Dot(axes[0], axis)) +
               halfSize.y * Mathf.Abs(Vector3.Dot(axes[1], axis)) +
               halfSize.z * Mathf.Abs(Vector3.Dot(axes[2], axis));
    }

    /// <summary>
    /// Détecte la collision entre une sphère et un cube
    /// </summary>
    public bool DetectSphereCollision(Vector3 sphereCenter, float sphereRadius, RigidBody3D cube, out CollisionInfo collision)
    {
        collision = new CollisionInfo();
        if (cube == null || cube.isKinematic) return false;
        
        // Trouver le point le plus proche sur le cube
        Vector3 localPoint = cube.transform.InverseTransformPoint(sphereCenter);
        Vector3 halfSize = cube.size * 0.5f;
        
        // Clamper aux limites du cube
        Vector3 closestLocal = new Vector3(
            Mathf.Clamp(localPoint.x, -halfSize.x, halfSize.x),
            Mathf.Clamp(localPoint.y, -halfSize.y, halfSize.y),
            Mathf.Clamp(localPoint.z, -halfSize.z, halfSize.z)
        );
        
        Vector3 closestWorld = cube.transform.TransformPoint(closestLocal);
        Vector3 delta = sphereCenter - closestWorld;
        float distSq = delta.sqrMagnitude;
        
        if (distSq < sphereRadius * sphereRadius + collisionTolerance)
        {
            float dist = Mathf.Sqrt(distSq);
            Vector3 normal;
            float penetration;
            
            // Si la sphère est à l'intérieur du cube (dist très petit)
            if (dist < 0.001f)
            {
                // Utiliser la direction du centre du cube vers la sphère
                Vector3 cubeToSphere = sphereCenter - cube.transform.position;
                if (cubeToSphere.sqrMagnitude < 0.001f)
                {
                    cubeToSphere = Vector3.up; // Fallback
                }
                normal = cubeToSphere.normalized;
                penetration = sphereRadius;
            }
            else
            {
                // Normal pointe du point de contact vers le centre de la sphère
                normal = delta / dist;
                penetration = sphereRadius - dist;
            }
            
            collision = new CollisionInfo(null, cube, closestWorld, normal, penetration);
            return true;
        }
        
        return false;
    }

    /// <summary>
    /// Résout une collision en appliquant des impulsions
    /// </summary>
    public void ResolveCollision(CollisionInfo collision, float elasticity)
    {
        RigidBody3D bodyA = collision.bodyA;
        RigidBody3D bodyB = collision.bodyB;
        
        if (bodyA != null && bodyA.isKinematic && bodyB != null && bodyB.isKinematic)
            return;
        
        Vector3 normal = collision.contactNormal;
        Vector3 contactPoint = collision.contactPoint;
        
        // Résoudre d'abord la pénétration pour éviter que les objets restent coincés
        ResolvePenetration(collision);
        
        // Calculer les vitesses relatives au point de contact
        Vector3 velA = bodyA != null ? bodyA.GetVelocityAtPoint(contactPoint) : Vector3.zero;
        Vector3 velB = bodyB != null ? bodyB.GetVelocityAtPoint(contactPoint) : Vector3.zero;
        Vector3 relativeVel = velB - velA;
        
        float velAlongNormal = Vector3.Dot(relativeVel, normal);
        
        // Ne résoudre que si les objets s'approchent (avec une petite tolérance)
        if (velAlongNormal > -0.01f) return;
        
        // Calculer le coefficient de restitution moyen
        float restitution = elasticity;
        if (bodyA != null && bodyB != null)
            restitution = (bodyA.restitution + bodyB.restitution) * 0.5f * elasticity;
        else if (bodyA != null)
            restitution = bodyA.restitution * elasticity;
        else if (bodyB != null)
            restitution = bodyB.restitution * elasticity;
        
        // Calculer l'impulsion
        float massA = bodyA != null && !bodyA.isKinematic ? bodyA.mass : float.MaxValue;
        float massB = bodyB != null && !bodyB.isKinematic ? bodyB.mass : float.MaxValue;
        
        float invMassA = bodyA != null && !bodyA.isKinematic ? 1.0f / massA : 0f;
        float invMassB = bodyB != null && !bodyB.isKinematic ? 1.0f / massB : 0f;
        
        float j = -(1 + restitution) * velAlongNormal;
        j /= invMassA + invMassB;
        
        Vector3 impulse = normal * j;
        
        // Appliquer l'impulsion
        if (bodyA != null && !bodyA.isKinematic)
        {
            bodyA.AddImpulseAtPoint(-impulse, contactPoint);
        }
        
        if (bodyB != null && !bodyB.isKinematic)
        {
            bodyB.AddImpulseAtPoint(impulse, contactPoint);
        }
        
        // Friction
        ApplyFriction(collision, impulse);
    }

    /// <summary>
    /// Résout la pénétration en séparant les objets
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
        
        // Ajouter un petit facteur pour garantir la séparation complète
        float separationFactor = 1.1f;
        Vector3 correction = collision.contactNormal * (collision.penetrationDepth * separationFactor) / totalInvMass;
        
        if (bodyA != null && !bodyA.isKinematic)
        {
            bodyA.transform.position -= correction * invMassA;
        }
        
        if (bodyB != null && !bodyB.isKinematic)
        {
            bodyB.transform.position += correction * invMassB;
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
        
        // Composante tangentielle
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