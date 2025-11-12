using UnityEngine;
using System.Collections.Generic;
using PhysicsUnity.Core;
using PhysicsUnity.Indiv_Work.Aziz;

/// <summary>
/// Gestionnaire principal de la simulation physique - VERSION PURE MATH
/// FIXED: Better handling of null references and body registration
/// Refactored to use shared utilities from PhysicsUnity.Core
/// </summary>
public class PhysicsManager : MonoBehaviour
{
    #region Inspector Properties
    [Header("Configuration")]
    public float timeStep = PhysicsConstants.DEFAULT_TIME_STEP;
    public int substeps = PhysicsConstants.DEFAULT_SUBSTEPS;
    public float globalElasticity = 0.8f;
    
    [Header("Sol")]
    public float groundLevel = 0f;
    public float groundRestitution = 0.2f;
    public float groundFriction = 0.6f;
    
    [Header("Debugging")]
    public bool showDebugInfo = true;
    public bool pauseSimulation = false;
    #endregion

    #region Private Fields
    private List<RigidBody3D> rigidBodies = new List<RigidBody3D>();
    private List<RigidConstraint> constraints = new List<RigidConstraint>();
    private CollisionDetector collisionDetector;
    private float accumulator = 0f;
    #endregion

    #region Initialization
    void Start()
    {
        collisionDetector = gameObject.AddComponent<CollisionDetector>();
        RegisterAllBodies();
    }

    public void RegisterAllBodies()
    {
        rigidBodies.RemoveAll(body => body == null);
        constraints.RemoveAll(constraint => constraint == null);

        RigidBody3D[] foundBodies = FindObjectsOfType<RigidBody3D>();
        RigidConstraint[] foundConstraints = FindObjectsOfType<RigidConstraint>();

        foreach (var body in foundBodies)
            if (!rigidBodies.Contains(body)) rigidBodies.Add(body);

        foreach (var constraint in foundConstraints)
            if (!constraints.Contains(constraint)) constraints.Add(constraint);
        
        Debug.Log($"PhysicsManager: {rigidBodies.Count} corps rigides et {constraints.Count} contraintes enregistrés");
    }

    public void RegisterBody(RigidBody3D body)
    {
        if (!rigidBodies.Contains(body)) rigidBodies.Add(body);
    }

    public void RegisterConstraint(RigidConstraint constraint)
    {
        if (!constraints.Contains(constraint)) constraints.Add(constraint);
    }
    #endregion

    #region Physics Update
    void FixedUpdate()
    {
        if (pauseSimulation) return;

        rigidBodies.RemoveAll(body => body == null);
        constraints.RemoveAll(constraint => constraint == null);

        float deltaTime = timeStep / substeps;

        for (int i = 0; i < substeps; i++)
        {
            SolveConstraints(deltaTime);
            IntegratePhysics(deltaTime);
            DetectAndResolveCollisions();
            HandleGroundCollisions();
        }
    }

    void SolveConstraints(float deltaTime)
    {
        foreach (var constraint in constraints)
            if (constraint != null && !constraint.isBroken)
                constraint.SolveConstraint(deltaTime);
    }

    void IntegratePhysics(float deltaTime)
    {
        foreach (var body in rigidBodies)
            if (body != null)
                body.IntegratePhysics(deltaTime);
    }

    void DetectAndResolveCollisions()
    {
        for (int i = 0; i < rigidBodies.Count; i++)
        {
            for (int j = i + 1; j < rigidBodies.Count; j++)
            {
                if (rigidBodies[i] == null || rigidBodies[j] == null) continue;

                CollisionInfo collision;
                if (collisionDetector.DetectCubeCollision(rigidBodies[i], rigidBodies[j], out collision))
                {
                    collisionDetector.ResolveCollision(collision, globalElasticity);
                }
            }
        }
    }

    void HandleGroundCollisions()
    {
        foreach (var body in rigidBodies)
        {
            if (body == null || body.isKinematic) continue;

            Vector3 pos = body.position;
            float halfHeight = body.size.y * 0.5f;
            
            Vector3 contactPoint;
            float penetration;
            if (CollisionUtils.CheckGroundCollision(pos, halfHeight, groundLevel, out contactPoint, out penetration))
            {
                body.position = new Vector3(pos.x, groundLevel + halfHeight, pos.z);
                body.UpdateVisualTransform();

                if (body.velocity.y < 0)
                {
                    body.velocity.y = -body.velocity.y * groundRestitution * globalElasticity;

                    Vector3 horizontalVel = new Vector3(body.velocity.x, 0, body.velocity.z);
                    horizontalVel *= (1f - groundFriction);
                    body.velocity = new Vector3(horizontalVel.x, body.velocity.y, horizontalVel.z);

                    body.angularVelocity *= (1f - groundFriction);
                }
            }
        }
    }
    #endregion

    #region Special Effects
    public void ApplyExplosion(Vector3 center, float radius, float force)
    {
        foreach (var body in rigidBodies)
        {
            if (body == null || body.isKinematic) continue;

            Vector3 direction = body.position - center;
            float distance = direction.magnitude;

            if (distance < radius && distance > PhysicsConstants.SEPARATION_THRESHOLD)
            {
                float falloff = 1f - (distance / radius);
                falloff = falloff * falloff;

                Vector3 explosionDir = direction.normalized;
                Vector3 explosionForce = explosionDir * force * falloff;

                body.AddImpulse(explosionForce / body.mass);

                Vector3 randomTorque = new Vector3(
                    Random.Range(-0.5f, 0.5f),
                    Random.Range(-0.5f, 0.5f),
                    Random.Range(-0.5f, 0.5f)
                ) * force * falloff * 0.01f;
                body.AddTorque(randomTorque);
            }
        }
    }

    public void BreakConstraintsInRadius(Vector3 center, float radius)
    {
        foreach (var constraint in constraints)
        {
            if (constraint == null || constraint.isBroken) continue;

            Vector3 constraintPos = constraint.transform.position;
            float distance = Vector3.Distance(constraintPos, center);

            if (distance < radius) constraint.Break();
        }
    }
    #endregion

    #region Simulation Control
    public void ResetSimulation()
    {
        foreach (var constraint in constraints)
            if (constraint != null) constraint.Repair();

        foreach (var body in rigidBodies)
        {
            if (body != null)
            {
                body.velocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
    }

    public string GetSimulationStats()
    {
        int activeBodies = 0;
        int activeConstraints = 0;
        float totalEnergy = 0f;

        foreach (var body in rigidBodies)
        {
            if (body != null && !body.isKinematic)
            {
                activeBodies++;
                totalEnergy += body.GetKineticEnergy();
            }
        }

        foreach (var constraint in constraints)
            if (constraint != null && !constraint.isBroken) activeConstraints++;

        return $"Corps actifs: {activeBodies}\n" +
               $"Contraintes actives: {activeConstraints}/{constraints.Count}\n" +
               $"Énergie cinétique totale: {totalEnergy:F2} J\n" +
               $"Élasticité globale: {globalElasticity:F2}";
    }
    #endregion

    #region Debug Visualization
    void OnDrawGizmos()
    {
        if (!showDebugInfo) return;
        Gizmos.color = Color.gray;
        Gizmos.DrawWireCube(new Vector3(0, groundLevel - 0.5f, 0), new Vector3(50, 1, 50));
    }
    #endregion
}
