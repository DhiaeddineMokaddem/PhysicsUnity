using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Système de détection et résolution de collisions personnalisé
/// </summary>
public class CustomCollisionSystem
{
    public struct CollisionContact
    {
        public CustomRigidBody bodyA;
        public CustomRigidBody bodyB;
        public Vector3 point;
        public Vector3 normal;
        public float penetration;

        public CollisionContact(CustomRigidBody a, CustomRigidBody b, Vector3 p, Vector3 n, float depth)
        {
            bodyA = a;
            bodyB = b;
            point = p;
            normal = n;
            penetration = depth;
        }
    }

    private float restitution = 0.4f;
    private float friction = 0.3f;
    private List<CollisionContact> contacts = new List<CollisionContact>();

    /// <summary>
    /// Détecte les collisions entre deux boîtes orientées
    /// </summary>
    public bool DetectBoxCollision(CustomRigidBody boxA, CustomRigidBody boxB, out CollisionContact contact)
    {
        contact = new CollisionContact();

        Vector3 centerA = boxA.position;
        Vector3 centerB = boxB.position;
        Vector3 delta = centerB - centerA;

        // Axes de séparation
        Vector3[] axesA = new Vector3[] { boxA.GetRight(), boxA.GetUp(), boxA.GetForward() };
        Vector3[] axesB = new Vector3[] { boxB.GetRight(), boxB.GetUp(), boxB.GetForward() };

        Vector3 halfSizeA = boxA.size * 0.5f;
        Vector3 halfSizeB = boxB.size * 0.5f;

        float minPenetration = float.MaxValue;
        Vector3 minAxis = Vector3.zero;

        // Test des 15 axes de séparation (SAT)
        // 6 axes des faces
        for (int i = 0; i < 3; i++)
        {
            if (!TestAxis(axesB[i], delta, ProjectOntoAxis(halfSizeA, axesA, axesB[i]), halfSizeB[i], ref minPenetration, ref minAxis))
                return false;
        }

        for (int i = 0; i < 3; i++)
        {
            if (!TestAxis(axesB[i], delta, ProjectOntoAxis(halfSizeA, axesA, axesB[i]), halfSizeB[i], ref minPenetration, ref minAxis))
                return false;
        }

        // 9 axes des arêtes (produits vectoriels)
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Vector3 axis = Vector3.Cross(axesA[i], axesB[j]);
                if (axis.sqrMagnitude < 0.0001f) continue;
                axis.Normalize();

                float projA = ProjectOntoAxis(halfSizeA, axesA, axis);
                float projB = ProjectOntoAxis(halfSizeB, axesB, axis);

                if (!TestAxis(axis, delta, projA, projB, ref minPenetration, ref minAxis))
                    return false;
            }
        }

        // S'assurer que la normale pointe de A vers B
        if (Vector3.Dot(minAxis, delta) < 0)
            minAxis = -minAxis;

        Vector3 contactPoint = centerA + delta * 0.5f;
        contact = new CollisionContact(boxA, boxB, contactPoint, minAxis, minPenetration);
        return true;
    }

    /// <summary>
    /// Teste un axe de séparation
    /// </summary>
    bool TestAxis(Vector3 axis, Vector3 delta, float projA, float projB, ref float minPenetration, ref Vector3 minAxis)
    {
        float distance = Mathf.Abs(Vector3.Dot(delta, axis));
        float penetration = projA + projB - distance;

        if (penetration < -0.01f)
            return false;

        if (penetration < minPenetration)
        {
            minPenetration = penetration;
            minAxis = axis;
        }

        return true;
    }

    /// <summary>
    /// Projette une boîte sur un axe
    /// </summary>
    float ProjectOntoAxis(Vector3 halfSize, Vector3[] axes, Vector3 axis)
    {
        return halfSize.x * Mathf.Abs(Vector3.Dot(axes[0], axis)) +
               halfSize.y * Mathf.Abs(Vector3.Dot(axes[1], axis)) +
               halfSize.z * Mathf.Abs(Vector3.Dot(axes[2], axis));
    }

    /// <summary>
    /// Résout une collision avec impulsion
    /// </summary>
    public void ResolveCollision(CollisionContact contact)
    {
        CustomRigidBody bodyA = contact.bodyA;
        CustomRigidBody bodyB = contact.bodyB;

        if ((bodyA.isStatic && bodyB.isStatic) || bodyA == null || bodyB == null)
            return;

        // Séparation des corps
        ResolvePenetration(contact);

        // Vitesses au point de contact
        Vector3 velA = bodyA.isStatic ? Vector3.zero : bodyA.GetVelocityAtPoint(contact.point);
        Vector3 velB = bodyB.isStatic ? Vector3.zero : bodyB.GetVelocityAtPoint(contact.point);
        Vector3 relativeVel = velB - velA;

        float velAlongNormal = Vector3.Dot(relativeVel, contact.normal);

        // Ne pas résoudre si les objets s'éloignent
        if (velAlongNormal > 0)
            return;

        // Calcul de l'impulsion
        float e = restitution;
        float invMassA = bodyA.isStatic ? 0 : 1.0f / bodyA.mass;
        float invMassB = bodyB.isStatic ? 0 : 1.0f / bodyB.mass;

        Vector3 rA = contact.point - bodyA.position;
        Vector3 rB = contact.point - bodyB.position;

        Vector3 raCrossN = Vector3.Cross(rA, contact.normal);
        Vector3 rbCrossN = Vector3.Cross(rB, contact.normal);

        float angularEffect = 0;
        if (!bodyA.isStatic)
        {
            Matrix4x4 invInertiaA = GetWorldInertiaInverse(bodyA);
            Vector3 temp = MultiplyMatrixVector(invInertiaA, raCrossN);
            angularEffect += Vector3.Dot(contact.normal, Vector3.Cross(temp, rA));
        }
        if (!bodyB.isStatic)
        {
            Matrix4x4 invInertiaB = GetWorldInertiaInverse(bodyB);
            Vector3 temp = MultiplyMatrixVector(invInertiaB, rbCrossN);
            angularEffect += Vector3.Dot(contact.normal, Vector3.Cross(temp, rB));
        }

        float j = -(1 + e) * velAlongNormal;
        j /= (invMassA + invMassB + angularEffect);

        Vector3 impulse = contact.normal * j;

        // Appliquer l'impulsion
        if (!bodyA.isStatic)
        {
            bodyA.velocity -= impulse * invMassA;
            bodyA.angularVelocity -= MultiplyMatrixVector(GetWorldInertiaInverse(bodyA), raCrossN) * j;
        }

        if (!bodyB.isStatic)
        {
            bodyB.velocity += impulse * invMassB;
            bodyB.angularVelocity += MultiplyMatrixVector(GetWorldInertiaInverse(bodyB), rbCrossN) * j;
        }

        // Friction
        ApplyFriction(contact, impulse.magnitude);
    }

    /// <summary>
    /// Résout la pénétration
    /// </summary>
    void ResolvePenetration(CollisionContact contact)
    {
        if (contact.penetration <= 0.001f) return;

        float invMassA = contact.bodyA.isStatic ? 0 : 1.0f / contact.bodyA.mass;
        float invMassB = contact.bodyB.isStatic ? 0 : 1.0f / contact.bodyB.mass;
        float totalInvMass = invMassA + invMassB;

        if (totalInvMass <= 0) return;

        Vector3 correction = contact.normal * (contact.penetration * 1.01f) / totalInvMass;

        if (!contact.bodyA.isStatic)
        {
            contact.bodyA.position -= correction * invMassA;
            contact.bodyA.transform.position = contact.bodyA.position;
        }

        if (!contact.bodyB.isStatic)
        {
            contact.bodyB.position += correction * invMassB;
            contact.bodyB.transform.position = contact.bodyB.position;
        }
    }

    /// <summary>
    /// Applique la friction
    /// </summary>
    void ApplyFriction(CollisionContact contact, float normalImpulseMag)
    {
        Vector3 velA = contact.bodyA.isStatic ? Vector3.zero : contact.bodyA.GetVelocityAtPoint(contact.point);
        Vector3 velB = contact.bodyB.isStatic ? Vector3.zero : contact.bodyB.GetVelocityAtPoint(contact.point);
        Vector3 relativeVel = velB - velA;

        Vector3 tangent = relativeVel - contact.normal * Vector3.Dot(relativeVel, contact.normal);
        if (tangent.sqrMagnitude < 0.0001f) return;

        tangent.Normalize();

        float invMassA = contact.bodyA.isStatic ? 0 : 1.0f / contact.bodyA.mass;
        float invMassB = contact.bodyB.isStatic ? 0 : 1.0f / contact.bodyB.mass;

        float jt = -Vector3.Dot(relativeVel, tangent);
        jt /= (invMassA + invMassB);

        Vector3 frictionImpulse;
        if (Mathf.Abs(jt) < normalImpulseMag * friction)
            frictionImpulse = tangent * jt;
        else
            frictionImpulse = tangent * -normalImpulseMag * friction;

        if (!contact.bodyA.isStatic)
            contact.bodyA.velocity -= frictionImpulse * invMassA;

        if (!contact.bodyB.isStatic)
            contact.bodyB.velocity += frictionImpulse * invMassB;
    }

    /// <summary>
    /// Helpers pour les calculs matriciels
    /// </summary>
    Matrix4x4 GetWorldInertiaInverse(CustomRigidBody body)
    {
        // Cette méthode devrait être accessible depuis CustomRigidBody
        // Pour simplifier, on recalcule ici
        Matrix4x4 R = Matrix4x4.Rotate(body.rotation);
        Matrix4x4 Rt = TransposeMatrix(R);
        
        // Inertie inverse locale (simple pour une boîte)
        float m = body.mass;
        float w = body.size.x, h = body.size.y, d = body.size.z;
        float Ixx = (m / 12.0f) * (h * h + d * d);
        float Iyy = (m / 12.0f) * (w * w + d * d);
        float Izz = (m / 12.0f) * (w * w + h * h);
        
        Matrix4x4 IInv = Matrix4x4.zero;
        IInv.m00 = 1.0f / Ixx;
        IInv.m11 = 1.0f / Iyy;
        IInv.m22 = 1.0f / Izz;
        IInv.m33 = 1;

        return MultiplyMatrices(MultiplyMatrices(R, IInv), Rt);
    }

    Matrix4x4 MultiplyMatrices(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                result[i, j] = a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j];
        result.m33 = 1;
        return result;
    }

    Matrix4x4 TransposeMatrix(Matrix4x4 m)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                result[i, j] = m[j, i];
        result.m33 = 1;
        return result;
    }

    Vector3 MultiplyMatrixVector(Matrix4x4 m, Vector3 v)
    {
        return new Vector3(
            m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
            m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
            m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
        );
    }
}