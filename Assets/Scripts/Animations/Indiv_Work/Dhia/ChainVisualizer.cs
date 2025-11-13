using UnityEngine;
using PhysicsSimulation.Core; // for ManualMatrix

//Handles rendering
public class ChainVisualizer
{
    private float linkRadius;
    private float linkLength;
    private GameObject chainLinkPrefab;
    private Transform parentTransform;

    // Cache mesh/material from prefab to draw manually (no Transform setters)
    private Mesh _cachedMesh;
    private Material[] _cachedMaterials;
    private int _cachedLayer = 0;
    private bool _cacheReady = false;

    // Local mesh offset: rotate Y by 90 degrees and scale uniformly by 8
    private Matrix4x4 _localOffset;
    private bool _offsetReady = false;

    public ChainVisualizer(float radius, float length, GameObject prefab, Transform parent)
    {
        linkRadius = radius;
        linkLength = length;
        chainLinkPrefab = prefab;
        parentTransform = parent;
    }

    public void InitializeVisuals(ChainLink[] links)
    {
        if (chainLinkPrefab == null)
            return;

        for (int i = 0; i < links.Length; i++)
        {
            links[i].visualObject = GameObject.Instantiate(chainLinkPrefab, links[i].position, links[i].rotation, parentTransform);
            links[i].visualObject.name = "ChainLink_" + i;
        }
    }

    public void UpdateVisuals(ChainLink[] links)
    {
        EnsureCache();

        for (int i = 0; i < links.Length; i++)
        {
            if (links[i].visualObject != null)
            {
                // Hide first and last links (they're inside anchors)
                bool shouldRender = i != 0 && i != links.Length - 1;

                // Always disable the instantiated renderer to avoid double rendering
                links[i].visualObject.SetActive(false);

                // Custom manual draw: compute TR matrix from our simulated position/rotation
                if (shouldRender && _cacheReady)
                {
                    // Build manual rigid transform matrix (no Transform.position/rotation used)
                    ManualMatrix tr = ManualMatrix.TR(links[i].position, links[i].rotation);
                    Matrix4x4 m = ManualToUnity(tr);

                    // If a parent transform exists, multiply by parent world matrix (read-only). We avoid setting any Transform here.
                    if (parentTransform != null)
                    {
                        // Note: reading parentTransform.localToWorldMatrix is allowed; we only avoid writing to Transforms.
                        m = parentTransform.localToWorldMatrix * m;
                    }

                    // Apply fixed local offset (Ry 90deg, uniform scale 8)
                    if (_offsetReady)
                    {
                        m = m * _localOffset;
                    }

                    // Draw each submesh with its corresponding material
                    for (int sub = 0; sub < _cachedMesh.subMeshCount; sub++)
                    {
                        var mat = (sub < _cachedMaterials.Length) ? _cachedMaterials[sub] : _cachedMaterials[0];
                        Graphics.DrawMesh(_cachedMesh, m, mat, _cachedLayer);
                    }
                }
            }
        }
    }

    public void DrawGizmos(ChainLink[] links, ChainAnchor startAnchor, ChainAnchor endAnchor, float currentTension, float maxStretchDistance)
    {
        if (links == null || links.Length == 0)
            return;

        // Draw links
        for (int i = 0; i < links.Length; i++)
        {
            // Draw sphere at joint
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(links[i].position, linkRadius * 0.5f);

            // Draw cylinder representing link - color based on tension
            if (i < links.Length - 1)
            {
                float tensionRatio = Mathf.Clamp01(currentTension / maxStretchDistance);
                Gizmos.color = Color.Lerp(Color.cyan, Color.red, tensionRatio);
                DrawCylinder(links[i].position, links[i + 1].position, linkRadius);
            }

            // Draw rotation axes
            Gizmos.color = Color.red;
            Gizmos.DrawLine(links[i].position, links[i].position + links[i].rotation * Vector3.right * linkLength * 0.3f);

            Gizmos.color = Color.green;
            Gizmos.DrawLine(links[i].position, links[i].position + links[i].rotation * Vector3.up * linkLength * 0.3f);

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(links[i].position, links[i].position + links[i].rotation * Vector3.forward * linkLength * 0.3f);
        }

        // Draw start anchor
        if (startAnchor != null)
        {
            Gizmos.color = startAnchor.isActive ? Color.green : Color.gray;
            Gizmos.DrawWireSphere(startAnchor.position, linkRadius * 1.5f);
            Gizmos.DrawWireCube(startAnchor.position, Vector3.one * linkRadius);
        }

        // Draw end anchor
        if (endAnchor != null)
        {
            Gizmos.color = endAnchor.isActive ? Color.green : Color.gray;
            Gizmos.DrawWireSphere(endAnchor.position, linkRadius * 1.5f);
            Gizmos.DrawWireCube(endAnchor.position, Vector3.one * linkRadius);
        }

        // Draw tension indicator
        if (startAnchor.isActive && endAnchor.isActive && currentTension > 0)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(startAnchor.position, endAnchor.position);
        }
    }

    private void DrawCylinder(Vector3 start, Vector3 end, float radius)
    {
        Vector3 dir = end - start;
        int segments = 8;
        Vector3 perpendicular = Vector3.Cross(dir.normalized, Vector3.up);

        if (perpendicular.magnitude < 0.1f)
            perpendicular = Vector3.Cross(dir.normalized, Vector3.right);

        perpendicular = perpendicular.normalized * radius;

        for (int i = 0; i < segments; i++)
        {
            float angle1 = (float)i / segments * Mathf.PI * 2f;
            float angle2 = (float)(i + 1) / segments * Mathf.PI * 2f;

            Quaternion rot1 = Quaternion.AngleAxis(angle1 * Mathf.Rad2Deg, dir.normalized);
            Quaternion rot2 = Quaternion.AngleAxis(angle2 * Mathf.Rad2Deg, dir.normalized);

            Vector3 p1 = start + rot1 * perpendicular;
            Vector3 p2 = start + rot2 * perpendicular;
            Vector3 p3 = end + rot1 * perpendicular;
            Vector3 p4 = end + rot2 * perpendicular;

            // Draw lines forming cylinder
            Gizmos.DrawLine(p1, p2);
            Gizmos.DrawLine(p3, p4);
            Gizmos.DrawLine(p1, p3);
        }
    }

    public void Cleanup(ChainLink[] links)
    {
        if (links != null)
        {
            for (int i = 0; i < links.Length; i++)
            {
                if (links[i].visualObject != null)
                {
                    GameObject.Destroy(links[i].visualObject);
                }
            }
        }
    }

    // --- Helpers ---

    private void EnsureCache()
    {
        if (_cacheReady && _offsetReady)
            return;

        GameObject src = chainLinkPrefab;
        if (src == null)
        {
            _cacheReady = false;
            return;
        }

        if (!_cacheReady)
        {
            // Prefer child components to support nested prefabs
            var mf = src.GetComponentInChildren<MeshFilter>(true);
            var mr = src.GetComponentInChildren<MeshRenderer>(true);
            if (mf != null)
            {
                _cachedMesh = mf.sharedMesh;
            }
            if (mr != null)
            {
                _cachedMaterials = mr.sharedMaterials;
                _cachedLayer = src.layer;
            }

            // Fallbacks
            if (_cachedMesh == null)
            {
                _cachedMesh = new Mesh(); // empty mesh prevents null exception; Gizmos will still show chain
            }
            if (_cachedMaterials == null || _cachedMaterials.Length == 0)
            {
                _cachedMaterials = new Material[] { new Material(Shader.Find("Standard")) };
            }

            _cacheReady = _cachedMesh != null && _cachedMaterials != null && _cachedMaterials.Length > 0;
        }

        if (!_offsetReady)
        {
            // Build manual local offset matrix: Rotate Y by 90 degrees, then uniform scale by 8
            float deg = 90f * Mathf.Deg2Rad;
            float c = Mathf.Cos(deg);
            float s = Mathf.Sin(deg);
            float sc = 8f;

            // Row-major assignment consistent with ManualToUnity mapping
            Matrix4x4 o = new Matrix4x4();
            o.m00 = sc * c;   o.m01 = 0f;   o.m02 = sc * s;   o.m03 = 0f; // X row
            o.m10 = 0f;       o.m11 = sc;   o.m12 = 0f;       o.m13 = 0f; // Y row (scaled)
            o.m20 = -sc * s;  o.m21 = 0f;   o.m22 = sc * c;   o.m23 = 0f; // Z row
            o.m30 = 0f;       o.m31 = 0f;   o.m32 = 0f;       o.m33 = 1f; // W row

            _localOffset = o;
            _offsetReady = true;
        }
    }

    private static Matrix4x4 ManualToUnity(ManualMatrix m)
    {
        // Construct a Unity Matrix4x4 with the same row-major data
        Matrix4x4 mm = new Matrix4x4();
        mm.m00 = m.m00; mm.m01 = m.m01; mm.m02 = m.m02; mm.m03 = m.m03;
        mm.m10 = m.m10; mm.m11 = m.m11; mm.m12 = m.m12; mm.m13 = m.m13;
        mm.m20 = m.m20; mm.m21 = m.m21; mm.m22 = m.m22; mm.m23 = m.m23;
        mm.m30 = m.m30; mm.m31 = m.m31; mm.m32 = m.m32; mm.m33 = m.m33;
        return mm;
    }
}

