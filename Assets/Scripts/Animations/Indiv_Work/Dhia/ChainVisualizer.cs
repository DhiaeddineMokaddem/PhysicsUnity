//Handles rendering
using UnityEngine;
using UnityEngine;

public class ChainVisualizer
{
    private float linkRadius;
    private float linkLength;
    private GameObject chainLinkPrefab;
    private Transform parentTransform;

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
        for (int i = 0; i < links.Length; i++)
        {
            if (links[i].visualObject != null)
            {
                // Hide first and last links (they're inside anchors)
                bool shouldRender = i != 0 && i != links.Length - 1;
                links[i].visualObject.SetActive(shouldRender);

                // Update the GameObject's Transform using our custom position/rotation
                if (shouldRender)
                {
                    links[i].visualObject.transform.position = links[i].position;
                    links[i].visualObject.transform.rotation = links[i].rotation;
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
        if (startAnchor.transform != null)
        {
            Gizmos.color = startAnchor.isActive ? Color.green : Color.gray;
            Gizmos.DrawWireSphere(startAnchor.position, linkRadius * 1.5f);
            Gizmos.DrawWireCube(startAnchor.position, Vector3.one * linkRadius);
        }

        // Draw end anchor
        if (endAnchor.transform != null)
        {
            Gizmos.color = endAnchor.isActive ? Color.green : Color.gray;
            Gizmos.DrawWireSphere(endAnchor.position, linkRadius * 1.5f);
            Gizmos.DrawWireCube(endAnchor.position, Vector3.one * linkRadius);
        }

        // Draw tension indicator
        if (startAnchor.isActive && endAnchor.isActive && currentTension > 0)
        {
            Vector3 midPoint = (startAnchor.position + endAnchor.position) * 0.5f;
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
}