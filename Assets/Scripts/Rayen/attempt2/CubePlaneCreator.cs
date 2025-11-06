namespace Rayen.attempt2
{
    using System.Collections.Generic;
    using UnityEngine;

    public class CubePlaneCreator : MonoBehaviour
    {
        //create a plane of cubes using cubeobject4 script not prefab
        public float width = 3;
        public float height = 3;
        public float spacing = 0.05f;

        public float cubeHeight = 0.05f;
        public float cubeWidth = 0.5f;
        public float cubeDepth = 0.5f;

        public float heightOffset = 3.0f;
        public Material cubeMaterial;

        private List<CustomRigidBody3D> customRigidBodies = new List<CustomRigidBody3D>();
        private List<GameObject> cubeGameObjects = new List<GameObject>();
        public CustomSceneObjects sceneObjects;

        // CreateCubePlane using CubeObject4
        void Start()
        {
            CreateCubePlane();
        }

        void Update()
        {
            for (int i = 0; i < customRigidBodies.Count; i++)
            {
                customRigidBodies[i].UpdatePhysics();
                // Custom collision with plane
                if (sceneObjects != null)
                {
                    float halfHeight = customRigidBodies[i].B * 0.5f;
                    // Plane collision (assume plane is infinite for now)
                    if (customRigidBodies[i].Position.y - halfHeight < sceneObjects.planeY)
                    {
                        customRigidBodies[i].Position.y = sceneObjects.planeY + halfHeight;
                        if (customRigidBodies[i].Velocity.y < 0)
                            customRigidBodies[i].Velocity.y = 0;
                    }
                    // Sphere collision (AABB center to sphere center)
                    Vector3 toCenter = customRigidBodies[i].Position - sceneObjects.spherePosition;
                    float minDist = sceneObjects.sphereRadius + Mathf.Max(customRigidBodies[i].A, customRigidBodies[i].B, customRigidBodies[i].C) * 0.5f;
                    if (toCenter.magnitude < minDist)
                    {
                        Vector3 normal = toCenter.normalized;
                        customRigidBodies[i].Position = sceneObjects.spherePosition + normal * minDist;
                        // Reflect velocity (simple elastic)
                        customRigidBodies[i].Velocity = Vector3.Reflect(customRigidBodies[i].Velocity, normal);
                    }
                }
                cubeGameObjects[i].transform.position = customRigidBodies[i].Position;
                MeshFilter mf = cubeGameObjects[i].GetComponent<MeshFilter>();
                Mesh mesh = mf.mesh;
                mesh.vertices = customRigidBodies[i].GetTransformedVertices();
                mesh.RecalculateNormals();
            }
        }

        void CreateCubePlane()
        {
            int numCubesX = Mathf.FloorToInt(width / (cubeWidth + spacing));
            int numCubesZ = Mathf.FloorToInt(height / (cubeDepth + spacing));

            for (int i = 0; i < numCubesX; i++)
            {
                for (int j = 0; j < numCubesZ; j++)
                {
                    float x = i * (cubeWidth + spacing);
                    float z = j * (cubeDepth + spacing);
                    Vector3 position = new Vector3(x, heightOffset, z);

                    CubeObject4 cube = new CubeObject4(cubeWidth, cubeHeight, cubeDepth, Color.green);
                    CustomRigidBody3D crb = new CustomRigidBody3D(cubeWidth, cubeHeight, cubeDepth, 1f, position, Color.green, cube.vertices, cube.triangles);
                    customRigidBodies.Add(crb);

                    GameObject cubeGO = new GameObject("Cube_" + i + "_" + j);
                    cubeGO.transform.position = position;
                    MeshFilter mf = cubeGO.AddComponent<MeshFilter>();
                    MeshRenderer mr = cubeGO.AddComponent<MeshRenderer>();
                    mr.material = cubeMaterial;
                    mr.material.color = Color.green;

                    Mesh mesh = new Mesh();
                    mesh.vertices = cube.vertices;
                    mesh.triangles = cube.triangles;
                    mesh.RecalculateNormals();
                    mf.mesh = mesh;
                    // add custom collisioncubecollisonmanager to each cube
                    cubeGO.AddComponent<CustomCubeCollisionManager>();

                    cubeGameObjects.Add(cubeGO);
                    // Attach force applier and set reference to rigid body
                    var forceApplier = cubeGO.AddComponent<CustomRigidBodyForceApplier>();
                    forceApplier.rigidBody = crb;
                    forceApplier.gravity = crb.Gravity;
                    forceApplier.damping = crb.Damping;
                    forceApplier.useFixedDeltaTime = crb.UseFixedDeltaTime;
                    forceApplier.customDt = crb.CustomDt;
                }
            }
        }
    }
}
