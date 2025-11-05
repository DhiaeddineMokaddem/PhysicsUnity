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

                    cubeGameObjects.Add(cubeGO);
                    // Attach force applier and set reference to rigid body
                    var forceApplier = cubeGO.AddComponent<CustomRigidBodyForceApplier>();
                    forceApplier.rigidBody = crb;
                    forceApplier.gravity = crb.Gravity;
                    forceApplier.damping = crb.Damping;
                    forceApplier.useFixedDeltaTime = crb.UseFixedDeltaTime;
                    forceApplier.customDt = crb.CustomDt;
                    //cubeGO.AddComponent<CustomFreeFallDamping>();
                    // Optionally set other parameters if needed
                    // damping.gravity = ...;
                    // damping.damping = ...;
                    // damping.useFixedDeltaTime = ...;
                    // damping.customDt = ...;
                }
            }
        }

    }
}
