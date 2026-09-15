// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System.Collections.Generic;
using System.Threading.Tasks;
using GLTFast;
using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityEngine.Networking;

namespace VrRos
{
    /// <summary>
    /// Loads the world published on /vr/scene: fetches the .glb over HTTP once, then reparents
    /// each exported body mesh under its own holder so the pose stream can drive it.
    ///
    /// Hierarchy per body:  Body_&lt;name&gt; (driven by /vr/body_poses) -> imported mesh node.
    ///
    /// The holder's child carries one fixed rotation, gltfCorrectionEuler. scene_export writes
    /// the .glb in glTF's Y-up convention; glTFast then applies its own right-to-left-handed
    /// flip on import. That flip composed with the export rotation differs from the ROS-to-Unity
    /// map used for live poses by exactly one rotation about Y, and this is it. If the world
    /// loads rotated 180 degrees, negate the Y value: those are the only two possibilities.
    /// </summary>
    public class SceneLoader : MonoBehaviour
    {
        public RosBridge bridge;

        [Tooltip("Optional: when set, the topic namespace comes from its config file")]
        public VrConfig config;

        [Tooltip("Namespace the PC publishes the world into")]
        public string outNs = "/vr";

        [Tooltip("Rotation reconciling glTFast's import flip with the live pose convention")]
        public Vector3 gltfCorrectionEuler = new Vector3(0f, 90f, 0f);

        [Tooltip("Draw a ground plane; scene_export leaves MuJoCo's unbounded floor out")]
        public bool drawGround = true;

        public float groundSize = 40f;
        public Material groundMaterial;

        [Tooltip("Collide the imported meshes so the pointer can select a body")]
        public bool addColliders = true;

        /// <summary>Body holders, indexed by MuJoCo body id. Null where a body has no geometry.</summary>
        public Transform[] Bodies { get; private set; } = new Transform[0];


        public bool Loaded { get; private set; }

        private string _loadedUrl;

        private void Start()
        {
            if (config != null && config.Active != null) outNs = config.Active.outNs;
            bridge.Subscribe($"{outNs}/scene", "std_msgs/msg/String", OnScene);

            // Somewhere to stand before the first scene arrives; the load replaces it.
            if (drawGround) CreateGround();
        }

        /// <summary>
        /// A floor the renderer draws properly, instead of the tens of thousands of baked quads
        /// an unbounded MuJoCo ground plane would become. It is scene dressing, not simulated
        /// geometry: contact still happens in MuJoCo against the real plane.
        /// </summary>
        /* Concave colliders. Convex ones are cheaper, but a convex hull of a cabinet or a counter
         * is a solid block: the ray stops on the hull and can never reach a cup standing on the
         * surface or an object inside an open shelf. Nothing here is simulated on the client -
         * these exist only to be pointed at - so the exact triangles are both affordable and the
         * only thing that gives the right answer. Moving a concave collider by its transform is
         * fine; only changing its mesh would force PhysX to re-cook it, and these never change. */
        private static void AddColliders(Transform meshRoot)
        {
            foreach (MeshFilter filter in meshRoot.GetComponentsInChildren<MeshFilter>(true))
            {
                if (filter.sharedMesh == null) continue;
                var collider = filter.gameObject.AddComponent<MeshCollider>();
                collider.sharedMesh = filter.sharedMesh;
                collider.convex = false;
            }
        }

        private void CreateGround()
        {
            var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
            ground.name = "Ground";
            ground.transform.SetParent(transform, false);
            ground.transform.localScale = Vector3.one * (groundSize / 10f); // Unity's plane is 10 m
            Destroy(ground.GetComponent<Collider>()); // nothing here is physically simulated

            if (groundMaterial != null) ground.GetComponent<Renderer>().material = groundMaterial;
        }

        private void OnScene(JObject msg)
        {
            if (msg == null) return;
            JObject payload;
            try
            {
                payload = JObject.Parse((string)msg["data"]);
            }
            catch (System.Exception e)
            {
                Debug.LogError($"scene: payload is not JSON: {e.Message}");
                return;
            }

            string url = (string)payload["url"];
            JObject manifest = payload["manifest"] as JObject;
            JObject env = payload["env"] as JObject;

            /* Either half is enough on its own. A simulated world needs the manifest; a place to
             * stand in and look at does not, and demanding one would force every scene through
             * the exporter for no benefit. */
            bool hasWorld = !string.IsNullOrEmpty(url) && manifest != null;
            if (!hasWorld && env == null)
            {
                Debug.LogError("scene: message carries neither a world nor scenery");
                return;
            }

            // Transient-local redelivers on every reconnect; only reload when something changed.
            string key = $"{url}|{(string)env?["url"]}";
            if (key == _loadedUrl) return;
            _loadedUrl = key;

            /* Fire and forget, but not silently: an unobserved Task swallows its exception, and
             * a scene that simply never appears is the least debuggable failure there is. */
            _ = LoadAsync(hasWorld ? url : null, manifest, env, payload)
                .ContinueWith(t => Debug.LogError($"scene: load failed: {t.Exception}"),
                              System.Threading.Tasks.TaskContinuationOptions.OnlyOnFaulted);
        }

        private async Task LoadAsync(string url, JObject manifest, JObject env, JObject payload)
        {
            Loaded = false;
            foreach (Transform child in transform) Destroy(child.gameObject);

            /* Scenery brings its own floor, and so does a model whose floor is real geometry
             * rather than an unbounded plane; either way a second one z-fights with it. */
            bool wantGround = (bool?)payload["ground"] ?? true;
            if (drawGround && wantGround && env == null) CreateGround();

            // Scenery only: nothing is simulated, so there are no bodies to map or to drive.
            if (url == null)
            {
                Bodies = new Transform[0];
                if (env != null) await LoadEnvAsync(env);
                return;
            }

            using UnityWebRequest request = UnityWebRequest.Get(url);
            await request.SendWebRequest();
            if (request.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError($"scene: GET {url} failed: {request.error}");
                return;
            }

            var gltf = new GltfImport();
            if (!await gltf.LoadGltfBinary(request.downloadHandler.data, new System.Uri(url)))
            {
                Debug.LogError($"scene: {url} is not a loadable glb");
                return;
            }

            var importRoot = new GameObject("ImportRoot").transform;
            importRoot.SetParent(transform, false);
            if (!await gltf.InstantiateMainSceneAsync(importRoot))
            {
                Debug.LogError("scene: instantiation failed");
                return;
            }

            var byName = new Dictionary<string, Transform>();
            foreach (Transform node in importRoot.GetComponentsInChildren<Transform>(true))
            {
                byName[node.name] = node;
            }

            JArray bodies = manifest["bodies"] as JArray;
            var holders = new Transform[bodies?.Count ?? 0];
            Quaternion correction = Quaternion.Euler(gltfCorrectionEuler);

            for (int i = 0; i < holders.Length; i++)
            {
                string name = (string)bodies[i]["name"];
                int node = (int)bodies[i]["node"];
                if (node < 0) continue; // body with no renderable geometry

                var holder = new GameObject($"Body_{name}").transform;
                holder.SetParent(transform, false);
                holders[i] = holder;

                // The pointer raycasts against the meshes and needs the hit mapped back to an index.
                holder.gameObject.AddComponent<VrBodyTag>().index = i;

                if (byName.TryGetValue(name, out Transform mesh))
                {
                    mesh.SetParent(holder, false);
                    mesh.localPosition = Vector3.zero;
                    mesh.localRotation = correction;
                    if (addColliders) AddColliders(mesh);
                }
                else
                {
                    Debug.LogWarning($"scene: manifest lists body '{name}' but the glb has no "
                                     + "node with that name");
                }
            }

            /* Whatever is left is scene dressing the manifest does not list - the sky dome and
             * the exported lights. It stays, parented to this object, instead of being destroyed
             * along with the import root. */
            int kept = 0;
            while (importRoot.childCount > 0)
            {
                Transform extra = importRoot.GetChild(0);
                extra.SetParent(transform, true);
                kept++;
            }

            Destroy(importRoot.gameObject);

            /* Shadows are what separate one white surface from another; without them a lit scene
             * still reads as flat clay. The importer leaves them off, so turn them on for the
             * lights the scene brought with it. */
            foreach (Light light in GetComponentsInChildren<Light>(true))
            {
                light.shadows = LightShadows.Soft;
                light.shadowBias = 0.02f;
            }

            /* Welded bodies come once instead of in the stream. Placed before Loaded goes true,
             * so no streamed frame lands on a scene still half at the origin. */
            int placed = 0;
            if (payload["static"] is JArray statics)
            {
                foreach (JToken entry in statics)
                {
                    int b = (int)entry[0];
                    if (b < 0 || b >= holders.Length || holders[b] == null) continue;
                    holders[b].SetLocalPositionAndRotation(
                        FrameConv.RosToUnity((float)entry[1], (float)entry[2], (float)entry[3]),
                        FrameConv.RosToUnity((float)entry[4], (float)entry[5], (float)entry[6],
                                             (float)entry[7]));
                    placed++;
                }
            }

            Bodies = holders;
            Loaded = true;
            Debug.Log($"scene: loaded {url} with {holders.Length} bodies ({placed} placed as "
                      + $"static) and {kept} other nodes");

            if (env != null) await LoadEnvAsync(env);
        }

        /// <summary>
        /// Loads the scenery: a textured .glb that is drawn and never simulated.
        ///
        /// It gets no body holder, no collider and no tag, because nothing on the PC knows it
        /// exists — a pose stream would have nothing to say about it and the pointer must not
        /// select it. Anything the user should collide with belongs in the MJCF as a geom.
        /// </summary>
        private async Task LoadEnvAsync(JObject env)
        {
            string url = (string)env["url"];
            if (string.IsNullOrEmpty(url)) return;

            using UnityWebRequest request = UnityWebRequest.Get(url);
            await request.SendWebRequest();
            if (request.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError($"env: GET {url} failed: {request.error}");
                return;
            }

            var gltf = new GltfImport();
            if (!await gltf.LoadGltfBinary(request.downloadHandler.data, new System.Uri(url)))
            {
                Debug.LogError($"env: {url} is not a loadable glb");
                return;
            }

            var root = new GameObject("Scenery").transform;
            root.SetParent(transform, false);
            if (!await gltf.InstantiateMainSceneAsync(root))
            {
                Debug.LogError("env: instantiation failed");
                return;
            }

            /* Placed in ROS coordinates like everything else the PC talks about, because the
             * environment's own origin is wherever its author put it. */
            JArray xyz = env["xyz"] as JArray;
            if (xyz != null && xyz.Count == 3)
            {
                root.localPosition =
                    FrameConv.RosToUnity((float)xyz[0], (float)xyz[1], (float)xyz[2]);
            }
            root.localRotation = Quaternion.Euler(0f, -(float)(env["yaw_deg"] ?? 0), 0f);
            float scale = (float)(env["scale"] ?? 1f);
            root.localScale = Vector3.one * (scale <= 0f ? 1f : scale);

            Debug.Log($"env: loaded {url} at {root.localPosition}, scale {scale}");
        }
    }
}
