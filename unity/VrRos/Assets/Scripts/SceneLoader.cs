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

        /// <summary>Body holders, indexed to match /vr/body_poses. Null where a body has no geometry.</summary>
        public Transform[] Bodies { get; private set; } = new Transform[0];

        public bool Loaded { get; private set; }

        private string _loadedUrl;

        private void Start()
        {
            if (config != null && config.Active != null) outNs = config.Active.outNs;
            bridge.Subscribe($"{outNs}/scene", "std_msgs/msg/String", OnScene);
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
            if (string.IsNullOrEmpty(url) || manifest == null)
            {
                Debug.LogError("scene: message carries no url or manifest; run scene_export first");
                return;
            }

            // Transient-local redelivers on every reconnect; only reload when the world changed.
            if (url == _loadedUrl) return;
            _loadedUrl = url;
            _ = LoadAsync(url, manifest);
        }

        private async Task LoadAsync(string url, JObject manifest)
        {
            Loaded = false;
            foreach (Transform child in transform) Destroy(child.gameObject);

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

                if (byName.TryGetValue(name, out Transform mesh))
                {
                    mesh.SetParent(holder, false);
                    mesh.localPosition = Vector3.zero;
                    mesh.localRotation = correction;
                }
                else
                {
                    Debug.LogWarning($"scene: manifest lists body '{name}' but the glb has no "
                                     + "node with that name");
                }
            }

            Destroy(importRoot.gameObject);
            Bodies = holders;
            Loaded = true;
            Debug.Log($"scene: loaded {url} with {holders.Length} bodies");
        }
    }
}
