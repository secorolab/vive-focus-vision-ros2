// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using Newtonsoft.Json.Linq;
using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// Drives the loaded body holders from /vr/body_poses. Pose i belongs to body i: the order
    /// is MuJoCo's body order, which is what scene_export wrote into the manifest.
    ///
    /// Parsing a PoseArray through JObject allocates per frame. That is fine for the tens of
    /// bodies a manipulation scene has; a scene with hundreds should move this topic to
    /// rosbridge's CBOR compression, or to a flat float32 array, before blaming the renderer.
    /// </summary>
    public class BodyPoseApplier : MonoBehaviour
    {
        public RosBridge bridge;
        public SceneLoader scene;

        [Tooltip("Optional: when set, the topic namespace comes from its config file")]
        public VrConfig config;

        [Tooltip("Namespace the PC publishes the world into")]
        public string outNs = "/vr";

        public int Received { get; private set; }

        private void Start()
        {
            if (config != null && config.Active != null) outNs = config.Active.outNs;
            bridge.Subscribe($"{outNs}/body_poses", "geometry_msgs/msg/PoseArray", OnPoses);
        }

        private void OnPoses(JObject msg)
        {
            if (msg == null || !scene.Loaded) return;
            JArray poses = msg["poses"] as JArray;
            if (poses == null) return;

            Transform[] bodies = scene.Bodies;
            if (poses.Count != bodies.Length)
            {
                // A mismatch means the running model is not the one that was exported.
                Debug.LogWarning($"body_poses has {poses.Count} poses but the scene has "
                                 + $"{bodies.Length} bodies; re-run scene_export for this model");
                return;
            }

            for (int i = 0; i < bodies.Length; i++)
            {
                if (bodies[i] == null) continue;
                JToken p = poses[i]["position"];
                JToken q = poses[i]["orientation"];
                bodies[i].SetLocalPositionAndRotation(
                    FrameConv.RosToUnity((float)p["x"], (float)p["y"], (float)p["z"]),
                    FrameConv.RosToUnity((float)q["x"], (float)q["y"], (float)q["z"],
                                         (float)q["w"]));
            }
            Received++;
        }
    }
}
