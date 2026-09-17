// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System.Collections.Concurrent;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// Drives the loaded body holders from /vr/body_poses, which carries only the bodies that
    /// moved: poses[i] belongs to body ids[i].
    ///
    /// The JSON walk and the frame conversion both happen on the socket task; the main thread
    /// only writes Transforms, which is the one part that cannot happen anywhere else. Every
    /// frame is applied: each carries only what moved since the last one.
    /// </summary>
    public class BodyPoseApplier : MonoBehaviour
    {
        public RosBridge bridge;
        public SceneLoader scene;

        [Tooltip("Optional: when set, the topic namespace comes from its config file")]
        public VrConfig config;

        [Tooltip("Namespace the PC publishes the world into")]
        public string outNs = "/vive_vr";

        public int Received { get; private set; }

        /// <summary>One decoded frame, already in Unity's convention.</summary>
        private sealed class Frame
        {
            public int[] Id = new int[0];
            public Vector3[] Position = new Vector3[0];
            public Quaternion[] Rotation = new Quaternion[0];
            public int Count;

            public void Resize(int n)
            {
                if (Position.Length >= n) return;
                Id = new int[n];
                Position = new Vector3[n];
                Rotation = new Quaternion[n];
            }
        }

        // Frames come back here after Apply, so a steady stream allocates nothing.
        private readonly ConcurrentBag<Frame> _pool = new ConcurrentBag<Frame>();
        private int _warned = -1;

        private void Start()
        {
            if (config != null && config.Active != null) outNs = config.Active.outNs;
            bridge.Subscribe<Frame>($"{outNs}/body_poses", "vive_vr_ros2/msg/BodyPoses", Decode, Apply);
        }

        /// <summary>Socket task: walks the JSON and converts to Unity's frame. No Unity API here.</summary>
        private Frame Decode(JObject msg)
        {
            JArray poses = msg?["poses"] as JArray;
            JArray ids = msg?["ids"] as JArray;
            if (poses == null || ids == null || ids.Count != poses.Count) return null;

            if (!_pool.TryTake(out Frame frame)) frame = new Frame();
            frame.Resize(poses.Count);
            frame.Count = poses.Count;

            for (int i = 0; i < poses.Count; i++)
            {
                JToken p = poses[i]["position"];
                JToken q = poses[i]["orientation"];
                frame.Id[i] = (int)ids[i];
                frame.Position[i] = FrameConv.RosToUnity((float)p["x"], (float)p["y"], (float)p["z"]);
                frame.Rotation[i] = FrameConv.RosToUnity((float)q["x"], (float)q["y"],
                                                         (float)q["z"], (float)q["w"]);
            }
            return frame;
        }

        /// <summary>Main thread: nothing but Transform writes.</summary>
        private void Apply(Frame frame)
        {
            if (scene.Loaded) Write(frame);
            _pool.Add(frame);
        }

        private void Write(Frame frame)
        {
            Transform[] bodies = scene.Bodies;
            for (int i = 0; i < frame.Count; i++)
            {
                int b = frame.Id[i];
                if (b < 0 || b >= bodies.Length)
                {
                    // An id outside the manifest means this is not the model that was exported.
                    if (_warned != b)
                    {
                        _warned = b;
                        Debug.LogWarning($"body_poses names body {b} but the scene has "
                                         + $"{bodies.Length}; re-run scene_export for this model");
                    }
                    continue;
                }
                if (bodies[b] == null) continue;
                bodies[b].SetLocalPositionAndRotation(frame.Position[i], frame.Rotation[i]);
            }
            Received++;
        }
    }
}
