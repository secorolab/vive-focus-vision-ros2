// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// Tints the gripper while the operator lines the controller up with it: red while far,
    /// amber as it closes, green once the PC says a grip press would engage.
    ///
    /// Green comes from the PC's own `ready` flag, never from a threshold held here, so green
    /// always means the press will take. Only the shade in between is computed locally, from the
    /// errors and the tolerances the status message carries.
    /// </summary>
    public class VrTeleopIndicator : MonoBehaviour
    {
        public RosBridge bridge;

        [Tooltip("Optional: when set, the topic namespace comes from its config file")]
        public VrConfig config;

        [Tooltip("The loaded world, whose body holders carry the gripper geometry")]
        public SceneLoader scene;

        [Tooltip("Namespace the PC publishes the world into")]
        public string outNs = "/vive_vr";

        [Tooltip("Arm whose status is shown")]
        public string arm = "right";

        [Tooltip("Body-name prefixes to tint; these are the gripper as the operator sees it")]
        public string[] highlightBodies = { "openarm_right_hand", "openarm_right_finger" };

        public Color ready = new Color(0.25f, 0.85f, 0.35f);
        public Color close = new Color(0.95f, 0.70f, 0.15f);
        public Color far = new Color(0.90f, 0.25f, 0.20f);

        [Tooltip("Multiple of the tolerance at which the tint is fully red")]
        public float farAtTolerances = 4f;

        private readonly List<Renderer> _renderers = new List<Renderer>();
        private MaterialPropertyBlock _block;
        private bool _tinted;
        private bool _have;
        private string _state = "";
        private bool _ready;
        private float _worst;

        private void Start()
        {
            if (config != null && config.Active != null) outNs = config.Active.outNs;
            _block = new MaterialPropertyBlock();
            bridge.Subscribe($"{outNs}/teleop/{arm}/status", "vive_vr_ros2/msg/TeleopStatus",
                             OnStatus);
        }

        private void OnStatus(JObject msg)
        {
            if (msg == null) return;
            _state = (string)msg["state"] ?? "";
            _ready = (bool?)msg["ready"] ?? false;

            float position = (float?)msg["position_error_m"] ?? 0f;
            float rotation = (float?)msg["rotation_error_rad"] ?? 0f;
            float positionTolerance = (float?)msg["position_tolerance_m"] ?? 0f;
            float rotationTolerance = (float?)msg["rotation_tolerance_rad"] ?? 0f;

            _worst = 0f;
            if (positionTolerance > 0f) _worst = Mathf.Max(_worst, position / positionTolerance);
            if (rotationTolerance > 0f) _worst = Mathf.Max(_worst, rotation / rotationTolerance);
            _have = true;
        }

        private void Update()
        {
            // A reload destroys the holders, so drop the cached renderers rather than restore them.
            if (scene == null || !scene.Loaded)
            {
                Forget();
                return;
            }
            if (!_have)
            {
                Restore();
                return;
            }

            // Engaged, or nothing to line up against: the gripper goes back to its own colours.
            if (_state == "engaged" || _state == "waiting_delta" || _state == "unreachable"
                || _state == "release_grip" || _state == "tracking_lost")
            {
                Restore();
                return;
            }

            if (_renderers.Count == 0) Collect();
            if (_renderers.Count == 0) return;

            float t = Mathf.Clamp01((_worst - 1f) / Mathf.Max(farAtTolerances - 1f, 0.01f));
            Color tint = _ready ? ready : Color.Lerp(close, far, t);

            _block.Clear();
            // The imported glTF materials name their base colour differently per shader variant.
            _block.SetColor("baseColorFactor", tint);
            _block.SetColor("_BaseColor", tint);
            _block.SetColor("_Color", tint);
            foreach (Renderer renderer in _renderers)
            {
                if (renderer != null) renderer.SetPropertyBlock(_block);
            }
            _tinted = true;
        }

        private void Collect()
        {
            foreach (Transform child in scene.transform)
            {
                if (!child.name.StartsWith("Body_")) continue;
                string body = child.name.Substring("Body_".Length);
                bool wanted = false;
                foreach (string prefix in highlightBodies)
                {
                    if (!string.IsNullOrEmpty(prefix) && body.StartsWith(prefix)) wanted = true;
                }
                if (!wanted) continue;
                _renderers.AddRange(child.GetComponentsInChildren<Renderer>(true));
            }
            if (_renderers.Count == 0)
            {
                Debug.LogWarning("teleop indicator: no body matched " +
                                 string.Join(", ", highlightBodies) + "; nothing to tint");
            }
        }

        private void Restore()
        {
            if (!_tinted) return;
            foreach (Renderer renderer in _renderers)
            {
                if (renderer != null) renderer.SetPropertyBlock(null);
            }
            _tinted = false;
        }

        /// <summary>The world reloads into new objects, so the cached renderers are stale.</summary>
        public void Forget()
        {
            _renderers.Clear();
            _tinted = false;
        }
    }
}
