// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

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
        private string _recoveryReason = "";
        private GameObject _recoveryPanel;
        private TextMeshProUGUI _recoveryTitle, _recoveryText;
        private float _worst;
        private float _positionTolerance;
        private float _statusAt = -100f, _controllerAt = -100f, _toolAt = -100f;
        private Vector3 _controller, _tool;
        private Quaternion _controllerRotation, _wantedRotation;
        private float _wantedAt = -100f;
        private LineRenderer _currentOutline, _wantedOutline;
        // Asymmetric controller silhouette in Unity grip coordinates: handle, top, forward arrow.
        private static readonly Vector3[] ControllerOutline = {
            new Vector3(-0.018f, -0.08f, 0), new Vector3(0.018f, -0.08f, 0),
            new Vector3(0.025f, 0.02f, 0), new Vector3(0.045f, 0.025f, 0.055f),
            new Vector3(-0.045f, 0.025f, 0.055f), new Vector3(-0.025f, 0.02f, 0),
            new Vector3(-0.018f, -0.08f, 0), new Vector3(-0.025f, 0.02f, 0),
            new Vector3(0.025f, 0.02f, 0), new Vector3(0, 0.02f, 0),
            new Vector3(0, 0.02f, 0.12f), new Vector3(-0.018f, 0.02f, 0.095f),
            new Vector3(0, 0.02f, 0.12f), new Vector3(0.018f, 0.02f, 0.095f)
        };
        private LineRenderer _guide, _targetRing;
        private Material _guideMaterial;
        private Camera _head;
        private const float StaleSeconds = 0.5f;

        // Works with existing generated scenes as well as fresh setup; never add duplicate arms.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void EnsureIndicators()
        {
            RosBridge bridge = Object.FindFirstObjectByType<RosBridge>();
            SceneLoader scene = Object.FindFirstObjectByType<SceneLoader>();
            if (bridge == null || scene == null) return;
            foreach (string side in new[] { "right", "left" })
            {
                bool exists = false;
                foreach (var indicator in Object.FindObjectsByType<VrTeleopIndicator>(FindObjectsSortMode.None))
                    if (indicator.arm == side) exists = true;
                if (exists) continue;
                var added = bridge.gameObject.AddComponent<VrTeleopIndicator>();
                added.arm = side;
                added.bridge = bridge;
                added.config = Object.FindFirstObjectByType<VrConfig>();
                added.scene = scene;
            }
        }

        private void Start()
        {
            highlightBodies = new[] { $"openarm_{arm}_hand", $"openarm_{arm}_finger" };
            if (config != null && config.Active != null) outNs = config.Active.outNs;
            _block = new MaterialPropertyBlock();
            BuildGuide();
            if (bridge == null) return;
            bridge.Subscribe($"{outNs}/teleop/{arm}/status", "vive_vr_ros2/msg/TeleopStatus",
                             OnStatus);
            bridge.Subscribe($"{outNs}/{arm}/pose", "geometry_msgs/msg/PoseStamped", msg =>
            {
                if (this == null || !ReadPosition(msg, out Vector3 point)
                    || !ReadRotation(msg, out Quaternion rotation)) return;
                _controllerRotation = rotation;
                _controller = point;
                _controllerAt = Time.unscaledTime;
            });
            bridge.Subscribe($"{outNs}/teleop/{arm}/alignment_pose", "geometry_msgs/msg/PoseStamped", msg =>
            {
                if (this == null || !ReadRotation(msg, out Quaternion rotation)) return;
                _wantedRotation = rotation;
                _wantedAt = Time.unscaledTime;
            });
            bridge.Subscribe($"{outNs}/sim/{arm}/ee_pose", "geometry_msgs/msg/PoseStamped", msg =>
            {
                if (this == null || !ReadPosition(msg, out Vector3 point)) return;
                _tool = point;
                _toolAt = Time.unscaledTime;
            });
        }

        private void OnStatus(JObject msg)
        {
            if (this == null || msg == null) return;
            _state = (string)msg["state"] ?? "";
            _ready = (bool?)msg["ready"] ?? false;

            float position = (float?)msg["position_error_m"] ?? 0f;
            float rotation = (float?)msg["rotation_error_rad"] ?? 0f;
            float positionTolerance = (float?)msg["position_tolerance_m"] ?? 0f;
            float rotationTolerance = (float?)msg["rotation_tolerance_rad"] ?? 0f;

            _worst = 0f;
            if (positionTolerance > 0f) _worst = Mathf.Max(_worst, position / positionTolerance);
            if (rotationTolerance > 0f) _worst = Mathf.Max(_worst, rotation / rotationTolerance);
            if (!Finite(position) || !Finite(rotation) || !Finite(positionTolerance)
                || !Finite(rotationTolerance) || position < 0f || rotation < 0f
                || positionTolerance < 0f || rotationTolerance <= 0f)
            {
                _have = false;
                return;
            }
            if (_state == "engaged" || _state == "waiting_delta") _recoveryReason = "";
            else if (_state == "release_grip")
            {
                string reason = (string)msg["stop_reason"] ?? "";
                if (reason == "translation_limit" || reason == "rotation_limit"
                    || reason == "tracking_timeout" || reason == "invalid_target"
                    || reason == "reference_mismatch") _recoveryReason = reason;
            }
            _positionTolerance = positionTolerance;
            _statusAt = Time.unscaledTime;
            _have = true;
        }

        private void Update()
        {
            if (_head == null) _head = Camera.main;
            bool connected = bridge != null && bridge.IsConnected;
            if (!connected)
            {
                // Do not revive an old green state when the socket reconnects.
                _have = false;
                _controllerAt = _toolAt = _wantedAt = -100f;
            }
            bool live = connected && _have && Time.unscaledTime - _statusAt <= StaleSeconds;
            bool loaded = scene != null && scene.Loaded;
            if (!loaded) Forget();
            bool matching = live && loaded && (_state == "align_pose" || _state == "ready");
            Color tint = _ready && matching ? ready
                : Color.Lerp(close, far, Mathf.Clamp01((_worst - 1f) / Mathf.Max(farAtTolerances - 1f, 0.01f)));

            RefreshRecovery(live, loaded);
            DrawGuide(matching, tint);
            DrawOrientation(matching);
            if (!matching)
            {
                Restore();
                return;
            }
            if (_renderers.Count == 0) Collect();
            _block.Clear();
            _block.SetColor("baseColorFactor", tint);
            _block.SetColor("_BaseColor", tint);
            _block.SetColor("_Color", tint);
            foreach (Renderer renderer in _renderers)
                if (renderer != null) renderer.SetPropertyBlock(_block);
            _tinted = true;
        }

        private void RefreshRecovery(bool live, bool loaded)
        {
            if (_recoveryPanel == null) return;
            bool unreachable = live && _state == "unreachable";
            bool show = _head != null && (_recoveryReason.Length > 0 || unreachable);
            _recoveryPanel.SetActive(show);
            if (!show) return;
            // Separate rows allow both arms to report a stop without covering each other.
            _recoveryPanel.transform.position = _head.transform.TransformPoint(
                new Vector3(0f, arm == "left" ? -0.04f : -0.27f, 1.3f));
            _recoveryPanel.transform.rotation = _head.transform.rotation;
            string hand = arm == "left" ? "LEFT HAND" : "RIGHT HAND";
            string cause = _recoveryReason == "translation_limit" ? "Movement limit reached"
                : _recoveryReason == "rotation_limit" ? "Rotation limit reached"
                : _recoveryReason == "tracking_timeout" ? "Tracking paused"
                : "Movement paused";
            _recoveryTitle.color = close;
            _recoveryTitle.text = hand + " — " + cause;
            if (!live || !loaded)
                _recoveryText.text = "Release the side grip.\nWait for the robot connection to return.";
            else if (unreachable)
            {
                _recoveryTitle.text = hand + " — Target out of reach";
                _recoveryText.text = "Move your hand back toward where you started,\nor release the grip and match the hand guide again.";
            }
            else if (_state == "ready" && _ready)
            {
                _recoveryTitle.text = hand + " — Ready to continue";
                _recoveryTitle.color = ready;
                _recoveryText.text = "Your hand is lined up.\nSqueeze the side grip to continue moving.";
            }
            else if (_state == "align_pose")
                _recoveryText.text = "Keep the side grip released. Rotate the white hand\noutline to match cyan, then squeeze when it turns green.";
            else if (_state == "tracking_lost")
                _recoveryText.text = "Release the side grip and bring the controller into view.\nWhen tracking returns, match the hand guide to continue.";
            else
                _recoveryText.text = "The arm is holding its position. Release the side grip,\nmatch the hand guide, then squeeze again when green.";
        }

        private void BuildRecoveryPanel()
        {
            _recoveryPanel = new GameObject("TeleopRecovery_" + arm, typeof(RectTransform), typeof(Canvas), typeof(Image));
            _recoveryPanel.transform.SetParent(transform, false);
            _recoveryPanel.transform.localScale = Vector3.one * 0.001f;
            _recoveryPanel.GetComponent<RectTransform>().sizeDelta = new Vector2(800f, 200f);
            _recoveryPanel.GetComponent<Canvas>().renderMode = RenderMode.WorldSpace;
            var background = _recoveryPanel.GetComponent<Image>();
            background.color = new Color(0.035f, 0.045f, 0.06f, 0.96f);
            background.raycastTarget = false;
            _recoveryTitle = RecoveryLabel("Title", 52f, 28f, 55f);
            _recoveryText = RecoveryLabel("Instructions", -30f, 25f, 110f);
            _recoveryPanel.SetActive(false);
        }

        private TextMeshProUGUI RecoveryLabel(string name, float y, float size, float height)
        {
            var go = new GameObject(name, typeof(RectTransform), typeof(TextMeshProUGUI));
            go.transform.SetParent(_recoveryPanel.transform, false);
            var text = go.GetComponent<TextMeshProUGUI>();
            text.rectTransform.anchoredPosition = new Vector2(0f, y);
            text.rectTransform.sizeDelta = new Vector2(760f, height);
            text.fontSize = size;
            text.alignment = TextAlignmentOptions.Center;
            text.color = Color.white;
            text.raycastTarget = false;
            return text;
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

        private static bool ReadPosition(JObject msg, out Vector3 point)
        {
            point = Vector3.zero;
            if ((string)msg?["header"]?["frame_id"] != "world") return false;
            JToken p = msg?["pose"]?["position"];
            if (p == null) return false;
            float x = (float?)p["x"] ?? float.NaN;
            float y = (float?)p["y"] ?? float.NaN;
            float z = (float?)p["z"] ?? float.NaN;
            if (!Finite(x) || !Finite(y) || !Finite(z)) return false;
            point = FrameConv.RosToUnity(x, y, z);
            return true;
        }

        private static bool ReadRotation(JObject msg, out Quaternion rotation)
        {
            rotation = Quaternion.identity;
            if ((string)msg?["header"]?["frame_id"] != "world") return false;
            JToken q = msg?["pose"]?["orientation"];
            float x = (float?)q?["x"] ?? float.NaN, y = (float?)q?["y"] ?? float.NaN;
            float z = (float?)q?["z"] ?? float.NaN, w = (float?)q?["w"] ?? float.NaN;
            if (!Finite(x) || !Finite(y) || !Finite(z) || !Finite(w)) return false;
            float norm = Mathf.Sqrt(x*x + y*y + z*z + w*w);
            if (Mathf.Abs(norm - 1f) > 0.1f) return false;
            rotation = FrameConv.RosToUnity(x/norm, y/norm, z/norm, w/norm);
            return true;
        }

        private void DrawOrientation(bool matching)
        {
            if (_wantedOutline == null || _currentOutline == null) return;
            bool show = matching && scene != null && Time.unscaledTime - _wantedAt <= StaleSeconds
                && Time.unscaledTime - _controllerAt <= StaleSeconds;
            _wantedOutline.enabled = _currentOutline.enabled = show;
            if (!show) return;
            Vector3 origin = scene.transform.TransformPoint(_controller);
            Quaternion current = scene.transform.rotation * _controllerRotation;
            Quaternion wanted = scene.transform.rotation * _wantedRotation;
            for (int i = 0; i < ControllerOutline.Length; ++i)
            {
                _currentOutline.SetPosition(i, origin + current * ControllerOutline[i]);
                _wantedOutline.SetPosition(i, origin + wanted * ControllerOutline[i]);
            }
            _currentOutline.startColor = _currentOutline.endColor = Color.white;
            _wantedOutline.startColor = _wantedOutline.endColor = _ready ? ready : Color.cyan;
        }

        private void DrawGuide(bool matching, Color tint)
        {
            if (_guide == null || _targetRing == null) return;
            bool show = matching && _positionTolerance > 0f && _head != null && scene != null
                && Time.unscaledTime - _controllerAt <= StaleSeconds
                && Time.unscaledTime - _toolAt <= StaleSeconds;
            _guide.enabled = _targetRing.enabled = show;
            if (!show) return;
            Vector3 target = scene.transform.TransformPoint(_tool);
            _guide.SetPosition(0, scene.transform.TransformPoint(_controller));
            _guide.SetPosition(1, target);
            _guide.startColor = _guide.endColor = tint;
            _targetRing.startColor = _targetRing.endColor = tint;
            for (int i = 0; i < _targetRing.positionCount; ++i)
            {
                float angle = i * Mathf.PI * 2f / _targetRing.positionCount;
                _targetRing.SetPosition(i, target + 0.035f *
                    (_head.transform.right * Mathf.Cos(angle) + _head.transform.up * Mathf.Sin(angle)));
            }
        }

        private void BuildGuide()
        {
            BuildRecoveryPanel();
            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null) return;
            _guideMaterial = new Material(shader);
            _guide = Line("ControllerToGripper", 2, false);
            _targetRing = Line("GripperTargetRing", 40, true);
            _currentOutline = Line("CurrentControllerOrientation", ControllerOutline.Length, false);
            _wantedOutline = Line("DesiredControllerOrientation", ControllerOutline.Length, false);
            _currentOutline.widthMultiplier = 0.002f;
            _wantedOutline.widthMultiplier = 0.004f;
        }

        private LineRenderer Line(string name, int points, bool loop)
        {
            var go = new GameObject(name);
            go.transform.SetParent(transform, false);
            var line = go.AddComponent<LineRenderer>();
            line.sharedMaterial = _guideMaterial;
            line.useWorldSpace = true;
            line.positionCount = points;
            line.loop = loop;
            line.widthMultiplier = 0.003f;
            line.enabled = false;
            return line;
        }

        private void OnDestroy()
        {
            Restore();
            Dispose(_recoveryPanel);
            if (_guide != null) Dispose(_guide.gameObject);
            if (_targetRing != null) Dispose(_targetRing.gameObject);
            if (_currentOutline != null) Dispose(_currentOutline.gameObject);
            if (_wantedOutline != null) Dispose(_wantedOutline.gameObject);
            Dispose(_guideMaterial);
        }

        private static void Dispose(Object value)
        {
            if (value == null) return;
            if (Application.isPlaying) Destroy(value);
            else DestroyImmediate(value);
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
            Restore();
            _renderers.Clear();
            _tinted = false;
        }
    }
}
