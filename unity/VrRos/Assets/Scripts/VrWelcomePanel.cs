// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace VrRos
{
    /// <summary>
    /// What the user sees before a world arrives: where the client is trying to connect, which
    /// input mode it is in, and what to run on the PC.
    ///
    /// It exists because the alternative is an empty floor, which looks the same whether the PC
    /// is unreachable, the launch was never started, or the scene failed to load — three problems
    /// with three different fixes. It comes back if the bridge drops, for the same reason.
    ///
    /// Built in code rather than authored as a prefab, so the scene stays reproducible from
    /// VrRosSetup in batch mode.
    /// </summary>
    public class VrWelcomePanel : MonoBehaviour
    {
        public RosBridge bridge;
        public VrConfig config;
        public SceneLoader scene;

        [Tooltip("Tracking space the panel hangs in; the camera offset, as the publishers use")]
        public Transform rig;

        [Tooltip("Metres in front of the rig, and metres below eye height")]
        public Vector2 offset = new Vector2(1.5f, 0.1f);

        [Tooltip("Panel width in metres")]
        public float width = 1.2f;

        [Tooltip("Seconds between refreshes; the contents change at human speed")]
        public float refreshSeconds = 0.25f;

        private TextMeshProUGUI _text;
        private GameObject _panel;
        private float _nextRefresh;

        private void Start()
        {
            Build();
            Refresh();
        }

        private void Update()
        {
            if (Time.unscaledTime < _nextRefresh) return;
            _nextRefresh = Time.unscaledTime + refreshSeconds;
            Refresh();
        }

        private void Refresh()
        {
            bool connected = bridge != null && bridge.IsConnected;
            bool loaded = scene != null && scene.Loaded;

            // A loaded world speaks for itself; a dropped bridge does not, so that brings it back.
            _panel.SetActive(!loaded || !connected);
            if (!_panel.activeSelf) return;

            VrConfig.Settings settings = config != null ? config.Active : null;
            string host = settings != null ? $"{settings.host}:{settings.port}" : "not configured";
            string mode = config != null && config.HandMode ? "hands" : "controllers";

            _text.text =
                "<size=190%><b>VrRos</b></size>\n\n"
                + Row("rosbridge", host, connected ? "connected" : "connecting…",
                      connected ? Good : Waiting)
                + Row("input", mode, "hold the left menu button to switch", Neutral)
                + Row("world", loaded ? "loaded" : "none", loaded ? "" : "waiting for /vr/scene",
                      loaded ? Good : Waiting)
                + "\n<size=72%><alpha=#99>On the PC:\n"
                + "  ros2 launch vr tracking.launch.py    poses only\n"
                + "  ros2 launch vr sim.launch.py model:=…    poses and a world\n\n"
                + $"config: {(config != null ? config.Path : "unknown")}</size>";
        }

        // Green is "this is working", amber "still waiting"; a setting is neither.
        private const string Good = "#7FD48A";
        private const string Waiting = "#E8C46A";
        private const string Neutral = "#DCE0E6";

        /// <summary>One status line: dim label, coloured value, dim note. pos aligns the columns.</summary>
        private static string Row(string label, string value, string note, string colour)
        {
            string tail = string.IsNullOrEmpty(note)
                            ? ""
                            : $"  <size=78%><alpha=#88>{note}<alpha=#FF></size>";
            return $"<alpha=#88>{label}<alpha=#FF><pos=26%><color={colour}>{value}</color>{tail}\n";
        }

        private void Build()
        {
            /* Under the rig, not the camera: head-locked text moves with every glance and is the
             * classic way to make someone ill. This hangs in front of where they are facing and
             * stays put while they look around it. */
            _panel = new GameObject("WelcomePanel", typeof(RectTransform), typeof(Canvas));
            var rect = (RectTransform)_panel.transform;
            rect.SetParent(rig != null ? rig : transform, false);
            rect.localPosition = new Vector3(0f, -offset.y, offset.x);
            rect.sizeDelta = new Vector2(1000f, 520f);
            // One layout unit is a millimetre of panel, so the sizes below read as a page.
            rect.localScale = Vector3.one * (width / 1000f);

            _panel.GetComponent<Canvas>().renderMode = RenderMode.WorldSpace;

            var background = new GameObject("Background", typeof(RectTransform), typeof(Image));
            background.transform.SetParent(rect, false);
            // Opaque: at 88% the skybox showed through and bleached the top of the text.
            background.GetComponent<Image>().color = new Color(0.09f, 0.10f, 0.12f);
            Stretch((RectTransform)background.transform, 0f);

            var text = new GameObject("Text", typeof(RectTransform), typeof(TextMeshProUGUI));
            text.transform.SetParent(rect, false);
            _text = text.GetComponent<TextMeshProUGUI>();
            _text.fontSize = 34f;
            _text.alignment = TextAlignmentOptions.TopLeft;
            _text.color = new Color(0.88f, 0.90f, 0.93f);
            Stretch((RectTransform)text.transform, 56f);
        }

        private static void Stretch(RectTransform rect, float margin)
        {
            rect.anchorMin = Vector2.zero;
            rect.anchorMax = Vector2.one;
            rect.offsetMin = new Vector2(margin, margin);
            rect.offsetMax = new Vector2(-margin, -margin);
        }
    }
}
