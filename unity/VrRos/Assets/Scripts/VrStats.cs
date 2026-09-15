// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// Logs the client's actual frame rate.
    ///
    /// Worth its few lines: the rate of a published topic is not the frame rate. It is the rate
    /// that survives the WebSocket and a Python rosbridge over Wi-Fi, and reading one for the
    /// other sends you optimising a renderer that was never the problem.
    /// </summary>
    public class VrStats : MonoBehaviour
    {
        [Tooltip("Seconds between reports; 0 disables")]
        public float intervalSeconds = 3f;

        [Tooltip("Optional: reports how many pose frames were applied, which the fps alone hides")]
        public BodyPoseApplier poses;

        private int _frames;
        private float _elapsed;
        private int _lastApplied;

        private void Update()
        {
            if (intervalSeconds <= 0f) return;

            _frames++;
            _elapsed += Time.unscaledDeltaTime;
            if (_elapsed < intervalSeconds) return;

            float fps = _frames / _elapsed;
            string applied = "";
            if (poses != null)
            {
                int n = poses.Received;
                applied = $", poses {(n - _lastApplied) / _elapsed:F1}/s applied";
                _lastApplied = n;
            }
            Debug.Log($"stats: {fps:F1} fps ({1000f / fps:F1} ms/frame), "
                      + $"eye buffer {UnityEngine.XR.XRSettings.eyeTextureWidth}"
                      + $"x{UnityEngine.XR.XRSettings.eyeTextureHeight}, "
                      + $"stereo {UnityEngine.XR.XRSettings.stereoRenderingMode}{applied}");
            _frames = 0;
            _elapsed = 0f;
        }
    }
}
