// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// Estimates the offset between the headset's clock and the PC's ROS clock so recorded
    /// controller stamps line up with sim state in a rosbag.
    ///
    /// This is a one-way estimate from /vive_vr/pc_time: the estimate is the smallest
    /// (local - remote) difference over a sliding window, which removes queueing jitter but not
    /// the constant one-way transit time, so stamps are biased late by roughly half the RTT
    /// (single-digit milliseconds on 5/6 GHz Wi-Fi). If demonstration data ever needs better
    /// than that, the fix is a round trip: publish a ping the PC echoes with both stamps.
    ///
    /// The offset only ever slews: a step backwards used to freeze every consumer that rejects
    /// stamps older than the last one it saw.
    /// </summary>
    public class ClockSync : MonoBehaviour
    {
        public RosBridge bridge;

        [Tooltip("Optional: when set, the topic namespace comes from its config file")]
        public VrConfig config;

        [Tooltip("Namespace the PC publishes its clock into")]
        public string outNs = "/vive_vr";

        [Tooltip("Length of the sliding window the best sample is taken over, in seconds")]
        public float windowSeconds = 30f;

        [Tooltip("Fastest the offset may be corrected, in seconds per second")]
        public double maxOffsetSlewPerSecond = 0.005;

        public bool HasOffset { get; private set; }

        /// <summary>Seconds to add to local time to get PC ROS time.</summary>
        public double Offset { get; private set; }

        private readonly struct Sample
        {
            public readonly float Time;
            public readonly double Delta;

            public Sample(float time, double delta)
            {
                Time = time;
                Delta = delta;
            }
        }

        // Candidates for the window minimum, oldest first and increasing in delta, so the front
        // is the smallest difference still inside the window.
        private readonly List<Sample> _window = new List<Sample>();
        private float _lastUpdate;

        /* Past this it is the suspend backlog, not jitter, and slewing it out would take hours. */
        private const double SnapSeconds = 1.0;

        private void Start()
        {
            if (config != null && config.Active != null) outNs = config.Active.outNs;
            bridge.Subscribe($"{outNs}/pc_time", "builtin_interfaces/msg/Time", OnPcTime);
        }

        private void OnPcTime(JObject msg)
        {
            if (msg == null) return;
            double remote = (int)msg["sec"] + (uint)msg["nanosec"] * 1e-9;
            float now = Time.unscaledTime;
            double delta = LocalSeconds() - remote;

            while (_window.Count > 0 && _window[_window.Count - 1].Delta >= delta)
            {
                _window.RemoveAt(_window.Count - 1);
            }
            _window.Add(new Sample(now, delta));
            while (_window.Count > 1 && now - _window[0].Time > windowSeconds)
            {
                _window.RemoveAt(0);
            }

            double wanted = -_window[0].Delta;
            double change = wanted - Offset;
            if (!HasOffset || Math.Abs(change) > SnapSeconds)
            {
                Offset = wanted;
                HasOffset = true;
                _lastUpdate = now;
                return;
            }

            double step = maxOffsetSlewPerSecond * Math.Max(0.0, now - _lastUpdate);
            _lastUpdate = now;
            if (change > step) change = step;
            else if (change < -step) change = -step;
            Offset += change;
        }

        /// <summary>Current PC ROS time, split the way builtin_interfaces/Time carries it.</summary>
        public void NowRos(out int sec, out uint nanosec)
        {
            double t = LocalSeconds() + Offset;
            sec = (int)Math.Floor(t);
            nanosec = (uint)Math.Round((t - sec) * 1e9);
            if (nanosec >= 1000000000u)
            {
                // Rounding can carry into the next second; Time messages reject nanosec >= 1e9.
                nanosec -= 1000000000u;
                sec += 1;
            }
        }

        private static double LocalSeconds() =>
            DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
    }
}
