// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// Estimates the offset between the headset's clock and the PC's ROS clock so recorded
    /// controller stamps line up with sim state in a rosbag.
    ///
    /// This is a one-way estimate from /vr/pc_time: it keeps the sample with the smallest
    /// (local - remote) difference seen in a sliding window, which removes queueing jitter but
    /// not the constant one-way transit time, so stamps are biased late by roughly half the RTT
    /// (single-digit milliseconds on 5/6 GHz Wi-Fi). If demonstration data ever needs better
    /// than that, the fix is a round trip: publish a ping the PC echoes with both stamps.
    /// </summary>
    public class ClockSync : MonoBehaviour
    {
        public RosBridge bridge;

        [Tooltip("Optional: when set, the topic namespace comes from its config file")]
        public VrConfig config;

        [Tooltip("Namespace the PC publishes its clock into")]
        public string outNs = "/vive_vr";

        [Tooltip("Seconds after which the best sample is discarded and re-estimated")]
        public float windowSeconds = 30f;

        public bool HasOffset { get; private set; }

        /// <summary>Seconds to add to local time to get PC ROS time.</summary>
        public double Offset { get; private set; }

        private double _bestDelta = double.MaxValue;
        private float _windowStart;

        private void Start()
        {
            if (config != null && config.Active != null) outNs = config.Active.outNs;
            bridge.Subscribe($"{outNs}/pc_time", "builtin_interfaces/msg/Time", OnPcTime);
        }

        private void OnPcTime(JObject msg)
        {
            if (msg == null) return;
            double remote = (int)msg["sec"] + (uint)msg["nanosec"] * 1e-9;
            double local = LocalSeconds();
            double delta = local - remote;

            if (Time.unscaledTime - _windowStart > windowSeconds)
            {
                _windowStart = Time.unscaledTime;
                _bestDelta = double.MaxValue;
            }

            if (delta < _bestDelta)
            {
                _bestDelta = delta;
                Offset = -delta;
                HasOffset = true;
            }
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
