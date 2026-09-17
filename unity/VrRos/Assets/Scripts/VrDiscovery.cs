// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// Finds the PC on the local network, so a changed address is not a file to edit.
    ///
    /// One UDP broadcast, and the first matching reply wins. The PC's `vive_vr_discovery` answers
    /// unicast, which is what makes this work on Android without a MulticastLock: a device only
    /// needs that permission to receive traffic that was not addressed to it.
    ///
    /// It cannot cross a router, and access points with client isolation drop it. That is why
    /// it never overrides a working address — see RosBridge — and why vr_config.json stays the
    /// manual way in.
    /// </summary>
    public static class VrDiscovery
    {
        [Serializable]
        public class Reply
        {
            public string host;
            public int port;
        }

        // Must match MAGIC in scripts/vive_vr_discovery.
        private const string Magic = "vrros-discover-1";

        /// <summary>
        /// Broadcasts one probe and waits up to timeoutMs for an answer. Returns null if nothing
        /// replies, which is the ordinary case on a network that blocks broadcast.
        /// </summary>
        public static async Task<Reply> ProbeAsync(int discoveryPort, int timeoutMs)
        {
            using (var udp = new UdpClient(AddressFamily.InterNetwork))
            {
                udp.EnableBroadcast = true;
                byte[] probe = Encoding.ASCII.GetBytes(Magic);
                var target = new IPEndPoint(IPAddress.Broadcast, discoveryPort);
                await udp.SendAsync(probe, probe.Length, target).ConfigureAwait(false);

                Task<UdpReceiveResult> receive = udp.ReceiveAsync();
                Task finished = await Task.WhenAny(receive, Task.Delay(timeoutMs))
                                          .ConfigureAwait(false);
                if (finished != receive) return null;

                string text = Encoding.UTF8.GetString(receive.Result.Buffer);
                if (!text.StartsWith(Magic, StringComparison.Ordinal)) return null;

                var reply = JsonUtility.FromJson<Reply>(text.Substring(Magic.Length).Trim());
                if (reply == null || string.IsNullOrEmpty(reply.host) || reply.port <= 0)
                {
                    Debug.LogWarning($"discovery: unusable reply: {text}");
                    return null;
                }
                return reply;
            }
        }
    }
}
