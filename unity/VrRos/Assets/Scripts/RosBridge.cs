// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// rosbridge v2 client: advertise/publish/subscribe over one WebSocket.
    /// Outgoing messages are built as strings by the callers, because the publish path runs at
    /// display rate and serializing through a JSON object model there allocates every frame.
    /// </summary>
    public class RosBridge : MonoBehaviour
    {
        [Tooltip("Optional: when set, host and port come from its config file instead")]
        public VrConfig config;

        [Tooltip("IP of the machine running rosbridge_websocket; overridden by config")]
        public string host = "192.168.1.10";

        public int port = 9090;

        [Tooltip("Seconds between reconnect attempts")]
        public float reconnectInterval = 2f;

        public bool IsConnected => _ws != null && _ws.State == WebSocketState.Open;

        private ClientWebSocket _ws;
        private CancellationTokenSource _cancel;
        private readonly ConcurrentQueue<string> _inbox = new ConcurrentQueue<string>();
        private readonly Dictionary<string, Action<JObject>> _handlers =
            new Dictionary<string, Action<JObject>>();
        private readonly List<(string topic, string type)> _advertised =
            new List<(string, string)>();
        private readonly List<(string topic, string type)> _subscribed =
            new List<(string, string)>();
        private float _nextConnectAttempt;

        private void Start()
        {
            if (config != null && config.Active != null)
            {
                host = config.Active.host;
                port = config.Active.port;
            }
        }

        private void Update()
        {
            if (!IsConnected && Time.unscaledTime >= _nextConnectAttempt)
            {
                _nextConnectAttempt = Time.unscaledTime + reconnectInterval;
                _ = ConnectAsync();
            }

            // Handlers touch Transforms, so they run on the main thread, never on the socket task.
            while (_inbox.TryDequeue(out string raw))
            {
                JObject envelope;
                try
                {
                    envelope = JObject.Parse(raw);
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"rosbridge: unparseable frame: {e.Message}");
                    continue;
                }

                if ((string)envelope["op"] != "publish") continue;
                string topic = (string)envelope["topic"];
                if (topic != null && _handlers.TryGetValue(topic, out var handler))
                {
                    handler(envelope["msg"] as JObject);
                }
            }
        }

        public void Advertise(string topic, string type)
        {
            _advertised.Add((topic, type));
            SendRaw($"{{\"op\":\"advertise\",\"topic\":\"{topic}\",\"type\":\"{type}\"}}");
        }

        public void Subscribe(string topic, string type, Action<JObject> handler)
        {
            _handlers[topic] = handler;
            _subscribed.Add((topic, type));
            SendRaw($"{{\"op\":\"subscribe\",\"topic\":\"{topic}\",\"type\":\"{type}\"}}");
        }

        /// <summary>Publishes a pre-built rosbridge envelope.</summary>
        public void Publish(string envelopeJson) => SendRaw(envelopeJson);

        private async Task ConnectAsync()
        {
            try
            {
                _cancel?.Cancel();
                _ws?.Dispose();
                _ws = new ClientWebSocket();
                _cancel = new CancellationTokenSource();
                await _ws.ConnectAsync(new Uri($"ws://{host}:{port}"), _cancel.Token);
                Debug.Log($"rosbridge: connected to ws://{host}:{port}");

                // A reconnect has to restate everything; the server keeps no client state.
                foreach (var (topic, type) in _advertised)
                {
                    SendRaw($"{{\"op\":\"advertise\",\"topic\":\"{topic}\",\"type\":\"{type}\"}}");
                }
                foreach (var (topic, type) in _subscribed)
                {
                    SendRaw($"{{\"op\":\"subscribe\",\"topic\":\"{topic}\",\"type\":\"{type}\"}}");
                }

                _ = ReceiveLoopAsync(_ws, _cancel.Token);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"rosbridge: connect failed: {e.Message}");
            }
        }

        private async Task ReceiveLoopAsync(ClientWebSocket socket, CancellationToken token)
        {
            var buffer = new byte[1 << 16];
            var message = new StringBuilder();
            try
            {
                while (socket.State == WebSocketState.Open && !token.IsCancellationRequested)
                {
                    var result = await socket.ReceiveAsync(new ArraySegment<byte>(buffer), token);
                    if (result.MessageType == WebSocketMessageType.Close) break;
                    message.Append(Encoding.UTF8.GetString(buffer, 0, result.Count));
                    if (!result.EndOfMessage) continue; // a frame larger than the buffer
                    _inbox.Enqueue(message.ToString());
                    message.Clear();
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"rosbridge: receive loop ended: {e.Message}");
            }
        }

        private void SendRaw(string json)
        {
            if (!IsConnected) return;
            var bytes = Encoding.UTF8.GetBytes(json);
            try
            {
                _ = _ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true,
                                  _cancel.Token);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"rosbridge: send failed: {e.Message}");
            }
        }

        private void OnDestroy()
        {
            _cancel?.Cancel();
            _ws?.Dispose();
        }
    }
}
