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

        [Tooltip("UDP port the PC's vive_vr_discovery answers on; 0 makes the configured host final")]
        public int discoveryPort = 9091;

        [Tooltip("Seconds between reconnect attempts")]
        public float reconnectInterval = 2f;

        public bool IsConnected => _ws != null && _ws.State == WebSocketState.Open;

        /// <summary>True while a discovery probe is out; the welcome panel says so.</summary>
        public bool IsSearching => _probing;

        [Tooltip("Most handler calls per frame; the rest wait, so a backlog cannot stall a frame")]
        public int maxHandlersPerFrame = 24;

        private ClientWebSocket _ws;
        private CancellationTokenSource _cancel;
        private readonly ConcurrentDictionary<string, Sink> _sinks =
            new ConcurrentDictionary<string, Sink>();
        private readonly List<Sink> _flushOrder = new List<Sink>();
        private readonly List<(string topic, string type)> _advertised =
            new List<(string, string)>();
        private readonly List<(string topic, string type)> _subscribed =
            new List<(string, string)>();
        private float _nextConnectAttempt;
        private bool _probing;
        private volatile VrDiscovery.Reply _found;
        private int _failures;

        private void Start()
        {
            if (config != null && config.Active != null)
            {
                host = config.Active.host;
                port = config.Active.port;
                discoveryPort = config.Active.discoveryPort;
            }
        }

        private void Update()
        {
            if (_found != null) AdoptDiscovered();

            if (!IsConnected && Time.unscaledTime >= _nextConnectAttempt)
            {
                _nextConnectAttempt = Time.unscaledTime + reconnectInterval;
                _ = ConnectAsync();

                /* Only once the configured address has actually failed. Asking first would let
                 * whatever answers the broadcast outrank a deliberate setting. */
                if (_failures > 0 && discoveryPort > 0 && !_probing)
                {
                    _probing = true;
                    _ = Task.Run(ProbeAsync);
                }
            }

            /* Parsing and decoding already happened on the socket task. What is left here is only
             * the part that cannot leave this thread - writing to Transforms - and it is budgeted,
             * because an unbounded drain makes a slow frame produce a bigger backlog, which makes
             * the next frame slower again. */
            int budget = maxHandlersPerFrame;
            for (int i = 0; i < _flushOrder.Count && budget > 0; i++)
            {
                budget -= _flushOrder[i].Flush(budget);
            }
        }

        /// <summary>Parses and decodes one frame. Runs on the socket task, never on the main thread.</summary>
        private void Route(string raw)
        {
            JObject envelope;
            try
            {
                envelope = JObject.Parse(raw);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"rosbridge: unparseable frame: {e.Message}");
                return;
            }

            if ((string)envelope["op"] != "publish") return;
            string topic = (string)envelope["topic"];
            if (topic == null || !_sinks.TryGetValue(topic, out Sink sink)) return;

            try
            {
                sink.Decode(envelope["msg"] as JObject);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"rosbridge: {topic} decode failed: {e.Message}");
            }
        }

        public void Advertise(string topic, string type)
        {
            _advertised.Add((topic, type));
            SendRaw($"{{\"op\":\"advertise\",\"topic\":\"{topic}\",\"type\":\"{type}\"}}");
        }

        /// <summary>
        /// Every message reaches the handler, on the main thread. For topics that carry events.
        /// </summary>
        public void Subscribe(string topic, string type, Action<JObject> handler)
            => Add(topic, type, new QueuedSink(handler));

        /// <summary>
        /// Decodes off the main thread; every message is applied, in order, on it.
        ///
        /// <paramref name="decode"/> runs on the socket task, so it must not touch the Unity API;
        /// <paramref name="apply"/> is where Transforms belong. A frame that carries only what
        /// changed cannot be skipped: its bodies would stay stale until the next full frame.
        /// </summary>
        public void Subscribe<T>(string topic, string type, Func<JObject, T> decode, Action<T> apply)
            => Add(topic, type, new DecodedSink<T>(decode, apply));

        private void Add(string topic, string type, Sink sink)
        {
            _sinks[topic] = sink;
            _flushOrder.Add(sink);
            _subscribed.Add((topic, type));
            SendRaw($"{{\"op\":\"subscribe\",\"topic\":\"{topic}\",\"type\":\"{type}\"}}");
        }

        /// <summary>Decoded off the socket task, applied on the main thread.</summary>
        private abstract class Sink
        {
            public abstract void Decode(JObject msg);

            /// <summary>Applies at most <paramref name="budget"/> messages; returns how many.</summary>
            public abstract int Flush(int budget);
        }

        private sealed class QueuedSink : Sink
        {
            private readonly Action<JObject> _handler;
            private readonly ConcurrentQueue<JObject> _queue = new ConcurrentQueue<JObject>();

            public QueuedSink(Action<JObject> handler) { _handler = handler; }

            public override void Decode(JObject msg) => _queue.Enqueue(msg);

            public override int Flush(int budget)
            {
                int n = 0;
                while (n < budget && _queue.TryDequeue(out JObject msg))
                {
                    _handler(msg);
                    n++;
                }
                return n;
            }
        }

        private sealed class DecodedSink<T> : Sink
        {
            private readonly Func<JObject, T> _decode;
            private readonly Action<T> _apply;
            private readonly ConcurrentQueue<T> _queue = new ConcurrentQueue<T>();

            public DecodedSink(Func<JObject, T> decode, Action<T> apply)
            {
                _decode = decode;
                _apply = apply;
            }

            public override void Decode(JObject msg)
            {
                T decoded = _decode(msg);
                if (decoded != null) _queue.Enqueue(decoded);
            }

            public override int Flush(int budget)
            {
                int n = 0;
                while (n < budget && _queue.TryDequeue(out T value))
                {
                    _apply(value);
                    n++;
                }
                return n;
            }
        }

        /// <summary>Publishes a pre-built rosbridge envelope.</summary>
        public void Publish(string envelopeJson) => SendRaw(envelopeJson);

        /// <summary>
        /// Calls a service with an empty request, and ignores the response. Enough for a Trigger;
        /// anything that needs the reply would have to match up the optional call id.
        /// </summary>
        public void CallService(string service) =>
            SendRaw($"{{\"op\":\"call_service\",\"service\":\"{service}\",\"args\":{{}}}}");

        /// <summary>Runs off the main thread; the answer is picked up by the next Update.</summary>
        private async Task ProbeAsync()
        {
            try
            {
                _found = await VrDiscovery.ProbeAsync(discoveryPort, 500).ConfigureAwait(false);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"discovery: probe failed: {e.Message}");
            }
            finally
            {
                _probing = false;
            }
        }

        private void AdoptDiscovered()
        {
            VrDiscovery.Reply found = _found;
            _found = null;
            if (found.host == host && found.port == port) return;

            Debug.Log($"discovery: {host}:{port} -> {found.host}:{found.port}");
            host = found.host;
            port = found.port;
            if (config != null) config.SaveHost(host, port);
            _nextConnectAttempt = 0f; // this frame, not two seconds from now
        }

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
                _failures = 0;

                // A reconnect has to restate everything; the server keeps no client state.
                foreach (var (topic, type) in _advertised)
                {
                    SendRaw($"{{\"op\":\"advertise\",\"topic\":\"{topic}\",\"type\":\"{type}\"}}");
                }
                foreach (var (topic, type) in _subscribed)
                {
                    SendRaw($"{{\"op\":\"subscribe\",\"topic\":\"{topic}\",\"type\":\"{type}\"}}");
                }

                /* Off the Unity synchronization context, or every await resumes on the main
                 * thread and the loop reads exactly one message per rendered frame: at 30 fps
                 * a 60 Hz stream falls behind and plays back late. */
                ClientWebSocket socket = _ws;
                CancellationToken token = _cancel.Token;
                _ = Task.Run(() => ReceiveLoopAsync(socket, token));
            }
            catch (Exception e)
            {
                _failures++;
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
                    var result = await socket.ReceiveAsync(new ArraySegment<byte>(buffer), token)
                                             .ConfigureAwait(false);
                    if (result.MessageType == WebSocketMessageType.Close) break;
                    message.Append(Encoding.UTF8.GetString(buffer, 0, result.Count));
                    if (!result.EndOfMessage) continue; // a frame larger than the buffer
                    Route(message.ToString());
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
