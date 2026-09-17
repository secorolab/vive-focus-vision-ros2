// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System;
using System.IO;
using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// Every address and topic name the client uses, read from a JSON file on the device so the
    /// PC's address or a topic layout can change without rebuilding the APK.
    ///
    /// The file lives at Application.persistentDataPath/vr_config.json, which on Android is
    /// /sdcard/Android/data/&lt;package&gt;/files/. Push a new one with:
    ///   adb push vr_config.json /sdcard/Android/data/sh.vamsi.vrros/files/vr_config.json
    /// If it is missing, the inspector defaults below are used and written out, so the file to
    /// edit always exists after the first run.
    ///
    /// Keep rawNs and outNs consistent with config/vr.yaml on the ROS side; they are the same
    /// contract seen from the two ends.
    /// </summary>
    public class VrConfig : MonoBehaviour
    {
        [Serializable]
        public class Settings
        {
            public string host = "192.168.1.10";
            public int port = 9090;

            // Where the PC's vr_discovery answers. 0 turns discovery off and makes host final.
            public int discoveryPort = 9091;
            public string rawNs = "/vr/raw";
            public string outNs = "/vr";
            public float maxRateHz = 90f;

            // Hands are the heaviest stream: 26 joints x 2 hands. Gaze is small but constant.
            public float handRateHz = 60f;
            public float gazeRateHz = 60f;

            public float moveSpeed = 1.5f;

            // One stick does both: push to walk, tilt to turn.
            public float turnSpeedDegPerSec = 90f;

            // How far the selection ray reaches, in metres.
            public float pointerRange = 8f;

            /* Standing eye height. The app tracks in device space so it needs no play area, and
             * the cost of that is that nothing measures the floor: this is where it is assumed. */
            public float eyeHeight = 1.6f;

            // Where the user starts in the world, in ROS coordinates. Everything is simulated,
            // so this is a free choice: put it clear of the scene rather than inside a table.
            public Vector3 spawnPosition = new Vector3(-1.5f, 0f, 0f);
            public float spawnYawDegrees = 0f;

            public string frameId = "world";

            // Restores the scene; matches the SceneNode's service in config/vr.yaml.
            public string resetService = "/vr_scene/reset";

            /* "controllers" or "hands". The runtime will not report hands while a controller is
             * awake, and publishing both would put two competing grab sources on one topic set. */
            public string inputMode = "controllers";
        }

        /* Live, not just what the file said: the menu button switches between controllers and
         * hands during a session, and the publishers read this every frame. */
        public bool HandMode { get; private set; }

        public void SetHandMode(bool on)
        {
            if (HandMode == on) return;
            HandMode = on;
            Debug.Log($"mode: {(on ? "hands" : "controllers")}");
        }

        [Tooltip("Used when no config file exists on the device yet")]
        public Settings defaults = new Settings();

        public Settings Active { get; private set; }

        public string Path => System.IO.Path.Combine(Application.persistentDataPath,
                                                     "vr_config.json");

        private void Awake()
        {
            Active = Load();
            HandMode = Active.inputMode.Equals("hands", StringComparison.OrdinalIgnoreCase);
            Debug.Log($"config: {Active.host}:{Active.port} raw={Active.rawNs} out={Active.outNs}"
                      + $" mode={Active.inputMode} ({Path})");
        }

        /// <summary>Writes a discovered address back, so the next start connects without probing.</summary>
        public void SaveHost(string host, int port)
        {
            if (Active == null) return;
            Active.host = host;
            Active.port = port;
            try
            {
                File.WriteAllText(Path, JsonUtility.ToJson(Active, true));
                Debug.Log($"config: saved {host}:{port} to {Path}");
            }
            catch (Exception e)
            {
                Debug.LogWarning($"config: could not write {Path}: {e.Message}");
            }
        }

        private Settings Load()
        {
            try
            {
                if (File.Exists(Path))
                {
                    var loaded = JsonUtility.FromJson<Settings>(File.ReadAllText(Path));
                    if (loaded != null) return loaded;
                    Debug.LogWarning($"config: {Path} is not valid JSON; using defaults");
                }
                else
                {
                    // Write it out so there is something to edit rather than something to guess.
                    File.WriteAllText(Path, JsonUtility.ToJson(defaults, true));
                    Debug.Log($"config: wrote defaults to {Path}");
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"config: could not read or write {Path}: {e.Message}");
            }
            return defaults;
        }
    }
}
