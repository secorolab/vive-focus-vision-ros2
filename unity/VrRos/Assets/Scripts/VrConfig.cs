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
            public string rawNs = "/vr/raw";
            public string outNs = "/vr";
            public float maxRateHz = 90f;

            // Hands are the heaviest stream: 26 joints x 2 hands. Gaze is small but constant.
            public float handRateHz = 60f;
            public float gazeRateHz = 60f;
            public string originFrame = "vr_origin";
        }

        [Tooltip("Used when no config file exists on the device yet")]
        public Settings defaults = new Settings();

        public Settings Active { get; private set; }

        public string Path => System.IO.Path.Combine(Application.persistentDataPath,
                                                     "vr_config.json");

        private void Awake()
        {
            Active = Load();
            Debug.Log($"config: {Active.host}:{Active.port} raw={Active.rawNs} out={Active.outNs}"
                      + $" ({Path})");
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
