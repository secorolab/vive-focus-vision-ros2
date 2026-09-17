// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

// VIVE.OpenXR's asmdef lists exactly these platforms, so a Linux player cannot reference the
// eye tracker at all; see VrRosSetup.BuildLinux.
#if UNITY_ANDROID || UNITY_EDITOR || UNITY_STANDALONE_WIN
#define VR_EYE_TRACKER
#endif

using System.Globalization;
using System.Text;
using UnityEngine;
#if VR_EYE_TRACKER
using UnityEngine.XR.OpenXR;
using VIVE.OpenXR.EyeTracker;
#endif

namespace VrRos
{
    /// <summary>
    /// Publishes eye gaze as vr/msg/EyeGaze: a ray per eye, each with its own validity, plus
    /// pupil diameter in millimetres.
    ///
    /// Requires the VIVE XR Eye Tracker (Beta) OpenXR feature, and eye tracking enabled in the
    /// headset's own settings - the runtime reports invalid gaze rather than failing when the
    /// user has not calibrated.
    ///
    /// Gaze is per-eye on purpose. Averaging to one ray here would throw away the case the
    /// recorded data most needs to explain: one eye tracking while the other does not.
    /// </summary>
    public class GazePublisher : MonoBehaviour
    {
        public RosBridge bridge;
        public ClockSync clock;

        [Tooltip("Optional: when set, namespace and frame come from its config file")]
        public VrConfig config;

        [Tooltip("Publish rate; the tracker itself runs at the display rate")]
        public float rateHz = 60f;

        public string rawNs = "/vr/raw";
        public string frameId = "world";

        [Tooltip("The XR rig, which maps tracking space into the world the scene is drawn in")]
        public Transform rig;

        private readonly StringBuilder _sb = new StringBuilder(1024);
        private float _nextPublish;
        private bool _warned;

        private void Start()
        {
            if (config != null && config.Active != null)
            {
                rawNs = config.Active.rawNs;
                frameId = config.Active.frameId;
                rateHz = config.Active.gazeRateHz;
            }

#if !VR_EYE_TRACKER
            Debug.LogWarning("gaze: no eye tracker on this platform; no gaze will be published");
#else
            _feature = OpenXRSettings.Instance != null
                ? OpenXRSettings.Instance.GetFeature<ViveEyeTracker>()
                : null;
            if (_feature == null || !_feature.enabled)
            {
                Debug.LogWarning("gaze: VIVE XR Eye Tracker feature is not enabled; "
                                 + "no gaze will be published");
                return;
            }
            bridge.Advertise($"{rawNs}/gaze", "vr/msg/EyeGaze");
#endif
        }

#if VR_EYE_TRACKER
        private ViveEyeTracker _feature;

        private void Update()
        {
            if (_feature == null || !_feature.enabled || !bridge.IsConnected) return;
            if (rateHz > 0f && Time.unscaledTime < _nextPublish) return;
            _nextPublish = Time.unscaledTime + (rateHz > 0f ? 1f / rateHz : 0f);

            if (!_feature.GetEyeGazeData(out XrSingleEyeGazeDataHTC[] gazes) || gazes == null
                || gazes.Length < 2)
            {
                if (!_warned)
                {
                    Debug.LogWarning("gaze: no data yet; is eye tracking enabled and calibrated "
                                     + "in the headset settings?");
                    _warned = true;
                }
                return;
            }

            // Pupil data is a separate call and can fail on its own while gaze stays valid.
            bool havePupil = _feature.GetEyePupilData(out XrSingleEyePupilDataHTC[] pupils)
                             && pupils != null && pupils.Length >= 2;

            clock.NowRos(out int sec, out uint nanosec);
            _sb.Clear();
            _sb.Append("{\"op\":\"publish\",\"topic\":\"").Append(rawNs)
               .Append("/gaze\",\"msg\":{\"header\":{\"stamp\":{\"sec\":").Append(sec)
               .Append(",\"nanosec\":").Append(nanosec).Append("},\"frame_id\":\"")
               .Append(frameId).Append("\"},");

            AppendEye("left", gazes[0]);
            _sb.Append(',');
            AppendEye("right", gazes[1]);

            _sb.Append(",\"left_valid\":").Append(B(gazes[0].isValid))
               .Append(",\"right_valid\":").Append(B(gazes[1].isValid));
            _sb.Append(",\"left_pupil_diameter_mm\":")
               .Append(F(havePupil ? pupils[0].pupilDiameter : 0f))
               .Append(",\"right_pupil_diameter_mm\":")
               .Append(F(havePupil ? pupils[1].pupilDiameter : 0f));
            _sb.Append(",\"left_pupil_valid\":")
               .Append(B(havePupil && pupils[0].isDiameterValid))
               .Append(",\"right_pupil_valid\":")
               .Append(B(havePupil && pupils[1].isDiameterValid));
            _sb.Append("}}");
            bridge.Publish(_sb.ToString());
        }

        private void AppendEye(string field, XrSingleEyeGazeDataHTC gaze)
        {
            // XrPosef is already right-handed Y-up like Unity's XR space, so the same conversion
            // used for controller poses applies.
            Vector3 up = new Vector3(gaze.gazePose.position.x, gaze.gazePose.position.y,
                                     gaze.gazePose.position.z);
            Quaternion uq = new Quaternion(gaze.gazePose.orientation.x, gaze.gazePose.orientation.y,
                                           gaze.gazePose.orientation.z, gaze.gazePose.orientation.w);
            if (rig != null)
            {
                up = rig.TransformPoint(up);
                uq = rig.rotation * uq;
            }

            Vector3 p = FrameConv.UnityToRos(up);
            Quaternion q = FrameConv.UnityToRos(uq);
            _sb.Append('"').Append(field).Append("\":{\"position\":{\"x\":").Append(F(p.x))
               .Append(",\"y\":").Append(F(p.y)).Append(",\"z\":").Append(F(p.z))
               .Append("},\"orientation\":{\"x\":").Append(F(q.x)).Append(",\"y\":").Append(F(q.y))
               .Append(",\"z\":").Append(F(q.z)).Append(",\"w\":").Append(F(q.w)).Append("}}");
        }
#endif

        private static string F(float v) => v.ToString("G7", CultureInfo.InvariantCulture);

        private static string B(bool v) => v ? "true" : "false";
    }
}
