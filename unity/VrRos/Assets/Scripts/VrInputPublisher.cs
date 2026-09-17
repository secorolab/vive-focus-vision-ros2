// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System.Globalization;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

namespace VrRos
{
    /// <summary>
    /// Publishes head and controller grip poses plus button state at display rate.
    ///
    /// Input is read through UnityEngine.XR.InputDevices / CommonUsages rather than any vendor
    /// SDK: the VIVE OpenXR plugin maps /interaction_profiles/htc/vive_focus3_controller onto
    /// exactly these usages, so the same code runs unchanged on other OpenXR headsets.
    ///
    /// Joy layout (sensor_msgs/Joy carries no schema, so this is the contract). The names are
    /// the ones printed in the Focus Vision manual, so that a binding can be discussed without
    /// translating:
    ///   buttons = [trigger, grip, A|X, B|Y, thumbstick_click, menu]
    ///   axes    = [thumbstick_x, thumbstick_y, trigger, grip]
    /// menu exists on the left controller only and reads 0 on the right; the VIVE button is
    /// reserved by the runtime and is not available to the application.
    /// </summary>
    public class VrInputPublisher : MonoBehaviour
    {
        public RosBridge bridge;
        public ClockSync clock;

        [Tooltip("Optional: when set, namespace, rate and frame come from its config file")]
        public VrConfig config;

        [Tooltip("Publish rate cap; 0 publishes every frame at the display rate")]
        public float maxRateHz = 90f;

        [Tooltip("The XR rig, which maps tracking space into the world the scene is drawn in")]
        public Transform rig;

        [Tooltip("Frame the poses are expressed in")]
        public string frameId = "world";

        [Tooltip("Namespace the raw poses are published into; InputNode reads from here")]
        public string rawNs = "/vr/raw";

        private readonly StringBuilder _sb = new StringBuilder(512);
        private float _nextPublish;

        private void Start()
        {
            if (config != null && config.Active != null)
            {
                rawNs = config.Active.rawNs;
                maxRateHz = config.Active.maxRateHz;
                frameId = config.Active.frameId;
            }

            /* Advertised in both modes even though publishing is gated: the mode can change at
             * runtime, and advertising on the switch would drop the first messages. */
            bridge.Advertise($"{rawNs}/head/pose", "geometry_msgs/msg/PoseStamped");
            foreach (string hand in new[] { "left", "right" })
            {
                bridge.Advertise($"{rawNs}/{hand}/pose", "geometry_msgs/msg/PoseStamped");
                bridge.Advertise($"{rawNs}/{hand}/joy", "sensor_msgs/msg/Joy");
            }
        }

        private void Update()
        {
            if (!bridge.IsConnected) return;
            if (maxRateHz > 0f && Time.unscaledTime < _nextPublish) return;
            _nextPublish = Time.unscaledTime + (maxRateHz > 0f ? 1f / maxRateHz : 0f);

            PublishPose($"{rawNs}/head/pose", InputDevices.GetDeviceAtXRNode(XRNode.CenterEye));
            if (config != null && config.HandMode) return;

            PublishHand("left", XRNode.LeftHand);
            PublishHand("right", XRNode.RightHand);
        }

        private void PublishHand(string hand, XRNode node)
        {
            InputDevice device = InputDevices.GetDeviceAtXRNode(node);
            if (!device.isValid) return;
            PublishPose($"{rawNs}/{hand}/pose", device);
            PublishJoy($"{rawNs}/{hand}/joy", device);
        }

        private void PublishPose(string topic, InputDevice device)
        {
            if (!device.isValid) return;
            if (!device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 p)) return;
            if (!device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion q)) return;

            /* Devices report in tracking space, which ignores where the rig has walked to. The
             * rig transform is what puts the hand where the user sees it in the scene. */
            if (rig != null)
            {
                p = rig.TransformPoint(p);
                q = rig.rotation * q;
            }

            Vector3 rp = FrameConv.UnityToRos(p);
            Quaternion rq = FrameConv.UnityToRos(q);

            _sb.Clear();
            _sb.Append("{\"op\":\"publish\",\"topic\":\"").Append(topic).Append("\",\"msg\":{");
            AppendHeader(_sb);
            _sb.Append(",\"pose\":{\"position\":{");
            AppendXyz(_sb, rp.x, rp.y, rp.z);
            _sb.Append("},\"orientation\":{");
            AppendXyz(_sb, rq.x, rq.y, rq.z);
            _sb.Append(",\"w\":").Append(F(rq.w));
            _sb.Append("}}}}");
            bridge.Publish(_sb.ToString());
        }

        private void PublishJoy(string topic, InputDevice device)
        {
            device.TryGetFeatureValue(CommonUsages.triggerButton, out bool trigger);
            device.TryGetFeatureValue(CommonUsages.gripButton, out bool grip);
            device.TryGetFeatureValue(CommonUsages.primaryButton, out bool aOrX);
            device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool bOrY);
            device.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool thumbstickClick);
            device.TryGetFeatureValue(CommonUsages.menuButton, out bool menu);
            device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 thumbstick);
            device.TryGetFeatureValue(CommonUsages.trigger, out float triggerValue);
            device.TryGetFeatureValue(CommonUsages.grip, out float gripValue);

            _sb.Clear();
            _sb.Append("{\"op\":\"publish\",\"topic\":\"").Append(topic).Append("\",\"msg\":{");
            AppendHeader(_sb);
            _sb.Append(",\"axes\":[").Append(F(thumbstick.x)).Append(',').Append(F(thumbstick.y))
               .Append(',').Append(F(triggerValue)).Append(',').Append(F(gripValue)).Append(']');
            _sb.Append(",\"buttons\":[").Append(B(trigger)).Append(',').Append(B(grip))
               .Append(',').Append(B(aOrX)).Append(',').Append(B(bOrY)).Append(',')
               .Append(B(thumbstickClick)).Append(',').Append(B(menu)).Append("]}}");
            bridge.Publish(_sb.ToString());
        }

        private void AppendHeader(StringBuilder sb)
        {
            clock.NowRos(out int sec, out uint nanosec);
            sb.Append("\"header\":{\"stamp\":{\"sec\":").Append(sec).Append(",\"nanosec\":")
              .Append(nanosec).Append("},\"frame_id\":\"").Append(frameId).Append("\"}");
        }

        private static void AppendXyz(StringBuilder sb, float x, float y, float z)
        {
            sb.Append("\"x\":").Append(F(x)).Append(",\"y\":").Append(F(y)).Append(",\"z\":")
              .Append(F(z));
        }

        // Invariant culture: a device set to a comma decimal separator would emit invalid JSON.
        private static string F(float v) => v.ToString("G7", CultureInfo.InvariantCulture);

        private static int B(bool v) => v ? 1 : 0;
    }
}
