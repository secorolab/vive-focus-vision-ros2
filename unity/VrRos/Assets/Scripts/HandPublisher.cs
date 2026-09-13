// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Management;

namespace VrRos
{
    /// <summary>
    /// Publishes the 26 tracked hand joints per hand as a PoseArray, in XRHandJointID order.
    ///
    /// Read through Unity's XRHandSubsystem rather than VIVE's own gesture API: the VIVE OpenXR
    /// plugin implements the subsystem provider, so this is the same vendor-neutral choice made
    /// for controllers with CommonUsages. The VIVE XR Hand Tracking OpenXR feature must be
    /// enabled or the subsystem never reports a tracked hand.
    ///
    /// The ROS side turns each array into a TF frame per joint; InputNode holds the joint names
    /// and the parent table, so the order here is a contract and must not be reordered.
    /// Published only while a hand is actually tracked, at a rate well below display rate:
    /// 26 joints x 2 hands is the largest stream in the system.
    /// </summary>
    public class HandPublisher : MonoBehaviour
    {
        public RosBridge bridge;
        public ClockSync clock;

        [Tooltip("Optional: when set, namespace and frame come from its config file")]
        public VrConfig config;

        [Tooltip("Publish rate; hand joints are the heaviest topic here")]
        public float rateHz = 60f;

        public string rawNs = "/vr/raw";
        public string frameId = "vr_origin";

        private readonly StringBuilder _sb = new StringBuilder(4096);
        private readonly List<XRHandSubsystem> _subsystems = new List<XRHandSubsystem>();
        private XRHandSubsystem _hands;
        private float _nextPublish;
        private bool _advertised;

        private void Start()
        {
            if (config != null && config.Active != null)
            {
                rawNs = config.Active.rawNs;
                frameId = config.Active.originFrame;
                rateHz = config.Active.handRateHz;
            }
        }

        private void Update()
        {
            if (!bridge.IsConnected) return;

            if (_hands == null || !_hands.running)
            {
                // The subsystem only exists once the XR loader has started, which is after Start.
                SubsystemManager.GetSubsystems(_subsystems);
                _hands = _subsystems.Count > 0 ? _subsystems[0] : null;
                if (_hands == null) return;
            }

            if (!_advertised)
            {
                foreach (string hand in new[] { "left", "right" })
                {
                    bridge.Advertise($"{rawNs}/{hand}/joints", "geometry_msgs/msg/PoseArray");
                }
                _advertised = true;
            }

            if (rateHz > 0f && Time.unscaledTime < _nextPublish) return;
            _nextPublish = Time.unscaledTime + (rateHz > 0f ? 1f / rateHz : 0f);

            PublishHand("left", _hands.leftHand);
            PublishHand("right", _hands.rightHand);
        }

        private void PublishHand(string name, XRHand hand)
        {
            if (!hand.isTracked) return;

            _sb.Clear();
            _sb.Append("{\"op\":\"publish\",\"topic\":\"").Append(rawNs).Append('/').Append(name)
               .Append("/joints\",\"msg\":{");
            clock.NowRos(out int sec, out uint nanosec);
            _sb.Append("\"header\":{\"stamp\":{\"sec\":").Append(sec).Append(",\"nanosec\":")
               .Append(nanosec).Append("},\"frame_id\":\"").Append(frameId).Append("\"},");
            _sb.Append("\"poses\":[");

            bool first = true;
            for (int i = XRHandJointID.BeginMarker.ToIndex(); i < XRHandJointID.EndMarker.ToIndex();
                 ++i)
            {
                XRHandJoint joint = hand.GetJoint(XRHandJointIDUtility.FromIndex(i));

                // An untracked joint still occupies its slot: the array length is the contract.
                Vector3 p = Vector3.zero;
                Quaternion q = Quaternion.identity;
                if (joint.TryGetPose(out Pose pose))
                {
                    p = pose.position;
                    q = pose.rotation;
                }

                Vector3 rp = FrameConv.UnityToRos(p);
                Quaternion rq = FrameConv.UnityToRos(q);

                if (!first) _sb.Append(',');
                first = false;
                _sb.Append("{\"position\":{\"x\":").Append(F(rp.x)).Append(",\"y\":")
                   .Append(F(rp.y)).Append(",\"z\":").Append(F(rp.z))
                   .Append("},\"orientation\":{\"x\":").Append(F(rq.x)).Append(",\"y\":")
                   .Append(F(rq.y)).Append(",\"z\":").Append(F(rq.z)).Append(",\"w\":")
                   .Append(F(rq.w)).Append("}}");
            }
            _sb.Append("]}}");
            bridge.Publish(_sb.ToString());
        }

        private static string F(float v) => v.ToString("G7", CultureInfo.InvariantCulture);
    }
}
