// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Hands;

namespace VrRos
{
    /// <summary>
    /// A ray from each controller, a highlight on whatever body it lands on, and that body's
    /// manifest index published to the PC.
    ///
    /// This exists because grabbing by proximity is not usable in a headset: it needs the
    /// controller physically inside a small radius of the target with nothing on screen saying
    /// whether it is close enough. Pointing shows the user what would be grabbed before they
    /// commit to grabbing it.
    ///
    /// The PC still decides what happens. This publishes a selection, not a command: vr::Grabber
    /// takes the index on <rawNs>/&lt;hand&gt;/target and applies its own spring-damper, so a body
    /// too heavy to move still will not move.
    /// </summary>
    public class VrPointer : MonoBehaviour
    {
        public RosBridge bridge;

        [Tooltip("Optional: when set, the namespace comes from its config file")]
        public VrConfig config;

        [Tooltip("The scene, which owns the body holders the ray hits")]
        public SceneLoader scene;

        [Tooltip("The tracked camera; hand rays are aimed from the eye through the pinch point")]
        public Camera head;

        /* Device and joint poses are reported in tracking space, which is the camera offset
         * object, not the XR Origin root: in device-space tracking the offset carries the eye
         * height, so converting through the root puts every ray a metre and a half out. */
        [Tooltip("The camera offset; poses from InputDevices and XR Hands are relative to this")]
        public Transform trackingSpace;

        public string rawNs = "/vr/raw";

        [Tooltip("Namespace the PC publishes into; the held body is reported there")]
        public string outNs = "/vr";

        [Tooltip("How far the ray reaches, in metres")]
        public float maxDistance = 8f;

        [Tooltip("Cast radius; 0 casts a bare ray. Widens what counts as a hit on small objects")]
        public float pointerRadius = 0.03f;

        [Tooltip("Weight on each new aim sample, 1 disables smoothing")]
        public float aimFilter = 0.35f;

        [Tooltip("Frames a new body must hold the ray before the selection moves to it")]
        public int switchFrames = 4;

        [Tooltip("How long the last aim survives a tracking dropout, in seconds")]
        public float trackingHoldSeconds = 0.25f;

        /* Only publishes on change, so this is a cap on how late the PC learns what the ray moved
         * onto, not a stream rate. Too low and a quick point-and-grab takes the previous body. */
        public float rateHz = 60f;

        public Color rayColour = new Color(0.35f, 0.7f, 1f, 0.9f);

        [Tooltip("What the ray is on: this is what a grab would take")]
        public Color highlightColour = new Color(1f, 0.85f, 0.2f, 1f);

        [Tooltip("What the PC reports as actually held, which is not always what was pointed at")]
        public Color heldColour = new Color(0.3f, 1f, 0.4f, 1f);

        /// <summary>Per-hand aim state: smoothed ray, and the debounced selection on it.</summary>
        private class Aim
        {
            public Vector3 origin;
            public Vector3 direction;
            public bool valid;
            public float lastValid;
            public int stable = -1;
            public int candidate = -1;
            public int candidateFrames;
        }

        private readonly Dictionary<string, Aim> _aim = new Dictionary<string, Aim>();

        private readonly Dictionary<string, LineRenderer> _rays =
            new Dictionary<string, LineRenderer>();
        private readonly Dictionary<string, int> _targets = new Dictionary<string, int>();
        private readonly Dictionary<string, int> _sent = new Dictionary<string, int>();

        // Restored when the tint moves off a body, so the scene keeps its exported colours.
        private readonly Dictionary<Renderer, Color> _originalColours =
            new Dictionary<Renderer, Color>();
        private readonly Dictionary<string, int> _held = new Dictionary<string, int>();
        private int _tintedBody = -1;
        private bool _tintedHeld;

        private float _nextPublish;
        private static readonly string[] Hands = { "left", "right" };

        [Tooltip("Thumb-to-index distance that counts as a pinch; matches GrabConf.pinch_close_m")]
        public float pinchCloseM = 0.025f;

        private readonly List<XRHandSubsystem> _subsystems = new List<XRHandSubsystem>();
        private XRHandSubsystem _handSubsystem;

        private bool HandMode => config != null && config.HandMode;

        // Falls back to this object so an unwired reference behaves as it did before device space.
        private Transform Space => trackingSpace != null ? trackingSpace : transform;

        private XRHand HandFor(string hand)
        {
            if (_handSubsystem == null || !_handSubsystem.running)
            {
                SubsystemManager.GetSubsystems(_subsystems);
                _handSubsystem = _subsystems.Count > 0 ? _subsystems[0] : null;
            }
            if (_handSubsystem == null) return default;
            return hand == "left" ? _handSubsystem.leftHand : _handSubsystem.rightHand;
        }

        private static bool JointPose(XRHand hand, XRHandJointID id, out Pose pose)
        {
            pose = default;
            return hand.isTracked && hand.GetJoint(id).TryGetPose(out pose);
        }

        private void Start()
        {
            if (config != null && config.Active != null)
            {
                rawNs = config.Active.rawNs;
                outNs = config.Active.outNs;
                maxDistance = config.Active.pointerRange;
            }

            foreach (string hand in Hands)
            {
                _rays[hand] = MakeRay(hand);
                _aim[hand] = new Aim();
                _targets[hand] = -1;
                _held[hand] = -1;
                _sent[hand] = int.MinValue; // so the first -1 is still published
                bridge.Advertise($"{rawNs}/{hand}/target", "std_msgs/msg/Int32");

                string h = hand;
                bridge.Subscribe($"{outNs}/{hand}/held", "std_msgs/msg/Int32",
                                 msg => _held[h] = msg == null ? -1 : (int)msg["data"]);
            }
        }

        private LineRenderer MakeRay(string hand)
        {
            var go = new GameObject($"Ray_{hand}");
            go.transform.SetParent(transform, false);

            var line = go.AddComponent<LineRenderer>();
            line.useWorldSpace = true;
            line.positionCount = 2;
            line.widthMultiplier = 0.004f;
            line.material = new Material(Shader.Find("Sprites/Default"));
            line.startColor = rayColour;
            line.endColor = rayColour;
            return line;
        }

        private void Update()
        {
            int pointed = -1;
            int held = -1;
            foreach (string hand in Hands)
            {
                int hit = Trace(hand);
                _targets[hand] = hit;
                if (hit > 0) pointed = hit;
                if (_held[hand] > 0) held = _held[hand];

                /* Show the grab the moment the button goes down rather than after the round trip
                 * to the PC and back. The PC is still the authority: if it refuses the body, the
                 * next /held says so and the colour drops back. */
                if (held < 0 && hit > 0 && GrabPressed(hand)) held = hit;
            }

            // Held wins: once something is in hand, that is the thing worth marking.
            Tint(held > 0 ? held : pointed, held > 0);

            if (rateHz > 0f && Time.unscaledTime < _nextPublish) return;
            _nextPublish = Time.unscaledTime + (rateHz > 0f ? 1f / rateHz : 0f);
            if (!bridge.IsConnected) return;

            foreach (string hand in Hands)
            {
                // Only on change: the PC holds the last value, so resending it every frame is noise.
                if (_targets[hand] == _sent[hand]) continue;
                _sent[hand] = _targets[hand];
                bridge.Publish($"{{\"op\":\"publish\",\"topic\":\"{rawNs}/{hand}/target\","
                               + $"\"msg\":{{\"data\":{_targets[hand].ToString(CultureInfo.InvariantCulture)}}}}}");
            }
        }

        /// <summary>What counts as "grabbing now", matching what vr::Grabber acts on per mode.</summary>
        /* Thumbstick click, matching grab_button on the PC. The grip button is the teleop
         * clutch, and one press must not both grab a body and engage an arm. */
        private bool GrabPressed(string hand)
        {
            if (HandMode)
            {
                XRHand h = HandFor(hand);
                if (!JointPose(h, XRHandJointID.ThumbTip, out Pose thumb)
                    || !JointPose(h, XRHandJointID.IndexTip, out Pose index))
                {
                    return false;
                }
                return Vector3.Distance(thumb.position, index.position) < pinchCloseM;
            }

            InputDevice device = InputDevices.GetDeviceAtXRNode(
                hand == "left" ? XRNode.LeftHand : XRNode.RightHand);
            return device.isValid
                   && device.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool click)
                   && click;
        }

        /// <summary>
        /// Where a hand points, in tracking space. The ray passes through the pinch point and is
        /// aimed from the eye, not along the hand: the hand's own axes run along the back of the
        /// hand and sit well above where the user believes they are pointing. Sighting from the
        /// eye through the fingers is what makes hand pointing land where it looks like it should,
        /// and the long eye-to-hand baseline means the few centimetres the fingers travel while
        /// closing move the aim by only a degree or two.
        /// </summary>
        private bool HandRay(string hand, out Vector3 origin, out Vector3 direction)
        {
            origin = default;
            direction = default;

            XRHand h = HandFor(hand);
            if (!JointPose(h, XRHandJointID.Wrist, out Pose wrist)) return false;

            origin = JointPose(h, XRHandJointID.ThumbTip, out Pose thumb)
                     && JointPose(h, XRHandJointID.IndexTip, out Pose index)
                       ? (thumb.position + index.position) * 0.5f
                       : wrist.position;

            if (head != null)
            {
                // The joints are tracking-space; the camera is not. Compare them in the world.
                Vector3 worldOrigin = Space.TransformPoint(origin);
                Vector3 fromEye = worldOrigin - head.transform.position;
                if (fromEye.sqrMagnitude > 1e-4f)
                {
                    direction = Space.InverseTransformDirection(fromEye.normalized);
                    return true;
                }
            }

            // No camera, or the hand is at the eye: fall back to the hand's own long axis.
            if (!JointPose(h, XRHandJointID.MiddleProximal, out Pose knuckle)) return false;
            Vector3 axis = knuckle.position - wrist.position;
            if (axis.sqrMagnitude < 1e-6f) return false;
            direction = axis.normalized;
            return true;
        }

        /// <summary>Casts one hand's ray and returns the body index under it, or -1.</summary>
        private int Trace(string hand)
        {
            LineRenderer line = _rays[hand];
            Aim aim = _aim[hand];

            bool got = HandMode ? HandRay(hand, out Vector3 local, out Vector3 localDir)
                                : ControllerRay(hand, out local, out localDir);

            if (got)
            {
                /* Hand joints in particular are noisy, and the noise is amplified by the length
                 * of the ray: a millimetre at the hand is centimetres at the far end. */
                if (aim.valid)
                {
                    local = Vector3.Lerp(aim.origin, local, aimFilter);
                    localDir = Vector3.Slerp(aim.direction, localDir, aimFilter).normalized;
                }
                aim.origin = local;
                aim.direction = localDir;
                aim.valid = true;
                aim.lastValid = Time.unscaledTime;
            }
            else if (!aim.valid || Time.unscaledTime - aim.lastValid > trackingHoldSeconds)
            {
                // Tracking has been gone long enough that the last aim is no longer believable.
                aim.valid = false;
                line.enabled = false;
                return Settle(aim, -1);
            }

            // Device and joint poses are in tracking space; the ray is drawn where the user is.
            Vector3 origin = Space.TransformPoint(aim.origin);
            Vector3 direction = Space.rotation * aim.direction;

            /* A sphere rather than a ray: the targets are centimetres across at arm's length, and
             * a zero-width ray demands more precision than hand tracking can deliver. */
            bool hit = pointerRadius > 0f
                         ? Physics.SphereCast(origin, pointerRadius, direction, out RaycastHit info,
                                              maxDistance)
                         : Physics.Raycast(origin, direction, out info, maxDistance);

            line.enabled = true;
            line.SetPosition(0, origin);
            line.SetPosition(1, hit ? info.point : origin + direction * maxDistance);

            int found = -1;
            if (hit)
            {
                VrBodyTag tag = info.collider.GetComponentInParent<VrBodyTag>();
                if (tag != null) found = tag.index;
            }
            return Settle(aim, found);
        }

        /// <summary>
        /// Debounces the selection. Without this the highlight flickers between two bodies that
        /// are close together, and a grab lands on whichever one happened to be under the ray on
        /// the frame the button went down.
        /// </summary>
        private int Settle(Aim aim, int found)
        {
            if (found == aim.stable)
            {
                aim.candidate = found;
                aim.candidateFrames = 0;
                return aim.stable;
            }

            if (found == aim.candidate) aim.candidateFrames++;
            else
            {
                aim.candidate = found;
                aim.candidateFrames = 1;
            }

            if (aim.candidateFrames >= switchFrames) aim.stable = aim.candidate;
            return aim.stable;
        }

        private static bool ControllerRay(string hand, out Vector3 origin, out Vector3 direction)
        {
            origin = default;
            direction = default;

            InputDevice device = InputDevices.GetDeviceAtXRNode(
                hand == "left" ? XRNode.LeftHand : XRNode.RightHand);
            if (!device.isValid
                || !device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 p)
                || !device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion q))
            {
                return false;
            }

            origin = p;
            direction = q * Vector3.forward;
            return true;
        }

        private void Tint(int index, bool isHeld)
        {
            if (index == _tintedBody && isHeld == _tintedHeld) return;

            foreach (var (renderer, colour) in _originalColours)
            {
                if (renderer != null) renderer.material.color = colour;
            }
            _originalColours.Clear();
            _tintedBody = index;
            _tintedHeld = isHeld;

            if (index < 0 || scene == null || scene.Bodies == null
                || index >= scene.Bodies.Length || scene.Bodies[index] == null)
            {
                return;
            }

            Color tint = isHeld ? heldColour : highlightColour;
            foreach (Renderer renderer in scene.Bodies[index].GetComponentsInChildren<Renderer>())
            {
                _originalColours[renderer] = renderer.material.color;
                renderer.material.color = tint;
            }
        }
    }
}
