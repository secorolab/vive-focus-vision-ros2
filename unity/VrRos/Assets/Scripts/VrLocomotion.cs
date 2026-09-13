// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using UnityEngine;
using UnityEngine.XR;

namespace VrRos
{
    /// <summary>
    /// Stick locomotion for the XR rig: one stick glides along the direction the user is looking,
    /// the other turns in fixed steps.
    ///
    /// Read through UnityEngine.XR.InputDevices like the rest of the client, rather than through
    /// the XR Interaction Toolkit's locomotion providers, so there is one input path and no input
    /// action assets to keep in sync.
    ///
    /// Turning is snapped, not smooth: continuous yaw is the main cause of motion sickness in VR.
    /// </summary>
    public class VrLocomotion : MonoBehaviour
    {
        [Tooltip("The XR Origin to move; defaults to this object")]
        public Transform rig;

        [Tooltip("The tracked camera, used for the look direction")]
        public Camera head;

        public VrConfig config;

        [Tooltip("Hand whose stick drives movement")]
        public bool moveWithLeftHand = true;

        public float speed = 1.5f;
        public float snapDegrees = 45f;
        public float deadzone = 0.2f;

        [Tooltip("Vertical movement on the face buttons, for looking into a scene from above")]
        public float verticalSpeed = 1.0f;

        private bool _turnArmed = true;

        private void Start()
        {
            if (rig == null) rig = transform;
            if (head == null) head = Camera.main;
            if (config != null && config.Active != null)
            {
                speed = config.Active.moveSpeed;
                snapDegrees = config.Active.snapDegrees;

                /* The spawn point is given in ROS coordinates, because that is the frame the
                 * scene and every published pose are in. */
                Vector3 p = config.Active.spawnPosition;
                rig.SetPositionAndRotation(FrameConv.RosToUnity(p.x, p.y, p.z),
                                           Quaternion.Euler(0f, -config.Active.spawnYawDegrees,
                                                            0f));
            }
        }

        private void Update()
        {
            if (head == null) return;

            InputDevice move = InputDevices.GetDeviceAtXRNode(
                moveWithLeftHand ? XRNode.LeftHand : XRNode.RightHand);
            InputDevice turn = InputDevices.GetDeviceAtXRNode(
                moveWithLeftHand ? XRNode.RightHand : XRNode.LeftHand);

            Translate(move);
            SnapTurn(turn);
            Elevate(move);
        }

        private void Translate(InputDevice device)
        {
            if (!device.isValid) return;
            if (!device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick)) return;
            if (stick.magnitude < deadzone) return;

            // Flatten the look direction: pitching the head should not drive the rig into the
            // floor or the sky.
            Vector3 forward = Vector3.ProjectOnPlane(head.transform.forward, Vector3.up).normalized;
            Vector3 right = Vector3.ProjectOnPlane(head.transform.right, Vector3.up).normalized;

            rig.position += (forward * stick.y + right * stick.x) * speed * Time.deltaTime;
        }

        private void SnapTurn(InputDevice device)
        {
            if (!device.isValid) return;
            if (!device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick)) return;

            if (Mathf.Abs(stick.x) < deadzone)
            {
                _turnArmed = true;
                return;
            }
            if (!_turnArmed) return;
            _turnArmed = false;

            // Rotate about the head, not the rig origin, or the user swings around the room.
            rig.RotateAround(head.transform.position, Vector3.up,
                             Mathf.Sign(stick.x) * snapDegrees);
        }

        private void Elevate(InputDevice device)
        {
            if (!device.isValid) return;
            device.TryGetFeatureValue(CommonUsages.primaryButton, out bool up);
            device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool down);
            if (up == down) return;
            rig.position += Vector3.up * (up ? 1f : -1f) * verticalSpeed * Time.deltaTime;
        }
    }
}
