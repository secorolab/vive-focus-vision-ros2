// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using UnityEngine;
using UnityEngine.XR;

namespace VrRos
{
    /// <summary>
    /// Stick locomotion for the XR rig, on one stick: pushing it glides along the direction the
    /// user is looking, tilting it turns continuously.
    ///
    /// Read through UnityEngine.XR.InputDevices like the rest of the client, rather than through
    /// the XR Interaction Toolkit's locomotion providers, so there is one input path and no input
    /// action assets to keep in sync.
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
        public float turnSpeedDegPerSec = 90f;
        public float deadzone = 0.2f;

        [Tooltip("Vertical movement on the face buttons, for looking into a scene from above")]
        public float verticalSpeed = 1.0f;

        private bool _recenterPressed;

        private void Start()
        {
            if (rig == null) rig = transform;
            if (head == null) head = Camera.main;
            if (config != null && config.Active != null)
            {
                speed = config.Active.moveSpeed;
                turnSpeedDegPerSec = config.Active.turnSpeedDegPerSec;

                /* Straight to the configured height here: at Start the camera has no tracked
                 * pose yet, so there is nothing to measure the correction against. */
                var origin = GetComponent<Unity.XR.CoreUtils.XROrigin>();
                if (origin != null) origin.CameraYOffset = config.Active.eyeHeight;
                MoveToSpawn();
            }
        }

        private void MoveToSpawn()
        {
            /* The spawn point is given in ROS coordinates, because that is the frame the scene
             * and every published pose are in. */
            Vector3 p = config.Active.spawnPosition;
            rig.SetPositionAndRotation(FrameConv.RosToUnity(p.x, p.y, p.z),
                                       Quaternion.Euler(0f, -config.Active.spawnYawDegrees, 0f));
        }

        /// <summary>
        /// Puts the user back at the configured spawn, facing the configured way, with the floor
        /// a sensible distance below their eyes.
        ///
        /// Device-space tracking measures no floor, so the ground sits eyeHeight below wherever
        /// the headset happened to be at launch: pick it up off a desk and the world is a metre
        /// low. This is the way back without editing a config file and restarting.
        /// </summary>
        public void Recenter()
        {
            if (config == null || config.Active == null) return;
            MoveToSpawn();

            // Shift the offset by however far the eyes are from where they should be.
            var origin = GetComponent<Unity.XR.CoreUtils.XROrigin>();
            if (origin != null && head != null)
            {
                float wanted = rig.position.y + config.Active.eyeHeight;
                origin.CameraYOffset += wanted - head.transform.position.y;
            }
        }

        private void Update()
        {
            if (head == null) return;

            InputDevice move = InputDevices.GetDeviceAtXRNode(
                moveWithLeftHand ? XRNode.LeftHand : XRNode.RightHand);

            /* Height is on the other hand's face buttons: this hand's stick already does both
             * walking and turning, and its Y button resets the simulation. */
            InputDevice other = InputDevices.GetDeviceAtXRNode(
                moveWithLeftHand ? XRNode.RightHand : XRNode.LeftHand);

            // One stick does both: push to walk, tilt to turn.
            DriveAndTurn(move);
            Elevate(other);
            PollRecenter(move);
        }

        /* X on the movement hand. Edge-triggered, or holding it would fight the stick. */
        private void PollRecenter(InputDevice device)
        {
            bool down = device.isValid
                        && device.TryGetFeatureValue(CommonUsages.primaryButton, out bool x) && x;
            if (down && !_recenterPressed)
            {
                Recenter();
                Debug.Log("recenter: back at spawn, eyes at the configured height");
            }
            _recenterPressed = down;
        }

        private void DriveAndTurn(InputDevice device)
        {
            if (!device.isValid) return;
            if (!device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick)) return;

            if (Mathf.Abs(stick.y) >= deadzone)
            {
                /* Flatten the look direction: pitching the head should not drive the rig into the
                 * floor or the sky. */
                Vector3 forward =
                    Vector3.ProjectOnPlane(head.transform.forward, Vector3.up).normalized;
                rig.position += forward * stick.y * speed * Time.deltaTime;
            }

            if (Mathf.Abs(stick.x) >= deadzone)
            {
                // Rotate about the head, not the rig origin, or the user swings around the room.
                rig.RotateAround(head.transform.position, Vector3.up,
                                 stick.x * turnSpeedDegPerSec * Time.deltaTime);
            }
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
