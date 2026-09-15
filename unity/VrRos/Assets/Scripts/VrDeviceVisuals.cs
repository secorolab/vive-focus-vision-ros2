// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using UnityEngine;
using UnityEngine.XR;

namespace VrRos
{
    /// <summary>
    /// Shows the controller or the hand models, one set or the other.
    ///
    /// Controllers are com.htc.upm.vive.openxr's ViveFocus3ControllerAimL/R: ControllerModelActions
    /// animates the buttons but does not place the controller, so the pose is written here from
    /// the same InputDevices read VrInputPublisher uses. Hands are the XR Hands package's sample
    /// prefabs, driven by XRHandSkeletonDriver from the same XRHandSubsystem HandPublisher reads,
    /// and need no help.
    ///
    /// Mode is not a preference: the runtime stops reporting hands while a controller is awake,
    /// so showing both would leave one set frozen wherever it was last seen.
    /// </summary>
    public class VrDeviceVisuals : MonoBehaviour
    {
        [Tooltip("Optional: when set, the mode comes from its config file")]
        public VrConfig config;

        [Tooltip("Show the hand meshes instead of the controllers")]
        public bool handMode;

        public GameObject leftController;
        public GameObject rightController;
        public GameObject leftHand;
        public GameObject rightHand;

        [Tooltip("Seconds the menu button must be held to switch modes; a tap is too easy to hit")]
        public float toggleHoldSeconds = 1f;

        private float _heldSince = -1f;

        private void Start()
        {
            if (config != null) handMode = config.HandMode;
            ApplyMode();
            Debug.Log($"visuals: {(handMode ? "hands" : "controllers")}");
        }

        private void Update()
        {
            PollToggle();

            if (config != null && config.HandMode != handMode)
            {
                handMode = config.HandMode;
                ApplyMode();
            }

            if (handMode) return;
            Place(leftController, XRNode.LeftHand);
            Place(rightController, XRNode.RightHand);
        }

        /* The menu button, which the Joy layout already reserves and nothing else reads. Only the
         * left controller has one. Held, not tapped: a tap mid-grab switched modes by accident. */
        private void PollToggle()
        {
            if (config == null) return;

            InputDevice device = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            bool down = device.isValid
                        && device.TryGetFeatureValue(CommonUsages.menuButton, out bool menu)
                        && menu;

            if (!down) _heldSince = -1f;
            else if (_heldSince < 0f) _heldSince = Time.unscaledTime;
            else if (Time.unscaledTime - _heldSince >= toggleHoldSeconds)
            {
                config.SetHandMode(!config.HandMode);
                _heldSince = float.PositiveInfinity; // once per hold
            }
        }

        private void ApplyMode()
        {
            SetActive(leftController, !handMode);
            SetActive(rightController, !handMode);
            SetActive(leftHand, handMode);
            SetActive(rightHand, handMode);
        }

        /// <summary>
        /// Positions a model in tracking space. The models are parented to the camera offset, so
        /// the local pose is exactly what the device reports and the rig's own motion is inherited.
        /// </summary>
        private static void Place(GameObject model, XRNode node)
        {
            if (model == null) return;

            InputDevice device = InputDevices.GetDeviceAtXRNode(node);
            if (!device.isValid
                || !device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 p)
                || !device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion q))
            {
                // Leaving it at the last pose reads as a frozen controller, which looks like a bug.
                if (model.activeSelf) model.SetActive(false);
                return;
            }

            if (!model.activeSelf) model.SetActive(true);
            model.transform.localPosition = p;
            model.transform.localRotation = q;
        }

        private static void SetActive(GameObject go, bool on)
        {
            if (go != null) go.SetActive(on);
        }
    }
}
