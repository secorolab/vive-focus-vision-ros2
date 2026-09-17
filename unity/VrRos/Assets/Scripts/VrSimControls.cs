// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using UnityEngine;
using UnityEngine.XR;

namespace VrRos
{
    /// <summary>
    /// Buttons that act on the simulation rather than on the rig.
    ///
    /// Only reset so far, on the left Y button. The service is the PC's, so the sim is restored
    /// for every client at once, not just for whoever pressed it.
    /// </summary>
    public class VrSimControls : MonoBehaviour
    {
        public RosBridge bridge;

        [Tooltip("Optional: when set, the namespace comes from its config file")]
        public VrConfig config;

        [Tooltip("Service that restores the scene; SceneNode advertises it under its node name")]
        public string resetService = "/vive_scene/reset";

        [Tooltip("Hand carrying the reset button")]
        public bool resetOnLeftHand = true;

        private bool _pressed;

        private void Start()
        {
            if (config != null && config.Active != null) resetService = config.Active.resetService;
        }

        private void Update()
        {
            if (bridge == null || !bridge.IsConnected) return;

            InputDevice device = InputDevices.GetDeviceAtXRNode(
                resetOnLeftHand ? XRNode.LeftHand : XRNode.RightHand);

            // Y on the left controller. Edge-triggered: holding it would reset every frame.
            bool down = device.isValid
                        && device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool y) && y;

            if (down && !_pressed)
            {
                bridge.CallService(resetService);
                Debug.Log($"reset: called {resetService}");
            }
            _pressed = down;
        }
    }
}
