// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using UnityEngine;
using UnityEngine.XR;
using UnityEngine.UI;
using TMPro;

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

        public static bool WorldLocked { get; private set; }
        private float _lockMenuSince = -1f;
        private bool _lockMenuWasDown;
        private bool _needNeutralStick;
        private TextMeshProUGUI _lockLabel;
        private bool _lockLabelPlaced;
        private bool _recenterPressed;
        private Vector3? _sceneSpawn;
        private float _sceneSpawnYaw;
        private float _pitch;

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

        /// <summary>Where the loaded world says to stand, which outranks the client's own default.</summary>
        public void SetSceneSpawn(Vector3 posRos, float yawDegrees)
        {
            _sceneSpawn = posRos;
            _sceneSpawnYaw = yawDegrees;
            MoveToSpawn();
        }

        public bool RobotBodyView { get; private set; }
        // Approximate body camera location in ROS coordinates, not a measured extrinsic.
        public Vector3 RobotEyeRos = new Vector3(-.10f, 0f, 1.0f);

        public void SetRobotBodyView(bool enabled)
        {
            RobotBodyView = enabled;
            if (enabled) MoveToRobotEye();
        }

        private void MoveToRobotEye()
        {
            if (rig == null || head == null) return;
            // Match the current eyes, including tracked height, to the robot eye.
            rig.rotation = Quaternion.identity;
            rig.position += FrameConv.RosToUnity(RobotEyeRos.x, RobotEyeRos.y, RobotEyeRos.z)
                            - head.transform.position;
        }

        private float _viewYawDegrees;

        public void SetViewYaw(float degrees)
        {
            _viewYawDegrees = degrees;
            MoveToSpawn();
        }

        private void MoveToSpawn()
        {
            if (RobotBodyView) { MoveToRobotEye(); return; }
            /* The spawn point is given in ROS coordinates, because that is the frame the scene
             * and every published pose are in. */
            Vector3 p = _sceneSpawn ?? config.Active.spawnPosition;
            float yaw = _sceneSpawn.HasValue ? _sceneSpawnYaw : config.Active.spawnYawDegrees;
            Quaternion orbit = Quaternion.Euler(0f, -_viewYawDegrees, 0f);
            rig.SetPositionAndRotation(orbit * FrameConv.RosToUnity(p.x, p.y, p.z),
                                       Quaternion.Euler(0f, -yaw - _viewYawDegrees, 0f));
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
            if (RobotBodyView) { MoveToRobotEye(); return; }
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
            EnsureLockLabel();
            PollWorldLock();
            if (head == null || WorldLocked || RobotBodyView) return;
            if (_needNeutralStick) {
                foreach (var hand in new[] { XRNode.LeftHand, XRNode.RightHand }) {
                    var device = InputDevices.GetDeviceAtXRNode(hand);
                    if (!device.isValid || !device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 axes)
                        || axes.magnitude > deadzone) return;
                }
                _needNeutralStick = false;
            }

            // No headset means the desktop player, where there is no stick to read.
            if (!XRSettings.isDeviceActive)
            {
                Desktop();
                return;
            }

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

        private void EnsureLockLabel()
        {
            if (rig==null || head==null) return;
            if (_lockLabel==null) {
                var panel=new GameObject("World lock status",typeof(RectTransform),typeof(Canvas),typeof(Image));
                panel.transform.SetParent(rig,false);
                var rect=panel.GetComponent<RectTransform>();
                rect.sizeDelta=new Vector2(720,150); rect.localScale=Vector3.one*0.001f;
                panel.GetComponent<Canvas>().renderMode=RenderMode.WorldSpace;
                var background=panel.GetComponent<Image>();
                background.color=new Color(0.03f,0.04f,0.05f,0.95f); background.raycastTarget=false;
                var label=new GameObject("Text",typeof(RectTransform),typeof(TextMeshProUGUI));
                label.transform.SetParent(panel.transform,false);
                _lockLabel=label.GetComponent<TextMeshProUGUI>();
                _lockLabel.rectTransform.sizeDelta=new Vector2(690,135);
                _lockLabel.fontSize=28; _lockLabel.alignment=TextAlignmentOptions.Center;
                _lockLabel.raycastTarget=false;
                UpdateLockLabel();
            }
            if (!_lockLabelPlaced || !WorldLocked) {
                PlaceLockLabel(); _lockLabelPlaced=true;
            }
        }
        private void PlaceLockLabel()
        {
            if (_lockLabel==null || head==null) return;
            var panel=_lockLabel.transform.parent;
            panel.position=head.transform.TransformPoint(new Vector3(0,-0.3f,1.1f));
            panel.rotation=head.transform.rotation;
        }
        private void UpdateLockLabel()
        {
            if (_lockLabel==null) return;
            string gripperHint = RobotBodyView ? "Rear trigger: hold open | release close" : "Stick up: open | down: close | center: stop";
            _lockLabel.text=WorldLocked ? "WORLD LOCKED\n" + gripperHint + "\nEnable robot to show wrist alignment guides" :
                "WORLD FREE - robot control disabled\nTap LEFT MENU to lock (do not hold)\nThen enable robot; side grip toggles arm following";
            _lockLabel.color=WorldLocked ? Color.green : Color.white;
        }

        private void PollWorldLock()
        {
            var device = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            if (!device.isValid) { _lockMenuWasDown=false; _lockMenuSince=-1f; return; }
            bool down=device.TryGetFeatureValue(CommonUsages.menuButton,out bool menu) && menu;
            if (down && !_lockMenuWasDown) _lockMenuSince=Time.unscaledTime;
            if (!down && _lockMenuWasDown && _lockMenuSince>=0f && Time.unscaledTime-_lockMenuSince<0.8f) {
                WorldLocked=!WorldLocked;
                _needNeutralStick=true;
                PlaceLockLabel();
                UpdateLockLabel();
                if (_lockLabel!=null) Debug.Log(_lockLabel.text);
            }
            _lockMenuWasDown=down;
        }

        /// <summary>WASD walks, Q and E change height, the right mouse button held looks around.</summary>
        private void Desktop()
        {
            Vector3 forward = Vector3.ProjectOnPlane(head.transform.forward, Vector3.up).normalized;
            Vector3 right = Vector3.Cross(Vector3.up, forward);
            float ahead = (Input.GetKey(KeyCode.W) ? 1f : 0f) - (Input.GetKey(KeyCode.S) ? 1f : 0f);
            float side = (Input.GetKey(KeyCode.D) ? 1f : 0f) - (Input.GetKey(KeyCode.A) ? 1f : 0f);
            float up = (Input.GetKey(KeyCode.E) ? 1f : 0f) - (Input.GetKey(KeyCode.Q) ? 1f : 0f);
            rig.position += (forward * ahead + right * side) * speed * Time.deltaTime
                            + Vector3.up * up * verticalSpeed * Time.deltaTime;

            if (!Input.GetMouseButton(1)) return;
            const float degreesPerUnit = 2.0f;
            rig.RotateAround(head.transform.position, Vector3.up,
                             Input.GetAxisRaw("Mouse X") * degreesPerUnit);
            _pitch = Mathf.Clamp(_pitch - Input.GetAxisRaw("Mouse Y") * degreesPerUnit, -89f, 89f);
            head.transform.localRotation = Quaternion.Euler(_pitch, 0f, 0f);
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
