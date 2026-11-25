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
        private const string RobotYawKey = "OpenArm.OperatorViewYaw";
        private float _robotYaw, _viewPitch;
        private bool _heightReady;
        private Vector2 _lookVelocity, _walkVelocity;
        private bool _operatorButtonReady;
        public static bool FacingHeadset { get; private set; }
        private static Vector3 _operatorOrigin, _operatorCenter;
        private static Quaternion _operatorRotation;

        // Shared by ROS publishing, controller meshes and pointing rays. The operator
        // frame stays fixed in the robot world when the viewing camera moves.
        public static void MapController(Transform space, ref Vector3 p, ref Quaternion q)
        {
            if (!FacingHeadset || space == null) return;
            Vector3 world = _operatorOrigin + _operatorRotation * (p - _operatorCenter);
            p = space.InverseTransformPoint(world);
            q = Quaternion.Inverse(space.rotation) * _operatorRotation * q;
        }

        private void PollOperatorFrame()
        {
            var left = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            bool down = left.isValid && left.TryGetFeatureValue(CommonUsages.primaryButton, out bool x) && x;
            if (WorldLocked || !RobotBodyView) { _operatorButtonReady = false; return; }
            if (!down) { _operatorButtonReady = true; return; }
            if (!_operatorButtonReady) return;
            _operatorButtonReady = false;
            if (FacingHeadset) { FacingHeadset = false; UpdateLockLabel(); return; }
            var right = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            if (!left.TryGetFeatureValue(CommonUsages.isTracked, out bool lt) || !lt
                || !right.TryGetFeatureValue(CommonUsages.isTracked, out bool rt) || !rt
                || !left.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 lp)
                || !right.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 rp)
                || !Finite(lp) || !Finite(rp)) return;
            var publisher = FindFirstObjectByType<VrInputPublisher>();
            if (publisher == null || publisher.rig == null) return;
            _operatorCenter = (lp + rp) * 0.5f;
            _operatorOrigin = publisher.rig.TransformPoint(_operatorCenter);
            _operatorRotation = Quaternion.Euler(0f, publisher.rig.eulerAngles.y + 180f, 0f);
            FacingHeadset = true;
            UpdateLockLabel();
        }
        private bool _rotationTriggerReady;
        private bool _robotRotating;
        private const float RobotTurnSpeed = 15f;
        private Vector3 _placementOffset, _placementHand;
        private bool _placementReady, _placing;
        private SceneLoader _robotScene;
        private Vector3 _robotOffset;
        private Vector3 _dragRigStart, _dragSceneStart;
        private float _dragLogAt;
        private string _placementStatus = "";
        private const string RobotPlacementKey = "OpenArm.RobotPlacement.";
        private bool _viewStickReady, _lookStickReady;
        private const string PlacementKey = "OpenArm.OperatorPlacement.";

        private void Awake()
        {
            WorldLocked = false;
            _robotOffset = new Vector3(PlayerPrefs.GetFloat(RobotPlacementKey+"x", 0f),
                PlayerPrefs.GetFloat(RobotPlacementKey+"y", 0f), PlayerPrefs.GetFloat(RobotPlacementKey+"z", 0f));
            if (!Finite(_robotOffset)) _robotOffset = Vector3.zero;
            FacingHeadset = false;
            _placementOffset = new Vector3(PlayerPrefs.GetFloat(PlacementKey+"x", 0f),
                PlayerPrefs.GetFloat(PlacementKey+"y", 0f), PlayerPrefs.GetFloat(PlacementKey+"z", 0f));
            if (!Finite(_placementOffset)) _placementOffset = Vector3.zero;
            _robotYaw = PlayerPrefs.GetFloat(RobotYawKey, 0f);
            _viewPitch = PlayerPrefs.GetFloat("OpenArm.OperatorViewPitch", 0f);
            if (float.IsNaN(_viewPitch) || float.IsInfinity(_viewPitch)) _viewPitch = 0f;
            _viewPitch = Mathf.Clamp(_viewPitch, -70f, 70f);
            if (float.IsNaN(_robotYaw) || float.IsInfinity(_robotYaw)) _robotYaw = 0f;
        }

        private void Start()
        {
            _robotScene = FindFirstObjectByType<SceneLoader>();
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
            _robotRotating = false;
            if (_robotScene != null)
                SetRobotDisplayPosition(enabled ? _robotOffset : Vector3.zero);
            if (enabled) MoveToRobotEye();
            UpdateLockLabel();
        }

        private void MoveToRobotEye()
        {
            if (rig == null || head == null) return;
            // Apply the saved orbit to both eye position and heading. The OpenArm
            // base is at ROS world zero; rotating the rig about it is the inverse
            // of rotating the displayed robot, with all ROS poses left consistent.
            Quaternion orbit = Quaternion.Euler(_viewPitch, _robotYaw, 0f);
            rig.rotation = orbit;
            Vector3 eye = FrameConv.RosToUnity(RobotEyeRos.x, RobotEyeRos.y, RobotEyeRos.z);
            rig.position += orbit * (eye + _placementOffset) - head.transform.position;
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
            PollOperatorFrame();
            PollRobotRotation();
            PollRobotPlacement();
            PollViewLook();
            PollViewMovement();
            PollViewHeight();
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
                rect.sizeDelta=new Vector2(720,230); rect.localScale=Vector3.one*0.001f;
                panel.GetComponent<Canvas>().renderMode=RenderMode.WorldSpace;
                var background=panel.GetComponent<Image>();
                background.color=new Color(0.03f,0.04f,0.05f,0.95f); background.raycastTarget=false;
                var label=new GameObject("Text",typeof(RectTransform),typeof(TextMeshProUGUI));
                label.transform.SetParent(panel.transform,false);
                _lockLabel=label.GetComponent<TextMeshProUGUI>();
                _lockLabel.rectTransform.sizeDelta=new Vector2(690,215);
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
                "WORLD FREE - robot control disabled\n" + (RobotBodyView ?  $"LEFT trigger: {(_robotRotating ? "STOP turning" : "start turning right")} ({_robotYaw:0}°)\nRIGHT trigger: hold and move to place robot\n{_placementStatus}\nLEFT stick: look | RIGHT stick: walk | A/B: up/down\nX: facing-headset { (FacingHeadset ? "ON" : "OFF") }\n" : "") + "Tap LEFT MENU to lock and save";
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
                _rotationTriggerReady=false;
                _robotRotating=false;
                _placing=false; _placementReady=false; _viewStickReady=false; _lookStickReady=false; _heightReady=false;
                if (WorldLocked && RobotBodyView) {
                    PlayerPrefs.SetFloat(RobotYawKey, _robotYaw);
                    PlayerPrefs.SetFloat("OpenArm.OperatorViewPitch", _viewPitch);
                    PlayerPrefs.SetFloat(RobotPlacementKey+"x", _robotOffset.x);
                    PlayerPrefs.SetFloat(RobotPlacementKey+"y", _robotOffset.y);
                    PlayerPrefs.SetFloat(RobotPlacementKey+"z", _robotOffset.z);
                    PlayerPrefs.SetFloat(PlacementKey+"x", _placementOffset.x);
                    PlayerPrefs.SetFloat(PlacementKey+"y", _placementOffset.y);
                    PlayerPrefs.SetFloat(PlacementKey+"z", _placementOffset.z);
                    PlayerPrefs.Save();
                }
                PlaceLockLabel();
                UpdateLockLabel();
                if (_lockLabel!=null) Debug.Log(_lockLabel.text);
            }
            _lockMenuWasDown=down;
        }

        // Rotate the viewing rig so rendering and published controller poses keep
        // sharing the same ROS frame. Never rotate just the robot mesh.
        private void PollRobotRotation()
        {
            if (!RobotBodyView || WorldLocked || rig == null || head == null || _robotScene == null) {
                _rotationTriggerReady = false;
                _robotRotating = false;
                UpdateLockLabel();
                return;
            }
            var device = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            if (!device.isValid) {
                _rotationTriggerReady = false;
                _robotRotating = false;
                UpdateLockLabel();
                return;
            }
            // Some XR providers expose the button without an analog value or
            // an isTracked feature. Accept either trigger representation.
            // An explicitly lost tracking state still blocks adjustment.
            if (device.TryGetFeatureValue(CommonUsages.isTracked, out bool tracked) && !tracked) {
                _rotationTriggerReady = false;
                _robotRotating = false;
                UpdateLockLabel();
                return;
            }
            bool hasButton = device.TryGetFeatureValue(CommonUsages.triggerButton, out bool button);
            bool hasAnalog = device.TryGetFeatureValue(CommonUsages.trigger, out float trigger);
            if (!hasButton && !hasAnalog) {
                _rotationTriggerReady = false;
                _robotRotating = false;
                UpdateLockLabel();
                return;
            }
            bool released = !button && (!hasAnalog || trigger < 0.2f);
            bool pressed = button || (hasAnalog && trigger >= 0.75f);
            if (released) _rotationTriggerReady = true;
            if (pressed && _rotationTriggerReady) {
                _rotationTriggerReady = false;
                _robotRotating = !_robotRotating;
                UpdateLockLabel();
            }
            if (!_robotRotating) return;
            float step = -RobotTurnSpeed * Mathf.Min(Time.unscaledDeltaTime, 0.05f);
            _robotYaw = Mathf.Repeat(_robotYaw + step, 360f);
            rig.RotateAround(_robotScene != null ? _robotScene.transform.position : Vector3.zero, Vector3.up, step);
            UpdateLockLabel();
            PlaceLockLabel();
        }

        private void PollViewLook()
        {
            var device = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            if (!RobotBodyView || WorldLocked || head == null || rig == null || _placing
                || !device.isValid
                || !device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick)
                || float.IsNaN(stick.x) || float.IsInfinity(stick.x)
                || float.IsNaN(stick.y) || float.IsInfinity(stick.y)) {
                _lookVelocity = Vector2.zero;
                _lookStickReady = false;
                return;
            }
            if (stick.magnitude <= deadzone) { _lookStickReady = true; _lookVelocity = Vector2.zero; return; }
            if (!_lookStickReady) return;
            float Axis(float value) => Mathf.Sign(value) * Mathf.Clamp01((Mathf.Abs(value)-deadzone)/(1f-deadzone));
            float dt = Mathf.Min(Time.unscaledDeltaTime, 0.05f);
            _lookVelocity = Vector2.Lerp(_lookVelocity, new Vector2(Axis(stick.x), Axis(stick.y)) * 35f,
                1f - Mathf.Exp(-dt / 0.12f));
            Vector3 eye = FrameConv.RosToUnity(RobotEyeRos.x, RobotEyeRos.y, RobotEyeRos.z);
            Vector3 savedEye = rig.rotation * (eye + _placementOffset);
            Vector3 pivot = head.transform.position;
            _robotYaw = Mathf.Repeat(_robotYaw + _lookVelocity.x*dt, 360f);
            _viewPitch = Mathf.Clamp(_viewPitch - _lookVelocity.y*dt, -70f, 70f);
            Quaternion orientation = Quaternion.Euler(_viewPitch, _robotYaw, 0f);
            Quaternion turn = orientation * Quaternion.Inverse(rig.rotation);
            rig.position = pivot + turn * (rig.position-pivot);
            rig.rotation = orientation;
            _placementOffset = Quaternion.Inverse(rig.rotation) *
                (pivot + turn * (savedEye-pivot)) - eye;
            _robotRotating = false;
            UpdateLockLabel();
            PlaceLockLabel();
        }

        // Desk-mounted headset: translate the viewpoint without needing head motion.
        private void PollViewMovement()
        {
            var device = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            if (!RobotBodyView || WorldLocked || head == null || rig == null || _placing
                || !device.isValid
                || !device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick)
                || float.IsNaN(stick.x) || float.IsNaN(stick.y)
                || float.IsInfinity(stick.x) || float.IsInfinity(stick.y)) {
                _walkVelocity = Vector2.zero;
                _viewStickReady = false;
                return;
            }
            float magnitude = Mathf.Min(stick.magnitude, 1f);
            if (magnitude <= deadzone) { _viewStickReady = true; _walkVelocity = Vector2.zero; return; }
            if (!_viewStickReady) return; // neutral first after unlocking/reconnecting
            Vector3 forward = Vector3.ProjectOnPlane(head.transform.forward, Vector3.up).normalized;
            if (forward.sqrMagnitude < 0.5f) forward = Quaternion.Euler(0f, _robotYaw, 0f) * Vector3.forward;
            Vector3 right = Vector3.Cross(Vector3.up, forward);
            Vector2 input = stick.normalized * ((magnitude-deadzone)/(1f-deadzone));
            float dt = Mathf.Min(Time.unscaledDeltaTime, 0.05f);
            _walkVelocity = Vector2.Lerp(_walkVelocity, input, 1f - Mathf.Exp(-dt / 0.10f));
            Vector3 delta = (right*_walkVelocity.x + forward*_walkVelocity.y) * 0.5f * dt;
            rig.position += delta;
            _placementOffset += Quaternion.Inverse(rig.rotation) * delta;
            _robotRotating = false;
            UpdateLockLabel();
            PlaceLockLabel();
        }

        private void PollViewHeight()
        {
            var device = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            if (!RobotBodyView || WorldLocked || rig == null || !device.isValid || _placing) {
                _heightReady = false;
                return;
            }
            if (!device.TryGetFeatureValue(CommonUsages.primaryButton, out bool up)
                || !device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool down)) {
                _heightReady = false;
                return;
            }
            if (!up && !down) { _heightReady = true; return; }
            if (!_heightReady || up == down) return;
            Vector3 delta = Vector3.up * (up ? 1f : -1f) * 0.3f
                * Mathf.Min(Time.unscaledDeltaTime, 0.05f);
            rig.position += delta;
            _placementOffset += Quaternion.Inverse(rig.rotation) * delta;
            _robotRotating = false;
            UpdateLockLabel();
            PlaceLockLabel();
        }

        private static bool Finite(Vector3 p) =>
            !float.IsNaN(p.x) && !float.IsInfinity(p.x) &&
            !float.IsNaN(p.y) && !float.IsInfinity(p.y) &&
            !float.IsNaN(p.z) && !float.IsInfinity(p.z);

        private void PlacementStatus(string message)
        {
            if (_placementStatus != message) Debug.Log("robot-drag-status: " + message);
            _placementStatus = message;
            UpdateLockLabel();
        }

        private void PollRobotPlacement()
        {
            var device = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            if (!RobotBodyView || WorldLocked || rig == null || _robotScene == null || !device.isValid) {
                _placing = _placementReady = false;
                return;
            }
            bool hasButton = device.TryGetFeatureValue(CommonUsages.triggerButton, out bool button);
            bool hasAnalog = device.TryGetFeatureValue(CommonUsages.trigger, out float trigger);
            if (!hasButton && !hasAnalog) {
                _placing = _placementReady = false;
                PlacementStatus("Right trigger input unavailable");
                return;
            }
            if (!button && (!hasAnalog || trigger < 0.2f)) {
                _placing = false;
                _placementReady = true;
                PlacementStatus("Right drag ready");
                return;
            }
            // Stop any latched orbit even if right-hand tracking cannot start a drag.
            _robotRotating = false;
            if ((device.TryGetFeatureValue(CommonUsages.isTracked, out bool tracked) && !tracked)
                || !device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 hand)
                || !Finite(hand)) {
                _placing = _placementReady = false;
                PlacementStatus("Right tracking unavailable — release trigger and retry");
                return;
            }
            Quaternion unusedRotation = Quaternion.identity;
            MapController(rig, ref hand, ref unusedRotation);
            if (!_placing) {
                if (!_placementReady) {
                    PlacementStatus("Release RIGHT trigger, then hold to drag robot");
                    return;
                }
                if (!button && (!hasAnalog || trigger < 0.75f)) return;
                _placementReady = false;
                _placing = true;
                _placementHand = hand;
                _dragRigStart = rig.position;
                _dragSceneStart = _robotScene.transform.position;
                _dragLogAt = 0f;
                PlacementStatus("PLACING ROBOT — move right hand up/down/left/right");
                return;
            }
            Vector3 delta = hand - _placementHand;
            _placementHand = hand;
            if (delta.magnitude > 0.25f) {
                _placing = _placementReady = false;
                PlacementStatus("Tracking jumped — release trigger and retry");
                return;
            }
            MoveRobotDisplay(delta);
            if (Time.unscaledTime >= _dragLogAt) {
                _dragLogAt = Time.unscaledTime + 0.5f;
                Debug.Log($"robot-drag: facing={FacingHeadset} rigDelta={rig.position - _dragRigStart:F3} sceneDelta={_robotScene.transform.position - _dragSceneStart:F3}");
            }
            PlacementStatus("PLACING ROBOT — move right hand up/down/left/right");
            PlaceLockLabel();
        }

        private void MoveRobotDisplay(Vector3 trackingDelta)
        {
            SetRobotDisplayPosition(_robotScene.transform.position + rig.TransformVector(trackingDelta));
            _robotOffset = _robotScene.transform.position;
        }

        private void SetRobotDisplayPosition(Vector3 position)
        {
            Transform ground = _robotScene.transform.Find("Ground");
            Vector3 groundPosition = ground != null ? ground.position : Vector3.zero;
            _robotScene.transform.position = position;
            if (ground != null) ground.position = groundPosition;
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
