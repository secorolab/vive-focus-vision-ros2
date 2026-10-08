using System.Collections;
using TMPro;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;
using UnityEngine.XR;

namespace VrRos
{
    /// <summary>Independent HTTP previews; never publishes robot commands.</summary>
    public class VrCameraPanels : MonoBehaviour
    {
        public RosBridge bridge;
        private GameObject root;
        private VrLocomotion locomotion;
        private float distance = 1.2f;
        private bool locked;
        private bool needNeutral = true;
        private readonly RawImage[] images = new RawImage[3];
        private readonly TextMeshProUGUI[] labels = new TextMeshProUGUI[3];
        private readonly float[] received = { -100f, -100f, -100f };
        private readonly string[] names = { "Main camera", "Left wrist camera", "Right wrist camera" };
        private readonly string[] keys = { "main", "wrist", "right" };

        private IEnumerator Start()
        {
            while (Camera.main == null || bridge == null) yield return null;
            locomotion = FindFirstObjectByType<VrLocomotion>();
            StartCoroutine(Stream(0));
            StartCoroutine(Stream(1));
            StartCoroutine(Stream(2));
        }

        private void Build()
        {
            root = new GameObject("Camera previews");
            // Parenting to the tracked camera also follows its late XR pose update.
            root.transform.SetParent(Camera.main.transform, false);
            root.transform.localPosition = new Vector3(0, .25f, distance);
            root.transform.localRotation = Quaternion.identity;
            for (int i = 0; i < 3; i++)
            {
                var panel = new GameObject(names[i], typeof(RectTransform), typeof(Canvas));
                panel.transform.SetParent(root.transform, false);
                panel.transform.localPosition = new Vector3(i == 0 ? 0f : (i == 1 ? -.88f : .88f), 0, 0);
                panel.transform.localScale = Vector3.one * (i == 0 ? .0015f : .001f);
                ((RectTransform)panel.transform).sizeDelta = new Vector2(680, 550);
                panel.GetComponent<Canvas>().renderMode = RenderMode.WorldSpace;
                var picture = new GameObject("Image", typeof(RectTransform), typeof(RawImage));
                picture.transform.SetParent(panel.transform, false);
                ((RectTransform)picture.transform).sizeDelta = new Vector2(640, 480);
                images[i] = picture.GetComponent<RawImage>();
                images[i].raycastTarget = false;
                var title = new GameObject("Status", typeof(RectTransform), typeof(TextMeshProUGUI));
                title.transform.SetParent(panel.transform, false);
                title.transform.localPosition = new Vector3(0, 270, 0);
                ((RectTransform)title.transform).sizeDelta = new Vector2(680, 60);
                labels[i] = title.GetComponent<TextMeshProUGUI>();
                labels[i].fontSize = 30;
                labels[i].alignment = TextAlignmentOptions.Center;
                labels[i].raycastTarget = false;
            }
        }

        private IEnumerator Stream(int index)
        {
            while (true)
            {
                string host = bridge.host;
                using (var request = UnityWebRequestTexture.GetTexture($"http://{host}:8081/cameras/{keys[index]}.jpg"))
                {
                    request.timeout = 2;
                    yield return request.SendWebRequest();
                    if (request.result == UnityWebRequest.Result.Success && host == bridge.host)
                    {
                        var texture = DownloadHandlerTexture.GetContent(request);
                        if (root == null) Build();
                        if (images[index].texture != null) Destroy(images[index].texture);
                        images[index].texture = texture;
                        received[index] = Time.unscaledTime;
                    }
                }
                yield return new WaitForSecondsRealtime(.125f);
            }
        }

        private void Update()
        {
            if (root == null) return;
            for (int i = 0; i < 3; i++)
            {
                bool live = Time.unscaledTime - received[i] < .5f;
                images[i].color = live ? Color.white : Color.black;
                labels[i].text = names[i] + (live ? " — LIVE" : " — CAMERA OFFLINE");
                labels[i].color = live ? Color.white : new Color(1, .65f, .25f);
            }
        }

        private void LateUpdate()
        {
            if (root == null || Camera.main == null) return;
            bool worldLocked = VrLocomotion.WorldLocked;
            if (worldLocked != locked)
            {
                locked = worldLocked;
                needNeutral = true;
                // Preserve the exact world pose on lock; resume eye-relative placement on unlock.
                root.transform.SetParent(locked ? null : Camera.main.transform, true);
            }
            if (locked) return;
            // In simulation the stick still belongs to locomotion. Adjust only in body view.
            if (locomotion != null && locomotion.RobotBodyView)
            {
                var device = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
                if (device.isValid && device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 axes))
                {
                    if (needNeutral) { if (axes.magnitude < .2f) needNeutral = false; }
                    else if (Mathf.Abs(axes.y) > .2f)
                        distance = Mathf.Clamp(distance + axes.y * .6f * Time.unscaledDeltaTime, .75f, 2.5f);
                }
                else needNeutral = true;
            }
            root.transform.localPosition = new Vector3(0, .25f, distance);
            root.transform.localRotation = Quaternion.identity;
        }

        private void OnDestroy()
        {
            StopAllCoroutines();
            foreach (var image in images)
                if (image != null && image.texture != null) Destroy(image.texture);
            if (root != null) Destroy(root);
        }
    }
}
