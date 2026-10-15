using System.Collections;
using System.Text;
using Newtonsoft.Json.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.XR;

namespace VrRos
{
    /// <summary>Episode UI only. Y explicitly requests the existing guarded return-to-zero routine.</summary>
    public class VrEpisodeRecorder : MonoBehaviour
    {
        public RosBridge bridge;
        public static bool Active { get; private set; }
        private float homeSince = -1;
        private bool homeSent;
        private TextMeshProUGUI label;
        private GameObject panel;
        private string state = "disabled";
        private float lastStatus = -100;
        private bool aWasDown, bWasDown, neutralRequired = true, busy;
        private float discardSince = -1;
        private bool discardSent;

        private IEnumerator Start()
        {
            while (bridge == null || Camera.main == null) yield return null;
            while (true)
            {
                string host = bridge.host;
                using (var request = UnityWebRequest.Get($"http://{host}:8081/recording/status"))
                {
                    request.timeout = 2;
                    yield return request.SendWebRequest();
                    if (request.result == UnityWebRequest.Result.Success && host == bridge.host)
                    {
                        JObject status = null;
                        try { status = JObject.Parse(request.downloadHandler.text); }
                        catch (System.Exception) { }
                        if (status != null)
                        {
                            state = (string)status["state"] ?? "disabled";
                            lastStatus = Time.unscaledTime;
                            if (state != "disabled")
                            {
                                if (panel == null) Build();
                                panel.SetActive(true);
                                label.text = $"<b>{state.ToUpperInvariant()} • RIGHT ARM</b>  {(float?)status["seconds"] ?? 0:0.0}s | saved {(int?)status["saved"] ?? 0}\n"
                                    + (string)status["message"] + "\nA: Start / Stop    B: Save    Hold X: Discard | Hold Y 2s: Discard + BOTH arms zero | A cancels return";
                                label.color = state == "recording" ? new Color(1f,.4f,.35f) : Color.white;
                            }
                            else if (panel != null) panel.SetActive(false);
                        }
                    }
                }
                yield return new WaitForSecondsRealtime(.25f);
            }
        }

        private void Build()
        {
            panel = new GameObject("Episode status", typeof(RectTransform), typeof(Canvas));
            panel.transform.SetParent(Camera.main.transform, false);
            panel.transform.localPosition = new Vector3(0, -.48f, 1.2f);
            panel.transform.localScale = Vector3.one * .001f;
            panel.GetComponent<Canvas>().renderMode = RenderMode.WorldSpace;
            var text = new GameObject("Text", typeof(RectTransform), typeof(TextMeshProUGUI));
            text.transform.SetParent(panel.transform, false);
            label = text.GetComponent<TextMeshProUGUI>();
            label.rectTransform.sizeDelta = new Vector2(1150,160);
            label.fontSize = 24;
            label.alignment = TextAlignmentOptions.Center;
            label.raycastTarget = false;
        }

        private void Update()
        {
            var right = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            var left = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            bool a = right.TryGetFeatureValue(CommonUsages.primaryButton, out bool av) && av;
            bool b = right.TryGetFeatureValue(CommonUsages.secondaryButton, out bool bv) && bv;
            bool x = left.TryGetFeatureValue(CommonUsages.primaryButton, out bool xv) && xv;
            bool y = left.TryGetFeatureValue(CommonUsages.secondaryButton, out bool yv) && yv;
            bool fresh = Time.unscaledTime-lastStatus < 2;
            Active = fresh && state != "disabled";
            if (panel != null && state != "disabled" && !fresh)
                label.text = "RECORDER DISCONNECTED — recording state unknown\nCheck laptop before starting another episode";
            if (!fresh || state == "disabled" || !VrLocomotion.WorldLocked || !right.isValid || !left.isValid)
            {
                neutralRequired = true;
                discardSince = -1;
                homeSince = -1; homeSent = false;
                aWasDown = a; bWasDown = b;
                return;
            }
            if (neutralRequired)
            {
                if (!a && !b && !x && !y) neutralRequired = false;
                aWasDown = a; bWasDown = b;
                return;
            }
            if (!busy)
            {
                if (a && !aWasDown && state == "returning")
                    StartCoroutine(Command("cancel_home"));
                else if (a && !aWasDown && !b && !x && !y && (state == "idle" || state == "home_done" || state == "recording"))
                    StartCoroutine(Command(state == "recording" ? "stop" : "start"));
                else if (b && !bWasDown && !a && !x && !y && (state == "review" || state == "export_error"))
                    StartCoroutine(Command(state == "review" ? "save" : "retry_export"));
                if (x && !a && !b && !y && (state == "review" || state == "fault" || state == "home_error"))
                {
                    if (discardSince < 0) discardSince = Time.unscaledTime;
                    if (!discardSent && Time.unscaledTime-discardSince >= 1.2f)
                    { discardSent = true; StartCoroutine(Command("discard")); }
                }
                else { discardSince = -1; discardSent = false; }
                if (y && !a && !b && !x && (state == "idle" || state == "recording" || state == "review" || state == "fault" || state == "home_done" || state == "home_error"))
                {
                    if (homeSince < 0) homeSince = Time.unscaledTime;
                    if (!homeSent && label != null)
                        label.text = "Hold Y: discard unsaved attempt and return BOTH arms to zero\nKeep the path clear; release Y to abort this countdown";
                    if (!homeSent && Time.unscaledTime-homeSince >= 2f)
                    { homeSent = true; StartCoroutine(Command("discard_home")); }
                }
                else { homeSince = -1; if (!y) homeSent = false; }
            }
            aWasDown = a; bWasDown = b;
        }

        private IEnumerator Command(string action)
        {
            busy = true;
            using (var request = new UnityWebRequest($"http://{bridge.host}:8081/recording/command", "POST"))
            {
                request.uploadHandler = new UploadHandlerRaw(Encoding.UTF8.GetBytes("{\"action\":\""+action+"\"}"));
                request.downloadHandler = new DownloadHandlerBuffer();
                request.SetRequestHeader("Content-Type", "application/json");
                request.timeout = 2;
                yield return request.SendWebRequest();
                if (request.result != UnityWebRequest.Result.Success && label != null)
                    label.text = "Recording command not confirmed; check status before retrying";
            }
            yield return new WaitForSecondsRealtime(.5f);
            busy = false;
        }

        private void OnDestroy() { Active = false; if (panel != null) Destroy(panel); }
    }
}
