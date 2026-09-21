// SPDX-License-Identifier: MIT
// Run with -executeMethod VrTeleopIndicatorCheck.Run.
using System;
using System.Reflection;
using Newtonsoft.Json.Linq;
using UnityEngine;
using TMPro;
using VrRos;

public static class VrTeleopIndicatorCheck
{
    private const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
    private static object Get(object o, string name) => o.GetType().GetField(name, Private).GetValue(o);
    private static void Set(object o, string name, object value) => o.GetType().GetField(name, Private).SetValue(o, value);
    private static void Call(object o, string name, params object[] args) => o.GetType().GetMethod(name, Private).Invoke(o, args);
    private static void Check(bool yes, string why) { if (!yes) throw new Exception(why); }
    private static JObject Status(bool ready, float distance = 2f) => new JObject {
        ["state"] = ready ? "ready" : "align_pose", ["ready"] = ready,
        ["position_error_m"] = distance, ["rotation_error_rad"] = 0.1f,
        ["position_tolerance_m"] = 0f, ["rotation_tolerance_rad"] = 0.2f
    };

    public static void Run()
    {
        UnityEditor.SceneManagement.EditorSceneManager.NewScene(UnityEditor.SceneManagement.NewSceneSetup.EmptyScene);
        var root = new GameObject("GuideCheck", typeof(RosBridge));
        var cameraObject = new GameObject("GuideCamera", typeof(Camera));
        var camera = cameraObject.GetComponent<Camera>();
        var sceneObject = new GameObject("GuideScene", typeof(SceneLoader));
        try
        {
            var bootstrap = typeof(VrTeleopIndicator).GetMethod("EnsureIndicators", BindingFlags.Static | BindingFlags.NonPublic);
            bootstrap.Invoke(null, null);
            bootstrap.Invoke(null, null);
            var indicators = root.GetComponents<VrTeleopIndicator>();
            Check(indicators.Length == 2, "Expected one guide per arm, without duplicates");
            Check(indicators[0].arm != indicators[1].arm, "Both guides target the same arm");
            foreach (var indicator in indicators)
            {
                indicator.scene = sceneObject.GetComponent<SceneLoader>();
                Set(indicator, "_head", camera);
                Call(indicator, "BuildGuide");
                Call(indicator, "RefreshRecovery", true, true);
                var panel = (GameObject)Get(indicator, "_recoveryPanel");
                Check(!panel.activeSelf, "Recovery panel visible during normal use");
                Call(indicator, "OnStatus", Status(false));
                Set(indicator, "_controller", new Vector3(indicator.arm == "left" ? -0.2f : 0.2f, 0, 0));
                Set(indicator, "_controllerRotation", Quaternion.identity);
                Set(indicator, "_wantedRotation", Quaternion.Euler(90, 0, 0));
                Set(indicator, "_controllerAt", Time.unscaledTime);
                Set(indicator, "_wantedAt", Time.unscaledTime);
                Call(indicator, "DrawOrientation", true);
                var current = (LineRenderer)Get(indicator, "_currentOutline");
                var wanted = (LineRenderer)Get(indicator, "_wantedOutline");
                Check(current.enabled && wanted.enabled, "Orientation outlines missing");
                Check(wanted.startColor == Color.cyan, "Nonready guide is not cyan");
                Check(Vector3.Distance(current.GetPosition(10), wanted.GetPosition(10)) > 0.1f,
                      "Target orientation not rotated");
                Call(indicator, "OnStatus", Status(true));
                Call(indicator, "DrawOrientation", true);
                Check(Vector4.Distance((Vector4)wanted.startColor, (Vector4)indicator.ready) < 0.01f, "PC readiness not shown green");
                Call(indicator, "DrawGuide", true, Color.green);
                Check(!((LineRenderer)Get(indicator, "_guide")).enabled, "Relative mode still points to floor");
                Set(indicator, "_wantedAt", Time.unscaledTime - 1f);
                Call(indicator, "DrawOrientation", true);
                Check(!wanted.enabled && !current.enabled, "Stale guide visible");
                var stopped = Status(false);
                stopped["state"] = "release_grip";
                stopped["stop_reason"] = "rotation_limit";
                Call(indicator, "OnStatus", stopped);
                Call(indicator, "RefreshRecovery", true, true);
                Check(panel.activeSelf, "Limit recovery panel missing");
                var title = (TextMeshProUGUI)Get(indicator, "_recoveryTitle");
                var instructions = (TextMeshProUGUI)Get(indicator, "_recoveryText");
                Check(title.text.Contains("Rotation limit"), "Limit cause missing");
                Check(instructions.text.Contains("Release the side grip"), "Recovery action missing");
                Call(indicator, "OnStatus", Status(false));
                Call(indicator, "RefreshRecovery", true, true);
                Check(panel.activeSelf && instructions.text.Contains("match cyan"), "Release lost recovery instructions");
                Call(indicator, "OnStatus", Status(true));
                Call(indicator, "RefreshRecovery", true, true);
                Check(title.text.Contains("Ready to continue"), "Ready recovery prompt missing");
                Call(indicator, "RefreshRecovery", false, true);
                Check(!instructions.text.Contains("Squeeze"), "Stale status invites engagement");
                var engaged = Status(false);
                engaged["state"] = "engaged";
                Call(indicator, "OnStatus", engaged);
                Call(indicator, "RefreshRecovery", true, true);
                Check(!panel.activeSelf, "Panel remained visible after resuming");
                Call(indicator, "OnStatus", Status(true, float.NaN));
                Check(!(bool)Get(indicator, "_have"), "Invalid status accepted");
                Call(indicator, "OnStatus", Status(true));
                Call(indicator, "Update");
                Check(!(bool)Get(indicator, "_have"), "Disconnect retained readiness");
            }
            Debug.Log("VrTeleopIndicatorCheck: PASS — both arms, conditional recovery, resume, readiness, orientation, stale/disconnect checks");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(root);
            UnityEngine.Object.DestroyImmediate(sceneObject);
            UnityEngine.Object.DestroyImmediate(cameraObject);
        }
    }
}
