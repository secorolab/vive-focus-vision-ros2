// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using VrRos;

/// <summary>
/// Batch-mode project setup, so the scene wiring and Android settings are reproducible instead
/// of a list of things to click. Run with:
///   Unity -batchmode -quit -projectPath . -buildTarget Android -executeMethod VrRosSetup.SetupAll
/// The headset's rosbridge host comes from the VR_HOST environment variable.
/// </summary>
public static class VrRosSetup
{
    private const string ScenePath = "Assets/Scenes/Main.unity";

    [MenuItem("VrRos/Setup Scene and Android Settings")]
    public static void SetupAll()
    {
        ConfigurePlayerSettings();
        IncludeGltfShaders();
        BuildScene();
        Debug.Log("VrRosSetup: done");
    }

    /* Nothing in the project references glTFast's shaders — the materials only exist at runtime,
     * once a .glb has been fetched — so the build strips them and every imported mesh renders
     * magenta on the device while looking correct in the Editor. */
    public static void IncludeGltfShaders()
    {
        string[] wanted =
        {
            "glTF/PbrMetallicRoughness", "glTF/PbrSpecularGlossiness", "glTF/Unlit",
        };

        var graphics = AssetDatabase.LoadAssetAtPath<UnityEngine.Object>(
            "ProjectSettings/GraphicsSettings.asset");
        var settings = new SerializedObject(graphics);
        SerializedProperty included = settings.FindProperty("m_AlwaysIncludedShaders");

        foreach (string name in wanted)
        {
            Shader shader = Shader.Find(name);
            if (shader == null)
            {
                Debug.LogError($"VrRosSetup: shader '{name}' not found — is glTFast installed?");
                continue;
            }

            bool present = false;
            for (int i = 0; i < included.arraySize; i++)
            {
                if (included.GetArrayElementAtIndex(i).objectReferenceValue == shader)
                {
                    present = true;
                    break;
                }
            }
            if (present) continue;

            included.InsertArrayElementAtIndex(included.arraySize);
            included.GetArrayElementAtIndex(included.arraySize - 1).objectReferenceValue = shader;
            Debug.Log($"VrRosSetup: always-include {name}");
        }

        settings.ApplyModifiedProperties();
        AssetDatabase.SaveAssets();
    }

    public static void ConfigurePlayerSettings()
    {
        var android = NamedBuildTarget.Android;
        PlayerSettings.SetScriptingBackend(android, ScriptingImplementation.IL2CPP);
        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
        PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
        PlayerSettings.SetApiCompatibilityLevel(android, ApiCompatibilityLevel.NET_Unity_4_8);

        PlayerSettings.companyName = "vamsi";
        PlayerSettings.productName = "VrRos";
        PlayerSettings.SetApplicationIdentifier(android, "sh.vamsi.vrros");

        // A standalone headset app is landscape-only and must not show the Unity splash in VR.
        PlayerSettings.defaultInterfaceOrientation = UIOrientation.LandscapeLeft;
        PlayerSettings.SplashScreen.show = false;

        // The scene server is plain HTTP on the LAN; the default blocks the .glb fetch outright.
        PlayerSettings.insecureHttpOption = InsecureHttpOption.AlwaysAllowed;

        Debug.Log("VrRosSetup: player settings configured (IL2CPP, ARM64, API 29+, cleartext HTTP)");
    }

    /* VIVE's own models rather than stand-ins. They hang off the camera offset because both
     * driver scripts work in tracking space, so the rig's own motion has to come from the
     * parent. */
    private static void AddDeviceVisuals(GameObject xrOrigin, VrConfig cfg)
    {
        const string Prefabs = "Packages/com.htc.upm.vive.openxr/Runtime/Prefabs";

        Transform offset = xrOrigin.transform.Find("Camera Offset");
        if (offset == null)
        {
            Debug.LogWarning("VrRosSetup: no Camera Offset, so no controller or hand models");
            return;
        }

        var visuals = xrOrigin.AddComponent<VrDeviceVisuals>();
        visuals.config = cfg;
        visuals.leftController = Spawn($"{Prefabs}/ViveFocus3ControllerAimL.prefab", offset);
        visuals.rightController = Spawn($"{Prefabs}/ViveFocus3ControllerAimR.prefab", offset);
        visuals.leftHand = Spawn($"{Prefabs}/ViveHandL.prefab", offset);
        visuals.rightHand = Spawn($"{Prefabs}/ViveHandR.prefab", offset);
    }

    private static GameObject Spawn(string assetPath, Transform parent)
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
        if (prefab == null)
        {
            Debug.LogError($"VrRosSetup: {assetPath} not found — is the VIVE plugin installed?");
            return null;
        }

        var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab, parent);
        instance.transform.localPosition = Vector3.zero;
        instance.transform.localRotation = Quaternion.identity;
        return instance;
    }

    public static void BuildScene()
    {
        Scene scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects,
                                                  NewSceneMode.Single);

        // The XR Origin brings its own camera; the default scene camera would fight it.
        GameObject defaultCamera = GameObject.Find("Main Camera");
        if (defaultCamera != null) UnityEngine.Object.DestroyImmediate(defaultCamera);

        if (!EditorApplication.ExecuteMenuItem("GameObject/XR/XR Origin (VR)"))
        {
            Debug.LogError("VrRosSetup: could not create an XR Origin (VR) — is XR Interaction "
                           + "Toolkit installed?");
        }
        GameObject xrOrigin = GameObject.Find("XR Origin (VR)");

        var root = new GameObject("VrRos");
        var cfg = root.AddComponent<VrConfig>();
        var bridge = root.AddComponent<RosBridge>();
        var clock = root.AddComponent<ClockSync>();
        var input = root.AddComponent<VrInputPublisher>();
        var poses = root.AddComponent<BodyPoseApplier>();
        var hands = root.AddComponent<HandPublisher>();
        var gaze = root.AddComponent<GazePublisher>();
        var sim = root.AddComponent<VrSimControls>();

        // SceneLoader parents the downloaded world under its own transform, so it gets its own.
        var sceneRoot = new GameObject("VrScene");
        sceneRoot.transform.SetParent(root.transform, false);
        var loader = sceneRoot.AddComponent<SceneLoader>();

        /* Only the fallback: at runtime VrConfig reads vr_config.json from the device, so the
         * address can change without rebuilding. VR_HOST just seeds what ships in the APK. */
        string host = Environment.GetEnvironmentVariable("VR_HOST");
        if (!string.IsNullOrEmpty(host)) cfg.defaults.host = host;
        bridge.host = cfg.defaults.host;

        bridge.config = cfg;
        clock.bridge = bridge;
        clock.config = cfg;
        input.bridge = bridge;
        input.clock = clock;
        input.config = cfg;
        poses.bridge = bridge;
        poses.scene = loader;
        poses.config = cfg;
        loader.bridge = bridge;
        loader.config = cfg;
        if (xrOrigin != null)
        {
            var locomotion = xrOrigin.AddComponent<VrLocomotion>();
            locomotion.rig = xrOrigin.transform;
            locomotion.head = xrOrigin.GetComponentInChildren<Camera>();
            locomotion.config = cfg;

            /* Every publisher needs the rig: tracking-space poses alone would not say where the
             * user has walked to. It is the camera offset rather than the origin root, because
             * in device-space tracking the offset carries the eye height and device poses are
             * reported relative to it. */
            Transform space = xrOrigin.transform.Find("Camera Offset") ?? xrOrigin.transform;
            input.rig = space;
            hands.rig = space;
            gaze.rig = space;

            /* Device space. The runtime offers only VIEW, LOCAL and STAGE - no
             * XR_EXT_local_floor - and with no boundary configured its STAGE origin sits at the
             * headset rather than the floor ("floor bound enable false"), so floor space costs a
             * play area and still does not measure a floor. Device space at least puts the ground
             * a known eyeHeight below the eyes, and VrLocomotion can recentre it on demand. */
            var origin = xrOrigin.GetComponent<Unity.XR.CoreUtils.XROrigin>();
            if (origin != null)
            {
                origin.RequestedTrackingOriginMode =
                    Unity.XR.CoreUtils.XROrigin.TrackingOriginMode.Device;
                origin.CameraYOffset = cfg.defaults.eyeHeight;
            }

            AddDeviceVisuals(xrOrigin, cfg);

            /* On the rig, because it casts from tracking-space device poses and draws the ray in
             * the world the user is standing in. */
            var pointer = xrOrigin.AddComponent<VrPointer>();
            pointer.bridge = bridge;
            pointer.config = cfg;
            pointer.scene = loader;
            pointer.head = xrOrigin.GetComponentInChildren<Camera>();
            pointer.trackingSpace = space;
        }
        else
        {
            Debug.LogWarning("VrRosSetup: no XR Origin found, so locomotion was not added and "
                             + "published poses will ignore where the user walked");
        }

        hands.bridge = bridge;
        hands.clock = clock;
        hands.config = cfg;
        gaze.bridge = bridge;
        gaze.clock = clock;
        gaze.config = cfg;
        sim.bridge = bridge;
        sim.config = cfg;

        Directory.CreateDirectory(Path.GetDirectoryName(ScenePath));
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, ScenePath);

        EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
        Debug.Log($"VrRosSetup: scene written to {ScenePath}, rosbridge host = {bridge.host}");
    }

    [MenuItem("VrRos/Build APK")]
    public static void BuildApk()
    {
        string output = "Build/VrRos.apk";
        Directory.CreateDirectory("Build");
        var options = new BuildPlayerOptions
        {
            scenes = new[] { ScenePath },
            locationPathName = output,
            target = BuildTarget.Android,
            targetGroup = BuildTargetGroup.Android,
            options = BuildOptions.None,
        };

        var report = BuildPipeline.BuildPlayer(options);
        Debug.Log($"VrRosSetup: build {report.summary.result}, {report.summary.totalSize} bytes, "
                  + $"{report.summary.totalErrors} error(s) -> {output}");
        if (report.summary.result != UnityEditor.Build.Reporting.BuildResult.Succeeded)
        {
            throw new Exception($"APK build failed: {report.summary.result}");
        }
    }
}
