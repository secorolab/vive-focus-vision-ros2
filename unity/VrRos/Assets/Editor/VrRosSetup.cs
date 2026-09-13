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
        BuildScene();
        Debug.Log("VrRosSetup: done");
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

        Debug.Log("VrRosSetup: player settings configured (IL2CPP, ARM64, API 29+)");
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

        var root = new GameObject("VrRos");
        var bridge = root.AddComponent<RosBridge>();
        var clock = root.AddComponent<ClockSync>();
        var input = root.AddComponent<VrInputPublisher>();
        var poses = root.AddComponent<BodyPoseApplier>();

        // SceneLoader parents the downloaded world under its own transform, so it gets its own.
        var sceneRoot = new GameObject("VrScene");
        sceneRoot.transform.SetParent(root.transform, false);
        var loader = sceneRoot.AddComponent<SceneLoader>();

        string host = Environment.GetEnvironmentVariable("VR_HOST");
        if (!string.IsNullOrEmpty(host)) bridge.host = host;

        clock.bridge = bridge;
        input.bridge = bridge;
        input.clock = clock;
        poses.bridge = bridge;
        poses.scene = loader;
        loader.bridge = bridge;

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
