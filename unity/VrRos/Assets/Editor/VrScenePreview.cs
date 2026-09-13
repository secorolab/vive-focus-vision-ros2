// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Imports an exported scene.glb and renders it to a PNG, so a world can be checked in Unity
/// without a headset or a device build.
///
///   Unity -batchmode -quit -projectPath . -executeMethod VrScenePreview.Render
///     (VR_GLB=/path/scene.glb VR_PNG=/path/out.png)
/// </summary>
public static class VrScenePreview
{
    private const string ImportDir = "Assets/ImportedScenes";

    private static float CorrectionYaw =>
        float.TryParse(Environment.GetEnvironmentVariable("VR_CORRECTION_YAW"), out float y)
            ? y
            : 90f;

    /// <summary>
    /// Imports the glb and saves it as a scene to open in the Editor, rather than rendering a
    /// PNG. Set VR_GLB first.
    /// </summary>
    [MenuItem("VrRos/Open Exported Scene")]
    public static void MakeScene()
    {
        string glb = Environment.GetEnvironmentVariable("VR_GLB");
        if (string.IsNullOrEmpty(glb) || !File.Exists(glb))
        {
            Debug.LogError($"VrScenePreview: set VR_GLB to an exported scene.glb (got '{glb}')");
            return;
        }

        Directory.CreateDirectory(ImportDir);
        string assetPath = $"{ImportDir}/{Path.GetFileNameWithoutExtension(glb)}.glb";
        File.Copy(glb, assetPath, true);
        AssetDatabase.ImportAsset(assetPath, ImportAssetOptions.ForceSynchronousImport);

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
        if (prefab == null)
        {
            Debug.LogError($"VrScenePreview: nothing imported from {assetPath}");
            return;
        }

        var scene = UnityEditor.SceneManagement.EditorSceneManager.NewScene(
            UnityEditor.SceneManagement.NewSceneSetup.DefaultGameObjects,
            UnityEditor.SceneManagement.NewSceneMode.Single);

        PrefabUtility.InstantiatePrefab(prefab);

        // Put the scene camera somewhere the robot is visible instead of at the origin.
        var cam = UnityEngine.Object.FindFirstObjectByType<Camera>();
        if (cam != null)
        {
            cam.transform.position = new Vector3(1.6f, 1.2f, 1.6f);
            cam.transform.LookAt(new Vector3(0f, 0.6f, 0f));
        }

        Directory.CreateDirectory("Assets/Scenes");
        const string scenePath = "Assets/Scenes/Preview.unity";
        UnityEditor.SceneManagement.EditorSceneManager.SaveScene(scene, scenePath);
        Debug.Log($"VrScenePreview: wrote {scenePath}");
    }

    [MenuItem("VrRos/Preview Exported Scene")]
    public static void Render()
    {
        string glb = Environment.GetEnvironmentVariable("VR_GLB");
        string png = Environment.GetEnvironmentVariable("VR_PNG") ?? "/tmp/vr_preview.png";
        if (string.IsNullOrEmpty(glb) || !File.Exists(glb))
        {
            Debug.LogError($"VrScenePreview: set VR_GLB to an exported scene.glb (got '{glb}')");
            return;
        }

        Directory.CreateDirectory(ImportDir);
        string assetPath = $"{ImportDir}/{Path.GetFileNameWithoutExtension(glb)}.glb";
        File.Copy(glb, assetPath, true);
        AssetDatabase.ImportAsset(assetPath, ImportAssetOptions.ForceSynchronousImport);

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
        if (prefab == null)
        {
            Debug.LogError($"VrScenePreview: nothing imported from {assetPath}; is glTFast present?");
            return;
        }

        /* Nothing is placed here on purpose: the glb carries each body's loaded pose in its own
         * node transforms, so what this renders is what any glTF viewer would show. */
        var root = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        var renderers = root.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0)
        {
            Debug.LogError("VrScenePreview: the imported scene has no renderers");
            return;
        }

        /* Frame on the content, not the ground: a MuJoCo floor plane is tens of metres across and
         * would push the camera so far back that the robot is a speck. */
        float floorSize = 5f;
        var framed = renderers.Where(r => r.bounds.size.magnitude < floorSize).ToArray();
        if (framed.Length == 0) framed = renderers;

        Bounds bounds = framed[0].bounds;
        foreach (var r in framed.Skip(1)) bounds.Encapsulate(r.bounds);

        var lightGo = new GameObject("PreviewLight");
        var light = lightGo.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.2f;
        lightGo.transform.rotation = Quaternion.Euler(45f, 35f, 0f);

        var camGo = new GameObject("PreviewCamera");
        var cam = camGo.AddComponent<Camera>();
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.12f, 0.13f, 0.16f);

        // Three-quarter view, far enough back that the whole scene fits.
        float radius = Mathf.Max(bounds.extents.magnitude, 0.2f);
        Vector3 dir = new Vector3(1f, 0.6f, 1f).normalized;
        camGo.transform.position = bounds.center + dir * radius * 3f;
        camGo.transform.LookAt(bounds.center);

        var rt = new RenderTexture(1280, 800, 24);
        cam.targetTexture = rt;
        cam.Render();

        RenderTexture.active = rt;
        var shot = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        shot.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        shot.Apply();
        RenderTexture.active = null;
        cam.targetTexture = null;

        File.WriteAllBytes(png, shot.EncodeToPNG());
        Debug.Log($"VrScenePreview: {renderers.Length} renderers, bounds centre {bounds.center} "
                  + $"size {bounds.size} -> {png}");

        UnityEngine.Object.DestroyImmediate(root);
        UnityEngine.Object.DestroyImmediate(camGo);
        UnityEngine.Object.DestroyImmediate(lightGo);
    }

    [Serializable] private class Body { public string name; public int node; }
    [Serializable] private class Pose { public float[] p; public float[] q; }
    [Serializable]
    private class Manifest
    {
        public Body[] bodies;
        public Pose[] initial_poses;
    }
}
