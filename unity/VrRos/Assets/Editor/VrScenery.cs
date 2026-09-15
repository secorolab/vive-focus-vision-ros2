// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor.SceneManagement;

/// <summary>
/// Bakes a scenery .glb into the shipped scene.
///
/// The runtime env_url path imports glTF on the device every launch, which costs CPU, garbage and
/// memory, and can only ever light the result in real time. Baking moves all of that to build
/// time: the geometry becomes a static prefab in the APK, the lighting becomes lightmaps, and the
/// headset does nothing at run time but draw it.
///
/// The trade is that the environment is then part of the build, so changing it means rebuilding.
/// Keep env_url for trying scenes out; bake the one being kept.
/// </summary>
public static class VrScenery
{
    private const string GlbPath = "Assets/Scenery/apartment.glb";
    private const string ScenePath = "Assets/Scenes/Main.unity";

    [MenuItem("VrRos/Bake Scenery Into Scene")]
    public static void BakeScenery()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(GlbPath);
        if (prefab == null)
        {
            Debug.LogError($"VrScenery: {GlbPath} did not import; is glTFast's importer present?");
            return;
        }

        GameObject old = GameObject.Find("BakedScenery");
        if (old != null) Object.DestroyImmediate(old);

        var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        instance.name = "BakedScenery";
        instance.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

        /* Static in every sense the build cares about: lightmapped, batched, occluder and
         * occludee. Nothing here ever moves - the simulation does not know it exists. */
        int marked = 0;
        foreach (Transform t in instance.GetComponentsInChildren<Transform>(true))
        {
            GameObjectUtility.SetStaticEditorFlags(
                t.gameObject,
                StaticEditorFlags.ContributeGI | StaticEditorFlags.BatchingStatic
                    | StaticEditorFlags.OccluderStatic | StaticEditorFlags.OccludeeStatic);
            marked++;
        }

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, ScenePath);
        Debug.Log($"VrScenery: {marked} objects marked static in {ScenePath}");
    }

    [MenuItem("VrRos/Bake Lighting")]
    public static void BakeLighting()
    {
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);

        // Modest settings: this is a standalone headset, and a long bake helps nothing here.
        LightmapEditorSettings.lightmapper = LightmapEditorSettings.Lightmapper.ProgressiveGPU;
        LightmapEditorSettings.lightmapsMode = LightmapsMode.NonDirectional;
        Lightmapping.lightingSettings.lightmapResolution = 10f;
        Lightmapping.lightingSettings.lightmapMaxSize = 1024;
        Lightmapping.lightingSettings.directSampleCount = 16;
        Lightmapping.lightingSettings.indirectSampleCount = 64;
        Lightmapping.lightingSettings.ao = true;

        Debug.Log("VrScenery: baking...");
        if (!Lightmapping.Bake()) Debug.LogError("VrScenery: bake failed");
        else Debug.Log("VrScenery: bake done");

        EditorSceneManager.SaveOpenScenes();
    }
}
