using System;
using System.Reflection;
using UnityEngine;
using VrRos;

public static class VrRobotPlacementCheck
{
    public static void Run()
    {
        UnityEditor.SceneManagement.EditorSceneManager.NewScene(UnityEditor.SceneManagement.NewSceneSetup.EmptyScene);
        var rig = new GameObject("PlacementRig");
        var scene = new GameObject("PlacementScene", typeof(SceneLoader));
        var ground = new GameObject("Ground");
        ground.transform.SetParent(scene.transform, false);
        var camera = new GameObject("PlacementCamera");
        camera.transform.SetParent(rig.transform, false);
        camera.transform.localPosition = new Vector3(.2f, 1.4f, -.3f);
        var locomotion = rig.AddComponent<VrLocomotion>();
        locomotion.rig = rig.transform;
        const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
        typeof(VrLocomotion).GetField("_robotScene", flags).SetValue(locomotion, scene.GetComponent<SceneLoader>());
        var move = typeof(VrLocomotion).GetMethod("MoveRobotDisplay", flags);
        try
        {
            foreach (float yaw in new[] {0f, 90f, 180f, 270f})
            {
                rig.transform.SetPositionAndRotation(new Vector3(1, 2, 3), Quaternion.Euler(0, yaw, 0));
                scene.transform.position = new Vector3(-2, .1f, 4);
                Vector3 eye = camera.transform.position;
                Quaternion heading = camera.transform.rotation;
                Vector3 before = scene.transform.position;
                Vector3 floor = ground.transform.position;
                Vector3 delta = new Vector3(.1f, .05f, -.2f);
                move.Invoke(locomotion, new object[] {delta});
                if (Vector3.Distance(camera.transform.position, eye) > 1e-5f ||
                    Quaternion.Angle(camera.transform.rotation, heading) > .001f)
                    throw new Exception("Robot placement moved the viewing camera");
                if (Vector3.Distance(scene.transform.position, before + rig.transform.TransformVector(delta)) > 1e-5f)
                    throw new Exception("Robot placement did not follow the controller delta");
                if (Vector3.Distance(ground.transform.position, floor) > 1e-5f)
                    throw new Exception("Robot placement moved the floor grid");
                Vector3 world = rig.transform.TransformPoint(new Vector3(.3f, .8f, -.1f));
                Quaternion rotation = rig.transform.rotation * Quaternion.Euler(25, 10, -5);
                Vector3 local = world;
                Quaternion localRotation = rotation;
                FrameConv.WorldToScene(scene.transform, ref local, ref localRotation);
                if (Vector3.Distance(scene.transform.TransformPoint(local), world) > 1e-5f ||
                    Quaternion.Angle(scene.transform.rotation * localRotation, rotation) > .05f)
                    throw new Exception("Published pose no longer matches the displayed controller");
            }
            const BindingFlags statics = BindingFlags.Static | BindingFlags.NonPublic;
            var facing = typeof(VrLocomotion).GetField("<FacingHeadset>k__BackingField", statics);
            try
            {
                facing.SetValue(null, true);
                Quaternion operatorRotation = Quaternion.Euler(0, 180, 0);
                typeof(VrLocomotion).GetField("_operatorRotation", statics).SetValue(null, operatorRotation);
                typeof(VrLocomotion).GetField("_operatorOrigin", statics).SetValue(null, Vector3.zero);
                typeof(VrLocomotion).GetField("_operatorCenter", statics).SetValue(null, Vector3.zero);
                Vector3 a = new Vector3(.1f, .2f, .3f);
                Vector3 delta = new Vector3(.05f, -.02f, .1f);
                Vector3 b = a + delta;
                Quaternion qa = Quaternion.identity, qb = Quaternion.identity;
                VrLocomotion.MapController(rig.transform, ref a, ref qa);
                VrLocomotion.MapController(rig.transform, ref b, ref qb);
                Vector3 before = scene.transform.position;
                move.Invoke(locomotion, new object[] {b - a});
                if (Vector3.Distance(scene.transform.position - before, operatorRotation * delta) > 1e-5f)
                    throw new Exception("Drag disagrees with facing-headset controller movement");
            }
            finally { facing.SetValue(null, false); }
            Debug.Log("PASS: robot drag leaves camera/floor fixed; scene/controller pose round trip at four headings");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(camera);
            UnityEngine.Object.DestroyImmediate(rig);
            UnityEngine.Object.DestroyImmediate(scene);
        }
    }
}
