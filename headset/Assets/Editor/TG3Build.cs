using System;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.XR.Interaction.Toolkit.UI;

public static class TG3Build
{
    public static void Android()
    {
        PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Android,"edu.scu.tg3.wireless");
        PlayerSettings.productName="TG3 Wireless";
        PlayerSettings.bundleVersion="0.3.0";
        PlayerSettings.Android.bundleVersionCode=300;
        PlayerSettings.Android.targetArchitectures=AndroidArchitecture.ARM64;
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Android,ScriptingImplementation.IL2CPP);
        PlayerSettings.Android.minSdkVersion=AndroidSdkVersions.AndroidApiLevel30;
        var args=Environment.GetCommandLineArgs(); int index=Array.IndexOf(args,"-tg3Output");
        if(index<0 || index+1>=args.Length) throw new Exception("-tg3Output is required");
        var scenes=EditorBuildSettings.scenes.Where(s=>s.enabled).Select(s=>s.path).ToArray();
        if(scenes.Length==0) throw new Exception("Upstream enabled XR scene missing");
        var scene=EditorSceneManager.OpenScene(scenes[0],OpenSceneMode.Single);
        // Retain the upstream XR rig, SDK, interaction rays and input actions.
        // Remove its old networking, collection and enterprise camera UI from
        // this dedicated scene. No runtime toggling of the system boundary.
        foreach(var behaviour in UnityEngine.Object.FindObjectsOfType<MonoBehaviour>(true))
        {
            if(!behaviour) continue;
            var script=MonoScript.FromMonoBehaviour(behaviour);
            if(script && AssetDatabase.GetAssetPath(script).StartsWith("Assets/Scripts/"))
                UnityEngine.Object.DestroyImmediate(behaviour);
            else if(behaviour.GetType().Name.Contains("MoveProvider") || behaviour.GetType().Name.Contains("TurnProvider")
                || behaviour.GetType().Name.Contains("TeleportationProvider")) behaviour.enabled=false;
        }
        foreach(var canvas in UnityEngine.Object.FindObjectsOfType<Canvas>(true))
        {
            canvas.enabled=false;
            foreach(var raycaster in canvas.GetComponentsInChildren<UnityEngine.UI.GraphicRaycaster>(true)) raycaster.enabled=false;
        }
        if(!UnityEngine.Object.FindObjectOfType<EventSystem>())
            new GameObject("TG3 XR EventSystem",typeof(EventSystem),typeof(XRUIInputModule));
        if(!Camera.main) throw new Exception("Upstream tracked main camera missing");
        Camera.main.clearFlags=CameraClearFlags.SolidColor;
        Camera.main.backgroundColor=new Color(0,0,0,0);
        const string generated="Assets/TG3/TG3.unity";
        EditorSceneManager.SaveScene(scene,generated);
        var result=BuildPipeline.BuildPlayer(new[]{generated},args[index+1],BuildTarget.Android,BuildOptions.None);
        if(result.summary.result!=BuildResult.Succeeded) throw new Exception("Android build failed");
    }
}
