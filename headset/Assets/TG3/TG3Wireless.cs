// Added to XRoboToolkit-Unity-Client, pinned by prepare_project.py.
// Uses Unity XR poses ONLY. Never mix raw PXR poses (a different convention).
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Http;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using LitJson;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit.UI;
using Unity.XR.PXR;
using Debug = UnityEngine.Debug;

public sealed class TG3Wireless : MonoBehaviour
{
    const string Protocol = "tg3-native-1";
    readonly object gate = new object();
    readonly List<XRInputSubsystem> subsystems = new List<XRInputSubsystem>();
    readonly Dictionary<XRNode, ulong> devices = new Dictionary<XRNode, ulong>();
    readonly Dictionary<XRNode, int> generations = new Dictionary<XRNode, int>();
    Dictionary<string, object> latest;
    long captured;
    string session = Guid.NewGuid().ToString("N");
    long sequence;
    bool focused = true, paused, uiMode = true, menuWasDown, panelPlaced;
    string token, address, networkState = "Enter PC IP and pairing code";
    CancellationTokenSource cancellation;
    Task worker;
    Text status;
    InputField hostInput, pinInput;
    InputField editing;
    Canvas panel;
    float nextCapture;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        if (FindObjectOfType<TG3Wireless>() == null)
        {
            var obj = new GameObject("TG3 Wireless");
            DontDestroyOnLoad(obj);
            obj.AddComponent<TG3Wireless>();
        }
    }

    void Start()
    {
        Application.runInBackground = false;
        PXR_Manager.EnableVideoSeeThrough = true;
        SubsystemManager.GetInstances(subsystems);
        foreach (var system in subsystems) system.trackingOriginUpdated += OriginChanged;
        InputDevices.deviceDisconnected += Disconnected;
        MakePanel();
    }

    void OriginChanged(XRInputSubsystem unused) { ResetReference(); }
    void Disconnected(InputDevice device) { ResetReference(); }
    void ResetReference()
    {
        lock (gate)
        {
            session = Guid.NewGuid().ToString("N");
            sequence = 0;
            latest = null;
        }
    }
    void OnApplicationFocus(bool value) { focused = value; ResetReference(); }
    void OnApplicationPause(bool value)
    {
        paused = value;
        ResetReference();
        if (!value) PXR_Manager.EnableVideoSeeThrough = true;
    }

    Dictionary<string, object> ReadPose(InputDevice device, out bool tracked)
    {
        bool hasTracked = device.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked);
        bool hasState = device.TryGetFeatureValue(CommonUsages.trackingState, out InputTrackingState flags);
        bool hasPosition = device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 p);
        bool hasRotation = device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion q);
        var required = InputTrackingState.Position | InputTrackingState.Rotation;
        tracked = device.isValid && hasTracked && isTracked && hasState && (flags & required) == required
            && hasPosition && hasRotation && Finite(p.x) && Finite(p.y) && Finite(p.z)
            && Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w)
            && Mathf.Abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < .02f;
        if (!tracked) return null;
        return new Dictionary<string, object> {
            {"position", new double[] {p.x,p.y,p.z}},
            {"orientation_xyzw", new double[] {q.x,q.y,q.z,q.w}} };
    }
    static bool Finite(float value) { return !float.IsNaN(value) && !float.IsInfinity(value); }

    Dictionary<string, object> ReadHand(XRNode node, string primary, string secondary)
    {
        var device = InputDevices.GetDeviceAtXRNode(node);
        var pose = ReadPose(device, out bool tracked);
        bool controls = device.TryGetFeatureValue(CommonUsages.grip, out float grip)
            & device.TryGetFeatureValue(CommonUsages.trigger, out float trigger)
            & device.TryGetFeatureValue(CommonUsages.primaryButton, out bool first)
            & device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool second)
            & device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 axes);
        tracked &= controls && Finite(grip) && Finite(trigger) && Finite(axes.x) && Finite(axes.y);
        if (!tracked) pose = null;
        if (!devices.ContainsKey(node) || devices[node] != device.deviceId)
        {
            devices[node] = device.deviceId;
            generations[node] = generations.ContainsKey(node) ? generations[node]+1 : 0;
        }
        var buttons = new List<string>();
        if (first) buttons.Add(primary);
        if (second) buttons.Add(secondary);
        return new Dictionary<string, object> {
            {"pose",pose}, {"connected",device.isValid}, {"tracked",tracked},
            {"grip",tracked ? (double)Mathf.Clamp01(grip) : 0.0},
            {"trigger",tracked ? (double)Mathf.Clamp01(trigger) : 0.0}, {"buttons",buttons},
            {"axes",new double[] {tracked ? axes.x : 0, tracked ? axes.y : 0}},
            {"source_id",node+"-"+device.deviceId+"-"+generations[node]} };
    }

    void Update()
    {
        var leftDevice=InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        leftDevice.TryGetFeatureValue(CommonUsages.menuButton,out bool menuDown);
        if(menuDown && !menuWasDown)
        {
            uiMode=true; ResetReference(); PlacePanel();
            if(panel) panel.enabled=true;
        }
        menuWasDown=menuDown;
        if (Time.unscaledTime >= nextCapture)
        {
            nextCapture = Time.unscaledTime + .015f;
            bool usable = focused && !paused && !uiMode;
            var head = ReadPose(InputDevices.GetDeviceAtXRNode(XRNode.Head), out bool headTracked);
            var frame = new Dictionary<string, object> {
                {"source_time_s",Time.realtimeSinceStartupAsDouble}, {"source_age_s",0.0},
                {"source","xrobotoolkit"}, {"focused",usable && headTracked},
                {"scene_visible",PXR_Manager.EnableVideoSeeThrough}, {"head",head},
                {"left",ReadHand(XRNode.LeftHand,"X","Y")},
                {"right",ReadHand(XRNode.RightHand,"A","B")} };
            lock (gate)
            {
                frame["session"] = session;
                frame["sequence"] = sequence++;
                latest = frame;
                captured = Stopwatch.GetTimestamp();
            }
            var left = (Dictionary<string,object>)frame["left"];
            var right = (Dictionary<string,object>)frame["right"];
            if(headTracked && !panelPlaced) PlacePanel();
            if (status) status.text = "TG3 Wireless 0.3\n"+networkState+"\nHead: "+headTracked+
                "  L: "+left["tracked"]+"  R: "+right["tracked"]+
                "\n"+(uiMode ? "INPUT PAUSED: use START INPUT after pairing" : "INPUT ACTIVE")+
                "\nGrip = follow / release = hold\nTrigger = close + retain; Y / B = open\nRelease grip before walking. Hardware enable is on PC.";
        }
    }

    async Task<JsonData> Request(HttpClient client, string path, object body = null, string nonce = null)
    {
        using (var request = new HttpRequestMessage(body == null ? HttpMethod.Get : HttpMethod.Post, address+path))
        {
            if (token != null) request.Headers.TryAddWithoutValidation("Authorization", "Bearer "+token);
            if (nonce != null) request.Headers.TryAddWithoutValidation("X-TG3-Nonce",nonce);
            if (body != null) request.Content = new StringContent(JsonMapper.ToJson(body),Encoding.UTF8,"application/json");
            using (var response = await client.SendAsync(request,cancellation.Token))
            {
                string text = await response.Content.ReadAsStringAsync();
                if (text.Length > 16384) throw new Exception("Oversize server reply");
                response.EnsureSuccessStatusCode();
                return JsonMapper.ToObject(text);
            }
        }
    }

    async void Pair()
    {
        if (worker != null && !worker.IsCompleted) return;
        if (!IPAddress.TryParse(hostInput.text,out IPAddress ip) || ip.AddressFamily != System.Net.Sockets.AddressFamily.InterNetwork
            || pinInput.text.Length != 8) { networkState = "Check IPv4 and 8 digit code"; return; }
        address = "http://"+ip+":8877";
        token = null;
        cancellation = new CancellationTokenSource();
        try
        {
            using (var client = new HttpClient {Timeout = TimeSpan.FromSeconds(2)})
            {
                var reply = await Request(client,"/v1/pair",new Dictionary<string,object> {
                    {"protocol",Protocol},{"pin",pinInput.text},{"device",SystemInfo.deviceModel}});
                token = (string)reply["token"];
            }
            pinInput.text = "";
            PlayerPrefs.SetString("tg3_pc_ip",hostInput.text); // IP only; never save token or PIN.
            ResetReference();
            worker = Task.Run(Stream);
        }
        catch (Exception) { networkState = "Pair failed. Reopen pairing on PC."; }
    }

    async Task Stream()
    {
        long lastSequence = -1;
        string lastSession = "";
        using (var client = new HttpClient {Timeout = TimeSpan.FromMilliseconds(120)})
        {
            while (!cancellation.IsCancellationRequested)
            {
                try
                {
                    var challenge = await Request(client,"/v1/challenge");
                    Dictionary<string,object> frame;
                    lock (gate)
                    {
                        frame = latest == null ? null : new Dictionary<string,object>(latest);
                        if (frame != null) frame["source_age_s"] = (Stopwatch.GetTimestamp()-captured)/(double)Stopwatch.Frequency;
                    }
                    if (frame == null || (double)frame["source_age_s"] > .12
                        || ((long)frame["sequence"] == lastSequence && (string)frame["session"] == lastSession))
                    { await Task.Delay(15); continue; }
                    var envelope = new Dictionary<string,object> {
                        {"protocol",Protocol},{"coordinates","unity_lh_y_up"},{"frame",frame}};
                    await Request(client,"/v1/frame",envelope,(string)challenge["nonce"]);
                    lastSequence = (long)frame["sequence"]; lastSession = (string)frame["session"];
            networkState = "WIRELESS INPUT CONNECTED (not hardware authority)";
                    await Task.Delay(10);
                }
                catch (Exception)
                {
                    if (cancellation.IsCancellationRequested) break;
                    networkState = "Network lost / release all inputs to reconnect";
                    ResetReference();
                    await Task.Delay(250);
                }
            }
        }
    }

    void MakePanel()
    {
        var obj = new GameObject("TG3 Connection",typeof(Canvas),typeof(CanvasScaler),typeof(TrackedDeviceGraphicRaycaster));
        panel = obj.GetComponent<Canvas>(); panel.renderMode = RenderMode.WorldSpace;
        var rect = obj.GetComponent<RectTransform>(); rect.sizeDelta = new Vector2(720,720);
        obj.transform.localScale = Vector3.one*.0012f;
        var camera = Camera.main;
        if (camera) { obj.transform.position = camera.transform.position+camera.transform.forward*1.25f;
            obj.transform.rotation = Quaternion.LookRotation(camera.transform.forward,Vector3.up); }
        var background = obj.AddComponent<Image>(); background.color = new Color(.04f,.08f,.12f,.85f);
        status = MakeText(obj.transform,new Vector2(0,225),new Vector2(680,230),20);
        hostInput = MakeInput(obj.transform,new Vector2(0,75),PlayerPrefs.GetString("tg3_pc_ip","192.168.3.47"));
        pinInput = MakeInput(obj.transform,new Vector2(0,10),"");
        MakeButton(obj.transform,"PC IP",new Vector2(-260,75),new Vector2(120,50),()=>editing=hostInput);
        MakeButton(obj.transform,"PAIR CODE",new Vector2(-260,10),new Vector2(120,50),()=>editing=pinInput);
        editing = hostInput;
        string[] keys = {"1","2","3","4","5","6","7","8","9",".","0","<"};
        for (int i=0;i<keys.Length;i++)
        {
            string key=keys[i];
            MakeButton(obj.transform,key,new Vector2((i%3-1)*105,-70-(i/3)*54),new Vector2(95,48),()=> {
                string text = editing.text;
                editing.text = key=="<" ? (text.Length==0 ? "" : text.Substring(0,text.Length-1)) : text+key;
            });
        }
        MakeButton(obj.transform,"CONNECT",new Vector2(220,-100),new Vector2(175,60),Pair);
        MakeButton(obj.transform,"CLEAR",new Vector2(220,-170),new Vector2(175,60),()=>editing.text="");
        MakeButton(obj.transform,"DISCONNECT",new Vector2(220,-240),new Vector2(175,60),()=> {
            cancellation?.Cancel(); ResetReference(); networkState="Disconnected. Reopen pairing on PC."; });
        MakeButton(obj.transform,"START INPUT",new Vector2(0,-315),new Vector2(300,55),()=> {
            if(token==null) { networkState="Pair with PC first"; return; }
            uiMode=false; ResetReference(); panel.enabled=false;
        });
    }
    void PlacePanel()
    {
        var camera=Camera.main;
        if(!camera || !panel) return;
        panel.transform.position=camera.transform.position+camera.transform.forward*1.25f;
        panel.transform.rotation=Quaternion.LookRotation(camera.transform.forward,Vector3.up);
        panelPlaced=true;
    }
    Text MakeText(Transform parent,Vector2 location,Vector2 size,int fontSize)
    {
        var obj = new GameObject("Text",typeof(RectTransform),typeof(Text)); obj.transform.SetParent(parent,false);
        var rect=obj.GetComponent<RectTransform>(); rect.anchoredPosition=location; rect.sizeDelta=size;
        var text=obj.GetComponent<Text>(); text.font=Resources.GetBuiltinResource<Font>("Arial.ttf");
        text.fontSize=fontSize; text.color=Color.white; text.alignment=TextAnchor.MiddleCenter;
        return text;
    }
    InputField MakeInput(Transform parent,Vector2 location,string value)
    {
        var obj=new GameObject("Input",typeof(RectTransform),typeof(Image),typeof(InputField));
        obj.transform.SetParent(parent,false); var rect=obj.GetComponent<RectTransform>();
        rect.anchoredPosition=location+new Vector2(70,0); rect.sizeDelta=new Vector2(450,50);
        var input=obj.GetComponent<InputField>(); input.textComponent=MakeText(obj.transform,Vector2.zero,new Vector2(430,45),25);
        input.textComponent.color=Color.black; input.text=value; input.readOnly=true; return input;
    }
    void MakeButton(Transform parent,string label,Vector2 location,Vector2 size,UnityEngine.Events.UnityAction action)
    {
        var obj=new GameObject(label,typeof(RectTransform),typeof(Image),typeof(Button)); obj.transform.SetParent(parent,false);
        var rect=obj.GetComponent<RectTransform>(); rect.anchoredPosition=location; rect.sizeDelta=size;
        obj.GetComponent<Image>().color=new Color(.16f,.3f,.4f,1);
        var button=obj.GetComponent<Button>(); button.onClick.AddListener(action);
        MakeText(obj.transform,Vector2.zero,size,20);
        obj.GetComponentInChildren<Text>().text=label;
    }
    void OnDestroy()
    {
        cancellation?.Cancel();
        InputDevices.deviceDisconnected -= Disconnected;
        foreach (var system in subsystems) system.trackingOriginUpdated -= OriginChanged;
        if (panel) Destroy(panel.gameObject);
    }
}
