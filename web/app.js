"use strict";

// These pure readers are also exercised under Node without a headset.
const DualInput = (() => {
  const invalidController = (connected = false) => ({pose:null, connected, tracked:false,
    grip:0, trigger:0, buttons:[], axes:[0, 0]});
  function poseOf(value) {
    if (!value || value.emulatedPosition !== false) return null;
    const p = value.transform?.position, q = value.transform?.orientation;
    const position = [p?.x, p?.y, p?.z], orientation = [q?.x, q?.y, q?.z, q?.w];
    if (![...position, ...orientation].every(Number.isFinite) || position.some(v => Math.abs(v) > 20)) return null;
    const norm = Math.hypot(...orientation);
    if (Math.abs(norm - 1) > .02) return null;
    return {position, orientation_xyzw:orientation.map(v => v / norm)};
  }
  function controller(frame, space, source, side) {
    if (!source || !source.gripSpace || source.targetRayMode !== "tracked-pointer") return invalidController();
    const pad = source.gamepad;
    if (!pad || pad.connected !== true) return invalidController();
    if (pad.mapping !== "xr-standard" || !pad.buttons || pad.buttons.length < 6) return invalidController(true);
    const buttons = Array.from(pad.buttons).slice(0, 6);
    if (!buttons.every(b => b && typeof b.pressed === "boolean" && Number.isFinite(b.value) && b.value >= 0 && b.value <= 1)) return invalidController(true);
    const pose = poseOf(frame.getPose(source.gripSpace, space));
    if (!pose) return invalidController(true);
    const axes = Array.from(pad.axes || []);
    if (axes.length < 4 || !axes.every(v => Number.isFinite(v) && Math.abs(v) <= 1)) return invalidController(true);
    const names = side === "left" ? ["X", "Y"] : ["A", "B"];
    const pressed = [];
    if (buttons[3].pressed) pressed.push("thumbstick");
    if (buttons[4].pressed) pressed.push(names[0]);
    if (buttons[5].pressed) pressed.push(names[1]);
    if (buttons[2].pressed) pressed.push("menu");
    return {pose, connected:true, tracked:true, grip:buttons[1].value,
      trigger:buttons[0].value, buttons:pressed, axes:axes.slice(2, 4)};
  }
  return {invalidController, poseOf, controller};
})();
if (typeof module !== "undefined" && module.exports) module.exports = DualInput;

if (typeof document !== "undefined") (() => {
  const $ = id => document.getElementById(id);
  let session = null, referenceSpace = null, gl = null, renderer = null;
  let sessionId = crypto.randomUUID(), sequence = 0, lastWireTime = -1;
  let sources = {left:null, right:null}, pendingInvalid = null, inFlight = false;
  const sourceIds = new WeakMap();
  let nextSourceId = 0, sourceTracked = {left:false, right:false};
  let nonce = null, nonceAt = -Infinity, blurred = false;
  let supportsAR = false, supportsVR = false, lastUI = -Infinity;
  let latest = null, message = "等待进入采集", lastReply = {};
  const sceneVisible = () => session?.environmentBlendMode === "alpha-blend" || session?.environmentBlendMode === "additive";
  const focused = () => !!session && session.visibilityState === "visible" && document.visibilityState === "visible" && !blurred;
  function setStatus(title, detail) { $("status").textContent = title; $("detail").textContent = detail; }
  function rotate() {
    sessionId = crypto.randomUUID(); sequence = 0; lastWireTime = -1;
    nonce = null; nonceAt = -Infinity; sources = {left:null, right:null};
    sourceTracked = {left:false, right:false};
  }
  function invalidFrame(reason, side = null) {
    const frame = {source_time_s:performance.now()/1000, focused:false,
      head:null, left:DualInput.invalidController(),
      right:DualInput.invalidController(), source:"webxr", scene_visible:sceneVisible()};
    frame.reason = reason;
    return frame;
  }
  function invalidate(reason, side = null, newSession = false) {
    if (newSession || !side) rotate();
    if (side) sources[side] = null; else sources = {left:null, right:null};
    pendingInvalid = invalidFrame(reason, side); latest = pendingInvalid;
    message = reason;
    show(latest, true); pump(null);
  }
  async function request(path, options = {}) {
    const abort = new AbortController(), timer = setTimeout(() => abort.abort(), 500);
    try {
      const response = await fetch(path, {credentials:"same-origin", mode:"same-origin", redirect:"error",
        cache:"no-store", ...options, signal:abort.signal});
      const reply = await response.json();
      if (!response.ok) { const error = new Error(reply.error || `HTTP ${response.status}`); error.reply = reply; throw error; }
      return reply;
    } finally { clearTimeout(timer); }
  }
  function takeNonce(reply) {
    const value = reply?.next_nonce || reply?.nonce;
    nonce = typeof value === "string" && value.length >= 16 ? value : null;
    nonceAt = performance.now();
  }
  function pump(sample) {
    if (inFlight) return;
    const chosen = pendingInvalid || sample;
    if (!chosen) return;
    const generation = sessionId;
    if (!nonce || performance.now() - nonceAt > 85) {
      inFlight = true;
      request("/api/challenge").then(reply => { if (generation === sessionId) takeNonce(reply); })
        .catch(error => { nonce = null; message = `连接中断：${error.message}`; })
        .finally(() => { inFlight = false; if (pendingInvalid) pump(null); });
      return;
    }
    // Never send a queued old valid pose when a previous request completes.
    // Only the current animation frame can initiate a valid send.
    const invalid = pendingInvalid;
    const sourceTime = invalid ? Math.max(chosen.source_time_s, lastWireTime + .000001) : chosen.source_time_s;
    if (sourceTime <= lastWireTime) return;
    const age = Math.max(0, performance.now()/1000 - chosen.source_time_s);
    if (age > .100 && !invalid) return;
    const body = {nonce, session:generation, sequence:++sequence, source_time_s:sourceTime,
      source_age_s:invalid ? 0 : age, focused:chosen.focused, head:chosen.head,
      left:chosen.left, right:chosen.right, source:"webxr", scene_visible:chosen.scene_visible};
    nonce = null; lastWireTime = sourceTime; inFlight = true;
    request("/api/frame", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body)})
      .then(reply => {
        if (generation !== sessionId) return;
        takeNonce(reply); lastReply = reply; message = "输入连接正常";
        if (pendingInvalid === invalid) pendingInvalid = null;
      }).catch(error => {
        if (generation !== sessionId) return;
        if (error.reply) takeNonce(error.reply);
        message = `输入未接受：${error.message}`;
        // A fresh session and release barriers replace an uncertain transport
        // outcome. The receiver independently expires any previously sent pose.
        rotate(); pendingInvalid = invalidFrame(message); latest = pendingInvalid;
      }).finally(() => { inFlight = false; if (pendingInvalid) pump(null); });
  }
  function show(frame, force = false) {
    if (!force && performance.now()-lastUI < 100) return;
    lastUI = performance.now();
    for (const side of ["left", "right"]) {
      const input = frame[side];
      $(side).textContent = JSON.stringify({连接:input.connected, 跟踪:input.tracked,
        需先松开:lastReply[`${side}_release_required`] ?? true, 握持:input.grip, 扳机:input.trigger,
        按键:input.buttons, 摇杆:input.axes, 位姿:input.pose}, null, 2);
    }
    $("head").textContent = JSON.stringify({头部位姿:frame.head, 聚焦:frame.focused,
      现场透视:frame.scene_visible, 状态:message}, null, 2);
    setStatus(frame.focused ? "正在采集双手柄" : "输入暂停", `${message}。${frame.scene_visible ? "现场透视已开启" : "未开启现场透视"}。`);
  }
  function readFrame(timestamp, frame) {
    if (!focused()) return invalidFrame("头显页面未聚焦");
    const viewer = frame.getViewerPose(referenceSpace);
    const result = {source_time_s:timestamp/1000, focused:true, head:DualInput.poseOf(viewer),
      source:"webxr", scene_visible:sceneVisible()};
    for (const side of ["left", "right"]) {
      const candidates = Array.from(session.inputSources).filter(source => source.handedness === side);
      if (candidates.length !== 1) {
        if (sources[side]) sourceIds.delete(sources[side]);
        sources[side] = null; sourceTracked[side] = false;
        result[side] = DualInput.invalidController(); continue;
      }
      const source = candidates[0];
      if (sources[side] && sources[side] !== source) {
        sources[side] = source;
        result[side] = DualInput.invalidController(true); // Independent reconnect release barrier.
      } else {
        sources[side] = source;
        result[side] = DualInput.controller(frame, referenceSpace, source, side);
      }
      if (!result[side].tracked && sourceTracked[side]) sourceIds.delete(source);
      sourceTracked[side] = result[side].tracked;
      if (!sourceIds.has(source)) sourceIds.set(source, `${side}-${++nextSourceId}`);
      result[side].source_id = sourceIds.get(source);
    }
    return result;
  }
  function createRenderer(context) {
    function shader(type, source) {
      const value = context.createShader(type); context.shaderSource(value, source); context.compileShader(value);
      if (!context.getShaderParameter(value, context.COMPILE_STATUS)) throw new Error("显示层着色器初始化失败");
      return value;
    }
    const program = context.createProgram();
    context.attachShader(program, shader(context.VERTEX_SHADER, "attribute vec2 p;attribute vec2 uv;uniform mat4 projection;varying vec2 tex;void main(){tex=uv;gl_Position=projection*vec4(p.x*.60,p.y*.25+.65,-1.9,1.0);}"));
    context.attachShader(program, shader(context.FRAGMENT_SHADER, "precision mediump float;varying vec2 tex;uniform sampler2D panel;void main(){vec4 c=texture2D(panel,tex);gl_FragColor=vec4(c.rgb*c.a,c.a);}"));
    context.linkProgram(program);
    if (!context.getProgramParameter(program, context.LINK_STATUS)) throw new Error("显示层初始化失败");
    const buffer = context.createBuffer(); context.bindBuffer(context.ARRAY_BUFFER, buffer);
    context.bufferData(context.ARRAY_BUFFER, new Float32Array([-1,-1,0,0,1,-1,1,0,-1,1,0,1,1,1,1,1]), context.STATIC_DRAW);
    const texture = context.createTexture(); context.bindTexture(context.TEXTURE_2D, texture);
    context.texParameteri(context.TEXTURE_2D, context.TEXTURE_MIN_FILTER, context.LINEAR);
    context.texParameteri(context.TEXTURE_2D, context.TEXTURE_MAG_FILTER, context.LINEAR);
    context.texParameteri(context.TEXTURE_2D, context.TEXTURE_WRAP_S, context.CLAMP_TO_EDGE);
    context.texParameteri(context.TEXTURE_2D, context.TEXTURE_WRAP_T, context.CLAMP_TO_EDGE);
    const panel = document.createElement("canvas"); panel.width = 1024; panel.height = 360;
    const ctx = panel.getContext("2d");
    return (frame, input) => {
      const viewer = frame.getViewerPose(referenceSpace), layer = session?.renderState.baseLayer;
      if (!viewer || !layer || !ctx) return;
      ctx.clearRect(0, 0, 1024, 360); ctx.fillStyle = "rgba(17,34,45,.88)"; ctx.fillRect(0, 0, 1024, 360);
      ctx.font = "bold 40px sans-serif"; ctx.fillStyle = "#7ce7bd"; ctx.fillText("PICO 双手柄采集", 35, 58);
      ctx.font = "30px sans-serif"; ctx.fillStyle = "#edf5fa";
      const handText = side => `${side === "left" ? "左" : "右"}手柄：${input[side].tracked ? "跟踪正常" : "跟踪无效"}  握持${Math.round(input[side].grip*100)}% 扳机${Math.round(input[side].trigger*100)}%`;
      [handText("left"), handText("right"), "左右握持键分别使能；松开保持", "断开后先松开按键，再重新握持", message.slice(0, 45)].forEach((line, i) => ctx.fillText(line, 35, 115+i*47));
      context.bindTexture(context.TEXTURE_2D, texture); context.pixelStorei(context.UNPACK_FLIP_Y_WEBGL, true);
      context.texImage2D(context.TEXTURE_2D, 0, context.RGBA, context.RGBA, context.UNSIGNED_BYTE, panel);
      context.bindFramebuffer(context.FRAMEBUFFER, layer.framebuffer);
      context.clearColor(0, 0, 0, sceneVisible() ? 0 : 1); context.clear(context.COLOR_BUFFER_BIT | context.DEPTH_BUFFER_BIT);
      context.useProgram(program); context.bindBuffer(context.ARRAY_BUFFER, buffer);
      for (const [name, offset] of [["p", 0], ["uv", 8]]) {
        const location = context.getAttribLocation(program, name); context.enableVertexAttribArray(location);
        context.vertexAttribPointer(location, 2, context.FLOAT, false, 16, offset);
      }
      context.activeTexture(context.TEXTURE0); context.bindTexture(context.TEXTURE_2D, texture);
      context.uniform1i(context.getUniformLocation(program, "panel"), 0);
      for (const view of viewer.views) {
        const viewport = layer.getViewport(view); if (!viewport) continue;
        context.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
        context.uniformMatrix4fv(context.getUniformLocation(program, "projection"), false, view.projectionMatrix);
        context.drawArrays(context.TRIANGLE_STRIP, 0, 4);
      }
    };
  }
  function animate(timestamp, frame) {
    if (!session || frame.session !== session) return;
    try {
      latest = readFrame(timestamp, frame); show(latest); pump(latest); renderer?.(frame, latest);
    } catch (error) { invalidate(`读取输入失败：${error.message}`); }
    session?.requestAnimationFrame(animate);
  }
  function ended() {
    invalidate("已退出采集"); session = null; referenceSpace = null; renderer = null;
    $("start").disabled = false; $("stop").disabled = true;
  }
  async function start() {
    if (session) return;
    $("start").disabled = true;
    try {
      const opening = navigator.xr.requestSession(supportsAR ? "immersive-ar" : "immersive-vr", {optionalFeatures:["local-floor"]});
      session = await opening; rotate(); blurred = false;
      session.addEventListener("end", ended, {once:true});
      session.addEventListener("visibilitychange", () => { if (!focused()) invalidate("头显页面失去焦点"); });
      session.addEventListener("inputsourceschange", event => {
        // The next XR frame reads the other hand and head anew. Do not copy
        // their old poses into a synthetic frame with a fresh timestamp.
        for (const source of event.removed || []) if (["left", "right"].includes(source.handedness)) {
          sourceIds.delete(source); sources[source.handedness] = null;
          sourceTracked[source.handedness] = false; message = "对应手柄已断开，重新连接后先松开按键";
        }
      });
      try { referenceSpace = await session.requestReferenceSpace("local-floor"); }
      catch (_) { referenceSpace = await session.requestReferenceSpace("local"); }
      referenceSpace.addEventListener("reset", () => invalidate("定位参考已改变，请松开后重新握持", null, true));
      gl = $("xr-canvas").getContext("webgl", {xrCompatible:true, alpha:true, premultipliedAlpha:true});
      if (!gl || typeof XRWebGLLayer !== "function") throw new Error("浏览器未提供 WebXR 显示层");
      await gl.makeXRCompatible();
      session.updateRenderState({baseLayer:new XRWebGLLayer(session, gl, {alpha:true})});
      renderer = createRenderer(gl); $("stop").disabled = false;
      invalidate("已进入采集，先松开两侧握持键与扳机"); session.requestAnimationFrame(animate);
    } catch (error) {
      setStatus("未进入采集", error.message);
      if (session) { try { await session.end(); } catch (_) { ended(); } }
      else $("start").disabled = false;
    }
  }
  let diagnosticBusy = false;
  setInterval(async () => {
    if (diagnosticBusy) return;
    diagnosticBusy = true;
    try {
      const response = await fetch("/api/status", {cache:"no-store"});
      if (!response.ok) return;
      const status = await response.json(), runtime = status.runtime || {};
      $("runtime").textContent = JSON.stringify({运行:runtime.state, 接管:runtime.active,
        状态:runtime.reasons || runtime.reason, 离线计算:runtime.ik?.backend || "仅位置目标预览"}, null, 2);
      const canvas = $("model-canvas"), ctx = canvas?.getContext("2d");
      if (!ctx) return;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#c5d9e2"; ctx.font = "18px sans-serif";
      ctx.fillText("正面视图", 115, 28); ctx.fillText("侧面视图", 465, 28);
      const project = (point, panel) => [panel*350+175+(panel ? point[0] : -point[1])*250, 140-point[2]*250];
      for (const panel of [0, 1]) {
        for (const side of ["left", "right"]) {
          const points = Object.values(runtime.model_links?.[side] || {});
          if (!points.length) continue;
          ctx.beginPath(); ctx.strokeStyle = side === "left" ? "#79e2b8" : "#77bcff"; ctx.lineWidth = 4;
          points.forEach((point, index) => { const xy=project(point,panel); if (index) ctx.lineTo(...xy); else ctx.moveTo(...xy); });
          ctx.stroke();
          for (const point of points) { const xy=project(point,panel); ctx.beginPath(); ctx.arc(...xy,4,0,2*Math.PI); ctx.fillStyle=ctx.strokeStyle; ctx.fill(); }
          const target=runtime.targets?.[side];
          if (target) { const xy=project([target[0][3],target[1][3],target[2][3]],panel); ctx.strokeRect(xy[0]-6,xy[1]-6,12,12); }
        }
      }
    } catch (_) { /* Input validity is independent of this diagnostic display. */ }
    finally { diagnosticBusy=false; }
  }, 200);
  $("start").addEventListener("click", start);
  $("stop").addEventListener("click", () => session?.end());
  window.addEventListener("blur", () => { blurred = true; invalidate("页面失去焦点"); });
  window.addEventListener("focus", () => { blurred = false; });
  document.addEventListener("visibilitychange", () => { if (document.visibilityState !== "visible") invalidate("页面进入后台"); });
  (async () => {
    if (!isSecureContext || !navigator.xr) return setStatus("浏览器未提供 WebXR", "请通过 USB 转发打开 http://localhost:8890。");
    [supportsAR, supportsVR] = await Promise.all([navigator.xr.isSessionSupported("immersive-ar"), navigator.xr.isSessionSupported("immersive-vr")]);
    $("start").disabled = !(supportsAR || supportsVR);
    setStatus(supportsAR || supportsVR ? "可以开始采集" : "当前浏览器不支持沉浸式采集", supportsAR ? "可使用现场透视。进入后先松开所有按键。" : "当前未确认现场透视；请在控制端确认观察方式。");
  })().catch(error => setStatus("浏览器检查失败", error.message));
})();
