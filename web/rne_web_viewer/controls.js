/* Browser controls for RNE's kinematic playground. */
(function (root) {
  "use strict";
  function driveAxes(keys) {
    return {
      forward: Number(keys.has("KeyW") || keys.has("ArrowUp")) - Number(keys.has("KeyS") || keys.has("ArrowDown")),
      turn: Number(keys.has("KeyA") || keys.has("ArrowLeft")) - Number(keys.has("KeyD") || keys.has("ArrowRight")),
    };
  }
  if (typeof module !== "undefined") module.exports = { driveAxes };
  if (!root.document) return;
  const document = root.document;
  const keys = new Set();
  const pointers = new Map();
  const forward = document.getElementById("control-forward");
  const turn = document.getElementById("control-turn");
  function update() {
    const combined = new Set(keys);
    for (const key of pointers.values()) combined.add(key);
    const axes = driveAxes(combined);
    forward.value = String(axes.forward);
    turn.value = String(axes.turn);
  }
  function releaseAll() { keys.clear(); pointers.clear(); update(); }
  const driveCodes = new Set(["KeyW", "KeyS", "KeyA", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"]);
  root.addEventListener("keydown", event => {
    if (/^(INPUT|TEXTAREA|SELECT|BUTTON)$/.test(event.target.tagName) || event.target.isContentEditable) return;
    if (!driveCodes.has(event.code)) return;
    event.preventDefault();
    takeManualControl();
    keys.add(event.code);
    update();
  });
  root.addEventListener("keyup", event => { keys.delete(event.code); update(); });
  root.addEventListener("blur", releaseAll);
  document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });
  for (const button of document.querySelectorAll("[data-drive]")) {
    button.addEventListener("pointerdown", event => {
      event.preventDefault();
      takeManualControl();
      button.setPointerCapture(event.pointerId);
      pointers.set(event.pointerId, button.dataset.drive);
      update();
    });
    const release = event => { pointers.delete(event.pointerId); update(); };
    button.addEventListener("pointerup", release);
    button.addEventListener("pointercancel", release);
    button.addEventListener("lostpointercapture", release);
    // Enter/Space provide keyboard access to the same momentary buttons.
    button.addEventListener("keydown", event => {
      if (event.code !== "Space" && event.code !== "Enter") return;
      event.preventDefault();
      takeManualControl();
      keys.add(button.dataset.drive); update();
    });
    button.addEventListener("keyup", event => {
      if (event.code !== "Space" && event.code !== "Enter") return;
      keys.delete(button.dataset.drive); update();
    });
    button.addEventListener("blur", () => { keys.delete(button.dataset.drive); update(); });
  }
  for (const input of document.querySelectorAll("[data-angle]")) {
    const output = document.getElementById(input.id + "-value");
    const show = () => { output.value = Number(input.value).toFixed(2) + " rad"; };
    input.addEventListener("input", show); show();
  }
  document.getElementById("control-stop").addEventListener("click", () => { releaseAll(); takeManualControl(); });
  function homeTargets() {
    for (const input of document.querySelectorAll("[data-angle]")) {
      input.value = input.defaultValue;
      input.dispatchEvent(new Event("input"));
    }
  }
  function signal(id) {
    if (["experiment-run","experiment-compare","experiment-manual"].includes(id)) {
      document.getElementById("nav-replay-enabled").value="0";
      document.getElementById("mobile-replay-enabled").value="0";
      document.getElementById("arm-replay-enabled").value="0";
      document.getElementById("arm-intent").value="0";
      document.getElementById("mobile-intent").value="0";
    }
    const intent = {"experiment-manual": "0", "experiment-run": "1", "experiment-compare": "2"}[id];
    if (intent !== undefined) document.getElementById("experiment-intent").value = intent;
    const counter = document.getElementById(id);
    counter.value = String((Number(counter.value) + 1) % 4294967296);
  }
  const sceneEditor = root.RNESceneEditor.init({document, signal, releaseAll});
  const experimentPanel = document.getElementById("experiment-panel");
  if (root.innerWidth > 700) experimentPanel.open = true;
  for (const field of ["task", "planner", "controller"]) {
    const select = document.getElementById("experiment-" + field + "-select");
    select.addEventListener("change", () => { document.getElementById("experiment-" + field).value = select.value; if (field === "task") sceneEditor.selectTask(select.value); });
  }
  const taskNotes = [
    "基本 · 障害物のない床で、到達時間と経路への追従を比較します。",
    "基本 · 直線上の障害物を迂回。計画失敗と制御の成否を見比べます。",
    "応用 · 2秒後に通路を塞ぎます。再計画と停止・迂回の挙動に注目。",
    "自由設定 · 編集した同じシーンで4組を比較します。",
    "中級 · 幅0.96mの通路。追従誤差と障害物までの最小余裕を比較。",
    "難関 · 障害物が0.4m/sで往復。現在位置を使って3Hzで再計画します。待機設定で再試行できます。未来位置の予測は含みません。",
    "中級 · U字の壁から一度遠回りして脱出。大域的な迂回計画に注目。",
    "中級 · 番号順に3地点を巡回。目標の切替と曲がり角の追従を比較。"
  ];
  const taskSelect = document.getElementById("experiment-task-select");
  const updateTaskNote = () => { document.getElementById("experiment-task-description").textContent = taskNotes[Number(taskSelect.value)]; };
  taskSelect.addEventListener("change", updateTaskNote);
  new MutationObserver(updateTaskNote).observe(document.getElementById("scene-editor"), {attributes:true,attributeFilter:["open"]});
  for (const checkbox of document.querySelectorAll("[data-trace]")) {
    checkbox.addEventListener("change", () => {
      let hidden = 0;
      for (const item of document.querySelectorAll("[data-trace]")) if (!item.checked) hidden |= 1 << Number(item.dataset.trace);
      document.getElementById("experiment-hidden-traces").value = String(hidden);
    });
  }
  const seedEditor = document.getElementById("experiment-seed-editor");
  seedEditor.addEventListener("change", () => {
    const value = Number(seedEditor.value);
    const seed = document.getElementById("experiment-seed");
    if (seedEditor.value !== "" && Number.isInteger(value) && value >= 0 && value <= 4294967295) seed.value = String(value);
    else seedEditor.value = seed.value;
  });
  for (const action of ["run", "pause", "step", "compare", "manual"]) {
    document.getElementById("experiment-" + action + "-button").addEventListener("click", () => {
      releaseAll();
      document.getElementById("nav-replay-enabled").value="0";
      if (action === "run" || action === "compare") { homeTargets(); sceneEditor.close(); }
      if (action === "manual") sceneEditor.close();
      signal("experiment-" + action);
    });
  }
  document.getElementById("experiment-results-button").addEventListener("click", () => {
    document.querySelector(".experiment-results").scrollIntoView({ block: "nearest" });
  });
  document.getElementById("experiment-recovery-toggle").addEventListener("change",event=>{document.getElementById("experiment-recovery").value=event.target.checked?"1":"0";});
  for (const action of ["run","compare","benchmark","pause","step","stop"]) {
    document.getElementById("arm-"+action+"-button").addEventListener("click",()=>{
      releaseAll();document.getElementById("arm-replay-enabled").value="0";
      if (action==="run" || action==="compare" || action==="benchmark") {
        sceneEditor.close();homeTargets();signal("experiment-manual");
        document.getElementById("arm-intent").value=action==="benchmark"?"3":action==="compare"?"2":"1";
      }
      if (action==="stop") document.getElementById("arm-intent").value="0";
      signal("arm-"+(action==="benchmark"?"compare":action));
    });
  }
  for (const action of ["run","compare","pause","step","stop"]) {
    document.getElementById("mobile-"+action+"-button").addEventListener("click",()=>{
      releaseAll();document.getElementById("mobile-replay-enabled").value="0";
      if (action==="run" || action==="compare") {
        sceneEditor.close();homeTargets();signal("experiment-manual");
        document.getElementById("mobile-intent").value=action==="compare"?"2":"1";
      }
      if (action==="stop") document.getElementById("mobile-intent").value="0";
      signal("mobile-"+action);
    });
  }
  document.getElementById("mobile-example-button").addEventListener("click",()=>{
    for (const [id,value] of [["experiment-planner-select","1"],["experiment-controller-select","1"],["arm-planner","2"],["arm-controller","0"],["experiment-seed-editor","42"]]) {
      const input=document.getElementById(id);input.value=value;input.dispatchEvent(new Event("change",{bubbles:true}));
    }
    document.getElementById("mobile-run-button").click();
  });
  for (const prefix of ["arm","mobile"]) {
    document.getElementById(prefix+"-export-button").addEventListener("click",()=>{
      const csv=document.getElementById(prefix+"-export-data").textContent;
      if (!csv) return;
      const url=URL.createObjectURL(new Blob(["\uFEFF",csv],{type:"text/csv;charset=utf-8"}));
      const link=document.createElement("a");
      link.href=url;link.download="rne-"+prefix+"-results.csv";
      document.body.appendChild(link);link.click();link.remove();
      setTimeout(()=>URL.revokeObjectURL(url),1000);
    });
  }
  for (const prefix of ["nav","arm","mobile"]) {
    const select=document.getElementById(prefix+"-replay-record"), slider=document.getElementById(prefix+"-replay-frame"), enabled=document.getElementById(prefix+"-replay-enabled");
    let times=[0];
    const show=()=>{const tick=times[Number(slider.value)]||0;document.getElementById(prefix+"-replay-tick").value=String(tick);document.getElementById(prefix+"-replay-time").value=(tick/30).toFixed(2)+" s";};
    const configure=()=>{const option=select.selectedOptions[0];times=option?option.dataset.times.split(",").map(Number):[0];slider.disabled=!option;slider.max=String(times.length-1);slider.value="0";show();};
    new MutationObserver(()=>{enabled.value="0";configure();}).observe(select,{childList:true});
    select.addEventListener("change",()=>{configure();enabled.value="1";});
    slider.addEventListener("input",()=>{enabled.value="1";show();});
    document.getElementById(prefix+"-replay-exit").addEventListener("click",()=>{enabled.value="0";});
  }
  function takeManualControl() {
    sceneEditor.close();
    if (document.getElementById("playground").dataset.experimentActive === "true") signal("experiment-manual");
  }
  document.getElementById("control-wave").addEventListener("click", () => signal("control-wave-counter"));
  const motionButton = document.getElementById("control-motion");
  motionButton.addEventListener("click", () => {
    const enabled = document.getElementById("control-motion-enabled");
    enabled.value = enabled.value === "1" ? "0" : "1";
    motionButton.setAttribute("aria-pressed", String(enabled.value === "1"));
    motionButton.textContent = "歩行モーション " + (enabled.value === "1" ? "ON" : "OFF");
  });
  document.getElementById("mission-start").addEventListener("click", () => {
    sceneEditor.close(); releaseAll(); homeTargets(); signal("control-mission-counter");
  });
  document.getElementById("control-reset").addEventListener("click", () => {
    sceneEditor.close(); releaseAll(); homeTargets(); signal("control-reset-counter");
  });
})(typeof window === "undefined" ? globalThis : window);
