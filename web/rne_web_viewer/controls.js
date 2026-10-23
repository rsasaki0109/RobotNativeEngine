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
      if (action === "run" || action === "compare") { homeTargets(); sceneEditor.close(); }
      if (action === "manual") sceneEditor.close();
      signal("experiment-" + action);
    });
  }
  document.getElementById("experiment-results-button").addEventListener("click", () => {
    document.querySelector(".experiment-results").scrollIntoView({ block: "nearest" });
  });
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
