/* Task presets that launch the existing navigation, arm and delivery controls. */
(function (root) {
  "use strict";
  const base = { "experiment-seed-editor": "42", "experiment-planner-select": "1", "experiment-controller-select": "1" };
  const arm = { "arm-planner": "2", "arm-controller": "0" };
  const presets = [
    { id: "nav-detour", title: "障害物を迂回", kind: "移動 · 4手法", note: "直線とA*、2つの制御で迂回を比較。seed 42。", domain: "nav", action: "experiment-compare-button", status: "experiment-status", values: { ...base, "experiment-task-select": "1" } },
    { id: "nav-crossing", title: "横切る箱を待つ", kind: "移動 · 4手法", note: "往復する箱に対する停止・再試行を比較。seed 42。", domain: "nav", action: "experiment-compare-button", status: "experiment-status", recovery: true, values: { ...base, "experiment-task-select": "5" } },
    { id: "arm-obstacle", title: "腕で障害物を避ける", kind: "腕 · 6手法", note: "関節直結・上方経由・RRTと2つの制御を比較。seed 42。", domain: "arm", action: "arm-compare-button", status: "arm-status", values: { ...base, ...arm, "arm-task": "2" } },
    { id: "delivery", title: "棚から取って運ぶ", kind: "運搬 · 4手法", note: "腕RRTを固定し、移動・把持・運搬・配置を比較。seed 42。", domain: "mobile", action: "mobile-compare-button", status: "mobile-status", values: { ...base, ...arm, "mobile-scenario": "0" } },
    { id: "delivery-arm", title: "腕の手法を変えて運ぶ", kind: "運搬 · 腕6手法", note: "A*＋DWAを固定し、腕の3計画×2制御で把持と配置を比較。seed 42。", domain: "mobile", action: "mobile-arm-compare-button", status: "mobile-status", values: { ...base, ...arm, "mobile-scenario": "0" } },
    { id: "closure", title: "運搬経路が閉鎖", kind: "運搬 · 4手法", note: "把持後の閉鎖で、失敗する段階を見比べる。seed 42。", domain: "mobile", action: "mobile-compare-button", status: "mobile-status", values: { ...base, ...arm, "mobile-scenario": "1" } },
    { id: "recovery-pair", title: "待って運搬を再開", kind: "再試行 · 2設定", note: "A*＋DWA・腕RRTで、再試行なし／ありを比較。5秒後に開通。seed 42。", domain: "mobile", action: "mobile-recovery-compare-button", status: "mobile-status", values: { ...base, ...arm, "mobile-scenario": "3" } },
  ];

  function init(document) {
    const panel = document.getElementById("experiment-panel");
    const gallery = document.getElementById("demo-gallery");
    const grid = document.getElementById("demo-grid");
    const message = document.getElementById("demo-launch-status");
    const resultLink = document.getElementById("demo-result-link");
    const engine = document.getElementById("engine-status");
    const Event = document.defaultView.Event;
    const buttons = [];
    let selected = null;
    const ready = () => engine.textContent.startsWith("操作できます");
    const setValue = (id, value) => {
      const input = document.getElementById(id);
      input.value = value;
      input.dispatchEvent(new Event("change", { bubbles: true }));
    };
    const setRecovery = (prefix, enabled) => {
      const input = document.getElementById(prefix + "-recovery-toggle");
      input.checked = enabled;
      input.dispatchEvent(new Event("change", { bubbles: true }));
    };
    function showProgress(preset) {
      panel.open = true;
      if (preset.domain === "arm") document.getElementById("manipulation-panel").open = true;
      if (preset.domain === "mobile") document.getElementById("mobile-panel").open = true;
      const status = document.getElementById(preset.status);
      status.setAttribute("tabindex", "-1");
      status.scrollIntoView({ block: "nearest" });
      status.focus({ preventScroll: true });
    }
    function launch(preset, button) {
      if (!ready()) return;
      for (const [id, value] of Object.entries(preset.values)) setValue(id, value);
      setRecovery("experiment", Boolean(preset.recovery));
      setRecovery("mobile", false);
      if (preset.domain === "nav") {
        for (const input of document.querySelectorAll("[data-trace]")) {
          input.checked = true;
          input.dispatchEvent(new Event("change", { bubbles: true }));
        }
      }
      panel.open = true;
      document.getElementById("manipulation-panel").open = preset.domain === "arm";
      document.getElementById("mobile-panel").open = preset.domain === "mobile";
      document.getElementById(preset.action).click();
      selected = { preset, button };
      for (const item of buttons) item.removeAttribute("data-last-selected");
      button.setAttribute("data-last-selected", "true");
      message.textContent = "最後に選んだデモ：" + preset.title + " · " + preset.kind + " · seed 42";
      resultLink.href = "#" + preset.status;
      resultLink.hidden = false;
      gallery.open = false;
      showProgress(preset);
    }
    for (const preset of presets) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "demo-card";
      button.dataset.demoId = preset.id;
      button.disabled = true;
      for (const [className, text] of [["demo-kind", preset.kind], ["demo-title", preset.title], ["demo-note", preset.note]]) {
        const span = document.createElement("span");
        span.className = className;
        span.textContent = text;
        button.appendChild(span);
      }
      button.addEventListener("click", () => launch(preset, button));
      buttons.push(button);
      grid.appendChild(button);
    }
    const updateReady = () => {
      for (const button of buttons) button.disabled = !ready();
      if (!ready()) message.textContent = "エンジンの準備を待っています。";
      else if (!selected) message.textContent = "デモを選ぶと、条件を設定して比較を開始します。";
    };
    new document.defaultView.MutationObserver(updateReady).observe(engine, { childList: true, subtree: true, characterData: true });
    updateReady();
    resultLink.addEventListener("click", (event) => {
      event.preventDefault();
      if (selected) showProgress(selected.preset);
    });
    for (const link of document.querySelectorAll("[data-demo-back]")) {
      link.addEventListener("click", (event) => {
        event.preventDefault();
        panel.open = true;
        gallery.open = true;
        gallery.scrollIntoView({ block: "start" });
        const target = selected ? selected.button : buttons[0];
        target.scrollIntoView({ block: "nearest" });
        target.focus({ preventScroll: true });
      });
    }
  }
  if (typeof module !== "undefined") module.exports = { init };
  if (root.document) init(root.document);
})(typeof window !== "undefined" ? window : globalThis);
