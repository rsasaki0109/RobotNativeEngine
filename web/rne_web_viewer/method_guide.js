/* Method explanations select next-run form values; they never start an experiment. */
(function(root) {
  "use strict";
  const groups = [
    {field:"experiment-planner-select", title:"移動の計画", methods:[
      {name:"直線", description:"開始点と目標を結び、経路上の衝突を検証します。迂回路は探索しません。", watch:"障害物が直線を塞ぐと、移動前に計画失敗になる点を見てください。"},
      {name:"A*", description:"障害物を反映したコストマップ上で、目標までの経路を探索します。", watch:"障害物を回り込む青い計画と、実際に通った軌跡を見比べてください。"}
    ]},
    {field:"experiment-controller-select", title:"移動の制御", methods:[
      {name:"Pure Pursuit", description:"計画経路上の少し先を目指して、移動速度と旋回速度を決めます。", watch:"曲がり角での軌跡のふくらみと、狭い通路での追従誤差に注目してください。"},
      {name:"DWA", description:"短時間の自分の動きを複数予測し、目標・経路・障害物との余裕から速度を選びます。", watch:"狭い場所や横切る箱の近くで、減速・停止・旋回の違いを見てください。"}
    ]},
    {field:"arm-planner", title:"腕の計画", methods:[
      {name:"関節直結", description:"IKで求めた目標姿勢へ、関節空間で補間します。障害物を避ける経由点は探索しません。", watch:"手先は直線に動くとは限りません。障害物に触れて止まる段階を見てください。"},
      {name:"上方経由", description:"手先の上方に経由目標を追加し、それぞれをIKと関節補間でつなぎます。", watch:"持ち上げる動きと、経由点の前後で実軌跡が障害物を避けられるかに注目してください。"},
      {name:"RRT-Connect", description:"関節空間で衝突を調べながら探索木を伸ばし、開始姿勢と目標姿勢をつなぎます。探索にはseedを使います。", watch:"青い計画の迂回、探索回数、物体を保持した後の腕の通り道を見てください。"}
    ]},
    {field:"arm-controller", title:"腕の制御", methods:[
      {name:"速度制限P", description:"関節の目標との差に比例して速度を決め、指令速度に上限を設けます。", watch:"経由点への追従と、目標へ近づくときの減速を見てください。"},
      {name:"加速度制限P", description:"比例制御の指令に対し、速度の変化も抑えて関節を動かします。", watch:"速度制限Pと同じ計画で、動き出し・目標切り替え・到達時間の違いを比べてください。"}
    ]}
  ];
  function init(document) {
    const container = document.getElementById("method-guide-content");
    const guide = document.getElementById("method-guide");
    const Event = document.defaultView.Event;
    const headings = new Map();
    for (const group of groups) {
      const section = document.createElement("section");
      const heading = document.createElement("h3");
      heading.id = "method-guide-" + group.field;
      heading.tabIndex = -1;
      heading.textContent = group.title;
      section.setAttribute("role", "group");
      section.setAttribute("aria-labelledby", heading.id);
      section.appendChild(heading);
      headings.set(group.field, heading);
      const select = document.getElementById(group.field);
      const choices = group.methods.map((method, index) => {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "method-card";
        button.dataset.methodField = group.field;
        button.dataset.methodValue = String(index);
        button.setAttribute("aria-controls", group.field);
        const badge = document.createElement("span");
        badge.className = "method-selected";
        badge.textContent = "次の実行に選択中";
        for (const [tag, text] of [["strong", method.name], ["span", method.description], ["span", method.watch]]) {
          const element = document.createElement(tag);
          element.textContent = text;
          button.appendChild(element);
        }
        button.appendChild(badge);
        button.addEventListener("click", () => {
          if (select.value === String(index)) return;
          select.value = String(index);
          select.dispatchEvent(new Event("change", {bubbles:true}));
        });
        section.appendChild(button);
        return {button, badge};
      });
      function update() {
        choices.forEach(({button, badge}, index) => {
          const selected = select.value === String(index);
          button.setAttribute("aria-pressed", String(selected));
          badge.hidden = !selected;
        });
      }
      select.addEventListener("change", update);
      update();
      container.appendChild(section);
    }
    for (const button of document.querySelectorAll("[data-method-guide-open]")) {
      button.addEventListener("click", () => {
        guide.open = true;
        const heading = headings.get(button.dataset.methodGuideOpen);
        heading.scrollIntoView({block:"nearest"});
        heading.focus({preventScroll:true});
      });
    }
  }
  const api = {groups, init};
  if (typeof module !== "undefined") module.exports = api;
  root.RNEMethodGuide = api;
})(typeof window === "undefined" ? globalThis : window);
