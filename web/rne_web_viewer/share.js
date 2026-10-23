/* Versioned experiment-condition links. Loading previews conditions; starting is explicit. */
(function(root) {
  "use strict";
  const editor = typeof module !== "undefined" ? require("./scene_editor.js") : root.RNESceneEditor;
  const modes = {
    "nav-run": "experiment-run-button", "nav-compare": "experiment-compare-button",
    "arm-run": "arm-run-button", "arm-compare": "arm-compare-button", "arm-benchmark": "arm-benchmark-button",
    "mobile-run": "mobile-run-button", "mobile-compare": "mobile-compare-button", "mobile-arm": "mobile-arm-compare-button",
    "mobile-recovery": "mobile-recovery-compare-button", "mobile-benchmark": "mobile-benchmark-button",
  };
  const MAX_BYTES = 16384;
  const error = () => { throw Error("共有リンクの形式・設定を確認してください"); };
  const keys = (value, names) => {
    if (!value || typeof value !== "object" || Array.isArray(value) || Object.keys(value).length !== names.length || names.some(n=>!Object.hasOwn(value,n))) error();
  };
  const integer = (v,max) => Number.isInteger(v) && v>=0 && v<=max;
  function validate(value) {
    keys(value,["version","mode","seed","nav","arm","mobile","scene"]);
    keys(value.nav,["task","planner","controller","recovery"]);
    keys(value.arm,["task","planner","controller"]);
    keys(value.mobile,["scenario","recovery"]);
    if (value.version !== 1 || typeof value.mode !== "string" || !Object.hasOwn(modes,value.mode) || !integer(value.seed,4294967295) ||
        !integer(value.nav.task,7) || !integer(value.nav.planner,1) || !integer(value.nav.controller,1) || typeof value.nav.recovery !== "boolean" ||
        !integer(value.arm.task,3) || !integer(value.arm.planner,2) || !integer(value.arm.controller,1) ||
        !integer(value.mobile.scenario,3) || typeof value.mobile.recovery !== "boolean") error();
    if ((value.nav.task===3) !== (value.scene!==null)) error();
    const scene = value.scene===null ? null : editor.validate(value.scene);
    // Reconstruct the allowlisted document; no unrecognized data enters a UI or URL.
    return {version:1,mode:value.mode,seed:value.seed,nav:{task:value.nav.task,planner:value.nav.planner,controller:value.nav.controller,recovery:value.nav.recovery},
      arm:{task:value.arm.task,planner:value.arm.planner,controller:value.arm.controller},mobile:{scenario:value.mobile.scenario,recovery:value.mobile.recovery},scene};
  }
  function encode(value) {
    const bytes=new TextEncoder().encode(JSON.stringify(validate(value)));
    if(bytes.length>MAX_BYTES) error();
    return btoa(String.fromCharCode(...bytes)).replace(/\+/g,"-").replace(/\//g,"_").replace(/=+$/g,"");
  }
  function decode(hash) {
    if(!hash.startsWith("#rne=")) return null;
    const token=hash.slice(5);
    if(!token || token.length>Math.ceil(MAX_BYTES*4/3) || !/^[A-Za-z0-9_-]+$/.test(token) || token.length%4===1) error();
    let value;
    try {
      const bytes=Uint8Array.from(atob(token.replace(/-/g,"+").replace(/_/g,"/")+"=".repeat((4-token.length%4)%4)),c=>c.charCodeAt(0));
      if(bytes.length>MAX_BYTES) error();
      value=JSON.parse(new TextDecoder("utf-8",{fatal:true}).decode(bytes));
    } catch(_) { error(); }
    return validate(value);
  }
  function link(base,value,language) {
    const url=new URL(base);
    url.searchParams.set("lang",language==="en"?"en":"ja");
    url.hash="rne="+encode(value);
    return url.href;
  }
  function init({document,sceneEditor}) {
    const byId=id=>document.getElementById(id);
    const message=byId("share-status"), output=byId("share-url"), start=byId("share-start"), copy=byId("share-copy");
    const engine=byId("engine-status");
    const Event=document.defaultView.Event;
    let shared=null;
    const tell=(text,failed=false)=>{message.textContent=text;message.dataset.error=String(failed);};
    const ready=()=>{start.disabled=!shared || engine.dataset.ready!=="true";};
    const set=(id,value)=>{const input=byId(id);input.value=String(value);input.dispatchEvent(new Event("change",{bubbles:true}));};
    const toggle=(id,value)=>{const input=byId(id);input.checked=value;input.dispatchEvent(new Event("change",{bubbles:true}));};
    function capture() {
      return validate({version:1,mode:byId("share-mode").value,seed:Number(byId("experiment-seed").value),
        nav:{task:Number(byId("experiment-task-select").value),planner:Number(byId("experiment-planner-select").value),controller:Number(byId("experiment-controller-select").value),recovery:byId("experiment-recovery-toggle").checked},
        arm:{task:Number(byId("arm-task").value),planner:Number(byId("arm-planner").value),controller:Number(byId("arm-controller").value)},
        mobile:{scenario:Number(byId("mobile-scenario").value),recovery:byId("mobile-recovery-toggle").checked},
        scene:byId("experiment-task-select").value==="3" ? sceneEditor.currentDocument().scene : null});
    }
    function restore(value) {
      const next=validate(value); // Validate everything before any control or running state changes.
      if(next.scene) sceneEditor.restore(JSON.stringify(editor.documentFor(next.scene,{planner:next.nav.planner,controller:next.nav.controller,seed:next.seed})));
      byId("experiment-manual-button").click();
      set("experiment-task-select",next.nav.task);
      set("experiment-planner-select",next.nav.planner);
      set("experiment-controller-select",next.nav.controller);
      set("experiment-seed-editor",next.seed);
      set("arm-task",next.arm.task);set("arm-planner",next.arm.planner);set("arm-controller",next.arm.controller);
      set("mobile-scenario",next.mobile.scenario);
      toggle("experiment-recovery-toggle",next.nav.recovery);toggle("mobile-recovery-toggle",next.mobile.recovery);
      set("share-mode",next.mode);
      byId("experiment-panel").open=true;
      byId("manipulation-panel").open=next.mode.startsWith("arm-");
      byId("mobile-panel").open=next.mode.startsWith("mobile-");
      shared=next;
      ready();
    }
    function showLink(value) {
      output.value=link(root.location.href,value,document.documentElement.lang);
      copy.disabled=false;
    }
    byId("share-create").addEventListener("click",()=>{
      try {const next=capture();showLink(next);shared=next;ready();tell("現在の設定リンクを作成しました。開始ボタンで同じ条件を実行できます。");}
      catch(e) {tell(e.message,true);}
    });
    copy.addEventListener("click",async()=>{
      try {await root.navigator.clipboard.writeText(output.value);tell("共有リンクをコピーしました");}
      catch(_) {output.focus();output.select();tell("リンク欄を選択しました。手動でコピーしてください。");}
    });
    start.addEventListener("click",()=>{
      if(!shared || engine.dataset.ready!=="true") return;
      try {
        const next=shared;
        restore(next);
        byId(modes[next.mode]).click();
        byId(next.mode.startsWith("mobile-")?"mobile-status":next.mode.startsWith("arm-")?"arm-status":"experiment-status").scrollIntoView({block:"nearest"});
        tell("共有された条件で実行を開始しました");
      } catch(e) {tell(e.message,true);}
    });
    new root.MutationObserver(ready).observe(engine,{attributes:true,attributeFilter:["data-ready"]});
    byId("language-select").addEventListener("change",()=>{if(shared)showLink(shared);});
    function loadFromURL() {
      try {
        const incoming=decode(root.location.hash);
        if(incoming) {
          restore(incoming);
          showLink(incoming);
          byId("share-panel").open=true;
          tell("共有された条件を読み込みました。開始ボタンで実行します。");
        }
      } catch(e) {
        shared=null;
        output.value="";
        copy.disabled=true;
        ready();
        byId("share-panel").open=true;
        tell(e.message,true);
      }
    }
    root.addEventListener("hashchange",loadFromURL);
    loadFromURL();
    ready();
  }
  const api={validate,encode,decode,link,init,modes};
  if(typeof module!=="undefined")module.exports=api;
  root.RNEExperimentShare=api;
})(typeof window==="undefined"?globalThis:window);
