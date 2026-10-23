/* Display-only filters: original record indices, summaries and CSV stay intact. */
(function(root) {
  "use strict";
  function matches(record, filter) {
    return (filter.outcome === "all" || record.outcome === filter.outcome) &&
      (filter.stage === "all" || String(record.stage) === filter.stage) &&
      (filter.seed === "all" || String(record.seed) === filter.seed);
  }
  function init(document) {
    const byId=id=>document.getElementById(id);
    const controls={outcome:byId("mobile-filter-outcome"),stage:byId("mobile-filter-stage"),seed:byId("mobile-filter-seed")};
    const roots=["mobile-records","mobile-timeline","mobile-diagnostics"].map(byId);
    const count=byId("mobile-filter-count"), empty=byId("mobile-filter-empty");
    let lastSeeds="";
    const data=e=>({outcome:e.dataset.mobileOutcome,stage:e.dataset.mobileStage,seed:e.dataset.mobileSeed});
    function apply() {
      const cards=[...byId("mobile-diagnostics").querySelectorAll("[data-mobile-index]")];
      const seeds=[...new Set(cards.map(e=>e.dataset.mobileSeed))].sort((a,b)=>Number(a)-Number(b));
      const key=seeds.join(",");
      if(key!==lastSeeds) {
        const previous=controls.seed.value;
        const options=[new document.defaultView.Option("全seed","all"),...seeds.map(seed=>new document.defaultView.Option(seed,seed))];
        controls.seed.replaceChildren(...options);
        controls.seed.value=seeds.includes(previous)?previous:"all";
        lastSeeds=key;
      }
      const filter={outcome:controls.outcome.value,stage:controls.stage.value,seed:controls.seed.value};
      for(const container of roots) for(const element of container.querySelectorAll("[data-mobile-index]")) element.hidden=!matches(data(element),filter);
      const visible=cards.filter(e=>!e.hidden), failures=visible.filter(e=>e.dataset.mobileOutcome==="failure").length;
      count.textContent=cards.length ? `表示 ${visible.length}/${cards.length}試行 · 失敗 ${failures} · 成功 ${visible.length-failures}` : "完了した試行はまだありません。";
      empty.hidden=cards.length===0 || visible.length!==0;
    }
    for(const input of Object.values(controls)) input.addEventListener("change",apply);
    byId("mobile-filter-reset").addEventListener("click",()=>{for(const input of Object.values(controls))input.value="all";apply();});
    const observer=new document.defaultView.MutationObserver(apply);
    for(const element of roots)observer.observe(element,{childList:true});
    apply();
  }
  const api={matches,init};
  if(typeof module!=="undefined")module.exports=api;
  root.RNEMobileResults=api;
})(typeof window==="undefined"?globalThis:window);
