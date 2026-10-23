/* Presentation clock for saved 30 Hz snapshots; never advances a simulation. */
(function(root) {
  "use strict";
  const rate=value=>[0.5,1,2,4].includes(Number(value))?Number(value):1;
  function indexAt(times,tick) {
    let low=0,high=times.length;
    while(low<high) {const mid=(low+high)>>>1;if(times[mid]<=tick)low=mid+1;else high=mid;}
    return Math.max(0,low-1);
  }
  class ReplayClock {
    constructor(tick=0) {this.reset(tick);}
    reset(tick) {this.tick=tick;this.last=null;}
    advance(timestamp,speed,lastTick) {
      if(this.last!==null)this.tick=Math.min(lastTick,this.tick+Math.max(0,timestamp-this.last)*30/1000*rate(speed));
      this.last=timestamp;
      return this.tick;
    }
  }
  function init({document,prefix,getTimes,seek,onPlay}) {
    const byId=suffix=>document.getElementById(prefix+"-replay-"+suffix);
    const slider=byId("frame"),enabled=byId("enabled"),play=byId("play"),previous=byId("previous"),next=byId("next"),speed=byId("speed");
    const window=document.defaultView;
    const clock=new ReplayClock();
    let playing=false,frame=null;
    const available=()=>!slider.disabled && getTimes().length>0;
    function refresh() {
      const active=available(),index=Number(slider.value);
      play.disabled=!active;speed.disabled=!active;
      previous.disabled=!active || index<=0;next.disabled=!active || index>=getTimes().length-1;
      const state=String(playing);
      if(play.getAttribute("aria-pressed")!==state) {
        play.setAttribute("aria-pressed",state);
        play.textContent=playing?"リプレイを一時停止":"リプレイを再生";
      }
    }
    function pause() {playing=false;if(frame!==null)window.cancelAnimationFrame(frame);frame=null;refresh();}
    function animate(timestamp) {
      if(!playing || !available() || enabled.value!=="1" || document.hidden) {pause();return;}
      const times=getTimes(),last=times[times.length-1];
      const tick=clock.advance(timestamp,speed.value,last),index=indexAt(times,tick);
      if(index!==Number(slider.value))seek(index);
      if(tick>=last) {pause();return;}
      frame=window.requestAnimationFrame(animate);
    }
    function start() {
      if(!available() || document.hidden)return;
      onPlay();
      const times=getTimes();
      let index=Number(slider.value);
      if(index>=times.length-1)index=0;
      clock.reset(times[index]);
      playing=true;
      seek(index);
      refresh();
      if(times[times.length-1]<=times[index]) {pause();return;}
      frame=window.requestAnimationFrame(animate);
    }
    play.addEventListener("click",()=>{if(playing)pause();else start();});
    for(const [button,offset] of [[previous,-1],[next,1]])button.addEventListener("click",()=>{
      pause();if(!available())return;onPlay();seek(Math.max(0,Math.min(getTimes().length-1,Number(slider.value)+offset)));refresh();
    });
    speed.addEventListener("change",()=>{speed.value=String(rate(speed.value));clock.reset(getTimes()[Number(slider.value)]||0);});
    document.addEventListener("visibilitychange",()=>{if(document.hidden)pause();});
    refresh();
    return {pause,sync(){if(!available())pause();else refresh();}};
  }
  const api={indexAt,ReplayClock,init};
  if(typeof module!=="undefined")module.exports=api;
  root.RNEReplayPlayer=api;
})(typeof window==="undefined"?globalThis:window);
