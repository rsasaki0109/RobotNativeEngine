/* Editable navigation geometry and versioned local scene documents. */
(function (root) {
  'use strict';
  const FORMAT = 'rne-navigation-scene';
  const STORAGE_KEY = 'rne.navigation-scene.v1';
  const clone = value => JSON.parse(JSON.stringify(value));
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  function keys(value, allowed) {
    if (!value || typeof value !== 'object' || Array.isArray(value) || Object.keys(value).some(k => !allowed.includes(k))) throw Error('JSONの項目を確認してください');
  }
  function clearance(o, p) {
    return Math.hypot(Math.max(0, Math.abs(p.x_m-o.x_m)-o.width_m/2), Math.max(0, Math.abs(p.z_m-o.z_m)-o.depth_m/2))-0.28;
  }
  function validate(scene) {
    keys(scene, ['goal', 'obstacles']); keys(scene.goal, ['x_m', 'z_m']);
    const g = scene.goal;
    if (![g.x_m,g.z_m].every(finite) || Math.abs(g.x_m)>3.4 || Math.abs(g.z_m)>3.4 || Math.hypot(g.x_m,g.z_m)<0.55) throw Error('目標はX/Z ±3.4m以内、開始点から0.55m以上離してください');
    if (!Array.isArray(scene.obstacles) || scene.obstacles.length>24) throw Error('障害物は24個までです');
    const obstacles = scene.obstacles.map(o => {
      keys(o,['x_m','z_m','width_m','depth_m','appear_step']);
      const item = {...o, appear_step: o.appear_step === undefined ? 0 : o.appear_step};
      if (![item.x_m,item.z_m,item.width_m,item.depth_m].every(finite) || item.width_m<0.2 || item.width_m>3 || item.depth_m<0.2 || item.depth_m>3 || Math.abs(item.x_m)+item.width_m/2>3.65 || Math.abs(item.z_m)+item.depth_m/2>3.65 || !Number.isInteger(item.appear_step) || item.appear_step<0 || item.appear_step>900) throw Error('障害物の寸法・範囲・出現ステップを確認してください');
      if (item.appear_step===0 && (clearance(item,{x_m:0,z_m:0})<0.15 || clearance(item,g)<0.15)) throw Error('開始点・目標と障害物の間を0.43m以上離してください');
      return item;
    });
    return {goal:{x_m:g.x_m,z_m:g.z_m},obstacles};
  }
  function preset(name) {
    const box=(x,z,w,d)=>({x_m:x,z_m:z,width_m:w,depth_m:d,appear_step:0});
    const scenes = {
      open:{goal:{x_m:2.8,z_m:-0.55},obstacles:[]},
      narrow:{goal:{x_m:3,z_m:0},obstacles:[box(1.6,-0.55,2,0.2),box(1.6,0.55,2,0.2)]},
      slalom:{goal:{x_m:3.25,z_m:0},obstacles:[box(1.2,-0.4,0.4,0.9),box(2.1,0.45,0.4,0.9)]},
      blocked:{goal:{x_m:2,z_m:0},obstacles:[box(1.1,0,0.2,2),box(2.9,0,0.2,2),box(2,-0.9,1.8,0.2),box(2,0.9,1.8,0.2)]},
    };
    if (!scenes[name]) throw Error('プリセットを選んでください');
    return validate(scenes[name]);
  }
  function documentFor(scene, selection) {
    keys(selection,['planner','controller','seed']);
    if (![selection.planner,selection.controller].every(v=>Number.isInteger(v) && v>=0 && v<=1) || !Number.isInteger(selection.seed) || selection.seed<0 || selection.seed>4294967295) throw Error('手法・seedを確認してください');
    return {format:FORMAT,version:1,selection:clone(selection),scene:validate(scene)};
  }
  function parseDocument(text) {
    if (new TextEncoder().encode(text).length>65536) throw Error('ファイルは64 KiB以下にしてください');
    let data;
    try { data=JSON.parse(text); } catch { throw Error('JSONを読み取れませんでした'); }
    keys(data,['format','version','selection','scene']);
    if (data.format!==FORMAT || data.version!==1) throw Error('対応していないシーン形式・バージョンです');
    return documentFor(data.scene,data.selection);
  }
  function init({document, signal, releaseAll}) {
    const byId=id=>document.getElementById(id);
    const panel=byId('scene-editor');
    const map=byId('scene-editor-map');
    const message=byId('scene-editor-status');
    let scene=preset('open');
    const undo=[];
    let loadSerial=0;
    let pendingFloor=null;
    const cancelFloor=()=>{if(pendingFloor!==null)tell("床の配置を取り消しました");pendingFloor=null;for(const id of ["experiment-run-button","experiment-compare-button"])byId(id).disabled=false;};
    const tell=(text,error=false)=>{message.textContent=text;message.dataset.error=String(error);};
    const custom=()=>{byId('experiment-task-select').value='3';byId('experiment-task').value='3';};
    function paint() {
      let svg="<rect x='-4' y='-4' width='8' height='8' fill='#101d28'/>";
      for (let i=-3;i<=3;i++) svg+=`<path d='M ${i} -4 V 4 M -4 ${i} H 4' fill='none' stroke='#274354' stroke-width='.015'/>`;
      scene.obstacles.forEach((o,i)=>{svg+=`<rect x='${o.x_m-o.width_m/2}' y='${o.z_m-o.depth_m/2}' width='${o.width_m}' height='${o.depth_m}' fill='${o.appear_step ? 'none' : '#c68446'}' stroke='#e6b07a' stroke-width='.035' ${o.appear_step ? "stroke-dasharray='.12 .08'" : ''}/><text x='${o.x_m}' y='${o.z_m}' font-size='.22' fill='#fff' text-anchor='middle'>${i+1}</text>`;});
      svg+=`<circle cx='0' cy='0' r='.28' fill='#dce9ef' fill-opacity='.35' stroke='#dce9ef' stroke-width='.025'/><text x='.35' y='.15' font-size='.24' fill='#dce9ef'>開始</text><circle cx='${scene.goal.x_m}' cy='${scene.goal.z_m}' r='.14' fill='#ffd773'/><text x='-3.7' y='-3.55' font-size='.25' fill='#b9d7e7'>X → / Z ↓ · 1目盛=1m</text>`;
      map.innerHTML=svg;
      byId('scene-obstacles').textContent=`障害物 ${scene.obstacles.length}/24 · 目標 X ${scene.goal.x_m.toFixed(2)} / Z ${scene.goal.z_m.toFixed(2)} m`;
      byId('scene-undo').disabled=undo.length===0;
    }
    function publish() {byId('scene-json').value=JSON.stringify(scene);signal('scene-revision');paint();}
    function commit(candidate,text,history=true) {
      const next=validate(candidate);
      cancelFloor();
      if (history) {undo.push(clone(scene));if(undo.length>40)undo.shift();}
      scene=next;custom();releaseAll();signal('experiment-manual');publish();tell(text);
    }
    function guard(action) {try {action();} catch(error){tell(error.message,true);}}
    function place(x,z) {
      guard(()=>{
        if (!finite(x)||!finite(z)) throw Error('床の位置を確認してください');
        if (byId('scene-snap').checked) {x=Math.round(x*10)/10;z=Math.round(z*10)/10;}
        byId('scene-x').value=String(x);byId('scene-z').value=String(z);
        const next=clone(scene); const mode=byId('scene-tool').value;
        if(mode==='goal')next.goal={x_m:x,z_m:z};
        else if(mode==='obstacle')next.obstacles.push({x_m:x,z_m:z,width_m:byId('scene-width').valueAsNumber,depth_m:byId('scene-depth').valueAsNumber,appear_step:byId('scene-delayed').checked?60:0});
        else if(mode==='erase') {
          const index=next.obstacles.findLastIndex(o=>Math.abs(x-o.x_m)<=o.width_m/2 && Math.abs(z-o.z_m)<=o.depth_m/2);
          if(index<0)throw Error('削除する障害物をクリックしてください');
          next.obstacles.splice(index,1);
        } else return;
        commit(next,'配置を更新しました。選択条件で実行できます。');
      });
    }
    panel.addEventListener('toggle',()=>{
      byId('scene-editor-visible').value=panel.open?'1':'0';
      if(panel.open){custom();releaseAll();signal('experiment-manual');} else cancelFloor();
    });
    byId('scene-preset-apply').addEventListener('click',()=>guard(()=>commit(preset(byId('scene-preset').value),'プリセットを適用しました')));
    byId('scene-clear').addEventListener('click',()=>guard(()=>commit({...scene,obstacles:[]},'障害物を削除しました')));
    byId('scene-undo').addEventListener('click',()=>guard(()=>{if(undo.length)commit(undo.pop(),'1つ前の配置に戻しました',false);}));
    byId('scene-place').addEventListener('click',()=>place(byId('scene-x').valueAsNumber,byId('scene-z').valueAsNumber));
    map.addEventListener('pointerup',event=>{
      if(event.button!==0)return;
      const p=map.createSVGPoint();p.x=event.clientX;p.y=event.clientY;
      const matrix=map.getScreenCTM();if(!matrix)return;
      const local=p.matrixTransform(matrix.inverse());place(local.x,local.y);
    });
    byId('scene-tool').addEventListener('change',cancelFloor);
    const canvas=byId('canvas'); let down=null;
    canvas.addEventListener('pointerdown',e=>{if(e.button===0)down={x:e.clientX,y:e.clientY,id:e.pointerId};});
    canvas.addEventListener('pointercancel',()=>{down=null;});
    canvas.addEventListener('pointerup',e=>{
      const start=down;down=null;
      if(!start||start.id!==e.pointerId||!panel.open||byId('scene-tool').value==='view'||Math.hypot(start.x-e.clientX,start.y-e.clientY)>5)return;
      const rect=canvas.getBoundingClientRect();
      byId('scene-click-x').value=String((e.clientX-rect.left)/rect.width*2-1);
      byId('scene-click-y').value=String(1-(e.clientY-rect.top)/rect.height*2);
      signal('scene-floor-click');
      pendingFloor=Number(byId('scene-floor-click').value);
      for(const id of ['experiment-run-button','experiment-compare-button'])byId(id).disabled=true;
      tell('床の位置を反映中…');
    });
    const result=byId('scene-floor-result');
    new MutationObserver(()=>{
      const parts=result.textContent.split(',').map(Number);
      if(parts[0]!==pendingFloor)return;
      cancelFloor();
      if(panel.open){if(parts.length===3 && parts.every(finite))place(parts[1],parts[2]);else tell('床が見える位置をクリックしてください',true);}
    }).observe(result,{childList:true});
    const selection=()=>({planner:Number(byId('experiment-planner').value),controller:Number(byId('experiment-controller').value),seed:Number(byId('experiment-seed').value)});
    function currentDocument() { if(pendingFloor!==null)throw Error('床の反映が完了してから保存してください');return documentFor(scene,selection()); }
    function restore(saved) {
      // Parse and validate the entire document before touching any editor state.
      const next=parseDocument(saved);
      commit(next.scene,'設定を読み込みました（まだ実行していません）');
      for(const field of ['planner','controller']) {byId('experiment-'+field).value=String(next.selection[field]);byId('experiment-'+field+'-select').value=String(next.selection[field]);}
      byId('experiment-seed').value=String(next.selection.seed);byId('experiment-seed-editor').value=String(next.selection.seed);
      panel.open=true;
    }
    byId('scene-save-local').addEventListener('click',()=>guard(()=>{
      root.localStorage.setItem(STORAGE_KEY,JSON.stringify(currentDocument()));tell('このブラウザに保存しました');
    }));
    byId('scene-load-local').addEventListener('click',()=>guard(()=>{
      const text=root.localStorage.getItem(STORAGE_KEY);if(!text)throw Error('保存済みの設定がありません');restore(text);
    }));
    byId('scene-export').addEventListener('click',()=>guard(()=>{
      const blob=new Blob([JSON.stringify(currentDocument(),null,2)],{type:'application/json'});
      const url=URL.createObjectURL(blob);const anchor=document.createElement('a');anchor.href=url;anchor.download='rne-navigation-scene-v1.json';anchor.click();
      setTimeout(()=>URL.revokeObjectURL(url),1000);tell('JSONを書き出しました');
    }));
    byId('scene-import').addEventListener('change',async event=>{
      const serial=++loadSerial;const file=event.target.files[0];if(!file)return;
      try {if(file.size>65536)throw Error('ファイルは64 KiB以下にしてください');const text=await file.text();if(serial===loadSerial)restore(text);}
      catch(error){tell(error.message,true);} finally {event.target.value='';}
    });
    publish();
    return {
      currentDocument, restore,
      close(){cancelFloor();panel.open=false;byId('scene-editor-visible').value='0';},
      selectTask(task){if(task==='3')panel.open=true;else this.close();},
    };
  }
  const api={validate,preset,documentFor,parseDocument,init};
  if(typeof module!=='undefined')module.exports=api;
  root.RNESceneEditor=api;
})(typeof window==='undefined'?globalThis:window);
