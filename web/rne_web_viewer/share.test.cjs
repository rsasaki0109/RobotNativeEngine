const assert=require('node:assert/strict');
const {validate,encode,decode,link,modes}=require('./share.js');
const editor=require('./scene_editor.js');
const base={version:1,mode:'nav-compare',seed:42,nav:{task:1,planner:1,controller:1,recovery:true},arm:{task:2,planner:2,controller:0},mobile:{scenario:3,recovery:false},scene:null};
const clone=v=>JSON.parse(JSON.stringify(v));
for(const mode of Object.keys(modes))for(const seed of [0,42,4294967295]) {
 const settings={...clone(base),mode,seed};
 for(const language of ['ja','en']) {
  const url=new URL(link('https://example.github.io/RobotNativeEngine/?other=1#demo-gallery',settings,language));
  assert.equal(url.pathname,'/RobotNativeEngine/');assert.equal(url.searchParams.get('other'),'1');assert.equal(url.searchParams.get('lang'),language);
  assert.deepEqual(decode(url.hash),settings);
 }
}
for(const name of ['open','narrow','slalom','blocked']) {
 const scene=editor.preset(name);if(scene.obstacles.length)scene.obstacles[0].appear_step=60;
 const settings={...clone(base),nav:{...base.nav,task:3},scene};
 assert.deepEqual(decode('#rne='+encode(settings)),settings);
}
assert.equal(decode('#demo-gallery'),null);
assert.equal(decode(''),null);
const reject=mutate=>{const bad=clone(base);mutate(bad);assert.throws(()=>validate(bad));};
for(const key of Object.keys(base))reject(v=>{delete v[key];});
for(const bad of [-1,1.5,'42',4294967296,Infinity,NaN])reject(v=>v.seed=bad);
for(const mode of ['constructor','__proto__','toString','unknown',['nav-run'],null,7])reject(v=>v.mode=mode);
reject(v=>v.version=2);reject(v=>v.extra=true);reject(v=>v.nav.task=8);reject(v=>v.nav.planner=2);reject(v=>v.arm.planner=3);reject(v=>v.arm.task=4);reject(v=>v.mobile.scenario=4);
reject(v=>v.nav.recovery=1);reject(v=>v.mobile.recovery='false');reject(v=>v.arm.extra=0);reject(v=>v.nav.task=3);
reject(v=>v.scene=editor.preset('open'));
reject(v=>{v.nav.task=3;v.scene={goal:{x_m:99,z_m:0},obstacles:[]};});
reject(v=>{v.nav.task=3;v.scene=editor.preset('open');v.scene.obstacles=Array(25).fill({});});
for(const hash of ['#rne=','#rne=*','#rne=A','#rne='+ 'a'.repeat(22000),'#rne='+btoa('{"version":1}'),'#rne='+btoa('not json'),'#rne=_w'])assert.throws(()=>decode(hash));
const value=clone(base);const encoded=encode(value);value.nav.task=7;assert.equal(decode('#rne='+encoded).nav.task,1);
console.log('PASS: all 10 modes × 3 seeds × 2 locales; custom layouts, scheduled obstacles, bounds, strict fields/types and malformed links');
