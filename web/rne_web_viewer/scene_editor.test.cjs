const assert = require('node:assert/strict');
const {validate,preset,documentFor,parseDocument} = require('./scene_editor.js');
const selection={planner:1,controller:1,seed:42};
for (const name of ['open','narrow','slalom','blocked']) {
  const scene=preset(name);
  const saved=documentFor(scene,selection);
  assert.deepEqual(parseDocument(JSON.stringify(saved)),saved);
  assert.deepEqual(saved.scene,scene);
}
const base=preset('open');
for(const scene of [
  {...base,goal:{x_m:NaN,z_m:1}}, {...base,goal:{x_m:Infinity,z_m:1}}, {...base,goal:{x_m:0,z_m:0}},
  {...base,goal:{x_m:3.5,z_m:0}}, {...base,extra:true},
  {...base,obstacles:Array(25).fill({x_m:1.5,z_m:1.5,width_m:0.5,depth_m:0.5})},
  {...base,obstacles:[{x_m:0,z_m:0,width_m:1,depth_m:1}]},
  {...base,obstacles:[{x_m:2.8,z_m:-0.55,width_m:0.5,depth_m:0.5}]},
  {...base,obstacles:[{x_m:3.6,z_m:2,width_m:1,depth_m:1}]},
  {...base,obstacles:[{x_m:1,z_m:1,width_m:-1,depth_m:0.5}]},
])assert.throws(()=>validate(scene));
const delayed=validate({...base,obstacles:[{x_m:1.8,z_m:0,width_m:0.4,depth_m:1,appear_step:60}]});
assert.equal(delayed.obstacles[0].appear_step,60);
for (const bad of ['{',' '.repeat(65537),JSON.stringify({...documentFor(base,selection),version:2}),JSON.stringify({...documentFor(base,selection),format:'other'}),JSON.stringify(documentFor(base,selection)).replace('"seed":42','"seed":-1')])assert.throws(()=>parseDocument(bad));
assert.deepEqual(base,preset('open'),'invalid proposals must not mutate an existing scene');
console.log('Scene schema, presets, versioned roundtrip and invalid-input checks passed');
