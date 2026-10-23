const assert=require('node:assert/strict');
const {matches}=require('./result_filters.js');
const records=[
 {index:0,outcome:'failure',stage:0,seed:42},
 {index:2,outcome:'success',stage:4,seed:0},
 {index:4,outcome:'failure',stage:1,seed:42},
 {index:9,outcome:'failure',stage:2,seed:42},
 {index:11,outcome:'failure',stage:3,seed:4294967295},
 {index:15,outcome:'success',stage:4,seed:42},
];
const before=JSON.stringify(records);
const select=(outcome='all',stage='all',seed='all')=>records.filter(r=>matches(r,{outcome,stage,seed})).map(r=>r.index);
assert.deepEqual(select(),[0,2,4,9,11,15]);
assert.deepEqual(select('failure'),[0,4,9,11]);
assert.deepEqual(select('success'),[2,15]);
assert.deepEqual(select('failure','1','42'),[4]);
assert.deepEqual(select('failure','2','42'),[9]);
assert.deepEqual(select('failure','3','4294967295'),[11]);
assert.deepEqual(select('success','4','0'),[2]);
assert.deepEqual(select('success','0'),[]);
assert.deepEqual(select('all','all','42'),[0,4,9,15]);
assert.deepEqual(select('all','all','99'),[]);
assert.equal(JSON.stringify(records),before);
console.log('PASS: combined outcome/stage/seed filters, sparse original indices, empty results and immutable records');
