const assert = require('node:assert/strict');
const fs = require('node:fs');
const {translate,chooseLanguage} = require('./i18n.js');
assert.equal(chooseLanguage('en','ja'),'en');
assert.equal(chooseLanguage('ja','en'),'ja');
assert.equal(chooseLanguage('xx','en'),'en');
assert.equal(chooseLanguage(null,null),'ja');
assert.equal(chooseLanguage('constructor','__proto__'),'ja');
assert.equal(translate('  腕6組を比較\n','en'),'  Compare 6 arm pairs\n');
assert.equal(translate('比較表を広く表示','en'),'Expand comparison table');
assert.equal(translate('通常表示に戻す','en'),'Return to normal view');
assert.equal(translate('再開','en'),'Resume');
assert.equal(translate('Replay inspector','ja'),'リプレイ検査');
assert.equal(translate('left knee','ja'),'左膝');
assert.equal(translate('RRT-Connect','ja'),'RRT-Connect');
assert.equal(translate('0.03 rad','en'),'0.03 rad');
assert.equal(translate('成功.json: loaded 8 frames','en'),'成功.json: loaded 8 frames');
assert.equal(translate('成功.json: loaded 8 frames','ja'),'成功.json: 8フレーム読込完了');
const html=fs.readFileSync(__dirname+'/index.html','utf8').replace(/<script[\s\S]*?<\/script>/g,'').replace(/<style[\s\S]*?<\/style>/g,'');
const strings=[...html.matchAll(/>([^<>]+)</g)].map(m=>m[1].trim());
const {groups}=require('./method_guide.js');
assert.equal(groups.reduce((n,g)=>n+g.methods.length,0),9);
for(const group of groups) {
  const select=html.match(new RegExp('<select id="'+group.field+'"[^>]*>([\\s\\S]*?)</select>'));
  assert.ok(select,group.field);
  assert.deepEqual([...select[1].matchAll(/value="(\d+)"/g)].map(m=>Number(m[1])),group.methods.map((_,i)=>i));
  strings.push(group.title,...group.methods.flatMap(m=>[m.name,m.description,m.watch]));
}
strings.push(...[...html.matchAll(/(?:aria-label|title|placeholder)="([^"]+)"/g)].map(m=>m[1]));
for(const text of strings) {
  if(['Language / 言語','日本語'].includes(text))continue;
  assert.ok(!/[\u3040-\u30ff\u3400-\u9fff]/.test(translate(text,'en')), 'Missing static English: '+text);
}
for(const text of [
 '標準の運搬 · 全4段階完了 · 成功 · 27.83 s · 未保持 · 待機 0.00 s · 再試行 0回 · リプレイ',
 '評価完了 · 完了 30/30試行。段階の成功率は、その段階まで到達した完了試行のみで計算します。',
 '試行1 把持 9.77秒から10.00秒',
 'A* × DWA · 腕 RRT-Connect × 速度制限P · seed 42 · 再試行あり',
 '同条件の比較 · 4/4組完了 · 成功 2/4組（各1試行）',
 'G1読み込みエラー: asset unavailable',
 '手先リーチ · 関節直結 × 速度制限P · 腕が障害物に接触 · 1.20 s · 完了段階 0 · 探索 1回 · 未保持',
 '障害物 1/24 · 目標 X 2.80 / Z -0.55 m',
 '最後に選んだデモ：腕の手法を変えて運ぶ · 運搬 · 腕6手法 · seed 42',
 '標準の運搬 · seed 42 · 再試行なし。未実行は失敗に含めません。比較完了後にセルを選ぶと終了地点を再生します。表は横にスクロールできます。',
 '移動4組 × 腕6組 · 運搬24試行',
 '未実行',
 '評価中',
 '成功組のランキング・失敗理由',
 '途中集計 · 成功 0 · 失敗 12 · 未完了 12',
 '確定集計 · 成功 6 · 失敗 18 · 未完了 0',
 'この配置・seed・再試行設定での結果です。成功した完了試行を総時間順に表示します。同時間は同順位。未完了は失敗に含めません。',
 '成功組 · 総時間の短い順',
 '成功した完了試行はありません。',
 '失敗した完了試行はありません。',
 '成功組のランキング（横スクロール）',
 '失敗理由の集計（横スクロール）',
 '順位', '総時間 s', '失敗した段階・理由', '実際の終了理由', '件数', '該当試行のリプレイ',
 '試行 21 · 終了地点へ',
 'クリア！ 4/4 · 15.00 秒（プレビュー時間）'
]) assert.ok(!/[\u3040-\u30ff\u3400-\u9fff]/.test(translate(text,'en')), 'Missing dynamic English: '+text);
console.log('PASS: language precedence, whitespace, static UI/attributes and dynamic result/error translations');
