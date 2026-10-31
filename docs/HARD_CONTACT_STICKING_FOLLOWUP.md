# 接触反復の追加・力積配分の引き継ぎを評価する

基準は `b3135af7`（#493）。200反復・2 ms刻み・正則化0・既存の残差許容値を固定し、
横運動と傾きのある4段・8段、各100ステップで3案を実測した。
**3案とも採用しない。実行コードは#493と同一に戻し、試作patchと結果を保存する。**

## 試作の定義

1. **残差を確認して停止**: 従来の反復変化量による停止候補に対し、元の接触則も満たす場合だけ停止する。
   小さな変化量だけで止まったステップは200反復まで継続する。sticking blockはOFF。
2. **seedからCG開始**: #493のsticking blockをONにし、activeな行の初期解をwarm-startの力積にする。
   初期残差は `-free - G * seed`。ゼロ開始で失われる零空間成分の配分を残す狙い。
   active setが変わると、除去された行の力積は0になる。候補の円錐・元の接触則の検査は変更しない。
3. **GS途中の再試行**: 2に加え、GSの8・32・128反復後に現在の力積で候補を再計算する。
   検証成功時だけ返し、棄却時は元のGS反復を続ける。成功時のGS反復数は保持する。
   試作のsticking active-set回数は途中の全棄却試行を累積していないため、演算量比較には使わない。

2/3のpatchでは測定exampleの省略時のモードもONにしている。これは試作専用であり、製品の既定値を変える提案ではない。

## 品質結果

表は外部残差不合格ステップ数 / 最大荷重法線速度残差(m/s)。基準値は#493の比較Aから。
案1はOFF基準、案2/3はON基準と比較する。

| シーン | #493 OFF | 案1: 残差停止 | #493 ON | 案2: seed開始 | 案3: GS途中 |
|---|---:|---:|---:|---:|---:|
| tower-slide 4 | 78 / 0.000766869 | 77 / 0.000766879 | 29 / 0.000783834 | 35 / 0.000783821 | 31 / 0.000783821 |
| tower-slide 8 | 78 / 0.00153693 | 77 / 0.00153693 | 56 / 0.00175101 | 56 / 0.001751 | 56 / 0.001751 |
| tower-tilt 4 | 84 / 0.000550661 | 80 / 0.000551673 | 80 / 0.000550424 | 83 / 0.000550482 | 82 / 0.000550094 |
| tower-tilt 8 | 84 / 0.000948026 | 80 / 0.000947951 | 80 / 0.00100862 | 83 / 0.00288033 | 82 / 0.002813 |

全12実行で10 µmのめり込み基準と支持鎖は合格したが、総合品質は不合格。
案1の横運動は78→77で最大残差もほぼ変わらない。200反復まで続けるだけでは解決しない。
案2/3は傾き8段の最大残差がON基準の約0.00101 m/sから約0.00288/0.00281 m/sへ悪化した。
「検証済みのその場の候補」が、後続ステップも含むシーン全体の誤差を必ず減らすわけではない。

案1で「停止したが内部証明なし」は全4ケースで0。内部証明に成功しながら外部残差に失敗した
ステップは4/4/3/3あり、すべてエネルギーの絶対誤差基準の違いによるものだった。
外部の速度・力積基準との不一致は0。停止フラグだけを品質合格としない。

## 費用と検証範囲

CPU固定、ウォームアップ後に監査OFFの計時を5回実施。値は実時間秒 / シミュレーション秒の中央値。
これらは試作ごとに順番に測った値で、同時期の交互比較ではない。速度向上・低下の因果的な主張には使わない。

| シーン | 案1 | 案2 | 案3 |
|---|---:|---:|---:|
| tower-slide 4 | 0.48994 | 0.23103 | 0.23865 |
| tower-slide 8 | 1.49965 | 1.45821 | 2.43846 |
| tower-tilt 4 | 0.54957 | 0.25742 | 0.35950 |
| tower-tilt 8 | 1.34696 | 1.66484 | 2.75109 |

- 各案4ケースで全監査ステップを再実行し一致を確認。計時5回すべてで最終状態ハッシュ一致。
- バイナリ・検査スクリプトのSHA256を照合。全3patchが基準commitに適用できることを確認。
- 製品コードは基準に復元。今回の試作を製品機能としてテスト済みとは扱わない。
- 全workspace CIは実施していない。前回からのディスク容量制約は継続中。

## 再現

別のcheckoutで `b3135af7` を取得し、各patchを単独で適用する。3案のpatchを重ねない。

```bash
git apply docs/evidence/contact-certified-stop-trial.patch
cargo build --release -p rne_physics_native --example contact_crowd
python scripts/contact_benchmark/contact_disturbed_tower.py target/release/examples/contact_crowd /tmp/certified-stop.json
python scripts/contact_benchmark/contact_disturbed_tower.py target/release/examples/contact_crowd /tmp/certified-stop.json --verify
```

案2/3はpatch名を `contact-seeded-sticking-trial.patch` / `contact-sticking-polish-trial.patch` に置き換える。
patchはこのPRの `docs/evidence` から用意する。基準commitにはまだ含まれない。

[案1](evidence/contact-certified-stop-trial-summary.json)、
[案2](evidence/contact-seeded-sticking-trial-summary.json)、
[案3](evidence/contact-sticking-polish-trial-summary.json)、
[provenance](evidence/contact-sticking-followup-provenance.json)。
生データはワークスペース内の一時保管。要約には全接触フレームがなく、恒久的な生データ公開URLもない。

## 次の実装対象

停止条件や同じ静止摩擦候補の再試行に追加投資せず、開離・静止摩擦・滑りを同時に扱う候補へ進む。
具体的には `vn >= 0, jn >= 0, jn*vn = 0` と
`jt = projection_disk(jt - alpha*vt, mu*jn)` を残差として、接触状態の変化を許す解法を試す。
これは次の実装案であり、今回完成した機能ではない。

先に失敗したステップの行列・自由速度・摩擦・warm startを固定fixtureとして取り出す。
元の物理方程式での残差、摩擦円錐、エネルギー検査を独立に通し、特異な行列への対応を検証する。
その後、この4シーンを同じ反復予算で再実行して最大誤差と計時を比較する。
許容値緩和やコンプライアンス追加で合格扱いにせず、全体軌道で悪化する候補は既定にしない。

後続: [固定接触問題とNewton試作](HARD_CONTACT_FROZEN_NEWTON.md)。
