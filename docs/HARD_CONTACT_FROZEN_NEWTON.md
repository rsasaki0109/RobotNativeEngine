# 固定した接触問題で開離・静止摩擦・滑りを同時に解く

基準は `67a35679`（#494）。シーンの軌道が変わる影響を切り離すため、失敗した接触問題の
Delassus行列G、自由速度b、摩擦係数、投影済みwarm start、GS終了時の力積を保存した。
**オフラインのNewton試作は8問題中5問題で独立検査に合格した。実行時の解法には未導入。**
シーン全体の品質や計算コストの改善、MuJoCoに対する優位性を示す結果ではない。

## 固定問題の選び方

sticking blockはOFF、正則化0、刻み2 ms、GS上限200、その他の許容値も従来のまま。
横運動/傾き × 4/8段の4シーンで、内部接触則検査に失敗したGS終了時だけを採取する。
各シーンから最初の失敗と、荷重のある法線速度の最大残差が最も大きい失敗を選ぶ。
同値の場合は先に現れたものを選ぶ。全件が16または32接触である。
ラベルの `failed-solve-N` は採取された失敗solveの通し番号で、シミュレーションのstep番号ではない。

[固定問題](evidence/contact-frozen-systems.json)には選択した8件の全行列とベクトルを含む。
このファイルだけでオフラインの比較を再実行できる。採取前後の4シーンの監査記録は
計時を除いて基準バイナリと一致した。採取時のstderr出力費用を性能結果として使わない。
全失敗solveの生キャプチャは一時ワークスペース保管で、恒久公開URLはない。

## 解法

各接触で `v = Gp + b`、`alpha = 1 / max(diag(Gii))` とする。
法線はFischer–Burmeister残差
`sqrt(pn² + (alpha*vn)²) - pn - alpha*vn`、
接線は `pt - projection_disk(pt - alpha*vt, mu*max(pn,0))` を使う。
これにより、開離・静止・滑りを候補の中で切り替える。摩擦円を多角形に近似しない。

解析的な一般化Jacobianを組み、NumPyのSVDでNewton方向を計算する。
特異値の相対しきい値は1e-12。方向のノルムが `1 + ||p||` を超える場合は
SVD係数を減衰させる（初期値 `1e-12*smax²`、10倍ずつ最大30回）。
これはNewtonステップの制御で、物理行列Gや自由速度bにコンプライアンスを足す操作ではない。
最大50回、最大12回の半減バックトラックで、二乗残差を十分減らす候補だけを進める。
不成功は `line_search_failed` または `iteration_limit` として保存し、成功扱いにしない。

成功判定はNewton残差から独立した、元のG・bによる検査で行う。
非負法線力積・摩擦円錐は1e-10 Ns、閉じる速度・荷重法線速度は1e-8 m/s、
内側の滑り速度・滑り方向の誤差はsqrt(2)*1e-8 m/s、散逸誤差は1e-10 Jを上限とする。
既存の外部検査より内側滑り速度のしきい値が厳しいが、緩めて合格にしてはいない。

## 結果

「初期力積」はGS前のwarm start、「GS後」は同じ問題のGS出力からNewtonを開始したもの。
反復数は採用したNewtonステップ数で、SVD・バックトラック費用を含む演算量ではない。

| 固定問題 | 初期力積から | GS後から |
|---|---:|---:|
| 横運動4段・最初の失敗 | 合格、5回 | 合格、1回 |
| 横運動4段・最大残差 | 不合格、8回 | 不合格、7回 |
| 横運動8段・最初の失敗 | 合格、5回 | 合格、1回 |
| 横運動8段・最大残差 | 合格、14回 | 不合格、5回 |
| 傾き4段・最初の失敗 | 合格、5回 | 合格、1回 |
| 傾き4段・最大残差 | 不合格、20回 | 不合格、3回 |
| 傾き8段・最初の失敗 | 合格、5回 | 合格、1回 |
| 傾き8段・最大残差 | 不合格、15回 | 不合格、2回 |

横運動8段の最大荷重法線速度残差は、GSの約1.54e-3 m/sから約7.71e-15 m/sになり、
他の摩擦・散逸検査も合格した。一方、同じ問題でもGS後からは失敗する。
初期値依存があり、現在の試作を本体の代替として採用する根拠には不足する。

先行比較として法線を `min(pn,alpha*vn)` で表す版と、方向を制限しないFB版も保存した。
前者は初期力積から4/8、後者は5/8が合格。方向を制限する最終版も5/8であり、
制限を加えたことで合格件数が増えたとは主張しない。
未制限版ではGS後からのNewton方向のノルムが約2.3万〜10万に達する最大残差問題があり、
初期力積ノルム約1.6〜3.0に比べて大きすぎる方向が短いバックトラックでは採用されなかった。
方向を制限しても残る停滞が、次の課題である。

## 再現と検証

NumPyを利用できるPython環境で実行する。BLASの並列数を固定する。

```bash
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_frozen_newton.py \
  docs/evidence/contact-frozen-systems.json /tmp/frozen-newton.json
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_frozen_newton.py \
  docs/evidence/contact-frozen-systems.json /tmp/frozen-newton.json --verify
OPENBLAS_NUM_THREADS=1 python -m unittest discover \
  -s scripts/contact_benchmark -p test_frozen_newton.py -v
```

`--verify` はソース・入力のハッシュ、NumPy版と、再計算した力積・全判定を照合する。
異なるBLAS/数値環境でのビット一致は保証しない。
既存の保存結果を検証するときは `/tmp/frozen-newton.json` の代わりに
`docs/evidence/contact-frozen-newton-results.json` を指定する。

採取をやり直す場合は、別checkoutの `67a35679` に
[evidence/contact-frozen-capture.patch](evidence/contact-frozen-capture.patch) を適用し、
releaseの `contact_crowd` を構築する。そのバイナリと未変更の基準バイナリを用意して実行する。

```bash
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_freeze.py \
  /path/to/instrumented-contact-crowd /path/to/original-contact-crowd \
  /tmp/new-frozen-capture /tmp/new-frozen-systems.json
```

採取用のpatchは製品コードに適用しない。CLIとスクリプトはこのPRから用意する。
検査済みの最終例バイナリは元のものに復元した。

新規6テストは開離・静止・滑り・摩擦なし、階数不足、解析Jacobianと数値微分の一致、
独立検査の棄却、反復上限、不正入力、選択規則、レポート改変の検出を対象とする。
Python全120テスト合格。Rust製品ソースは変更していない。
workspace全体のCIはディスク容量制約のため未実施。

## 証拠と次の判断

- [最終版の全結果](evidence/contact-frozen-newton-results.json)
- [min版の全結果](evidence/contact-frozen-newton-min-results.json)と[ソース](evidence/contact-frozen-newton-min.py)
- [未制限FB版の全結果](evidence/contact-frozen-newton-unbounded-results.json)と[ソース](evidence/contact-frozen-newton-unbounded.py)
- [provenance](evidence/contact-frozen-newton-provenance.json)

次は固定問題で、接触状態が切り替わる付近の停滞と初期値依存を解消する。
残差と力積の大きさを追いながら、方向の制限・ステップ採用規則を比較する。
固定問題の改善後にRustで有界な候補解法を実装し、元の接触則で検証してから採用する。
シーン全体の最大めり込み、接触残差、計算コストを再評価するまでは既定の解法を変更しない。

後続: [零空間を使った接触荷重の解放](HARD_CONTACT_NULLSPACE_RELEASE.md)。
