# 接触行列構築でのゼロ係数の省略

## 対象と変更

基準は `a0fdb425`（#485）。前回は接触反復中の速度計算を改善した。
今回は `block_delassus` が接触行列 `G = J M̃⁻¹ Jᵀ` を作る際の内積を改善する。
以前は、接触に関係しない物体も含む全自由度の成分を各内積で走査していた。

Jacobian各行の厳密な非ゼロ係数を列順に保存し、応答列との内積に使う。
非ゼロ値の大きさによる切り捨てはない。物体ごとの質量行列の分解・応答列の解法、
接触行列の対称成分のコピー、接触反復・停止条件は変えない。
元の密なJacobian、応答列、接触行列も維持するため、密行列の格納をなくす変更ではない。

全一般化速度自由度が48未満、またはその行の非ゼロ係数数が全自由度の1/4を
超える場合は従来の内積を使う。この分岐は物理の近似ではなく、
小規模・密な行でインデックス構築の費用を避ける実装上の選択。
48自由度以上では行の検査や一時領域が増え、密な入力でも無料になるわけではない。

前回の[サンプリング](evidence/contact-sparse-bias-profile-after.txt)では
`coupled::step_with_external` のexclusive比率が約72%だった。
インライン展開を含むため、これを行列構築だけの時間とはみなさず、
今回の同一fixtureの前後計測で変更全体の効果を評価する。

## 数値挙動を保つ仕組み

有限の応答列だけに省略経路を使い、残す係数の乗算・加算順序を維持する。
ゼロ×無限大・NaNを隠さないよう、非有限の応答列は従来計算を使う。
積や和がオーバーフローして非有限になった場合も元の全項を計算し直す。

内積の結果がゼロの場合、単に正のゼロへ置き換えると元の負のゼロを変える可能性がある。
そこで保存したゼロ係数と対応する応答の符号を調べ、正のゼロの積が元の内積に
少なくとも一つ存在する場合だけ正のゼロを返す。それを確認できなければ、
従来の `Iterator::sum` で再計算する。
これは通常のround-to-nearest演算における符号付きゼロの扱いを前提とする。

新規テストは48/96/186自由度で、1e-300/1e300スケール、正負のゼロ、
無限大・NaN、オーバーフローを含む入力を従来の内積とビット比較する。
負のゼロだけの和、正のゼロの積が存在する場合、密な行、極小の非ゼロ値も検査する。

## 評価方法と制約

exampleと評価スクリプトは前回と同じ。CPU固定・ウォームアップ後、
bridgeとclustersを各5試行の前後交互測定で比較する。
1箱は同じ着地を新規ワールドで200回繰り返すバッチを9試行、開始順を反転して2セット。
診断は計時中に無効とし、監査の姿勢・接触診断・状態ハッシュとの一致を別に確認する。

[ソース・バイナリのSHA-256](evidence/contact-sparse-delassus-provenance.json)。
大きな生データは `/workspace/artifacts` に保存し、要約とハッシュをコミットする。
永続的に共有できる生データURLはまだない。
これは当該Native fixtureの比較であり、MuJoCoや全般的なエンジン優位性を示すものではない。

## 検証

Rust releaseライブラリテスト: dynamics 56件、native 63件成功。
Python接触ベンチマークテスト91件成功。
有限ギャップ36条件の全軌跡・接触診断が保存済みの変更前バイナリと一致。
両crateのrelease all-targets Clippy（warnings禁止）、fmt、依存境界検査成功。
ワークスペース全体CIは容量制約で引き続き未検証（過去の全体ビルドは容量不足）。

## 連結シーンの結果

| 箱数 | 変更前 秒/物理秒 | 変更後 秒/物理秒 | 時間短縮 |
| ---: | ---: | ---: | ---: |
| 15 | 5.436 | 3.160 | 41.9% |
| 31 | 47.618 | 11.536 | 75.8% |

[bridge比較](evidence/contact-sparse-delassus-bridge-summary.json)。
全100ステップの姿勢・支持辺・状態ハッシュ・接触診断・反復数・停止フラグが完全一致。
最大床めり込みは3.90e-10 m未満、箱間めり込みは1.93e-10 m未満で変更なし。
31箱の物理時間0.2秒は約9.52秒から約2.31秒になった。実時間より依然約11.5倍遅く、
これを実時間性能やMuJoCoに勝ったという主張へ広げない。
過去の計測との比率を掛け合わせず、今回の同時期の前後比較を用いる。

## 補助条件と再実行

[小グループの比較](evidence/contact-sparse-delassus-clusters.json)では、
stacksは64箱4.8%短縮・128箱1.8%悪化、clustersは64箱0.3%悪化・128箱0.5%短縮。
小グループのソルバーでは自由度の条件を満たさず、今回の省略経路は使わない。
全ケースで速くなるとは主張せず、これらの小さな差も保存する。

1箱の200回バッチは[A](evidence/contact-sparse-delassus-batch-a.json)で0.39%悪化、
[B](evidence/contact-sparse-delassus-batch-b.json)で2.55%短縮だった。
小規模ケースでの速度向上や統計的無劣化の証明には使わない。

bridge、clusters、両バッチは保存結果の再計算に加えて、新規プロセスでの
監査再実行も成功した。全姿勢・診断・状態ハッシュが一致し、
バッチでは全200回の最終ハッシュも監査と一致した。

```bash
python scripts/contact_benchmark/contact_bridge.py BEFORE AFTER /tmp/bridge.json
python scripts/contact_benchmark/contact_bridge.py BEFORE AFTER /tmp/bridge.json --verify
python scripts/contact_benchmark/contact_clusters.py BEFORE AFTER /tmp/clusters.json
python scripts/contact_benchmark/contact_batch.py BEFORE AFTER /tmp/batch.json
```
