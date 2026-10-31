# 荷重解放の候補評価で未使用Jacobianを作らない

基準は `f6eb548e`（#497）。候補を絞る試作は一部の合格を失ったため、
まず**候補数・選択順・物理条件を変えずに、順位付けで使わないJacobianの計算を省く**。
引き続きオフラインPython試作で、Rustの本体には未導入。

## 試した変更

開発用8問題と#497の16問題、計24問題×2初期値で、解放する候補を
`vn * pn` の大きい順に1件または4件に絞って比較した。同値は接触番号順。
全件評価と同じNewton・射影・独立検査を使う。

| 方法 | 合格 / 48実行 | 候補評価数 | 全件評価から失った合格 |
|---|---:|---:|---:|
| 全件評価 | 32 | 912 | 0 |
| 上位1件 | 34 | 71 | 2 |
| 上位4件 | 36 | 292 | 2 |

候補制限は合格総数を増やすが、以前解けた2実行を失う。
今回は既存結果を保つ高速化を選ぶ。候補制限が常に劣るという結論ではなく、
別解法としての品質・費用評価は残る。この試作の結果も隠さず保存する。

全候補を評価する現在のコードは、残差二乗和だけで候補を選ぶのに
`residual_jacobian` から残差とJacobianを毎回生成していた。
新しい `contact_release_fast.residual` は、元と同じ順序・式で残差だけを計算する。
さらに、Newtonのバックトラックも試行点の残差だけを使うため、同じ省略を適用した。
採用方向を求める点では従来どおりJacobianを作る。Newtonの方程式、全候補の走査、
SVD、射影、採用判定、勾配診断は変えない。
採用Newtonステップは合計50以下、解放は最大8回のまま。

比較用に元のソースを保持し、新しいモジュールに候補評価を省力化した版を置いた。
Newton反復と解放制御の重複部分がずれないよう、成功・失敗を含む全出力の完全一致を検査する。

## 評価用入力

前回の生キャプチャから、開発用2件と#497の4件を各シーンで除外する。
残りから元の順序で中点四分位の4件ずつを選ぶ。追加16問題は、既存24問題と重複しない。
選択には今回のソルバー結果を使わない。新しい形状ではなく、同じ4シーンの別ステップである。
入力全体は `docs/evidence/contact-release-fast-validation/` に保存した。

CPUを1つに固定し、BLASも1スレッドにする。
全件評価版と高速版を各入力で1回実行してから、5回交互に測る。
さらに開始順を反転してA/Bの2セットを取る。計時対象はPythonのsolve全体で、
検査や履歴の作成を含み、ファイルI/Oを含まない。
集計は各入力の中央値の合計で、Rustカーネルやシーン実時間の測定ではない。

## 結果

開発用24問題×2初期値の48実行と、追加16問題×2初期値の32実行、
合計80実行で、元の全件評価版と最終高速版の全出力が一致した。
力積、合格判定、失敗理由、採用ステップ、解放候補数、選択履歴、勾配診断も同じ。
追加32実行の合格は両方20/32で、不合格12実行もそのまま残す。

| 省略対象 | セット | 元の全件評価版 (s) | 高速版 (s) | 短縮率 |
|---|---|---:|---:|---:|
| 候補評価のみ | A | 6.204643 | 5.989526 | 3.47% |
| 候補評価のみ | B | 6.170692 | 5.957842 | 3.45% |
| 候補評価＋バックトラック | A | 6.213361 | 4.263708 | 31.38% |
| 候補評価＋バックトラック | B | 6.048381 | 4.177598 | 30.93% |

A/Bで開始順を反転しても短縮を確認した。これは同じ数値結果を保ったPython試作の改善である。
実時間は環境負荷の影響を受け、異なる行の元版の計時も変動する。段階間の単独計時ではなく
各行の交互比較を使う。候補数と採用Newtonステップ数は減っていない。
既存の荷重解放なしNewton法やMuJoCoとの費用逆転を示したわけではない。

## 再現

```bash
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_release_fast_compare.py run \
  docs/evidence/contact-release-fast-validation /tmp/fast-a.json
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_release_fast_compare.py run \
  docs/evidence/contact-release-fast-validation /tmp/fast-b.json --first release
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_release_fast_compare.py run \
  docs/evidence/contact-release-fast-validation /tmp/fast-a.json --verify
OPENBLAS_NUM_THREADS=1 python -m unittest discover \
  -s scripts/contact_benchmark -p test_release_fast.py -v
```

入力選択をやり直す場合は、rawキャプチャ、開発用fixture、前回の16問題を指定する。

```bash
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_release_fast_compare.py freeze \
  /path/to/contact-frozen-final-raw docs/evidence/contact-frozen-systems.json \
  docs/evidence/contact-release-holdout /tmp/new-fast-validation
```

候補制限の試作は保存済みのソースから全48実行を再計算できる。

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=scripts/contact_benchmark \
  python docs/evidence/contact-release-cap-trial.py \
  docs/evidence/contact-frozen-systems.json docs/evidence/contact-release-holdout /tmp/cap-screen.json
```

新規テストは開離・静止・滑り・境界での残差完全一致、固定seedのPSD行列、
成功/失敗の全履歴一致、不正な上限、既存24問題との非重複を検証する。
Python全132テスト合格。最終A/Bは新規実行で結果とハッシュを検証した。
候補制限試作も全結果の再現を確認した。Rustコードと従来のソルバーは未変更。
全workspace CIはディスク容量制約のため未実施。

## 証拠

- [最終A](evidence/contact-release-fast-a.json)・[最終B](evidence/contact-release-fast-b.json)
- [候補のみA](evidence/contact-release-candidate-only-a-summary.json)・[B](evidence/contact-release-candidate-only-b-summary.json)・[ソース](evidence/contact-release-candidate-only.py)
- [候補制限の全判定](evidence/contact-release-cap-screen-summary.json)・[ソース](evidence/contact-release-cap-trial.py)
- [provenance](evidence/contact-release-fast-provenance.json)

最終版の全入力と全結果を収録する。途中版の大きな生結果はワークスペース内の一時保管で、
要約にはSHA256を記録した。過去の計時値そのものの再現は保証しない。

次は品質を失った候補制限ケースの分析と、Rust実装に向けたSVD・再試行費用の見積もりを進める。
本体への導入前にはシーン全体の品質・費用を別途測る。
