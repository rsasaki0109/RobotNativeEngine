# 荷重解放法を未使用ステップで評価する

基準は `0080e467`（#496）。Newton法と荷重解放法を変更せず、既存の開発用8固定問題を
除外した別の16問題を評価した。**品質改善は別ステップにも見られるが、Python試作の
実行時間は増える。品質とコストの両面で勝てたという結果ではない。**

## 選択を結果から切り離す

#495で採取した4シーンの全失敗solveの生データを使う。まず各シーンの開発用
「最初の失敗・最大残差」の2件を除外する。残りを元の順序に並べ、
インデックス `floor((2k+1)*N/8), k=0..3` の4件を選ぶ。
選択に今回のNewton法や荷重解放法の成否を使わない。
入力行列・ベクトルも含めて16件は互いに異なり、開発用8件とも重複しないことを検査した。

これは**同じ4シーンの別ステップ**であり、新しい形状・摩擦係数・ロボット・長期軌道への
汎化を示すものではない。この結果を見た後の調整では、この16件を未知データ扱いしない。
選択した全入力は `docs/evidence/contact-release-holdout/` に収録する。

## 比較方法

各問題を元のwarm start（seed）とGS終了時の力積（gs）から解く。合計32実行。
比較対象は#495のNewton法、追加版は#496の荷重解放法で、両方のソースは変更していない。
法線・円錐・滑り方向・散逸の独立検査も同一。採用Newtonステップ上限50、
追加版の荷重解放上限8回を維持する。

CPU affinityを1 CPUに固定し、`OPENBLAS_NUM_THREADS=1` で実行する。
各入力について両版を1回ずつ実行してから、5回交互に計測する。
入力ごとの開始順も交互にし、さらに開始順を反転したA/Bの2セットを取る。
全計測で力積・判定・再試行履歴が事前実行と完全一致することを確認する。

計時対象は**Pythonのsolve関数全体**で、行列の読み込み・SVD・残差計算・
独立検査・戻り値の作成を含む。ファイルI/Oは除く。
Rustの接触カーネル時間やシミュレーション1秒当たりの費用ではない。
全体時間は32入力それぞれの5回の中央値を合計したもの。中央値の合計であり、
32入力を一括して走らせた測定値ではない。

## 結果

A/Bの品質結果と全Newton履歴は一致した。

| 初期値 | 従来Newton | 荷重解放あり | 新たな合格 | 失った合格 |
|---|---:|---:|---:|---:|
| seed | 3/16 | 12/16 | 9 | 0 |
| gs | 0/16 | 8/16 | 8 | 0 |

全32実行では3→20合格、17実行が改善し、合格から不合格への変化は0だった。
ただし12実行はまだ不合格。合格数は入力×初期値の数であり、32種類の別形状を試した意味ではない。

| 計測セット | 従来Newtonの中央値合計 (s) | 荷重解放版 (s) | 比率 |
|---|---:|---:|---:|
| 予備測定 | 2.048242 | 5.293388 | 2.58× |
| A | 1.931550 | 5.145086 | 2.66× |
| B（開始順反転） | 1.949223 | 5.253064 | 2.69× |

採用Newtonステップ数の合計は316→762、
追加の解放候補評価は756件だった。品質改善に追加費用を払っている。
現段階で高速化は主張しない。PythonからRustへの移植でこの比率が維持されるとも限らない。
予備測定後は検証時のインデックス処理と計測開始順の指定だけを整えた。ソルバーは変えておらず、
予備測定・A・Bの力積と全判定は同じだった。予備測定の計時も保存する。

## 再現

選択済み入力からの測定には生キャプチャは不要。NumPyを用意して実行する。

```bash
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_release_holdout.py run \
  docs/evidence/contact-release-holdout /tmp/holdout-a.json
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_release_holdout.py run \
  docs/evidence/contact-release-holdout /tmp/holdout-b.json --first release
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_release_holdout.py run \
  docs/evidence/contact-release-holdout /tmp/holdout-a.json --verify
```

`--verify` は全32組を再実行し、力積・独立検査・履歴、入力/依存ソースのSHA256、
NumPy版、計時標本の数・有限性・集計を確認する。過去の実時間そのものを再現する検査ではない。
保存済み結果は出力パスに `docs/evidence/contact-release-holdout-a.json` を指定して検証できる。
ビット一致は同じ数値環境に限り、異なるBLAS環境では保証しない。

生キャプチャから選び直す場合は、#495の採取手順で生成したrawディレクトリを指定する。
元のfixtureにある各キャプチャのSHA256と一致しなければ処理を中止する。

```bash
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_release_holdout.py freeze \
  /path/to/contact-frozen-final-raw docs/evidence/contact-frozen-systems.json /tmp/new-holdout
```

## 検証・証拠

- 選択の決定性・開発用との非重複、rawハッシュ、全32組のcoverage、計時/集計/ソース改変の検出をテスト。
- Python全129テスト合格。全計時実行が事前実行と一致し、A/Bをさらに新規実行して検証。
- Rustソース、Newton法、荷重解放法は未変更。全workspace CIはディスク容量制約で未実施。
- [結果A](evidence/contact-release-holdout-a.json)、[結果B](evidence/contact-release-holdout-b.json)。
- [予備計時](evidence/contact-release-holdout-preliminary-summary.json)と[当時のスクリプト](evidence/contact-release-holdout-initial.py)。
- [provenance](evidence/contact-release-holdout-provenance.json)。各ファイルは1 MiB未満。

## 次の実装判断

荷重解放は開発用8件以外でも改善する一方、756候補の追加評価と長い再試行が必要だった。
次は候補の上限・優先順位・同一接触の再選択を制御して費用を減らす。
今回の16件を使って調整した場合は、さらに別の問題で確認する。
本体導入前には、固定問題の合格だけでなく、シーン全体の最大残差・めり込み・実時間を確認する。

後続: [未使用Jacobianを省く同値高速化](HARD_CONTACT_RELEASE_RESIDUAL_COST.md)。
