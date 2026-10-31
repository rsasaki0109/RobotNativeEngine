# Hard-contact収束判定の早期終了

最新の200回バッチ計測では、最終版の1箱は0.64%悪化〜3.93%短縮だった。
小規模用分岐の明確な利点は確認できず撤回した。測定の詳細は末尾を参照。

## 変更と目的

基準は `3edd40ca`（#482）。接触ソルバーの反復後に計算する
`max |G δλ| <= velocity_tolerance` を、許容値を超えた最初の行で
falseを返す判定へ置き換えた。一時試作した4接触以下の特別扱いは、バッチ比較を受けて撤回した。
各行の内積の加算順序、接触の順序、
許容値、インパルス更新、停止条件のOR結合は変えない。
収束する反復では全行を評価するため、常に計算量が小さくなるわけではない。
行列やソルバーの物理モデルを近似する変更ではない。

旧 `f64::max` がNaNを無視する挙動も維持する。負またはNaNの許容値では
旧判定同様falseを返す。これは既存挙動の保存であり、非有限の物理状態を
正しいものとして認定する検査ではない。

## 計測から選んだ対象

GNU gprofng 2.44で基準版の128箱clustersを30回外部からサンプリングした。
[関数表](evidence/contact-iteration-profile.txt)では
`solve_contact_impulses` のexclusive比率35.43%、収束判定16.23%、
`coupled::step_with_external` 14.57%だった。inclusiveは重複する。
インライン展開のため、coupledのexclusiveを行列組み立て時間とは解釈しない。

表示CPU時間3.022秒は30実行の合計約32秒と一致していないため、
この表は候補選びの粗い比率にのみ使い、絶対時間や厳密な工程別割合を主張しない。
プロファイルは `/workspace/artifacts/contact-cluster-profile-30.er`。
`-a off` のためバイナリと共有ライブラリを内包せず、再解析には元のファイルが必要。
再現コマンド:

```bash
gprofng collect app -a off -F on -o profile.er \
  /bin/bash -c 'for ((i=0;i<30;i++)); do /path/to/baseline 128 timing clusters; done'
gprofng display text -functions profile.er
```

## 不採用にしたキャッシュ案

先に試した3×3 sticking solveの行列式・余因子キャッシュは、状態・診断の
一致を保てたが、64/128箱clustersで2回とも2.3〜3.4%遅くなった。
本変更には含めない。原因をキャッシュミスなどと断定する証拠はない。
[試作差分](evidence/rejected-sticking-cache.patch)は基準コミットに適用できる。
[A](evidence/sticking-cache-a-summary.json)、
[B](evidence/sticking-cache-b-summary.json)、
[非接触配置](evidence/sticking-cache-separated-summary.json)に全条件を残した。
1箱の短い実行だけで見えた改善は採用理由にしない。

## 評価条件と限界

[接触シーン](HARD_CONTACT_CLUSTER_SWEEP.md)と同じ64/128箱、stacks/clusters、
2 ms刻み100ステップ。単一CPUへ固定し、ウォームアップ後に前後交互で5回実行し、
中央値を比較する。診断は速度計測では無効。比較するのはNativeの変更前後で、
MuJoCoとの比較ではない。全箱が一体になった山積み、把持、長時間安定性は未評価。

大きな生データはローカルの `/workspace/artifacts` に保存し、要約とSHA-256を
リポジトリへ置く。永続的に共有できる外部URLはまだない。

## 検証

- Rust releaseライブラリテスト: dynamics 52件、native 63件成功。
- Python接触ベンチマークテスト: 80件成功。
- 有限ギャップ36条件: 変更前後の全軌跡・接触診断が一致。
- 両crateのrelease all-targets Clippy（warnings禁止）、fmt、依存境界検査成功。
- ワークスペース全体のClippy/test/ci-headless/ciは未検証。
  この環境は残容量約1.5 GiBで、以前の全体ビルドは容量不足になっている。

次は多数の箱が一つの接触グループになる条件で効果を確認する。
今回の独立した2箱・4箱のグループだけから、山積み全般への効果は判断しない。

## 初版の実測結果（3b801079）

| 計測 | 箱数 | 配置 | 変更前 ms/物理秒 | 変更後 ms/物理秒 | 時間短縮 |
| --- | ---: | --- | ---: | ---: | ---: |
| A | 64 | stacks | 987.92 | 950.56 | 3.8% |
| A | 64 | clusters | 2578.60 | 2184.82 | 15.3% |
| A | 128 | stacks | 1944.59 | 1860.89 | 4.3% |
| A | 128 | clusters | 5154.51 | 4442.81 | 13.8% |
| B | 64 | stacks | 984.42 | 920.29 | 6.5% |
| B | 64 | clusters | 2600.29 | 2221.89 | 14.6% |
| B | 128 | stacks | 1941.89 | 1881.52 | 3.1% |
| B | 128 | clusters | 5190.11 | 4466.65 | 13.9% |

2セットとも全100ステップの状態ハッシュ、支持ペア、候補数、接触診断、
反復回数・停止フラグが一致した。clustersで13.8〜15.3%、stacksで3.1〜6.5%短縮。
これは当該fixtureの結果であり、エンジン全般の高速化率ではない。

[A](evidence/velocity-stop-a-summary.json)、[B](evidence/velocity-stop-b-summary.json)、
[離して配置した1/16/64/128箱](evidence/velocity-stop-separated-summary.json)、
[ソース・バイナリとプロファイルのハッシュ](evidence/velocity-stop-provenance.json)。
離して配置したケースも[追加セットB](evidence/velocity-stop-separated-b-summary.json)を測定した。
1箱は最初の5試行で53.5%悪化、追加5試行で2.7%悪化した。
さらに同じCPU固定・ウォームアップ・前後交互で51試行を実施したところ、
中央値は100ステップあたり1.299 msから1.369 msへ約5.4%悪化した
（[全試行](evidence/velocity-stop-one-box-followup.json)）。全最終ハッシュは一致。
大きな変動を除いても、小規模ケースの悪化は解消したとはいえない。
16箱は2セットとも約10%短縮、64/128箱は約1〜5%短縮。
このため全サイズで勝つ変更とはせず、接触の多い条件と小規模条件の
トレードオフとしてレビューする。この初版の結果を受け、以下の小規模用経路を追加した。
51試行も既存 `contact_crowd.run(binary, 1, 'timing')` を使用し、
偶数試行でbefore→after、奇数試行でafter→beforeの順とした。

```bash
python scripts/contact_benchmark/contact_clusters.py BEFORE AFTER /tmp/velocity-stop.json
python scripts/contact_benchmark/contact_clusters.py BEFORE AFTER /tmp/velocity-stop.json --verify
```

## 小規模用経路の追補

4接触以下は全行の内積の最大値を計算し、5接触以上は早期終了する。
箱の一つの接触面が4点までという規模を基準にした切り替えで、
許容誤差や物理モデルの変更ではない。小さい行列で毎行の分岐を避ける意図であり、
実行時間の差を特定のCPU命令やキャッシュ要因へ帰属する測定はしていない。
単体テストは2/4/5/8接触で、先頭から末尾の各行の不成立、境界値、
相殺、オーバーフロー、特殊な許容値を旧最大値計算と比較する。

追補の対照は初版ではなく、変更前の `3edd40ca` の保存バイナリ。
1箱は51試行×2セットで、セットBは前後の実行順を反転。
接触の多い4条件も従来と同じ5試行×2セットで測定する。

| 1箱・51試行 | 変更前 ms/100ステップ | 追補後 ms/100ステップ | 時間短縮 |
| --- | ---: | ---: | ---: |
| A | 1.251164 | 1.250556 | 0.05% |
| B（順序反転） | 1.325775 | 1.312969 | 0.97% |

前回見られた約5%の悪化はこの2セットでは再現せず、ほぼ同等だった。
小さい改善値を高速化の主張には使わない。異なるCPUやコンパイラでも
悪化しないことを保証する結果ではない。
[1箱A](evidence/velocity-small-one-a.json)、[1箱B](evidence/velocity-small-one-b.json)、
[51試行の実行スクリプト](evidence/velocity-small-samples.py)、
[追補ソースとバイナリのSHA-256](evidence/velocity-small-provenance.json)。
実行スクリプトはrepo rootで実行し、保存バイナリのパスは環境に合わせる。

追補後もRust 115件、Python 80件、有限ギャップ36条件の前後比較、
両crateのrelease all-targets Clippy、fmt、依存境界検査を再実行して成功した。
ワークスペース全体CIの未検証という制約は変わらない。

| 計測 | 箱数 | 配置 | 変更前 ms/物理秒 | 追補後 ms/物理秒 | 時間短縮 |
| --- | ---: | --- | ---: | ---: | ---: |
| A | 64 | stacks | 960.47 | 940.15 | 2.1% |
| A | 64 | clusters | 2562.43 | 2192.90 | 14.4% |
| A | 128 | stacks | 1903.15 | 1848.49 | 2.9% |
| A | 128 | clusters | 5125.14 | 4414.40 | 13.9% |
| B | 64 | stacks | 952.77 | 930.63 | 2.3% |
| B | 64 | clusters | 2566.05 | 2212.62 | 13.8% |
| B | 128 | stacks | 1932.77 | 1878.79 | 2.8% |
| B | 128 | clusters | 5205.41 | 4419.51 | 15.1% |

[接触A](evidence/velocity-small-a-summary.json)、[接触B](evidence/velocity-small-b-summary.json)、
[離して配置したケース](evidence/velocity-small-separated-summary.json)。
接触4条件とも全100ステップの状態ハッシュ・接触診断・反復数は一致した。
タイミング実行の最終ハッシュも監査と一致。生データはローカル保存のみ。

### 追補の追加測定と未解決点

接触シーン計測後の通常5試行では、1箱が13.5%悪化した。
16/64/128箱の離して配置したケースは0.09〜0.17%悪化でほぼ同等。
1箱をさらに同じ51試行×2セットで確認すると、Aは0.29%悪化、Bは7.23%悪化した。

| 計測後の1箱・51試行 | 変更前 ms/100ステップ | 追補後 ms/100ステップ |
| --- | ---: | ---: |
| A | 1.306430 | 1.310233 |
| B（順序反転） | 1.263970 | 1.355363 |

[追加A](evidence/velocity-small-one-post-a.json)、
[追加B](evidence/velocity-small-one-post-b.json)。実行手順は上のスクリプトと同じで、
出力名のみ `velocity-small-one-post-` に変更した。先の測定は上書きしていない。
従って先の2セットだけを使って「悪化解消」と判断しない。
1箱の測定をより長い実行で評価することが次の課題であり、
4接触という切り替えを全般的に最適な閾値として認定していない。
今回もマージせず、接触の多い条件の改善と小規模の未確定要素を分けてレビューする。

## 200回バッチ比較と最終判断

短い約1.3 msの試行だけで小規模用の最適化を採否判断しないため、
`contact_crowd 1 timing separated 200` を追加した。
毎回新しいbackendとECSを作り、同じ着地100ステップを200回実行する。
各回のstepとECS書き戻しだけを計時し、合計を比較する。
初期化・ハッシュ・JSON出力は除外する。20000ステップを連続実行する方式ではなく、
着地後の静止時間の比率は元のfixtureと同じ。合計物理時間は40秒。
1サンプルの計測区間合計は約0.25秒で、200個の時刻区間の和である。

CPUを固定し、各版を1バッチでウォームアップ後、交互順序で9サンプル実行。
セットBは開始順を反転。比較順もA:早期終了→小規模用、B:小規模用→早期終了とした。
全200回の最終ハッシュを単独監査の最終ハッシュと照合し、監査の全100ステップも比較する。
全試行を保存し、検証時には監査とバッチを両バイナリで再実行する。

3版とも今回の同じexampleを使ってビルドし、`contact.rs`だけを差し替えた。
変更前は3edd40ca、早期終了のみは3b801079、小規模用分岐ありは35d5415c。
作業後は早期終了のみの実装に戻し、広げた2/4/5/8接触の単体テストは維持した。
[ビルドのソース・バイナリSHA-256](evidence/contact-batch-provenance.json)。

| 対象 | セット | 変更前 ms/物理秒 | 対象版 ms/物理秒 | 時間短縮 |
| --- | --- | ---: | ---: | ---: |
| 早期終了のみ | A | 6.42146 | 6.39425 | 0.42% |
| 早期終了のみ | B | 6.31027 | 6.20808 | 1.62% |
| 小規模用分岐あり | A | 6.40263 | 6.43829 | -0.56% |
| 小規模用分岐あり | B | 6.49077 | 6.31577 | 2.70% |

[早期終了A](evidence/contact-batch-early-a.json)、[早期終了B](evidence/contact-batch-early-b.json)、
[小規模用A](evidence/contact-batch-hybrid-a.json)、[小規模用B](evidence/contact-batch-hybrid-b.json)。
全監査・全試行のハッシュは一致し、全4レポートの再実行検証が成功した。
ペアごとの時間差にはなお外れがあり、統計的同等性や全CPUでの無劣化の証明ではない。
このバッチ条件で旧来の約5%悪化を再現しなかった、という範囲にとどめる。

小規模用の分岐を追加する一貫した利点を確認できないため撤回し、
より単純な早期終了のみの版を最終実装とする。過去の悪化測定も削除しない。
小規模性能を改善したとの主張ではなく、特殊な閾値を導入する根拠がないという判断。

```bash
python scripts/contact_benchmark/contact_batch.py BASE CANDIDATE /tmp/batch-a.json
python scripts/contact_benchmark/contact_batch.py BASE CANDIDATE /tmp/batch-b.json --first after
python scripts/contact_benchmark/contact_batch.py BASE CANDIDATE /tmp/batch-a.json --verify
```

通常の引数では従来と同じ1試行・JSON形式を保つ。繰り返しは1箱のtimingだけに制限し、
1〜1000回の範囲を検査する。追加テストは途中のハッシュ不一致、時間集計の改ざん、
非有限時間、試行欠落、CLI制限、単独監査と繰り返しの一致を検査する。

### 最終ビルドでの再確認

拡充したテストを残した最終ソースからビルドしたexampleのSHA-256は、
探索時の早期終了版と異なるため、最終バイナリを改めて測定した。
本変更の最終的な1箱の数値はこちらを使う。

| セット | 変更前 ms/物理秒 | 最終版 ms/物理秒 | 時間短縮 |
| --- | ---: | ---: | ---: |
| A | 6.49751 | 6.53893 | -0.64% |
| B（順序反転） | 6.64908 | 6.38749 | 3.93% |

[最終版A](evidence/contact-batch-selected-a.json)、
[最終版B](evidence/contact-batch-selected-b.json)。
ペア差には大きなばらつきが残るため、これを厳密な無劣化証明には使わない。
元の約5%悪化が一貫して再現する結果ではなく、小規模用分岐を加える根拠もない。
これ以上この1箱の細かな数値に合わせて分岐を調整せず、
接触の多い条件の改善を本変更の対象とする。

最終ソースでRust 115件・Python 84件、release all-targets Clippy（両crate）、
fmt、依存境界検査成功。ワークスペース全体CIは容量制約により引き続き未検証。

最終バイナリで接触4条件も1セット（各5試行）再測定し、
全ステップのハッシュ・接触診断・反復数の一致と短縮を確認した。
[最新の接触比較](evidence/contact-batch-selected-clusters-summary.json)。

| 箱数 | 配置 | 変更前 ms/物理秒 | 最終版 ms/物理秒 | 時間短縮 |
| ---: | --- | ---: | ---: | ---: |
| 64 | stacks | 1065.57 | 1003.89 | 5.8% |
| 64 | clusters | 2615.52 | 2265.07 | 13.4% |
| 128 | stacks | 2004.74 | 1918.60 | 4.3% |
| 128 | clusters | 5549.63 | 4834.40 | 12.9% |
