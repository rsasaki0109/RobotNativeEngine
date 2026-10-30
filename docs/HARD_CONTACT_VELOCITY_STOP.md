# Hard-contact収束判定の早期終了

最新追補では接触の多い条件の短縮を維持したが、1箱の時間変動が残る。
小規模ケースの悪化解消は未確定であり、Draftのまま評価する。

## 変更と目的

基準は `3edd40ca`（#482）。接触ソルバーの反復後に計算する
`max |G δλ| <= velocity_tolerance` を、許容値を超えた最初の行で
falseを返す判定へ置き換えた。追補では4接触以下を従来の全行最大値計算に戻した。
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
