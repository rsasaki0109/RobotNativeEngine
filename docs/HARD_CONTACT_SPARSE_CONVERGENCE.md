# 非ゼロ係数一覧を再利用した収束判定

基準は `8f13f63a`（#486）。行列構築の改善後、31箱bridgeを10回外部から
サンプリングすると、exclusive比率は `solve_contact_impulses` が約59%、
`coupled::step_with_external` が約24%だった。
[関数表](evidence/contact-sparse-convergence-profile.txt)はインライン展開を含み、
ソルバー内部の各処理を分離できない。表示CPU時間2.742秒も実行全体と整合しないため、
厳密な工程別時間として使わず、次の候補選びに限る。全10回の最終ハッシュは監査と一致。

## 変更

ソルバーに残る密な `G δλ` の行走査を対象にした。
既存の `SparseBiasRows` が持つ、自接触ブロックを除いた非ゼロ係数の一覧を再利用する。
既存のソート済み列を二分探索し、前の列→元の自接触3要素→後の列という順で加算する。
係数の列順を変えず、極小の非ゼロ値も残す。

自接触の3要素は元のDelassus行列から読み、局所ソルバーで加えた正則化を含めない。
同じ接触速度変化を測るためであり、正則化・停止許容値・反復上限の変更ではない。
許容値を超えた最初の行で未収束とする既存の早期終了も維持する。

追加の係数配列や行ごとの索引配列は作らない。
疎な一覧の採用条件（32接触以上かつ自己ブロック外の非ゼロが全要素の1/4以下）も変えない。
収束判定への適用は64接触（192列）以上に限定する。一覧がない場合や短い行では、
元の密なループへ反復ごとに一度だけ分岐し、行ごとの省略経路の判定を避ける。
適用する反復には差分の有限性チェックが追加される。

## 数値の扱い

差分 `δλ` のどれかが非有限なら密な計算を使い、ゼロ×無限大などを省略しない。
省略経路で和が非有限になった行も従来計算で評価し直す。
有限な場合のゼロ項の省略は、通常のround-to-nearest演算で絶対値を保つ。
符号付きゼロの違いは、この判定で元から使う絶対値により消える。
元のNaN無視の最大値判定と、非正・非有限の許容値に対する挙動も維持する。

新しいテストは32/33/63/64/65接触で、自接触の対角・交差成分を含む各行の
絶対速度変化を従来式とビット比較する。正負のゼロ、極小/巨大値、差分の
オーバーフロー、無限大/NaN、許容値の境界を含め、疎・密の停止判定も比較する。

## 評価と検証

CPU固定・ウォームアップ後、bridgeとclustersは各条件5試行の前後交互測定。
1箱は200回の新規ワールドをまとめたバッチを9試行、開始順を反転して2セット。
exampleや評価スクリプトは変更せず、前回の保存バイナリと比較する。
[ソース・バイナリ・プロファイルのSHA-256](evidence/contact-sparse-convergence-provenance.json)。

Rust releaseライブラリテスト: dynamics 57件、native 63件成功。
Python接触ベンチマークテスト91件成功。
有限ギャップ36条件の全軌跡・接触診断は保存済みの変更前バイナリと一致。
両crateのrelease all-targets Clippy（warnings禁止）、fmt、依存境界検査成功。
ワークスペース全体CIは容量制約で未検証（過去の全体ビルドは容量不足）。

大きな生データ・プロファイルは `/workspace/artifacts` に保存し、要約とハッシュを
コミットする。永続的に共有できる生データURLはまだない。
プロファイルは `gprofng collect app -a off -F on` で保存したため、
再解析には元のバイナリと共有ライブラリが必要になる。
本評価はNativeの前後比較であり、MuJoCoや全ワークロードへの優位性を示すものではない。

## 短い行での試作と修正

最初は既存の疎な一覧があれば全ての収束判定に使った。
bridgeは15箱16.3%、31箱22.7%短縮したが、独立した128箱clustersでは9.4%悪化した。
[最初のclusters比較](evidence/contact-sparse-convergence-v1-clusters.json)、
[試作差分](evidence/contact-sparse-convergence-v1.patch)、
[全試作レポートの要約・来歴](evidence/contact-sparse-convergence-v1-provenance.json)。

これを受けて、短い行は元の密なループに直接戻し、192列以上だけで再利用する形に変更した。
接触数の境界63/64/65も数値一致テストに含める。
192列は今回の実装上の費用を抑えるガードであり、全CPUに最適な閾値とは主張しない。
最終版は再ビルド後に前回版と改めて比較し、試作の計測値と混ぜない。

さらに短い行を従来ループへ戻した試作でも、小グループのwall時間が大きく振れた。
[2回目の試作の全結果](evidence/contact-sparse-convergence-v2-provenance.json)、
[差分](evidence/contact-sparse-convergence-v2.patch)。64箱clustersで43.3%悪化したセットがあった。
外部の `resource.getrusage(RUSAGE_CHILDREN)` で別途9組測ると、中央値は
全プロセスCPU時間0.451→0.460秒、step wall時間0.448→0.456秒だった。
[CPU診断](evidence/contact-sparse-convergence-v2-cpu64.json)、
[実行スクリプト](evidence/contact-sparse-convergence-cpu-check.py)。
CPU時間には初期化・出力も含むため、通常のstep計時と同じコスト指標ではない。
この別測定だけで43%の差の原因を断定したり、全体の無劣化を証明したりしない。

最終実装では索引配列の追加も撤回し、既存のソート済み係数を二分探索して
自接触成分の位置を求める。小規模でも使われる疎な一覧の構造・構築処理を
元に戻し、新たな処理を長い行の収束判定内に限定する。

## 最終版の結果

最終版はprototypeの結果を流用せず、bridge/clustersを各2セット測定した。

| セット | 箱数 | 変更前 秒/物理秒 | 変更後 秒/物理秒 | 時間短縮 |
| --- | ---: | ---: | ---: | ---: |
| A | 15 | 3.302 | 2.792 | 15.5% |
| A | 31 | 12.028 | 8.962 | 25.5% |
| B | 15 | 3.317 | 2.706 | 18.4% |
| B | 31 | 12.024 | 8.959 | 25.5% |

[bridge A](evidence/contact-sparse-convergence-bridge-a.json)、
[bridge B](evidence/contact-sparse-convergence-bridge-b.json)。
15箱で15.5〜18.4%、31箱で両セットとも約25.5%短縮。
31箱の0.2物理秒は約2.41秒から約1.79秒になったが、依然実時間より約9倍遅い。
全100ステップの姿勢・支持辺・状態ハッシュ・接触診断・反復数・停止フラグが完全一致。
最大床めり込み3.90e-10 m未満、箱間めり込み1.93e-10 m未満で変化なし。

小グループの結果も省略しない。時間短縮が負なら悪化を表す。

| セット | 箱数 | 配置 | 時間短縮 |
| --- | ---: | --- | ---: |
| A | 64 | stacks | -1.6% |
| A | 64 | clusters | -1.5% |
| A | 128 | stacks | -0.4% |
| A | 128 | clusters | -2.1% |
| B | 64 | stacks | -1.9% |
| B | 64 | clusters | 1.1% |
| B | 128 | stacks | 1.9% |
| B | 128 | clusters | -1.8% |

[clusters/stacks A](evidence/contact-sparse-convergence-clusters-a.json)、
[clusters/stacks B](evidence/contact-sparse-convergence-clusters-b.json)。
この結果から小グループの無劣化・高速化を主張しない。

1箱の200回バッチは[A](evidence/contact-sparse-convergence-batch-a.json)で3.2%、
[B](evidence/contact-sparse-convergence-batch-b.json)で9.8%短縮したが、
ペアごとの差が大きく、新経路も使わない条件なので改善の根拠には使わない。

最終ソースでRust 120件・Python 91件、有限ギャップ36条件、Clippy、fmt、境界検査を再実行済み。
全6レポートは保存結果の再計算と新規プロセスでの監査再実行に成功。
バッチは全繰り返しの最終ハッシュも監査と一致した。

```bash
python scripts/contact_benchmark/contact_bridge.py BEFORE AFTER /tmp/bridge.json
python scripts/contact_benchmark/contact_bridge.py BEFORE AFTER /tmp/bridge.json --verify
python scripts/contact_benchmark/contact_clusters.py BEFORE AFTER /tmp/clusters.json
python scripts/contact_benchmark/contact_batch.py BEFORE AFTER /tmp/batch.json
```
