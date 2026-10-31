# 接触候補の採用距離を切り分ける追試

3 kg・一辺0.1 mの箱にCOM外力17.658 Nを加える。摩擦係数0.5、初期速度0、
初期姿勢は床に接する状態で、事前の静置はしない。観測は50 ms。
Nativeは2 ms、MuJoCo 3.9.0は8 µs。時間刻みを揃えた性能比較ではない。

## Nativeで候補距離だけを変える

診断用exampleの末尾に `CONTACT_MARGIN_M` を追加した。
選べる値は0、0.0001、0.02 m。既存の呼び出しとバックエンドの既定値は変わらない。
ギャップバイアス、摩擦、反復上限200、正則化0は固定する。

| 候補距離 | 保持点数 | 最大回転速度 | 最大めり込み | 回転0.01 rad/s超過 |
| --- | --- | ---: | ---: | ---: |
| 0 | 2・4 | 0.342618 rad/s | 61.783 µm | 4 ms |
| 0.1 mm | 常に4 | ECS観測値0 | 4.14e-5 µm | なし |
| 20 mm（既定） | 常に4 | ECS観測値0 | 4.14e-5 µm | なし |

0.1 mmと20 mmの状態トレースは完全一致した。正のギャップを持つ点が法線インパルスを
受けたサンプルはそれぞれ11件（閾値1e-10 N s）、最大ギャップは約0.889 nm。
したがって、このケースで20 mm先から支持しているわけではない。
接触境界付近の点を距離0で切り捨てることが不安定化するという観測であり、
大きな隙間や高速衝突に対する予測接触の有効性を実証したものではない。
ECS回転速度は姿勢差からの再構成なので、観測値0は数学的な厳密ゼロの保証ではない。

## MuJoCo対照条件の訂正

前回資料の「margin=gap=0.1 mmなら拘束開始距離は0」という説明は誤りだった。
今回、箱を床から50 µm離して `mj_forward` を実行し、実際の `contact.dist`、
`contact.includemargin`、`efc_address` を記録・テストした。
このモデルでは両geomに同じ値を指定する。

| 各geomのmargin / gap | 50 µmでの検出点数 | includemargin | 拘束採用 |
| --- | ---: | ---: | --- |
| 0 / 0 | 0 | — | なし |
| 0 / 0.1 mm | 4 | 0 | なし |
| 0.1 mm / 0.1 mm | 4 | 0.2 mm | 4点 |
| 0.1 mm / 0 | 4 | 0.2 mm | 4点 |

solref=[0.02,1]、通常の安全設定を維持した50 ms試行では:

- margin=0の2条件は、検出点数を除く記録済みステップ指標、点別の力・残差が完全一致。
  回転基準超過はどちらも18.944 ms。gapのみの追加は拘束の切替を変えなかった。
- margin=0.1 mmの2条件も同じ記録項目が完全一致し、超過は41.640 ms。
  接触前から拘束を採用しても、この条件では回転を除去できない。
- 全条件で拘束点数は0・2・4、反復上限到達は0。

MuJoCoのmargin変更は拘束の基準距離にも影響するため、Nativeの候補距離と同義ではない。
今回はNativeの予測ギャップバイアス自体は変更しておらず、その独立の効果は未分離。
性能計測、2秒品質ゲート、未使用の検証用質量での合格も主張しない。
次は非ゼロの初期隙間を持つ落下・滑走で、予測接触の効果と必要な候補距離を確認する。

## 再現

```bash
cargo build --release -p rne_physics_native --example friction_threshold
python scripts/contact_benchmark/contact_admission.py \
  target/release/examples/friction_threshold /tmp/contact-admission
python scripts/contact_benchmark/contact_admission.py \
  target/release/examples/friction_threshold /tmp/contact-admission --verify
```

検証はバイナリ・ソース・生配列のSHA-256、集計の再計算、両エンジンの新規再実行との
完全一致を確認する。候補欠落、設定の不一致、採用距離外の点、無効なCLI値の拒否もテストする。

[要約とハッシュ](evidence/contact-admission-summary.json)。生レポートとNPZは
`/workspace/artifacts/contact-admission-v3`（合計3,267,304 bytes）に保存。
永続公開URLはない。ADR034に従い大容量の生データはGitに追加していない。
全workspace CIは未実行で、マージ前の確認が残る。

今回の検証: Python 70件、Native releaseライブラリ59件、対象クレート全targetの
release Clippy、fmt、依存境界チェックを通過。
