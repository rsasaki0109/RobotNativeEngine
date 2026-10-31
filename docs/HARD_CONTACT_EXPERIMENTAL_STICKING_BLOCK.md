# 静止摩擦の一括解法を明示的な実験機能にする

基準は `afe4421b`（#492）。残差を測れるようになったので、動き・傾きのある積層で
接触をまとめて解く候補を評価した。採用する実装は**既定OFFの実験機能**である。
残差不合格のステップ数が減る条件はあるが、最大残差やコストが悪化する条件もあり、
既定の解法を置き換える根拠には達していない。

## アルゴリズム

`ContactSolverConfig.experimental_sticking_block = true` のとき、従来の法線のみの
一括解法で解けなかった島に、全接線速度をゼロにする静止摩擦候補を試す。
対象は2〜64接触、全摩擦係数が正、正則化0、relaxation 1、有限で正の速度許容値。
力積許容値も有限・非負で、反復予算が正の場合に限る。

初期のactive setは閉じる自由法線速度またはwarm startの正荷重から決める。
activeな接触の3成分をまとめ、元のDelassus行列にゼロ開始の共役勾配法を適用する。
整合する半正定値・階数不足の系に対して、range spaceの候補を得るための方法であり、
対角に微小値やコンプライアンスを加えない。
CGは最大 `2 * active_rows + 1` 回。非正の曲率、非有限値、精度不足は棄却する。
CG内部の速度残差目標は既存許容値の1/4で、最後の元の方程式による検査を優先する。

負の法線荷重が出た場合は最も負の接触を外す。非activeな接触が閉じる場合は追加する。
active-set更新は `min(max_iterations, 8)` 回で打ち切る。
候補の力積が非負・摩擦円錐内であること、および元の全接触行列に対する
`contact_law_satisfied` を確認してから採用する。
滑りが必要で円錐を超える候補は射影して採用せず、元のwarm startでGSに戻る。
すべて固定順序で、シミュレーション中に時計や乱数は使わない。

## APIと利用方法

```rust
config.contact.solver.experimental_sticking_block = true;
```

```bash
target/release/examples/contact_crowd 4 audit tower-slide 1 sticking-block
```

従来のCLI呼び出しと既定設定では無効。有効時だけJSONに
`experimental_sticking_block: true` を追加する。
`ContactSolution` に `sticking_block_solved` と `sticking_block_iterations` を追加した。
後者はactive-setの試行数で、CG内側の反復数ではない。
GS反復数0は一括候補が解けたことを表す場合があり、演算費用がゼロという意味ではない。
公開structリテラルを外部コードで作る場合は新フィールドの指定が必要。

## 先行して棄却した試作

- 偶数回のGS反復で接触の処理順を反転する方法は、横運動4段の箱間めり込みが
  1.08e-5 mに増え、既存の1e-5 m基準を超えたため不採用。
- 静止摩擦一括解法に加え、8反復ごとに法線荷重をまとめて補正する方法も試した。
  全接触残差が5%以上下がる候補だけ採用する設計だったが、今回の全4ケースの
  最終監査結果は一括解法のみの試作と一致し、計時は増加した。内部採用回数は計測していない。
  観測できる残差改善がなく、不採用とした。

## 評価方法

最終バイナリを共用し、無効・有効を各5回交互に測る。
CPU固定・ウォームアップ後、開始順を反転して2セット保存する。
無効側にも従来の法線一括解法は含む。
接触点の詳細監査を無効にしたstepとECS同期を計時し、初期化・監査・出力を除く。
各モードの全ステップ再実行と、計時・監査の最終ハッシュ一致を検証する。
形状、摩擦、刻み幅、反復上限、正則化、許容値はモード間で同じ。

幾何・支持鎖・接触残差の基準は従来のまま。失敗をレポートから除かない。
候補の検証成功と、0.2秒のシーン全体の品質合格は別である。
過去のステップで力積配分が変わるため、GSに戻った後の誤差が増える可能性もある。
短時間試験であり、長期安定性やロボット操作、MuJoCoに対する優位性は未評価。

## 結果

100ステップ中の外部接触残差不合格数。A/Bとも品質結果は一致。

| シーン | 無効→有効 | 最大荷重法線速度残差 (m/s)、無効→有効 |
|---|---:|---:|
| tower-slide 4 | 78 → 29 | 0.000766869 → 0.000783834 |
| tower-slide 8 | 78 → 56 | 0.00153693 → 0.00175101 |
| tower-tilt 4 | 84 → 80 | 0.000550661 → 0.000550424 |
| tower-tilt 8 | 84 → 80 | 0.000948026 → 0.00100862 |

全4ケースで幾何・支持鎖は合格するが、残差・収束を含む総合品質は未達。
有効側で証明成功なのに外部の速度・力積検査が失敗したステップは0。
静止積層は従来の法線一括解法で成功する。橋は64接触を超えて対象外で、残差不合格78ステップのまま。

計時の中央値（実時間秒 / シミュレーション秒）。正の短縮率は高速化。

| シーン | A 無効→有効 | B 無効→有効 | 短縮率 A / B |
|---|---:|---:|---:|
| tower-slide 4 | 0.22781 → 0.20293 | 0.37312 → 0.39871 | +10.9% / -6.9% |
| tower-slide 8 | 1.87583 → 1.91757 | 2.70876 → 2.96743 | -2.2% / -9.5% |
| tower-tilt 4 | 0.25571 → 0.26321 | 0.31824 → 0.30151 | -2.9% / +5.3% |
| tower-tilt 8 | 1.68401 → 2.75813 | 1.51473 → 1.87539 | -63.8% / -23.8% |
| tower 4 | 0.04302 → 0.05164 | 0.03516 → 0.03491 | -20.0% / +0.7% |
| tower 8 | 0.15041 → 0.13060 | 0.09937 → 0.11862 | +13.2% / -19.4% |
| tower 16 | 0.47418 → 0.47496 | 0.35472 → 0.36677 | -0.2% / -3.4% |
| bridge 15 | 3.65628 → 2.67645 | 5.58511 → 4.00870 | +26.8% / +28.2% |
| bridge 31 | 15.07067 → 10.87120 | 21.26468 → 20.48315 | +27.9% / +3.7% |
| separated 1 | 0.00425 → 0.00552 | 0.00689 → 0.00495 | -30.0% / +28.1% |

対象外の橋や単一箱でも大きな計時差があり、ホスト負荷の影響を切り離せない。
横運動4段もAは短縮、Bは増加する。全体の高速化や品質・コスト両面の勝利は主張しない。

## 検証と再現

- Rust: dynamics 69件 + native 63件、計132件合格。対象2crateのrelease Clippy合格。
- Pythonベンチマーク検査114件合格。fmt・境界lint合格。
- 既定設定17ケースで#492の全監査記録と一致（計時のみ除外）。
- 最終再ビルドのexampleは測定バイナリとバイト一致。
- workspace全体のCIはディスク不足のため未検証。

```bash
cargo build --release -p rne_physics_native --example contact_crowd
python scripts/contact_benchmark/contact_sticking_compare.py target/release/examples/contact_crowd /tmp/sticking-a.json
python scripts/contact_benchmark/contact_sticking_compare.py target/release/examples/contact_crowd /tmp/sticking-b.json --first after
python scripts/contact_benchmark/contact_sticking_compare.py target/release/examples/contact_crowd /tmp/sticking-a.json --verify
```

[比較A](evidence/sticking-opt-a-summary.json)・[比較B](evidence/sticking-opt-b-summary.json)には全計時と品質判定を保存。
[provenance](evidence/contact-sticking-provenance.json)にソース・生データ・バイナリのSHA256を記録。
生データはワークスペース内の一時保管で、恒久的な公開URLはない。要約だけでは全ステップの再検査はできない。

棄却試作は `docs/evidence/contact-*-trial-summary.json`、同名 `.patch` と `.rs` に保存。
試作再現は基準 `afe4421b` にpatchを適用し、該当する `.rs` を
`crates/rne_dynamics/src/contact/sticking.rs` に置く（交互GSには追加ファイルなし）。

次の課題は滑りを含む候補解法と棄却時の費用削減。最大残差とコストの両方が
安定して改善するまで、この実験機能を既定にしない。

後続: [残差停止・seed開始・GS途中再試行の棄却結果](HARD_CONTACT_STICKING_FOLLOWUP.md)。
