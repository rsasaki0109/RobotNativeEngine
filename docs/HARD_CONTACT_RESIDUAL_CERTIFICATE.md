# 停止と接触残差の合否を分ける

基準は `95a3f4d4`（#491）。従来の `converged` / `solver_converged` は、
反復間の変化量が小さくなったかを表す。実際の接触条件が解けたことを意味しない。
既存の停止条件を維持し、停止後の接触条件を表す独立したフラグを追加する。

- `ContactSolution.contact_law_satisfied`
- `ContactStep` / `CoupledStep` / `NativeContactImpulse` の `solver_contact_law_satisfied`
- example監査のフレーム集約 `all_contact_law_flags`

公開結果structへのフィールド追加であり、外部コードでstructリテラルを組み立てる場合は
新フィールドの指定が必要になる。既存の停止フラグの意味は変えない。

## 検査内容

GS経路では終了時に一度、現在の全力積を用いて `v = free + G λ` を計算する。
法線には既存の正則化と同じ `regularization * mean(diag Gii) * λn` を加える。
従って、正則化付きの場合は柔らかい法線条件を検査し、剛体の残差とは区別する。

力積・速度の有限性、非負の法線力積、摩擦円錐、閉じる法線速度、荷重点の法線速度を確認する。
接線力積が円錐内部なら滑りが小さいこと、境界なら滑り方向と力積が逆向きであることも確認する。
円錐の数値余裕は既存の相対1e-12と力積許容値の大きい方を使う。
荷重点と微小な円錐半径の分類には既存の力積許容値を使う。
GS検査の力積・速度許容値は有限・非負を必要とし、それ以外はfalseを返す。
ゼロの速度許容値は残差ゼロを要求するが、従来の停止条件には影響しない。

摩擦方向は速度単位で検査する。接線ノルムには2成分分の `sqrt(2) * velocity_tolerance` を使う。
**外部監査の絶対散逸エネルギー基準1e-10 Jとは同じ条件ではない。**
新フラグが真でも、この別のエネルギー基準に不合格のステップはありうる。
法線一括解法の経路では、既存の法線相補性・荷重点接線速度の候補検証結果を引き継ぐ。

これはソルバー行列に対する許容値付き検査であり、外部の幾何監査や接触候補の完全性の証明ではない。
同じ島の診断点には島全体の結果を伝える。関節制約を含む島では、その制約も検査対象となる。
空のフレームにおけるexampleの集約値はtrue（検査する接触点がない）とする。

## 採用しなかった試作

最初は残差が不合格ならGS反復を続ける形を試した。
4段の横運動で0.257→0.611秒/物理秒、傾きで0.307→0.678秒/物理秒となり、
15箱bridgeは2.851→11.653秒/物理秒へ悪化した。
横運動4段の最大荷重点法線速度残差は約7.67e-4 m/sのままで、最大値の改善は得られなかった。
そのため、この試作を採用せず、停止・軌跡を変えない診断追加に絞った。
試作の生データ・差分・ソースも保存し、最終版の計測とは分ける。

## 評価方法

横運動・傾きのある4/8段、静止方向の4/8/16段、15/31箱bridge、単箱を測定する。
CPU固定、ウォームアップ後、各5回の交互測定。開始順を反転した2セットを保存する。
速度測定は接触点の詳細診断を無効にし、stepとECS同期だけを計時する。
新しい島単位の検査はソルバーの出力として常に計算されるため、その追加コストは計時に含む。
各版内で全ステップの再実行と計時・監査の最終ハッシュ一致を検証する。

新フラグは、速度・力積についての外部残差基準と比較する。
絶対散逸エネルギーも含む外部監査との不一致は別に計数し、隠さない。
この変更自体は物理的な収束改善や高速化を実現するものではない。
通常反復の収束加速を次に評価する際、停止フラグだけで改善と判断しないための基盤である。

## 最終版の結果

全10条件で、追加フラグと計測時間を除く全ステップの状態・姿勢・接触診断・反復数が
旧版と完全一致した。測定順を反転した両セットで同じ結果。

| 条件 | 段数/箱数 | 新フラグ不合格 / 100 | 停止フラグ真かつ新フラグ不合格 |
| --- | ---: | ---: | ---: |
| tower-slide | 4 | 78 | 61 |
| tower-slide | 8 | 78 | 5 |
| tower-tilt | 4 | 84 | 58 |
| tower-tilt | 8 | 84 | 8 |
| tower | 4 | 0 | 0 |
| tower | 8 | 0 | 0 |
| tower | 16 | 0 | 0 |
| bridge | 15 | 78 | 78 |
| bridge | 31 | 78 | 78 |

最終版の全towerでは、新フラグが真で外部監査の速度・力積基準が不合格となるステップは0。
絶対散逸エネルギーも含めた不一致も今回は0だった。ただし試作ではエネルギー基準のみ
不合格の例があったため、両者が一般に同じ判定だとはしない。
bridgeでは幾何と従来停止基準は合格するが、新しい島単位の残差検査は78ステップで不合格。
これは検出された課題であり、物理的に解消したという報告ではない。

| セット | 配置 | 箱数 | 変更前 秒/物理秒 | 変更後 秒/物理秒 | 時間短縮（負は悪化） |
| --- | --- | ---: | ---: | ---: | ---: |
| A | tower-slide | 4 | 0.267875 | 0.280833 | -4.8% |
| A | tower-slide | 8 | 1.689537 | 1.646691 | 2.5% |
| A | tower-tilt | 4 | 0.289995 | 0.265159 | 8.6% |
| A | tower-tilt | 8 | 1.464143 | 1.475426 | -0.8% |
| A | tower | 4 | 0.034539 | 0.036832 | -6.6% |
| A | tower | 8 | 0.111793 | 0.102377 | 8.4% |
| A | tower | 16 | 0.377259 | 0.386748 | -2.5% |
| A | bridge | 15 | 2.745048 | 2.727381 | 0.6% |
| A | bridge | 31 | 8.995237 | 9.207848 | -2.4% |
| A | separated | 1 | 0.004778 | 0.005958 | -24.7% |
| B | tower-slide | 4 | 0.243903 | 0.240609 | 1.4% |
| B | tower-slide | 8 | 1.696136 | 1.778869 | -4.9% |
| B | tower-tilt | 4 | 0.271903 | 0.274723 | -1.0% |
| B | tower-tilt | 8 | 1.458447 | 1.454441 | 0.3% |
| B | tower | 4 | 0.042733 | 0.034255 | 19.8% |
| B | tower | 8 | 0.117340 | 0.107562 | 8.3% |
| B | tower | 16 | 0.399987 | 0.384890 | 3.8% |
| B | bridge | 15 | 2.820877 | 2.731883 | 3.2% |
| B | bridge | 31 | 10.256329 | 9.576039 | 6.6% |
| B | separated | 1 | 0.004686 | 0.004807 | -2.6% |

[セットA](evidence/contact-residual-flag-a.json)、[開始順反転B](evidence/contact-residual-flag-b.json)。
高速化は主張しない。追加検査の費用とホストの実行時間変動を含む観測値。
短い単箱計時ではAが24.7%、Bが2.6%遅くなったため、別途200ワールドをまとめた
9組の交互バッチを実行した。[バッチA](evidence/contact-residual-flag-batch-a.json)は1.1%悪化、
[開始順反転B](evidence/contact-residual-flag-batch-b.json)は2.4%短縮。各ペアも変動しており、
全条件の無劣化を証明する結果ではない。

## 検証と再現

Rust releaseライブラリテスト127件（dynamics 64、native 63）、Python接触ベンチマーク112件成功。
既存テストの期待値は変更しない。新規テストは微小更新と残差の区別、摩擦方向・法線・円錐条件、
正則化、非有限値、Native診断までの伝搬、計測レポートの状態・フラグ整合性を検査する。
release all-targets Clippy、fmt、依存境界検査成功。
ワークスペース全体CIは容量制約で未検証（空き約1.4 GB、過去に全体ビルドで容量不足）。

```bash
python scripts/contact_benchmark/contact_stopping_compare.py \
  /workspace/artifacts/contact-disturbed-tower-v1 /workspace/artifacts/contact-residual-flag-after \
  /tmp/contact-certificate.json
python scripts/contact_benchmark/contact_stopping_compare.py \
  /workspace/artifacts/contact-disturbed-tower-v1 /workspace/artifacts/contact-residual-flag-after \
  /tmp/contact-certificate.json --verify
```

[来歴・全ソース・バイナリ・生データのハッシュ](evidence/contact-residual-flag-provenance.json)。
大きな生データは `/workspace/artifacts/residual-flag-{a,b}.json`、不採用試作は
`/workspace/artifacts/residual-stop-a.json` に保存した。永続共有URLは未整備。

[不採用試作の全条件](evidence/contact-residual-rejected-continue.json)、
[既存ファイルへの差分](evidence/contact-residual-rejected-contact.patch)、
[試作の新規モジュール](evidence/contact-residual-rejected-residual.rs)、
[試作用比較スクリプト](evidence/contact-residual-rejected-compare.py)も保存する。
再構成時は基準コミットに差分を適用し、新規モジュールを `crates/rne_dynamics/src/contact/residual.rs`、
比較スクリプトを `scripts/contact_benchmark/contact_stopping_compare.py` として配置する。
試作のPythonテストには2件の失敗があり、最終版ではその既存テストも元の期待値で通過している。

次の課題は、検出した残差を低い費用で下げる反復改善。今回、許容値を緩めたり、
失敗フラグを合格へ置き換えたりして物理品質が改善したようには扱わない。
