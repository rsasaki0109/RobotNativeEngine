# SO-101の肩旋回応答差：比較ランナーの減衰条件を修正

2026-10-29。対象はRNE/RapierとMuJoCo 3.9.0の固定台座・無重力・接触なし比較。

## 結論

以前の比較ランナーは、URDFから`dynamics`を除去すれば損失ゼロになると扱っていた。
しかしRNE/Rapierは`JointPassiveDynamics`がない関節について既存の数値減衰を維持する。
このため、MuJoCoだけが明示的に損失ゼロになっており、比較条件が一致していなかった。
トルク比較ランナーの全可動関節に減衰・摩擦ゼロを明示すると、shoulder_pan入力時に
刻みを細かくしても縮まらなかったwrist_rollの応答差が大幅に減った。

修正対象は`--torque-benchmark`の初期化のみ。一般のURDF読み込みや接触世界の減衰設定は変更していない。
これは独自物理ソルバの改善やMuJoCoに対する優位性の実証ではなく、比較条件の修正である。

## 修正前後の測定

同一モデル、20–60 msの正トルク、60–100 msの負トルク、200 msまでの全軌跡を比較した。
表は全観測関節・全時刻の最大角度差。単位はµrad（10⁻⁶ rad）。

| 振幅 N·m | 刻み ms | 修正前 | 修正後 |
| ---: | ---: | ---: | ---: |
| 0.001 | 1 | 106.211 | 2.044 |
| 0.001 | 0.5 | 106.211 | 1.114 |
| 0.001 | 0.25 | 106.211 | 0.590 |
| 0.01 | 1 | 1063.554 | 20.368 |
| 0.01 | 0.5 | 1063.459 | 10.078 |
| 0.01 | 0.25 | 1063.278 | 5.051 |

0.001 N·m、1 msの最初の駆動ステップにおけるwrist_roll加速度（速度差分）は、
修正前0.1102661、修正後0.1106602、MuJoCo 0.1106601 rad/s²だった。
全関節の加速度・角度・速度・入力・実行ファイルのハッシュは
[6条件の修正前後記録](../examples/116_robot_workbench/python/results/so101_pan_zero_loss.json.gz)に保存した。
修正前実行ファイルは`179a4786`時点の比較実装で、同一ホスト・同一モデルで測定した。

根拠となるコードは`backend.rs`の`passive_viscous_damping`と、Rapierの
`MultibodyJoint::default_damping`。単一ローターの解析解テストだけでは、
関節を連結したときの既定減衰を検出できていなかった。
新しい回帰テストはSO-101全体で「dynamics省略」と「ゼロを明記」の軌跡が一致することを確認する。
このテストは修正前バイナリで失敗し、修正後で成功する。MuJoCoの有無に依存しない。

残る差は刻みを細かくすると減少する。ただし積分方法と浮動小数点精度が異なるため、
完全一致や原因の完全な分解を主張しない。一般の姿勢・重力・関節制限・接触は次の評価対象。

## 再現

```sh
cargo build -p robot_workbench --no-default-features
.venv-gym/bin/python examples/116_robot_workbench/python/diagnose_pan_response.py \
  target/debug/rne-workbench assets/robots/so101/so101.urdf /tmp/pan-audit.json.gz
# 修正前の実行ファイルがある場合は --reference-binary /path/to/old/rne-workbench を追加
.venv-gym/bin/python examples/116_robot_workbench/python/benchmark_mujoco.py \
  target/debug/rne-workbench assets/robots/so101/so101.urdf /tmp/zero-loss.json.gz
.venv-gym/bin/python examples/116_robot_workbench/python/benchmark_mujoco.py \
  target/debug/rne-workbench /tmp/zero-loss.json.gz --replay
```

出力先は未使用のパスを指定する。MuJoCo等の依存関係は[Workbench](ROBOT_WORKBENCH.md#so-101-torque-response-against-mujoco)を参照。
修正後14条件の記録は[so101_zero_loss_cross_sim.json.gz](../examples/116_robot_workbench/python/results/so101_zero_loss_cross_sim.json.gz)。
旧`so101_cross_sim.json.gz`と既存の図・6ページ資料は、調査の経緯を残す修正前の測定記録。
旧記録は修正後バイナリでは完全再生できない。現行の結果には修正後記録を使い、完全再生は同一環境で生成した記録で行う。
