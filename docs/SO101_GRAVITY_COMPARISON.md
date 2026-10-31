# SO-101：重力を加えたトルク応答比較

2026-10-29。開発計画Bのうち、重力の共通条件を追加した段階。
RNE/RapierとMuJoCo 3.9.0に同じ世界座標の重力 `[0, 0, -9.81] m/s²` を与える。
固定台座、初期角度・速度ゼロ、接触なし、関節制限なし、受動損失ゼロ。
[既定減衰の修正](SO101_ZERO_LOSS_DIAGNOSIS.md)を含むランナーを使用する。
`rne_physics_native`や実機との比較ではない。

## 独立した基準による確認

重心が回転軸からX方向に0.1 mずれた、質量1 kg・重心慣性0.02 kg·m²の
1自由度振り子を使う。回転軸はYで、実効慣性は `0.02 + 1 × 0.1² = 0.03 kg·m²`。
初期姿勢での重力トルクは `+0.981 N·m`、初期角加速度は `32.7 rad/s²` になる。

0.25 msの最初のステップで速度差分から角加速度を求め、両エンジンで解析値との
差が0.001 rad/s²以内になることをテストする。重力を+Zに反転すると符号が反転し、
重力を回転軸に平行な-Yへ変えると角加速度ゼロになることも検証する。
これは初期加速度と軸方向のテストであり、長時間のエネルギー保存を保証するものではない。

## SO-101の測定

入力は以前と同じ、各関節への+0.001 N·m（20–60 ms）、−0.001 N·m（60–100 ms）、
その後200 msまで無入力。無入力1条件と6関節入力を1 ms／0.5 msで計14条件実行した。
重力は入力区間にかかわらず全ステップで作用する。

| 指標 | 1 ms | 0.5 ms |
| --- | ---: | ---: |
| 全14条件中、各刻みの最大角度差 | 0.00614635 rad | 0.00306885 rad |
| 無入力条件の最大角度差 | 0.00612511 rad | 0.00305543 rad |

最大値は全観測関節・全時刻で集計する。無入力条件の最大差はwrist_flexに現れた。
刻みを半分にすると差もおおむね半減するが、これは一致性の測定であり、
どちらかの正確さや接触性能・学習性能の優位性を示すものではない。
制限を外したモデルなので、関節が実機の可動域内に留まることも要求していない。

全入力、位置・速度軌跡、差分指標、重力ベクトル、モデル・実行ファイルのハッシュは
[測定記録](../examples/116_robot_workbench/python/results/so101_gravity_cross_sim.json.gz)に保存した。

## 再現

```sh
cargo build -p robot_workbench --no-default-features
.venv-gym/bin/python examples/116_robot_workbench/python/benchmark_mujoco.py \
  target/debug/rne-workbench assets/robots/so101/so101.urdf /tmp/gravity.json.gz \
  --gravity 0 0 -9.81
.venv-gym/bin/python examples/116_robot_workbench/python/benchmark_mujoco.py \
  target/debug/rne-workbench /tmp/gravity.json.gz --replay
```

依存関係は[Workbench](ROBOT_WORKBENCH.md#so-101-torque-response-against-mujoco)を参照。
出力先は未使用のパスを指定する。`--gravity`を省略すると従来どおり無重力。
RustのJSON入力でも`gravity_m_s2`省略時はゼロ。各成分は有限値かつ絶対値100 m/s²以下を受け付ける。
再生時は保存済み重力を使い、`--gravity`による上書きを拒否する。
CIでも重力付き14条件を生成・完全再生し、無重力の結果と一緒にアップロードする。

次は関節制限を残した共通モデルを作り、ストッパー到達時の侵入角・速度・反力の扱いを比較する。
今回の結果だけで開発計画B（重力と関節制限）の全体完了とはしない。
