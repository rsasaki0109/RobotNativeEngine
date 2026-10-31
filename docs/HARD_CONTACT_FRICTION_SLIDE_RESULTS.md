# H4 摩擦付き滑走と停止

2026-10-30。Nativeの摩擦付き接触を解析的なCoulomb滑走と比較し、
**32/32条件が今回固定した品質ゲートを通過**した。
正則化0の最大めり込みは4.90×10⁻¹¹ m、正則化0.001は0.294 µm。
最大停止距離誤差は0.500 mm、停止時間誤差は1.226 msだった。
前段で追加した摩擦ゼロ専用の一括解法は、この摩擦付きケースでは使われない。

## 条件と評価

一辺0.1 mの立方体を水平床上に静置し、X方向の初速だけを与える。
質量0.3/3 kg、初速0.2/0.5 m/s、床と箱の摩擦係数0.2/0.5、
刻み1/2 ms、正則化0/0.001、反復上限200の全直積32条件。
重心は中心、慣性 `m a²/6`、重力Y=-9.81、反発係数0、評価時間2秒。
摩擦係数はmaterialのf32表現を解析式にも反映する。

解析解は `v(t)=max(v0-μgt,0)`、停止時間 `v0/(μg)`、
停止距離 `v0²/(2μg)`。今回の摩擦係数では理想的な接地圧中心が支持面内にあり、
転倒せず並進する解と比較できる。質量を変えても理想停止距離は変わらない。

事前に固定したゲートは最大めり込み10 µm、停止距離誤差1 mm、
停止時間誤差4 ms、全時刻のX速度誤差0.01 m/s、
最後の0.5秒の速度1 mm/s・角速度0.01 rad/s、横ずれ10 µm、
初期運動エネルギーに対する全機械エネルギーの超過率10⁻⁶。
停止時刻は速度ノルムが1 mm/sを最後に超えた直後のサンプルと定義する。
停止後の再滑走も時刻に反映する。

停止距離にはsemi-implicit Eulerの離散化誤差が残る。
例として初速0.5 m/s・刻み2 msの誤差は約0.5 mmで、
小さなめり込みがそのまま小さな軌道誤差を意味するわけではない。
ゲートはこの差を分けて評価する。

姿勢から全8頂点の床めり込みを測り、回転エネルギーも含める。
各条件で監査を2回実行して全トレース一致を要求し、
ウォームアップ後の新規ワールド5回の計測では監査との最終状態一致を要求する。
一つの論理CPUに固定し、費用はbackendループのみを記録する。
今回はMuJoCo比較や摩擦解法の高速化を実施していない。

## 再現と証拠

```bash
cargo build --locked --release -p rne_physics_native --example friction_slide
python scripts/contact_benchmark/slide.py target/release/examples/friction_slide /tmp/slide.json.xz
python scripts/contact_benchmark/slide.py target/release/examples/friction_slide /tmp/slide.json.xz --verify
RNE_SLIDE=target/release/examples/friction_slide python -m unittest discover -s scripts/contact_benchmark -p test_slide.py -v
```

[全32条件の集計](evidence/friction-slide/h4-summary.json)に設定、指標、費用サンプルと
バイナリ・評価器・生データのSHA-256を保存した。
生データ `friction-slide-h4.json.xz` は別成果物として提供し、ADR034に従いGitへ
入れない。恒久公開URLは未取得。CIは新規に計測した全トレースをartifactへ保存する。
検証器は全条件の存在、全状態からの品質再計算、最終状態と費用を検証する。

解析解の成功例、誤った摩擦係数、回転・横ずれ、初期状態不整合のテストに加え、
高摩擦条件での実Native統合テストを追加した。ソルバー本体の変更はない。
全workspaceチェックは既知のディスク不足により再実行していない。
次は外力を加えて、静止摩擦の限界から滑り始める押し動作を検証する。
今回の初速度を与えた滑走試験では、押し手との接触や静止摩擦の閾値は未検証。
