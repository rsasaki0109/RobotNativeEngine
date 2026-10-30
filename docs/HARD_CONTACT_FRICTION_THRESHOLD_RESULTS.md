# H5 静止摩擦の前後での一定重心荷重

2026-10-30。静止した箱に相当水平荷重 `F = r μ m g` を与える32条件は、
すべて今回の品質ゲートを通過した。`r = ±0.8` では静止を維持し、
`r = ±1.2` では正負両方向で解析解どおりに滑走した。
これは閾値の前後20%での分類試験であり、限界値そのものの精密同定ではない。

## 荷重の与え方と範囲

この測定時点ではNative backendは `apply_external_body_wrench` を未実装だった。
後続の[外力API実装と検証](NATIVE_EXTERNAL_WRENCH.md)とは測定方法を区別する。
この試験は単一の動的箱と固定床なので、世界の加速度を
`(r μ g, -g, 0)` とすることで、箱の重心に一定水平力 `F` を加えた運動を再現する。
固定床は加速しない。均一な体積力は重心まわりの追加モーメントを生じない。
ここで検証するのは接触摩擦の応答であり、汎用外力API、力の適用点、
外力の一ステップ消費、押し手と箱の接触はこの方法では検証していない。

一辺0.1 mの立方体、質量0.3/3 kg、摩擦係数0.2/0.5、荷重比±0.8/±1.2、
正則化0/0.001の32条件。床と箱は同じ摩擦係数、反発ゼロ、初速ゼロ、
刻み2 ms、反復上限200、時間2秒。摩擦係数のf32表現を解析式にも反映する。
箱は床の十分内側を移動し、今回の係数では理想接地圧中心が支持面内にある。

解析加速度は `sign(r) max(|r|-1,0) μg`。
全時刻で位置 `a t²/2` と速度 `a t` に照合する。
外力の仕事を差し引いたエネルギー残差 `E(t)-E(0)-F x(t)` も測る。
静止するべき条件は位置誤差だけでなく、ずれと速度の厳しいゲートで監査する。

固定ゲートは、最大めり込み10 µm、軌道位置誤差3 mm、速度誤差0.002 m/s、
静止条件でのずれ10 µm・速度0.1 mm/s、角速度0.01 rad/s、横ずれ10 µm、
仕事補正後のエネルギー超過率10⁻⁶（初期の鉛直位置エネルギーで正規化）。
初期姿勢と時計を検証し、幾何は姿勢から全頂点を計算する。

## 結果

- 全32条件合格。静止16条件の最大ずれ **0.0914 µm**、最大速度 **0.0211 mm/s**。
- 全条件の最大速度誤差 **9.81×10⁻⁶ m/s**、最大めり込み **0.2943 µm**。
- 最大位置誤差 **1.9621 mm**。2秒の一定加速を2 msのsemi-implicit Eulerで積分した
  理論的な離散化差 `a dt T / 2` と整合する。

各条件は監査を2回実行して完全一致し、ウォームアップ後5回の新規ワールド計測は
監査の最終状態と一致する。計算費用はbackendバッチのみ記録する。
この結果にMuJoCoとの速度比較や新しいソルバーの性能改善は含まない。

## 再現

```bash
cargo build --locked --release -p rne_physics_native --example friction_threshold
python scripts/contact_benchmark/threshold.py target/release/examples/friction_threshold /tmp/threshold.json.xz
python scripts/contact_benchmark/threshold.py target/release/examples/friction_threshold /tmp/threshold.json.xz --verify
RNE_THRESHOLD=target/release/examples/friction_threshold python -m unittest discover -s scripts/contact_benchmark -p test_threshold.py -v
```

[集計とハッシュ](evidence/friction-threshold/h5-summary.json)をGitに保存。
生データ `friction-threshold-h5.json.xz` は別成果物として提供し、ADR034に従って
Gitへ追加しない。恒久公開URLは未取得。CIは新しく採った全データを保存する。
全記録の再計算、両方向の解析解・誤方向・誤滑走のテストとNative統合テストを追加した。
ソルバー本体の変更はない。全workspaceチェックは既知のディスク不足で再実行していない。
次の実装対象はNativeへの汎用外力API接続と、一ステップだけの力・適用点モーメントの検証。
