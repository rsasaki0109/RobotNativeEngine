# 実外力による摩擦試験：共通2 ms刻み

2026-10-30。重心への実外力APIを使い、NativeとMuJoCo 3.9.0を16条件で比較した。
Nativeは全条件で品質ゲートを通過した。MuJoCoは今回の固定設定では全条件で
不合格だったが、実行コストは低かった。したがって、同品質での速度優位は未確立。

| 指標 | Native | MuJoCo |
| --- | ---: | ---: |
| 品質合格 | 16/16 | 0/16 |
| 各条件の中央値を平均した実行コスト（ms / 物理1秒） | 37.569 | 3.404 |
| 最大めり込み（m） | 9.5825e-11 | 9.6108e-4 |
| 最大位置誤差（m） | 0.0019620 | 0.0198457 |
| 最大速度誤差（m/s） | 2.1981e-7 | 0.40135 |
| 静止条件の最大移動（m） | 5.2246e-9 | 2.2206e-5 |
| 静止条件の最大速度（m/s） | 4.6039e-7 | 0.021131 |

最大値は各バックエンドの16条件全体から算出した。同品質比較が成立しないため、
集約レポートの速度比は `null` とし、比較可能フラグを `false` にした。
この結果はMuJoCoの最良設定を探索した結果ではない。

## 条件と計測範囲

一辺0.1 mの箱、質量0.3/3 kg、摩擦係数0.2/0.5、荷重比
`F / (μmg) = -1.2, -0.8, 0.8, 1.2`。世界重力は両方とも `(0, -9.81, 0)`。
2 ms刻みで2秒間、毎ステップ世界X方向の力を重心へ加える。
Nativeは `apply_external_body_wrench`、MuJoCoは重心に作用する
`xfrc_applied` を用いる。追加トルクはゼロ。摩擦係数はNativeのf32値に揃える。

Nativeは正則化0・最大200反復。MuJoCoはEuler・Newton・elliptic、最大200反復、
許容値1e-10、`solref = [2e-6, 10]`、`solimp = [0.999, 0.999, 0.001, 0.5, 2]`、
`condim=3`、ねじり・転がり摩擦ゼロ。MuJoCoの既定の安全制約は解除していない。
このsolrefは過去の積み上げ試験から固定して持ち込んだもので、摩擦向けの調整は未実施。

箱は床にちょうど接する姿勢・速度ゼロから開始する。事前静定や初期支持力の投入は
行わず、接触が立ち上がる最初のステップも品質判定に含む。MuJoCoの静止条件では
最初のステップから鉛直速度が発生しており、刻みや接触設定を調整する際にも
この過渡応答を落とさず検証する必要がある。

品質ゲートは既存H5と同一：めり込み10 µm、位置誤差3 mm、速度誤差2 mm/s、
静止移動10 µm、静止速度0.1 mm/s、回転0.01 rad/s、横ずれ10 µm、
外力仕事を差し引いたエネルギー増加率1e-6。MuJoCoの警告・時計も確認する。
今回の試験は接触インパルスをエンジン間比較していない。

計時は初期化とトレース生成を除き、外力の毎ステップ投入とステップ実行を含む。
Nativeでは移動する重心位置を取得するための毎ステップECS書き戻しも含む。
MuJoCoはPythonループで毎ステップ投入し、`nstep` の一括呼び出しは使わない。
このため結果は利用経路を含むコストであり、ソルバー単体の速度比較ではない。
CPU affinityを1コアに固定し、ウォームアップ後5回の中央値を使う。
監査トレースの完全再現と、計時時の最終姿勢・速度の一致を確認した。

## 再現・証拠

```bash
cargo build --release -p rne_physics_native --example friction_threshold
python scripts/contact_benchmark/force_friction.py \
  target/release/examples/friction_threshold force-friction.json.xz
python scripts/contact_benchmark/force_friction.py \
  target/release/examples/friction_threshold force-friction.json.xz --verify
```

Python環境にはMuJoCo 3.9.0とNumPyが必要。
集約結果・個別ゲート・5回の計時・ソースとバイナリのハッシュは
[evidence/force-friction-common-step-summary.json](evidence/force-friction-common-step-summary.json)
に保存した。生トレースはセッション成果物
`/workspace/artifacts/force-friction-common-step-v1.json.xz`（2,233,220 bytes）、
SHA-256 `b3274837d0b60d7aad15cbd17a66875e60f62ab132179aa8fe7976258b1eb17b`。
永続公開URLはなく、ADR 034に従って大容量の生データはGitに追加していない。
集約JSONだけでは生トレースからの独立再計算はできない。

44件のPython試験（既存40件と新規4件）、変更したRust exampleのrelease Clippy、
fmt、依存境界チェックを確認する。全workspace検証はローカル空き容量不足のため
未実施であり、マージ前のCIが必要。

## 次の実験

摩擦用の較正条件でMuJoCoの刻み・接触設定を調整し、全品質ゲートを通る設定を
固定してから、未使用の質量・荷重条件で検証する。Nativeの外力投入・Jacobian計算・
ECS同期も別途計測し、同品質での総コストと処理内訳を示す。今回の16条件を
後から検証用データと呼び替えない。
