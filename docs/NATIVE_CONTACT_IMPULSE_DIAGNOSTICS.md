# Nativeの点別接触インパルス診断

2026-10-30。Nativeに任意で有効化できる点別診断を追加した。
最初の50 msの滑走試験で、法線力の作用位置とモーメントの釣り合いを
MuJoCoと比較できるようになった。診断の有無で各ステップの状態ハッシュと
監査トレースが変わらないことを確認した。

## APIと寿命

```rust
backend.set_contact_diagnostics_enabled(world_id, true)?;
backend.step(world_id, dt)?;
let points = backend.contact_impulse_diagnostics(world_id)?;
```

これは `NativeBackend` の診断用APIで、共通 `PhysicsBackend` の能力宣言や
`ContactEvent` の意味は変更しない。ワールドごとに既定で無効。
無効時は診断サンプルを割り当てず、世界座標への変換も行わない。
分岐自体のコストは残るため、オーバーヘッドゼロとは主張しない。
有効時にはサンプルの割り当てと変換が増えるので、性能計測では無効にする。

`NativeContactImpulse` は、積分前の接触生成姿勢に対応する以下の値を返す。

- インパルスを受ける `body_entity` と、反対側の `other_entity`。
- 両物体のソルバー作用点 `point_world_m` / `other_point_world_m`。
- 反対側から受け側へ向く法線、符号付きギャップ。
- 法線インパルス、世界座標での接線インパルス、ステップ時間。

受け側の合インパルスは `normal * normal_impulse + tangential_impulse`。
反対側には逆符号で作用する。エンティティ順はソルバーの順であり、ID昇順ではない。
両作用点はギャップ分だけ離れ得るため、モーメント計算には対象物体側の点を使う。
中点を作用点と誤認しないよう、両方を公開した。

サンプルは重複除去・接触点削減後の、実際に解いた接触のみ。
正のギャップを持つ予測接触やゼロインパルスも残す。インパルスを `dt_s` で割ると
ステップ平均力になる。瞬時の連続時間接触力や、削減前の全接触候補ではない。

取得結果は最後に成功したステップのもの。有効化・無効化、および `sync_from_ecs`
でクリアする。`sync_to_ecs` では保持する。接触なしの成功ステップは空になる。
失敗したステップの途中サンプルは公開しないが、これは世界全体のロールバックを
新たに保証するものではない。存在しないワールドへのアクセスはエラー。

## 50 msの直接比較

前回と同じ3 kgの箱、摩擦係数0.5、重心への水平力17.658 N。
Nativeは2 ms刻み・正則化0・最大200反復で25ステップを計測した。
MuJoCoは前回保存した8 µs刻みの点別データをハッシュ検証・再集計して使用した。

| 観測 | Native（50 ms全体） | MuJoCo（時定数20 ms、margin=0、最初の回転基準超過ステップ） |
| --- | --- | --- |
| 荷重を持つ点数 | 3〜4点 | 2点 |
| 法線力の作用位置のX偏り | 約25 mm | 約50 mm |
| 法線モーメントZ | 約+0.73575 N m | 約+1.26605 N m |
| 接線モーメントZ | 約−0.73575 N m | 約−0.63302 N m |
| 合接触モーメント | 最大ノルム1.2161e-6 N m | Z成分+0.63302 N m |

Nativeでは法線・接線モーメントがほぼ相殺し、ECSで観測した角速度はゼロだった。
点別診断では微小な残留モーメントが見える。前回の「合モーメントゼロ」はECSの
速度差分から復元した値だったので、点別の直接観測値と区別する必要がある。

点別合力と速度変化の最大差は1.1469e-11 N、合モーメントとECS角速度差分の
最大差は1.0139e-6 N m。検証はステップ積分した量で、並進1e-8 N s・
角運動量1e-8 N m sの許容差を使う。ECS角速度は姿勢差分から再構成されるため、
直接の一般化速度・加速度との厳密一致を主張しない。

この比較は、Nativeが支持力の配分でモーメントを釣り合わせていることを示す。
ただし刻み・接触候補の生成法は異なり、MuJoCoの支持点切り替わりが増幅する
根本原因や、同品質の速度優位を確定したものではない。

## 再現・検証

```bash
cargo build --release -p rne_physics_native --example friction_threshold
# 最後のdiagnosticsで観測を有効化し、50 msのみ実行する。
target/release/examples/friction_threshold 2000000 3 1.2 0 200 audit 0.5 wrench diagnostics
python scripts/contact_benchmark/native_contact_diagnostics.py \
  target/release/examples/friction_threshold MUJOCO_DIAGNOSTICS_DIRECTORY native-diagnostics.json
python scripts/contact_benchmark/native_contact_diagnostics.py \
  target/release/examples/friction_threshold MUJOCO_DIAGNOSTICS_DIRECTORY native-diagnostics.json --verify
```

PythonにはNumPyとMuJoCo 3.9.0が必要。MuJoCo側の入力は
[前回の診断手順](HARD_CONTACT_FRICTION_CONTACT_DIAGNOSTICS.md)で再生成できる。
前回レポートのNativeバイナリとは異なるため、今回はMuJoCo点別トレースの
ハッシュと指標を検証し、Nativeは新バイナリで採り直している。

[点別データを含むレポート](evidence/native-contact-diagnostics.json)は約139 KBでGitに保存。
生のNative診断、初期・各ステップ状態、合力・モーメント、ソース／バイナリのハッシュ、
MuJoCo側の指標と入力ハッシュを含む。MuJoCoの大容量トレース自体は従来どおり別保管。

Rustでは有効／無効／再実行の状態ハッシュ一致、力とモーメントの釣り合い、
動的物体同士への作用反作用、ワールド分離、クリアと失敗時の寿命を検証。
Nativeライブラリ57テスト、Python60テスト、対象クレート全targetのrelease Clippy、
fmt、依存境界、差分チェックを確認した。全workspaceのCIは未実行で、マージ前に必要。

次は削減前の候補・採用点・拘束残差を揃え、支持点の切り替わりが起きる条件を調べる。
