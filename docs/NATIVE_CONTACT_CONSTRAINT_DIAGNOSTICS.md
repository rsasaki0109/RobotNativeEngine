# 接触候補の保持と拘束残差の診断

2026-10-30。Nativeの診断を拡張し、候補数の変化と解後の点速度を直接取得した。
今回の箱ではNativeは生成・重複除去・支持面削減の各段階で4点を保持した。
MuJoCoでは検出点を4点に保つ対照条件でも、拘束点が0・2・4点と切り替わり、
回転基準を超えた。観測した範囲では反復上限不足を示す結果は得られず、
次に調べる対象を拘束への採用条件と予測接触の扱いに絞る。

## Native APIの追加

[点別インパルス診断](NATIVE_CONTACT_IMPULSE_DIAGNOSTICS.md)と同じスイッチで有効化する。

- `contact_pipeline_diagnostics(world_id)`：世界全体の生成候補数、重複除去後の数、
  支持面削減後の採用数。未観測・無効時は`None`、接触ゼロの観測はゼロ件の値。
- `NativeContactImpulse.relative_velocity_world_m_s`：解後の一般化速度を
  積分前のJacobianに掛けた相対点速度。固定ベースの処方運動も加算する。
- `normal_constraint_velocity_m_s`：法線点速度にギャップ／侵入回復バイアスを加えた値。
- 摩擦係数、接触が属する島の反復数・停止フラグ。

追加のJacobian評価は診断が有効なときだけ行う。ソルバーの入力、接触の順序、
ウォームスタート、状態更新は変更しない。候補数はペア別ではなく世界全体の合計。
個々の不採用候補の位置や理由は今回のAPIに含まれない。
診断値の寿命・クリア条件は既存の点別診断と同じ。

法線拘束速度は `w = n·v + b`。ギャップが非負なら `b = gap/dt`、侵入していれば
`b = penetration_recovery * gap/dt`。正則化0の場合に、硬い法線拘束の
`w >= 0`、`lambda_n >= 0`、`lambda_n*w = 0` を調べられる。
正則化が非ゼロの場合のコンプライアンス項はこの値に含まれないため、比較スクリプトは
非ゼロの正則化を拒否する。停止フラグは独立した相補性の保証ではない。

## 今回の結果

前回と同じ箱・重心外力、最初の50 msを観測した。Nativeは2 ms刻み、
MuJoCo 3.9.0は8 µs刻み。Nativeは25ステップすべてで`4 → 4 → 4`だった。
荷重を持つ点数と候補数は区別する。

Nativeの最大値：

| 指標 | 値 |
| --- | ---: |
| 硬い法線拘束の速度違反 `max(0,-w)` | 2.0718e-8 m/s |
| 法線相補性 `abs(lambda_n*w)` | 4.5730e-10 N m |
| 摩擦円錐からのインパルス超過 | 1.7347e-18 N s |
| Gauss-Seidel反復数 | 43 / 上限200 |

全ステップで島の停止フラグが真だった。ただし収束判定は反復中の変化量であり、
独立に計算した法線速度違反が設定の速度変化許容値以下になる保証ではない。

## MuJoCoは別の最適性条件で確認

MuJoCoの柔らかい接触に、Nativeの硬い速度拘束の式をそのまま当てない。
この6自由度・elliptic cone・condim=3の条件では、拘束空間の目的関数の勾配を
`g = J*qacc - aref + R*f` として計算し、接触ごとに次を評価した。

- 主問題の摩擦円錐：`norm(f_t) <= mu*f_n`。
- 双対円錐：`g_n >= mu*norm(g_t)`。
- 相補性：`f·g = 0`。

この勾配は加速度単位。Nativeの速度残差とは単位もモデルも違うので、数値の大小で
バックエンドを順位付けしない。これは局所的な柔接触の最適性診断であり、
物理的なめり込みや長時間の姿勢安定性の合格判定とは別である。

| solref | 最大Newton反復 | 最大双対円錐違反（m/s²） | 最大相補性（N m/s²） | 回転基準超過 |
| --- | ---: | ---: | ---: | ---: |
| [0.02,1] | 4 | 3.77e-15 | 5.68e-14 | 18.944 ms |
| [0.005,1] | 5 | 7.11e-15 | 1.42e-13 | 10.208 ms |
| [2e-6,10] | 3 | 1.36e-12 | 7.45e-9 | 1.376 ms |

全条件で反復上限200に達したステップはゼロ。
今回の結果は「反復回数を増やせば解消する」という方向を支持していない。

## 検出点だけを増やす対照実験

MuJoCoの `[0.02,1]` に対し、geomの `margin=gap=0.0001 m` とした。
検出範囲は広がる一方、拘束の作用開始を決める `margin-gap` は0のまま。

| 条件 | 検出接触点数 | 拘束点数（efc_address >= 0） | 荷重点数 | 回転基準超過 |
| --- | --- | --- | --- | ---: |
| margin=gap=0 | 0・2・4 | 0・2・4 | 0・2・4 | 18.944 ms |
| margin=gap=0.1 mm | 常に4 | 0・2・4 | 0・2・4 | 41.640 ms |

対照条件でもNewton反復は最大5、双対円錐違反2.66e-15 m/s²、相補性2.84e-14 N m/s²。
候補の保持だけでは回転増加を除去できず、拘束への採用段階を調べる必要がある。
ただし軌道と超過時刻は変わったので、両条件の数値的な等価性や原因の一意性は主張しない。
50 msの対照実験であり、2秒の品質合格や速度優位も主張しない。

## 再現と検証

```bash
cargo build --release -p rne_physics_native --example friction_threshold
python scripts/contact_benchmark/contact_constraint_audit.py \
  target/release/examples/friction_threshold /tmp/contact-constraint-audit
python scripts/contact_benchmark/contact_constraint_audit.py \
  target/release/examples/friction_threshold /tmp/contact-constraint-audit --verify
```

MuJoCo 3.9.0とNumPyが必要。Nativeの有効・無効トレース一致、点別診断の完全再現、
MuJoCoの全配列の完全再現を確認した。検証器はソース・バイナリ・生配列のハッシュと
候補数・残差を再計算する。

[レポート](evidence/contact-constraint-audit.json)にはNativeの全サンプルとMuJoCoの指標を
保存した。MuJoCo生配列は `/workspace/artifacts/contact-constraint-audit-v3` に保存
（NPZ合計2,029,998 bytes）。永続公開URLはなく、大容量配列はGitに追加しない。

Rustでは16点から4点への支持面削減、正ギャップの予測接触、負ギャップの侵入回復、
診断の有無による状態ハッシュ一致を検証した。Nativeライブラリ59件、Python65件、
対象クレート全targetのrelease Clippy、fmt、依存境界・差分チェックを確認。
全workspaceのCIは未実行であり、マージ前の確認が必要。

次は拘束への採用距離とギャップバイアスを切り分ける小さな対照実験を行う。
今回観測した安定性の差を、反復回数・検出点数・拘束の有効化のどれに帰属できるかを
混同せず、同品質の性能比較へ戻れる条件を探す。
