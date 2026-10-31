# 法線を先に評価し、未使用の摩擦方向計算を省略する

基準は `33686953`（#487）。改善後の31箱bridgeを10回外部サンプリングした。
exclusive比率は `solve_contact_impulses` 約37%、`coupled::step_with_external` 約32%。
[関数表](evidence/contact-normal-first-profile.txt)はインライン処理を分離できず、
表示CPU時間2.572秒も実行全体と整合しない。候補選びの参考に限る。

## 変更と同値性

各接触の自己インパルスを除いた速度を、法線→接線1→接線2の順に計算する。
法線速度が非負、摩擦係数がゼロ、または法線応答が非正なら、接線2行の計算を省く。
既存の局所ソルバーが接線速度を読まずに終了する条件と一致させている。
接触候補の削除や、摩擦モデル・正則化・収束条件の変更はない。

各行内の加算順序と疎な計算のフォールバックを維持する。
3行の評価中はインパルスを更新しないため、独立した行の評価順序だけが変わる。
NaNの比較は元の分岐と同じにし、法線NaNを分離接触として扱わない。
符号付きゼロ、摩擦なし、非正の法線応答、非有限の接線中間値を単体テストに含める。
省略対象の接線評価が実行されないこと、および必要な場合の3行の評価も検査する。

## 評価方法

既存のexampleと比較スクリプトを変更せず、保存した前回バイナリと比較する。
bridgeと独立したstacks/clustersはCPU固定、ウォームアップ後5試行の交互測定。
1箱は200回の新規ワールドをまとめて9組測定し、開始順を反転した2セットを保存する。
時間はstepとECS同期のみで、初期化・監査・JSON出力を除く。
bridgeの全100ステップの姿勢を独立したOBB判定でも検査する。

大きな生データは `/workspace/artifacts` に置き、要約とSHA-256をコミットする。
永続共有URLは未整備。Nativeの前後比較であり、MuJoCoに対する優位性は未評価。

## 検証

Rust releaseライブラリテスト121件（dynamics 58、native 63）、Python接触ベンチマーク
91件成功。有限ギャップ36条件は保存済みの旧バイナリと全軌跡・接触診断が一致。
両crateのrelease all-targets Clippy（warnings禁止）、fmt、依存境界検査も成功。
ワークスペース全体CIは未検証。空き容量は約1.4GBで、過去の全体ビルドは容量不足だった。

比較中の全ステップの状態ハッシュ・接触診断・反復数・停止フラグは一致。
bridgeでは姿勢と支持辺も一致し、最大床めり込み3.90e-10 m未満、
箱間めり込み1.93e-10 m未満、未収束ステップ0を維持した。

## 計測結果

bridgeは各2セット実施した。

| セット | 箱数 | 変更前 秒/物理秒 | 変更後 秒/物理秒 | 時間短縮 |
| --- | ---: | ---: | ---: | ---: |
| A | 15 | 2.700 | 2.442 | 9.6% |
| A | 31 | 8.847 | 8.539 | 3.5% |
| B | 15 | 2.756 | 2.534 | 8.1% |
| B | 31 | 8.993 | 8.217 | 8.6% |

[bridge A](evidence/contact-normal-first-bridge-a.json)、[bridge B](evidence/contact-normal-first-bridge-b.json)。
31箱の0.2物理秒は約1.77〜1.80秒から約1.64〜1.71秒となった。
依然として実時間より約8倍遅く、この改善だけで目標達成とはしない。

小グループも全条件を掲載する。時間短縮が負なら悪化を表す。

| セット | 箱数 | 配置 | 変更前 秒/物理秒 | 変更後 秒/物理秒 | 時間短縮 |
| --- | ---: | --- | ---: | ---: | ---: |
| A | 64 | stacks | 0.956 | 0.983 | -2.8% |
| A | 64 | clusters | 2.133 | 1.977 | 7.3% |
| A | 128 | stacks | 2.575 | 2.251 | 12.6% |
| A | 128 | clusters | 14.272 | 11.149 | 21.9% |
| B | 64 | stacks | 1.010 | 0.994 | 1.6% |
| B | 64 | clusters | 2.222 | 2.091 | 5.9% |
| B | 128 | stacks | 1.967 | 1.980 | -0.6% |
| B | 128 | clusters | 4.447 | 4.479 | -0.7% |

[小グループ A](evidence/contact-normal-first-clusters-a.json)、[小グループ B](evidence/contact-normal-first-clusters-b.json)。
128箱clustersの変更前時間自体が14.27→4.45秒/物理秒と大きく変動している。
そのためセットAの21.9%短縮を再現性のある改善と主張しない。
測定環境のばらつきの原因は特定していない。64箱clustersは両セットで5.9〜7.3%短縮したが、
stacksなどには最大2.8%悪化した測定があり、全条件の無劣化は示していない。

1箱は[A](evidence/contact-normal-first-batch-a.json)で2.46%短縮、
開始順を反転した[B](evidence/contact-normal-first-batch-b.json)で0.05%悪化。
個々のペアも変動するため、明確な速度改善の根拠にはしない。
[生データ・バイナリ・ソース・プロファイルのハッシュ](evidence/contact-normal-first-provenance.json)。

再現例（リポジトリ直下、出力は未使用のパスを指定）:

```bash
/workspace/work/gym-venv/bin/python scripts/contact_benchmark/contact_bridge.py \
  /workspace/artifacts/contact-sparse-convergence-v3 \
  /workspace/artifacts/contact-normal-first-after /workspace/artifacts/normal-first-recheck.json
```

次はより深い積層や滑りを伴う連結負荷にも品質・コスト評価を広げる。
今回の2段bridgeだけでは深い積層やLeKiwi＋SO-101の操作タスクを代表できない。

続く評価: [摩擦あり4・8・16段積層の収束限界](HARD_CONTACT_DEEP_TOWER.md)。
