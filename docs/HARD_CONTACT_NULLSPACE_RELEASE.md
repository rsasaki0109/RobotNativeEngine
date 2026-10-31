# 停滞した接触の荷重を解放してNewton法を再開する

基準は `a2e121f9`（#495）。同じ8固定問題・同じ独立検査で、初期力積からの合格が
5/8→6/8、GS後の力積からの合格が4/8→6/8になった。
**オフライン試作であり、Rustの実行時解法は変更しない。** 全件合格でも初期値非依存でもない。

## 停滞の観測

まずNewton方向の減衰を採用比に応じて調整する試作を行ったが、両初期値とも4/8合格にとどまった。
この試作の終了状態で、横運動4段・傾き4段・傾き8段の最大残差問題では
残差二乗和の勾配 `JᵀF` のL2ノルムが約7e-16〜1.5e-15まで小さくなる一方、
接触則の残差は残った。勾配が小さいだけでは物理的な解ではない。
16/32接触の行列は48/96行に対してrank 24/48（診断用の絶対しきい値1e-10）だった。

法線速度が正、すなわち離れる向きに動いている接触にも正の力積が残る。
全体の速度をほとんど変えずに力積配分を変えられる零空間を利用し、その接触を開く
初期値を作ってNewton法を再試行する。
この観測は4固定問題の診断であり、すべての停滞原因を説明したとは扱わない。

## 解法と上限

1. #495のNewton法をそのまま実行する。合格判定と許容値は変更しない。
2. 不合格時だけGのSVDを計算し、相対特異値1e-12未満の右特異ベクトルを零空間Nとする。
3. `vn > vt` かつ `pn > it` の各接触iについて、`N_i z ≈ p_i` を最小二乗で解き、
   `p - N z` を候補にする。法線だけでなく、その接触の力積3成分を解放する狙い。
4. 候補を非負法線・摩擦円錐へ射影し、選択接触の力積を0にする。候補の元のNewton残差を計算する。
   有限な候補のうち二乗残差が最小のものを選ぶ。同値は接触番号が小さいものを優先する。
5. その候補からNewton法を再開し、元のG・自由速度・摩擦係数に対する独立検査で合格を確認する。

`G N z` は数値的にほぼ0でも、円錐への射影と選択接触のゼロ化後には速度が変わり得る。
その前後の速度変化を別々に記録する。候補生成だけで解けたと判定しない。
候補の残差が元の停滞点より大きくても再開を許す、非単調な試作である。

全再試行を合わせた**採用Newtonステップ数は50以下**、荷重解放は最大8回。
これは演算量を#495と同じにしたという意味ではない。
追加でGのSVD1回、最大8×64候補の小規模最小二乗・射影・残差計算、
各再試行での不採用Newton方向やバックトラックが必要となる。
原理上、同じ接触を繰り返し選ぶこともある。上限で必ず打ち切り、失敗を記録する。

## 結果

先頭4種類の「最初の失敗」は従来どおり両初期値で合格（初期力積5回、GS後1回）、解放0回。
以下は各シーンの最大残差問題。括弧内は採用Newtonステップ数 / 解放回数 / 評価候補数。

| 問題 | 初期力積から | GS後から |
|---|---|---|
| 横運動4段 | 不合格 (10 / 1 / 2) | 不合格 (13 / 1 / 2) |
| 横運動8段 | 合格 (14 / 0 / 0) | 不合格 (19 / 8 / 100) |
| 傾き4段 | **新たに合格** (30 / 2 / 6) | **新たに合格** (23 / 4 / 20) |
| 傾き8段 | 不合格 (50 / 1 / 6) | **新たに合格** (27 / 4 / 20) |

各初期値で6/8が合格するが、合格する問題は一致しない。
「どちらかで解ければよい」と数えると7/8になるものの、両方を試す統合解法は実装していない。
候補生成費用もあるため、実時間の速度向上やシーン全体での改善は未評価。

横運動4段の初期力積からの結果は、法線・円錐・滑り方向の検査を満たしたが、
散逸誤差1.0903e-10 Jが既存の1e-10 J上限を超えたため不合格とした。
GS後からの結果は負の法線力積・円錐誤差が残る。
微小誤差を丸めたり許容値を緩めたりして合格にはしていない。
横運動8段のGS初期値は解放8回の上限、傾き8段の初期力積は50ステップの上限に達した。

## 再現

```bash
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_frozen_release.py \
  docs/evidence/contact-frozen-systems.json /tmp/contact-release.json
OPENBLAS_NUM_THREADS=1 python scripts/contact_benchmark/contact_frozen_release.py \
  docs/evidence/contact-frozen-systems.json /tmp/contact-release.json --verify
OPENBLAS_NUM_THREADS=1 python -m unittest discover \
  -s scripts/contact_benchmark -p test_frozen_release.py -v
```

`--verify` は依存する#495のソースも含めたハッシュ、NumPy版、全再実行結果を確認する。
保存済み結果を検証する場合は出力パスを `docs/evidence/contact-frozen-release-results.json` にする。
別のBLAS環境でビット一致を保証するものではない。

棄却した適応減衰の試作も再現できる。

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=scripts/contact_benchmark \
  python docs/evidence/contact-frozen-adaptive-damping.py \
  docs/evidence/contact-frozen-systems.json /tmp/contact-damping.json
```

新規5テストは、冗長な接触条件を持つ合成問題、射影の実現可能性、零空間なし、
上限と不正入力、実際の傾き4段問題、共有反復予算、解放無効時の一致、
再実行とレポート改変の検出を確認する。
Python全125テスト合格。全workspace CIはディスク容量制約のため未実施。
Rustソースと既定動作は変更していない。

## 証拠と次の課題

- [荷重解放の全結果](evidence/contact-frozen-release-results.json)
- [棄却した適応減衰の結果](evidence/contact-frozen-adaptive-damping-results.json)
- [適応減衰後の停滞診断](evidence/contact-frozen-stationarity.json)
- [ソースと入力のprovenance](evidence/contact-frozen-release-provenance.json)

次は、候補数の多い横運動8段の再試行と、横運動4段の散逸・円錐誤差を扱う。
特定の固定問題にだけ合わせないため、候補の選択規則を変える際には
別ステップの接触問題を評価用に追加する。シーン全体と費用の評価前に本体へ導入しない。

後続: [未使用ステップ16件での品質・Python費用比較](HARD_CONTACT_RELEASE_HOLDOUT.md)。
