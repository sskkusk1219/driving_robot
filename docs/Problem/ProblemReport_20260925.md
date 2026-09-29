# 20260925 手順2を「車両に依らない測定パターン」に作り直す

**状態: クローズ（2026-09-28）** — 段7（手順2 の計測効率化）まで完了。段7d 実機（`drive_log_real_20260928_062254.csv`）で
G 校正表が期待どおりの形になり、所要時間は目標 1800s に対し 1750s。手順3（`drive_log_real_20260928_070218.csv`）の KPI は
段2 の基準（075149）と比べて悪化なし（ユーザー判断 2026-09-28）。詳細は末尾の「段7d 実機結果と手順3 比較・クローズ」を参照。

## 問題

手順2（閉ループパターン走行 → FF 逆モデル学習）の開度は「不感帯 + 固定の%」で、実質この試験車に合わせて手で調整した値になっている。
車両が変わると（660cc ⇔ EV など、開度→加速度の感度が数倍違う）同じ%でも意味が変わり、WLTP（Low/Mid/Hi/ExHi）の網羅が保証できない。

## 議論で確認した事実

| 項目 | 内容 |
|---|---|
| FF 逆モデルの入力 | 車速・この先の車速変化。WLTP が決めるので**車両に依らない** |
| FF 逆モデルの出力 | 開度。**車両ごとに変わる** |
| 網羅性の定義 | 「車速 × 加速度」の格子で測る（開度では測らない） |
| WLTP の範囲 | 0〜131.3 km/h、−5.46〜+6.06 km/h/s（−0.155〜+0.172 G）。ログ `drive_log_real_20260924_153749.csv` の基準車速から集計 |
| この車で WLTP に要るアクセル実開度 | 最大 13.6%。今の ACCEL_SWEEP（24〜80%）は 0.4G ガバナ任せで WLTP の外を測っている |
| 閉ループ追従中の開度をそのまま学習する問題 | 制御器の反応（ばたつき）を学んでしまう（閉ループ V/U = 1/K）。→ 測る区間は**開度固定**にする |
| 2次多項式 1 本の限界 | データの偏りがそのまま当てはめの偏りになる（低開度階段で低速改善・中速悪化）。→ WLTP の分布で**重み付け** |

## 検討した案

| 案 | 内容 | 判断 |
|---|---|---|
| 案1 | 開度を多めに並べ、G ガバナ（または WLTP 最大加速度）に達したら打ち切る | 刻みが車両依存で残る。打ち切り基準を WLTP にすれば案3 の一部として使える |
| 案2 | 閉ループで車速・加速度を追従し、そのときの開度を記録する | 制御器の反応が混ざる。不採用 |
| **案3（採用）** | WLTP の車速×加速度格子を狙う。開度は較正から車両ごとに自動で決め、測定区間は開度固定 | 採用 |

## ユーザー決定（2026-09-25）

- 案3 を採用。旧パターンは**置き換え**
- 学習サンプルの WLTP 重み付けも今回の範囲
- 旧パターン前提の解析ツールは**削除**（debug_a2a5 / debug_a3a4 / stair_gain / cruise_curve / kaizen `--part coverage` / accel_onset の階段表 / C6）
- 順番は**重み付けを先**（重みの効果とパターン変更の効果を分けて見る）
- **車両に不具合があり修理したため、挙動が以前と違う。過去の CSV は学習に使えない**
  → 段2 は修理後の車で現パターンの手順2 を 1 回走って基準を取り直す（この CSV は段5 の旧パターン側にもなる）

## 全体計画

計画の詳細: `/home/raspi5_16gb/.claude/plans/docs-problem-problemreport-20260910-md-goofy-lighthouse.md`（セッション計画ファイル）
1 段ずつ実装 → Claude が確認 → ユーザーが実行して確認 → 次の段へ。テストは変更箇所のみ。

| 段 | 内容 | 走行 | 状態 |
|---|---|---|---|
| 0 | 本ドキュメント作成 | なし | ✅ 完了 |
| 1 | WLTP 格子と網羅マップ（`tests/research/wltp_grid.py`） | なし（オフライン） | ✅ 完了（ユーザー確認 OK） |
| 2 | 学習サンプルの重み付け。修理後の車で現パターンの基準を取り直し、重みなし／ありを手順3 で比較 | 実機（手順2 ×1、手順3 ×2） | ✅ 完了（重み付けに優位性なし） |
| 3 | 新パターン「格子ステップ走行」（`grid_planner.py`、`pattern_loop.py` に `GridStationPattern`・`HOLD_STEP`） | スタブ | ✅ 完了（感度 1/5 以下の車への対策は後回し） |
| 4 | 手順2 のパターン列を置き換え、旧パターン・旧ツール・旧 config キーを削除 | スタブ | ✅ 完了（ユーザー判断 2026-09-26） |
| 4b | 格子の狙いを複数モード対応にし、しきい値・発進停車・強い加減速（助走）を見直す（段5 の前の議論。下記） | スタブ | ✅ 完了（段4 と合わせてユーザー判断 2026-09-26） |
| 5 | 実機確認（新パターンの手順2 → 手順3）とレポート | 実機 | ✅ 完了（1 回目で①〜③の問題を発見 → 5b へ） |
| 5b | 段5 の 1 回目の実機走行で見つかった測り方の修正（u0 補正・ブレーキ上限・停車ステップ等） | スタブ → 実機 | ✅ 完了（0.4G 超え 1 件を確認 → 段6 へ） |
| 6 | 上限 G 対策（G 校正・先読みガバナー・踏み込みランプ）と通し掃引 | スタブ → 実機 | ✅ 完了（段6d 実機: 0.4G 超え 0 行。ユーザー判断で手順3 へ） |
| 7 | 手順2 の計測効率化（惰行ステップ廃止・GRID_RETURN 等）。実機で見つかった不具合の修正（7c・7d） | スタブ → 実機 | ✅ 完了（2026-09-28。所要 1750s、手順3 の KPI 悪化なし → クローズ） |

## 段ごとの内容

### 段1: WLTP 格子と網羅マップ

- `wltp_cells(mode, bins)`: WLTP 基準車速から (車速, 加速度) の滞在時間ヒストグラムを作る。加速度は FF の regime ホライズン（1.0s 先）との差で、学習側と同じ定義。停車は除外
- `data_cells(logs, ff_params, spec)`: 学習 CSV の「ペダルが効いている行」を同じ格子で数える
- `coverage_table(...)`: セルごとの「WLTP で要る秒 / 学習データの秒」と、穴の一覧
- 格子の既定値は config `learning.grid`（車速 10 km/h 刻み 0〜140、加速度境界 [-7,-3,-1.5,-0.5,-0.1,0.1,0.5,1.5,3,7] km/h/s）
- WLTP は既存の `mode_drive.load_mode` + `ReferenceSpeed`（DB）を再利用

### 段2: 学習サンプルの重み付け

- `ff_candidate.train_inverse_model_effective` に `sample_weight` を渡す。重み = clip(WLTP 秒 / 学習データ秒, w_min, w_max)、ペダルごとに平均 1 へ正規化
- config `learning.sample_weight`: `enabled`（既定 false）、`w_min`、`w_max`
- CV 側（`model_analysis.py`、`kaizen.py`）にも同じ重みを入れる

### 段3: 格子ステップ走行

- ステーション（車速）ごとに、狙う加速度の列を WLTP 格子から作る。開度は較正した感度から自動で決め、測定区間は開度固定（`HOLD_STEP` フェーズ）
- 打ち切りは「その車速帯の WLTP 最大加速度 × 1.2」を超えたら
- PI 保持のゲインは感度で正規化

### 段4: パターン列の置き換え

- 新しい構成: コーストダウン ×2 → 格子ステップ走行 → クリープ発進 ×3 → クリープ域ブレーキ保持（不感帯 + frac × (停車保持開度 − 不感帯)）
- 決定（2026-09-25 段4 計画時）: C6（FF 候補。定速階段の実測テーブルが骨格）は**ごと削除**。旧キーを含むバックアップ yaml は**そのまま残す**（読むと ConfigError）。
  本番 `generate_patterns` 先頭のクリープ解放 5 段＋安定待ちは入れない（クリープ平衡は 2-0、クリープ加速はクリープ発進 ×3 が測る）

### 段4b: 格子の狙いを複数モード対応にする（2026-09-26 議論）

段5 の前に、ユーザーから次の要望が出た。将来はすべてのモードで KPI を満たす必要があるので、WLTP だけでなく
モードを選べるようにし、選んだモードの走りを網羅できるようにしたい。

**決定**

1. 狙うモードを config `modes.coverage_mode_names`（リスト）で選ぶ。最初は `["01_WLTP_Low,Mid,Hi,ExHi", "09_US06"]`。
   重なるマスは 1 回だけ測る。合成は、秒数はモードごとの**最大**、平均加速度は秒数で重み付けした平均、車速帯の最大・最小加速度は全モードの最大・最小
   （秒数を合計にしないのは、2 モードで 1.5 秒ずつ使うマスが合計 3 秒で「1 ステップ分」を超えるのを避けるため）。
   手順3 で走るモードと手順2 の MAE_WLTP は従来どおり `modes.wltp_mode_name`
2. 「マスを狙う最小秒数」は 5 秒（段1 の仮置き）をやめ、**1 ステップで測れる長さ = `grid_step_window_s − grid_step_lag_s`（2.5 秒）から自動**で決める。別の数値は持たない
3. 発進・停車セルは、担当する車速範囲（0〜20 km/h）だけ固定し、そこで測る加速度は選んだモードから自動で決める（config に狙いの値は持たない）
4. 加速度の列を ±7 から **±14 km/h/s** に広げる（`vehicle.max_decel_g` 0.4 G ≒ 14.1 km/h/s。G ガバナより強くは測れない）。WLTP のマスは変わらない
5. 強い加減速は**助走**をつけて測る（下記）

**5 秒に根拠が無かった件（実測）** — 各モードで、しきい値以上のマスが走行時間の何 % を占めるか:

| しきい値 | WLTP | UDDS | HWFET | US06 | SC03 | NEDC | AMA |
|---|---|---|---|---|---|---|---|
| 2 秒 | 99.4% | 99.6% | 98.3% | 93.8% | 97.2% | 98.8% | 99.8% |
| 5 秒（旧） | 97.8% | 96.2% | 92.4% | **79.9%** | 89.4% | 93.2% | 99.4% |
| 10 秒 | 93.5% | 89.5% | 86.8% | 57.1% | 52.2% | 86.1% | 97.0% |

WLTP だけなら 5 秒でも 2% しか落とさないが、US06 だと走行時間の 2 割を対象外にする。2.5 秒にすると WLTP の狙いは 74 → 79 ステップ。

**±7 を超える走り（旧い列だと数えずに捨てていた秒数）**: US06 61 秒（加速度 −11.1〜+13.5）、SC03 16 秒、AMA 283 秒。WLTP は −5.4〜+6.0 で 0 秒。

**強いステップの助走**: 1 ステップは開度固定で最大 3 秒、ステーションから ±10 km/h の帯を出たら終わる。
中心から踏むと、+13.5 km/h/s なら 0.74 秒で帯を出て、頭の 0.5 秒を除くと 2〜3 点しか残らず傾きが測れない。
そこで「帯の中心から踏んだとき、傾きを測れる時間（帯 ÷ |狙い| − 頭の除外）が `grid_step_min_fit_s`（1.0 秒 ≒ 0.1 秒刻みで 10 点）未満」の狙い
（今の値では |狙い| > 6.7 km/h/s）を強いステップとし、加速は帯の下の端（下限は `grid_station_min_kmh`）、減速は上の端（上限は最高車速の手前）で
PI 保持して落ち着いてから踏む。基準開度はその車速で落ち着いた開度 u0'。帯の反対側の端に出たら終える。助走で落ち着かなければそのステップだけとばす。
US06 を入れると、ステップは 79 → 98、うち強いステップは 9 本（15〜55 km/h の ±8〜9 km/h/s）。発進・停車は WLTP のみの
発進 [0.98, 2.22, 4.17]・停車 [−0.99, −2.16, −4.03] が、US06 込みで発進 [0.96, 2.23, 4.3, 9.39]・停車 [−0.98, −2.15, −4.4, −7.98] km/h/s になる

**調整値の根拠一覧**（新しい値は、データ由来か仮の値か・何に効くか・いつ決めるかを明示する）:

| 値 | 現在 | データの根拠 | 何に効くか | いつ決めるか |
|---|---|---|---|---|
| `grid_settle_tol_kmh` | ±1.0 km/h | **なし**（段3 の仮の値）。旧定速階段の PI 保持（実機 2 本）は車速が σ 0.5〜1.2 km/h 揺れ、±1 に 3 秒続けて入るまで 4.5〜20 秒ばらついた | 各ステップの前に毎回入る待ち時間（約 100 回）と、u0 の精度 | 段5 の実機データ（2-1 の終わりの「落ち着き判定の集計」）。ユーザー決定: それまで据え置き |
| `grid_settle_s` | 3.0 s | **なし**（仮）。3 秒窓の平均開度のばらつきは σ 0.06〜0.38%（一番弱い加速の踏み増し 約 0.8% に対し最大で約半分） | 同上 | 同上 |
| `grid_step_window_s` | 3.0 s | **なし**（仮） | 1 ステップの長さ = 傾きを当てはめる点数。狙う最小秒数の元 | 段5（同じ集計の「ステップの当てはめ」） |
| `grid_step_lag_s` | 0.5 s | **なし**（仮。ペダル・応答の遅れの目安） | 同上 | 同上 |
| `grid_step_band_max_kmh` | 10 km/h | **なし**（仮） | ステップで動ける車速幅（助走の判定にも効く） | 同上 |
| `grid_step_min_fit_s` | 1.0 s | 設計値（0.1 秒刻みで最低 10 点） | 助走を付ける境目（|狙い| > 6.7 km/h/s） | US06 の実機データ |
| 狙う最小秒数 | 2.5 s（自動） | 1 ステップで測れる長さ。5 秒だと US06 の 2 割を対象外にした（上の表） | 狙うマスの数・所要時間 | window・lag を決め直せば自動で追従 |
| 加速度の外側の端 | ±14 km/h/s | あり: `vehicle.max_decel_g` 0.4 G ≒ 14.1 km/h/s | 数えられる走りの範囲 | — |

### 段5: 実機確認

- 網羅表で WLTP のセルに穴が無いこと、手順3 を段2 の基準と比べる（1 本目は暖機で +0.13 km/h ずれるのでイベント単位で比べる）

## 実行手順（ユーザー実行）

各段が終わるたびにここへ追記する。

### 段1（走行なし・実機不要）

```bash
# 手順2 の CSV と、WLTP の基準車速を持つ CSV（手順3 のモード走行 CSV でよい。DB 不要）
.venv/bin/python -m tests.research.wltp_grid tests/research/results/drive_log_real_20260924_161325.csv \
    --ref-csv tests/research/results/drive_log_real_20260924_153749.csv
# DB の WLTP を使う場合は --ref-csv を外す
```

見方: セルは「WLTP が要る秒 / 学習データの秒」。`*` は穴（WLTP ≥ 5s かつ 学習 < 2s）。
単体テスト: `.venv/bin/python -m pytest tests/research/test_research_wltp_grid.py -q`

### 段2（重み付けの比較。実機）

```bash
# 1) 同じ手順2 CSV から、重みなし／ありの当てはまりを比べる（config は書き換えない）
.venv/bin/python -m tests.research.relearn tests/research/results/drive_log_real_20260925_070629.csv \
    --dry-run --weight off --ref-csv tests/research/results/drive_log_real_20260925_075149.csv
.venv/bin/python -m tests.research.relearn tests/research/results/drive_log_real_20260925_070629.csv \
    --dry-run --weight on  --ref-csv tests/research/results/drive_log_real_20260925_075149.csv
# 2) 重みありで作り直して保存（model_path 等を書き換える。2-0 の実測値は書き戻さない）
cp tests/research/config_testVehicle.yaml tests/research/results/config_testVehicle_before_20260925_weighted.yaml
.venv/bin/python -m tests.research.relearn tests/research/results/drive_log_real_20260925_070629.csv \
    --weight on --ref-csv tests/research/results/drive_log_real_20260925_075149.csv
# 3) 重みありで手順3（実機・約 35 分）
.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"
```

- `--weight on` はその実行だけ有効にする（config の `learning.sample_weight_enabled` は false のまま）
- 単体テスト: `.venv/bin/python -m pytest tests/research/test_research_sample_weight.py -q`

### 段4（スタブ。走行は実機不要）

```bash
R=tests/research/results/drive_log_real_20260925_075149.csv   # 基準車速を持つ CSV（DB 不要）
# 手順2 の全体構成（コースト → 格子 → クリープ）をスタブで走り、モデル作成（YAML は更新しない）まで通す
.venv/bin/python -m tests.research.grid_stub_run --ref-csv $R --full --stations 15 --gain-scale 5
# 単体テスト（変更箇所のみ）
.venv/bin/python -m pytest tests/research/test_research_pattern_drive.py tests/research/test_research_pattern_loop.py \
  tests/research/test_research_pattern_loop_grid.py tests/research/test_research_grid_patterns.py \
  tests/research/test_research_config.py tests/research/test_research_ff_candidate.py \
  tests/research/test_research_train_candidate_model.py tests/research/test_research_accel_onset.py \
  tests/research/test_research_kaizen.py tests/research/test_research_stop_brake_floor.py -q
.venv/bin/ruff check tests/research
```

見方: パターン一覧が「COAST_DOWN ×2 → GRID_STEP → GRID_LAUNCH → クリープ発進 ×3 → ブレーキ保持 ×8」になっていること、完走すること、
最後の 2-2 で惰行カーブ・クリープカーブ・停止下限・ペダルゲインの推定行が出ること（スタブなので数値の良し悪しは判断しない）。
既知の失敗 `test_stop_brake_floor_settings_are_validated`（作業ツリーの yaml `stop_brake_floor_offset_pct`=0.5）は段4 と無関係。

### 段5（実機。ユーザーが別ターミナルで実施）

```bash
# 1) 手順1 → 走行前チェック → 手順2（ペダル探索 → 新パターン列の走行 → モデル作成。実機なので config へ書き戻す）
.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"
# 2) 結果確認（Claude と議論）: 2-1 の終わりの「落ち着き判定の集計」・網羅表・所要時間から
#    grid_settle_tol_kmh・grid_settle_s・grid_step_window_s などの値と根拠を確定する（上の「調整値の根拠一覧」）
# 3) 問題なければ手順3（1 で書き戻したモデルで FF のみの WLTP 走行）
.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"
```

見るもの: 所要時間（`learning.timeout_s` 3600s に収まるか。US06 込みでステップ 98 本＋助走）、落ち着き判定の集計（待ち時間の合計・打切りの有無・開度の σ）、
強いステップの結果（助走つきの行が OK になるか、G ガバナで打切りになるか）、網羅表（WLTP + US06 の穴）。手順3 は段2 の基準（075149）とイベント単位で比べる。

### 段5b（手順2 の再走。ユーザーが実機で実施）

```bash
# 単体テスト（変更箇所のみ）
.venv/bin/python -m pytest tests/research/test_research_grid_planner.py tests/research/test_research_pattern_loop_grid.py \
  tests/research/test_research_grid_settle.py tests/research/test_research_pattern_loop.py -q
.venv/bin/ruff check tests/research
# 手順1 → 走行前チェック → 手順2（実機。config へ書き戻す）
.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"
```

見るもの（コーストダウン G。手順2 の先頭）: cap への到達時間・加速中の G の最大（0.3G 未満か）・G ガバナ作動・開度が 60 km/h あたりから 70% で頭打ちか。
見るもの: ブレーキ指令の最大（15 km/h の強い減速で 80% に張り付かないか）と G ガバナの作動、停車ステップの所要時間（クリープで 26 s 以上走り続けないか）、
定速ステップの実測が 0 に近づいたか、集計の「窓の傾き」、やり直し・未達・打切りの数（2 倍の値を決め直す材料）。問題なければ手順3（段2 の基準 075149 とイベント単位で比べる）。
既知の失敗: `test_research_config.py` の 3 件（`test_stop_brake_floor_settings_are_validated` ほか 2 件）は、実機走行が書き換えた yaml（`stop_brake_floor_offset_pct`・`onset_accel_kmhs`）を見るテストで今回の変更とは無関係。

### 段5 で走る手順2 の流れ（新パターン列。2026-09-26 時点）

`main --steps 1,2 --hw real` は、手順1（初期化）→ 走行前チェック → 手順2 を続けて走る。手順2 の中身:

| 段 | 内容 | 決まり方 |
|---|---|---|
| 準備 | `modes.coverage_mode_names`（既定 WLTP + US06）の全モードを DB から読み、車速 × 加速度の格子（車速 10 km/h 刻み 0〜140、加速度 −14〜+14 km/h/s の 9 列）に合成。読めなければ走る前に止まる。モードごとの「格子外 N 秒」を表示 | 重なるマスは 1 回だけ（秒数は最大） |
| 2-0 ペダル探索 | クリープ中に 0.5 mm ずつ踏み、アクセル・ブレーキの不感帯と停止確認開度を測る → 停車保持開度を決めて停車保持。実機は config へ保存 | 実測 |
| 2-1 ① コーストダウン ×2 | アクセル（最大 70%）で最高車速付近まで上げて両ペダル 0% で惰行し、5 km/h まで減速（惰行カーブ用） | 車速で終わる（車両非依存） |
| 2-1 ② 格子ステップ走行 | 車速ステーション（10〜20, 20〜30, … の帯の中心 15, 25, …, 125 km/h。狙うマスが無い帯は除く）を昇順に、下の手順を繰り返す | 開度は走行中に自動 |
| 2-1 ③ 発進・停車セル | 停車から固定アクセルで 20 km/h まで発進（LAUNCH）→ そのまま固定ブレーキで停車（STOP）を、狙いの加速度ぶん繰り返す | 狙いは選んだモードの 0〜20 km/h から自動 |
| 2-1 ④ クリープ発進 ×3 | 両ペダル解放で自走させ、クリープ平衡（車速の傾きが 0.1 km/h/s 未満が 2 秒続く）まで。通常の停車復帰（クリープカーブ用） | 実測 |
| 2-1 ⑤ クリープ域ブレーキ保持 ×8 | クリープ中にブレーキを 不感帯 + frac × (停車保持開度 − 不感帯) で保持（frac = 0.05〜1.0）。停止ブレーキの下限用 | `learning.creep_brake_hold_fracs` |
| 走行後 | アクセル 0% → 0.5 mm 刻みで 0.2G 付近の緩減速 → 停車保持 | 実測 |
| 2-2 モデル作成 | 2次多項式 + 標準化 + Ridge の逆モデル（アクセル/ブレーキ）、惰行カーブ・クリープカーブ・停止下限・ペダルゲインを推定して config へ書き戻す（MAE_WLTP は `modes.wltp_mode_name`） | — |

**②格子ステップ走行の 1 ステーション（車速 V）の流れ**

1. 停車から（または前のステーションから止まらずに）PI で V に保つ。V の ±1 km/h に 3 秒いたら「落ち着いた」とし、その 3 秒間のアクセル開度の平均を定常開度 u0（＝その車速で加速度 0 の開度）とする
2. 開度を固定して最大 3 秒走り、車速の傾き（頭の 0.5 秒を除く最小二乗）を加速度として測る。ステップの順は次のとおり。1 ステップ終わるごとに u0 へ戻して落ち着き直す
   - 定速（u0 のまま）→ 惰行（両ペダル 0%）
   - 減速の列: 緩い順。惰行より緩い狙いはアクセルを u0 から不感帯へ向けて絞り、強い狙いはブレーキ（不感帯 + (惰行の a − 狙い a) ÷ ブレーキ感度）
   - 加速の列: 小さい順。u0 + 狙い a ÷ アクセル感度
3. 感度（開度 1% あたりの加速度）は測るごとに割線で更新して次へ引き継ぐ。狙いのセルに入らなければ更新した感度で 1 回だけやり直す。
   実測が別の狙いのセルに入ったらそのセルも測れたことにしてとばす。実測が「その車速帯のモードの最大（最小）加速度 × 1.2」を超えるか、G ガバナが働いたら、その向きの残りを打ち切る
4. 強いステップ（|狙い| > 6.7 km/h/s）は助走をつける: 加速は帯の下の端（下限 10 km/h）、減速は上の端まで PI で移って落ち着いてから踏み、帯の反対側へ出たら終える。助走で落ち着かなければそのステップだけとばす
5. ステーションが終わったら止まらずに次のステーションへ。最後の後は停車復帰（DRIVE_BRAKE）
6. 車速が底まで落ちても残りのステップは捨てず、PI で戻って続ける

2-1 の終わりに「落ち着き判定の集計」（待ち時間・許容幅内の割合・開度のばらつき・ステップの当てはめ点数）が出る。段5 でこれを見て
`grid_settle_tol_kmh`・`grid_settle_s`・`grid_step_window_s` を決め直す。

**段5 の記録・採用の段取り（ユーザー決定 2026-09-26）**: 段5 が完了したら、結果と確定した設定値・根拠を RPD（`docs/Phase/RPD_*.md`）に記載し、この手順2 を正式採用とする。それまでは本ドキュメント（ProblemReport）の記録のみ。

### 段4b（スタブ。走行は実機不要。DB の 09_US06 を読むので `--ref-csv` は付けない）

```bash
# 手順2 の全体構成を、WLTP + US06 の合成でスタブ走行（15 km/h ステーションだけ。約 15〜20 分）
.venv/bin/python -m tests.research.grid_stub_run --full --stations 15 --gain-scale 5
# 網羅表（WLTP + US06 の合成。格子外の秒数も出る）
.venv/bin/python -m tests.research.wltp_grid tests/research/results/drive_log_stub_20260925_151114.csv
# 単体テスト（変更箇所のみ）
.venv/bin/python -m pytest tests/research/test_research_wltp_grid.py tests/research/test_research_grid_planner.py \
  tests/research/test_research_grid_settle.py tests/research/test_research_grid_patterns.py \
  tests/research/test_research_pattern_loop_grid.py tests/research/test_research_pattern_loop.py \
  tests/research/test_research_config.py tests/research/test_research_pattern_drive.py -q
.venv/bin/ruff check tests/research
```

見方: 走行の最初に「狙うモード（格子外 N 秒）」、パターン一覧の 15 km/h ステーションが「減速 4 列・加速 4 列」、発進・停車が「発進 4 列・停車 4 列」になること。
1 行ずつの格子ステップに、強いステップ（加速 +9 / 減速 −8 km/h/s）で「助走 10km/h」「助走 25km/h」が付くこと。走行の終わりに「落ち着き判定の集計」の表が出ること。
スタブは応答が実車の約 1/5 なので「未達」「打切り」が出てよい（数値の良し悪しは判断しない）。
既知の失敗 `test_stop_brake_floor_settings_are_validated`（作業ツリーの yaml `stop_brake_floor_offset_pct`=0.5）は段4b と無関係。

### 段3（スタブ。走行は実機不要）

```bash
R=tests/research/results/drive_log_real_20260925_075149.csv   # 基準車速を持つ CSV（DB 不要）
# 1 ステーションだけ（約 6 分。スタブは応答が実車の約 1/5 で遅い）
.venv/bin/python -m tests.research.grid_stub_run --ref-csv $R --stations 15 --no-launch
# 発進・停車セルまで
.venv/bin/python -m tests.research.grid_stub_run --ref-csv $R --stations 15 --gain-scale 5
# 感度が違う車でも流れが壊れないか（制御性能はスタブで判断しない）
.venv/bin/python -m tests.research.grid_stub_run --ref-csv $R --stations 15 --no-launch --gain-scale 0.2
```

見方: 1 ステップごとに 1 行「車速 / 種類 / 狙い a / ペダル・開度 / 実測 a / 感度 g の更新 / 判定」。
最後に感度 g の推移（スタブの真値付き）と、網羅マップ（開度固定のステップの行だけ／全行）が出る。
単体テスト: `.venv/bin/python -m pytest tests/research/test_research_grid_planner.py tests/research/test_research_pattern_loop_grid.py tests/research/test_research_grid_patterns.py -q`

## 段3 の結果（Claude 確認）

- 追加: `grid_planner.py`（狙い・開度・感度の割線更新・打ち切り）、`grid_patterns.py`（WLTP 集計 → パターン列）、
  `grid_stub_run.py`（スタブ直結 CLI）、`wltp_grid.wltp_cell_stats` / `data_cells(keep=)`、
  `pattern_loop.py` に `GridStationPattern`・`GridLaunchPattern`・`_Phase.HOLD_STEP`、`PatternKind.GRID_STEP/GRID_LAUNCH`、
  config `learning.grid_*` 17 キー。手順2 の `build_patterns` は**まだ変えていない**（段4）
- 単体テスト合格（既知の stop_brake_floor の 1 件を除く）、`ruff check` 合格
- 設計の実装上の決定（計画からの変更）:
  - 開度の基準は「u0 = PI が落ち着いた釣り合いの開度 = 加速度 0」。定速ステップの実測は PI のハンチングで揺れる（プラントで +1.7 km/h/s が出た）ため基準にせず記録だけ残す
  - PI の kp・ki は感度 g で割る（kp_norm 0.36・ki_norm 0.06）が、**レート上限は g で割らず固定 1.0 %/s**（実開度が追従できる速さで車両に依らない。g で割ると g の初期値が真値とずれた車で接近が遅すぎた）
  - 発進の開度は不感帯 + (狙い − 最寄りの惰行) ÷ g（不感帯では走行抵抗で減速する分を上乗せ。停車と同じ考え方）
  - 踏んだのに無反応（実測 a が変わらない）なら g を半分にして、やり直しは深く踏む
- スタブ 1 ステーション（15 km/h、感度×1）: 惰行・減速（ブレーキ 3 段）は狙いのセルに入った。加速は g の初期値 2.0 が真値 0.25 の 8 倍で 1 回目が空振り
  → 無反応時の半減を追加（上記）。スタブは接近・整定が遅く（1 ステップ約 80 s）、**全体の所要時間はスタブでは見積もれない**（計画の約 20 分は実車の見込み）
- スタブ 感度×5（15 km/h + 発進・停車セル、391 s）: 完走。感度 g はアクセル 0.63→0.90→1.11（真値 1.25）、ブレーキ 5.04→…→2.72（真値 2.50）へ数回で寄った。
  ブレーキの強すぎる停車ステップは「打切り」が働いた
- スタブ 感度×0.2（15 km/h のみ）: **ステップ 0 本**。g の初期値 2.0 が真値 0.05 の 40 倍ずれ、PI（kp = 0.36 / g）が弱すぎて 240 s で落ち着かずステーションを飛ばした（走行は壊れず完了）。
  → **未解決の懸念（ユーザー判断 2026-09-25: 後回し）**: 感度が今の車の約 1/5 以下だと最初のステーションで整定できない。対策案は「接近中に感度を推定して PI ゲインを補正」。段4 は別ターミナルで実施
## 段4 の結果（Claude 確認）

- **手順2 の新パターン列**（`pattern_drive.build_patterns(cfg, profile, wltp_stats)`）:
  コーストダウン ×2 → 格子ステップ走行（ステーション昇順 → 発進・停車セル）→ クリープ発進 ×3 → クリープ域ブレーキ保持 ×8。
  `main.step2_pattern_drive` は **2-0 の前**に WLTP の基準車速（DB）を読み `wltp_cell_stats` を作る（DB が読めないなら走る前に止まる）。
  `build_ff_model` には常に `wltp_mode` を渡す（重みは `sample_weight_enabled` が false なら無効のまま。MAE_WLTP が出るだけ）
- **クリープ域ブレーキ保持の開度** = 不感帯 + frac × (停車保持開度 − 不感帯)。新キー `learning.creep_brake_hold_fracs`
  （yaml は `[0.05, 0.15, 0.25, 0.45, 0.55, 0.7, 0.9, 1.0]` ≒ 修理後の実測で +0.6〜+11.4%）。開度列は `stop_brake_floor.creep_brake_hold_openings`
  1 か所で作り、パターン生成・停止下限の推定（候補開度）・`stop_brake_floor` CLI が同じ式を使う
- **削除**: パターン（ACCEL_SWEEP・BRAKE_HOLD・CRUISE_TRIM・不感帯プローブ・トリム階段・定速階段・低開度階段。`PatternKind` と
  `pattern_loop` の各クラス・フェーズ・PI、`learning_patterns.LearningDriveManager`）、config キー 約 30 個（`learning.*_offsets_pct`・
  `brake_hold_low/hard_*`・`trim_stair_*`・`cruise_hold_*`・`low_open_stair_*`・`creep_brake_hold_offsets_pct`）、
  ツール（`debug_a2a5`・`debug_a3a4`・`stair_gain`・`cruise_curve` とテスト、`kaizen --part coverage`、`accel_onset` の表3b、
  `train_candidate_model --cruise-curve-from`、FF 候補 C6）。`debug_a6` の「フェーズ所要時間」表も同時に削除（kaizen の関数に依存していたため）
- **残したもの**: `_Phase.CRUISE_HOLD`（格子の PI 保持。CSV の phase 名を段3 と揃えるため据え置き）、`_Phase.BRAKE_HOLD`（クリープ域ブレーキ保持）、
  COAST_DOWN・クリープ・G ガバナ・停車復帰。`learning.timeout_s` は yaml で 3600s のまま
- 動作確認用に `grid_stub_run --full` を追加（手順2 の全体構成で走り、2-2 のモデル作成まで通す）
- 単体テスト・`ruff check` の結果と、スタブ通しの結果は下の「段4 の確認結果」に追記

### 段4 の確認結果

- **単体テスト**: `tests/research` 全体 553 件合格（既知の `test_stop_brake_floor_settings_are_validated` 1 件のみ除く。作業ツリーの yaml `stop_brake_floor_offset_pct`=0.5 が原因で段4 と無関係）。`ruff check tests/research` 合格
  （全体を流して、削除したキーを参照していた `test_research_pedal_search.py` の 1 件も削除した）。`main`・`kaizen`・`accel_onset`・`train_candidate_model`・`debug_a6/a7`・`relearn` の import も確認
- **スタブ通し**（`grid_stub_run --full --stations 15 --gain-scale 5`。走行 912s＝15.2 分、スタブは応答が遅く実車の所要は見積もれない）:
  - パターン一覧は 15 本: COAST_DOWN ×2 → GRID_STEP（15 km/h）→ GRID_LAUNCH → クリープ発進 ×3 → ブレーキ保持 ×8。全パターンを通って完走し、緩減速・停車保持まで行った
  - クリープ域ブレーキ保持の開度（スタブ: 不感帯 8、停車保持 16）: 8.4 / 9.2 / 10.0 / 11.6 / 12.4 / 13.6 / 15.2 / 16.0%（= 不感帯 + frac × 8）。停止下限の候補開度も同じ列（`creep_brake_hold_openings`）
  - 格子ステップ: 10 本（OK 4・やり直し 2・未達 1・打切り 1・定速 1・惰行 1）。感度 g はアクセル 0.63→0.90→1.11（真値 1.25）、ブレーキ 5.04→…→2.72（真値 2.50）と段3 と同じ動き
  - 2-2 のモデル作成まで通った（逆モデル・惰行カーブ低速端・クリープ加速カーブ・停止下限・ペダルゲイン。YAML は更新せず）。
    スタブは 1 ステーションだけ走ったので網羅マップは穴だらけ（WLTP の 87%）。これは絞ったことによるもので、網羅の評価は段5（実機・全ステーション）で行う
- **注意**: 旧キーを含む yaml（`config_testVehicle_old.yaml`・`results/config_testVehicle_before_20260925_*.yaml`）は `ConfigError` で読めない（記録として残した）

## 段5 の結果（1 回目。Claude 確認 2026-09-26）と段5b

手順2 実機走行 `drive_log_real_20260926_055835.csv`（走行 3046s。格子ステップ走行は約 2832s で `timeout_s` に収まった）を生データで検証した。
モデル・yaml は 2-2 で書き戻し済み（`test_vehicle_20260926_064924.pkl`）だが、下の①〜③の修正前のデータなので手順3 は走らない（ユーザー決定）。

### 見つかった問題

| # | 問題 | 実測 |
|---|---|---|
| ① 安全 | 15 km/h の強い減速ステップでブレーキが上限 80% に張り付いた | 24.8→3.7 km/h を 1.5 s（平均 約 0.4G、最大 約 0.6G）。G ガバナは 0.8 s 後に作動。15 km/h ではブレーキ 7.4〜18.9% の減速が惰行（−1.47）とほぼ同じで、感度 g が極小になり狙い −8 の開度が外挿で上限まで伸びた |
| ② | 停車ステップが止まらない | ブレーキ 7.05% / 7.52%（不感帯ちょうど）で 5 km/h のクリープと釣り合い、40 s 打ち切りまで 26〜28 s 走行。傾きの当てはめがこの区間を含むため実測 a が 0 寄りになる。原因は g_brake を 135 km/h ステーションから引き継いだこと（約 1.4。低速の 30 倍）と、狙い −0.98 が惰行 −1.47 より弱いこと |
| ③ | u0 が近づく向きで偏る | 「±1 km/h・3 s」は近づく途中で通る。126 回すべて窓の中の車速が一方向に動いていた（下から +0.14〜+0.42、上から −0.06〜−0.42 km/h/s）。u0 のままの定速ステップの実測は +0.19〜+0.49 km/h/s。u0 の上下差は 15 km/h で 1.6%、65 km/h で 4.6%、125 km/h で 6% |
| ④ 構造 | 強い減速のマスは 1 ステップでは埋まらない | 強いステップは帯を 1.2〜2.3 s で抜け、穴の基準 2 s に届かない（10〜60 km/h × −14〜−3 km/h/s、0〜10 km/h の強い加速）。**今回は直さない**（次の手順2 の結果を見て決める） |

学習に入った行の出どころ（アクセル 2132 s の 86% が PI 保持、開度固定のステップは 10%。ブレーキ 360 s は停車復帰 31%・停車ステップ 27%・クリープ域保持 21%・格子ステップ 19%）は設計（測る区間は開度固定）とずれているが、
**ユーザー決定（2026-09-26）: 今は含めたまま**。PI が開度を 1 %/s までしか動かさないので、閉ループのばたつきを学ぶほどの悪さかは手順3 の実機比較で見る。

網羅表（全行）: 穴 6 セル＝WLTP 48s（3%）。開度固定＋惰行の行だけで数え直すと穴 12 セル＝90s（5%）。

### 調整値の評価

| 値 | 判定 | 根拠 |
|---|---|---|
| `grid_settle_tol_kmh` ±1 / `grid_settle_s` 3 s | 判定としては不適だが**据え置き**。u0 は定速ステップの実測で補正する（下の A） | 上の③。締めると待ち時間が増える（今でも走行時間の 65%＝1855 s、幅内 中央 21%） |
| `grid_step_lag_s` 0.5 s | 妥当 | 94 本を 0.5 s ごとに区切った傾き / 全体の傾き: 0.65, 1.05, 1.02, 1.02, 0.97 |
| `grid_step_window_s` 3 s | 妥当 | 残差 σ 中央 0.03 km/h、点数 中央 26 |
| ブレーキの踏み込みの伸びの倍率 2 倍（新規） | **仮の値**（データの根拠なし） | 次の手順2 のやり直し・未達の数で決め直す |

### 段5b の変更（ユーザー決定 2026-09-26）

- **A. u0 の補正**（`grid_planner.py`）: 基準を (u0, 加速度 0) から (u0, 定速ステップの実測 a_hold) に変更。加速の開度 = u0 + (狙い − a_hold) ÷ g、g の更新・緩い減速の補間・やり直しも a_hold 基準。
  助走の u0' は据え置き（助走の車速に定速ステップが無いため）
- **B. ブレーキの踏み込み上限**（`grid_planner.py`, `pattern_loop.py`）: 不感帯からの踏み込み量は、そのステーション（発進・停車セル）で試した最大の **2 倍**まで。未試行なら停車保持開度まで。15 km/h の並びに当てはめると強い減速の開度は 80% → 30.7%
- **C. 停車ステップ**: クリープ平衡（車速 ≤ クリープ速度 + 許容幅 かつ 傾き < `creep_launch_settle_kmhs` が `creep_launch_settle_s` 続く）で終える。当てはめは v ≥ クリープ速度 + 許容幅の点だけ（旧: v ≥ 2）。
  発進・停車セルの感度 g は最寄りのステーション（15 km/h）の最後の値から始める（旧: 直前の 135 km/h の値）
- **D. 集計に「窓の傾き」列を追加**（`grid_settle.py`）: 落ち着いた窓の車速の傾き。次の手順2 で A の効果（定速ステップの実測が 0 に近づくか、窓の傾きが減るか）を見る材料
- **E. コーストダウン前の加速を G 上限の刻み踏みに変更**（`pattern_loop.py`, `pattern_drive.py`）: 旧はアクセル 70% を 1.5 s でランプして固定。
  新は手順2 終了時の緩減速（`stop_decel.decelerate_to_stop`）と同じ考え方で、不感帯の手前まで先に上げ、その後 1 s ごとに加速度（傾き）を見て
  目標 0.2G 未満（0.18G 未満）なら +0.5 mm、0.3G 超えなら −0.5 mm（接近位置より浅くはしない）。70% は「指令」ではなく**頭打ち**。
  cap 手前で惰行へ・最高速超えでブレーキ・G ガバナは変更なし。設定は `decel_stop` を共用（`PatternLoopConfig.coast_accel_*` に写す。新しい調整値は増やしていない）。
  加速度の計算は `grid_planner.fit_slope` を流用（`stop_decel` を import すると hardware 側を引き込むため）。単体テスト 6 件追加、変更箇所 89 件・ruff 合格
- **F. 加速の打ち切り `accel_full_range_timeout_s` 30 s → 60 s**（`PatternLoopConfig`。ユーザー指示 2026-09-26）: **仮の値（データの根拠なし）**。
  0.5 mm × 1 s 刻みだと不感帯付近から 70% まで最悪約 100 s かかり、スタブでは 30 s 時点で開度約 19% のまま惰行へ入った（スタブは応答が実車の約 1/5 なので実車の値ではない）。
  効き方: 短すぎると cap（約 130 km/h）に届かず惰行カーブの測定範囲が狭まる／長すぎると頭打ちの車速で加速が続く。
  決め直し: 実機の手順2 の先頭（コーストダウン 2 本）で、到達車速（cap の手前約 128 km/h まで届いたか）・打ち切りで惰行に入っていないか・加速中の開度が 0.2G 前後で落ち着くか・`coast_decel_*` が前回とずれていないかを見る

- **G. コーストダウン前の加速を「G の余裕に比例して踏む」方式に置き換え**（`pattern_loop.py`, `pattern_drive.py`。E の刻み踏みを置換）:
  E・F の実機（`drive_log_real_20260926_110427.csv`）は、60 s で 83 km/h（1 本目・2 本目とも）までしか届かず、加速中の G は最大 0.06G だった。
  前回（70% 固定）は 28.7 s で 143 km/h・最大 0.32G。原因は踏む速さで、1 刻み 0.5 mm / 1 s（約 0.5%/s）は遅すぎ、G の判定は毎回「踏み増し」で上限を守る部分は一度も働かなかった
  （停車前のブレーキは効きが強く少し踏めば足りるが、アクセルは 0.2G に 50% 前後要る）。
  - 新方式（案1、ユーザー決定 2026-09-26）: 踏む速さ [%/s] = `coast_accel_rate_gain` ×（目標 0.2G − 今の G）。出だしはぐっと踏み、目標に近づくとゆっくり、超えたら同じ式で戻る。
    目標 ±0.02G は保持。範囲は [不感帯の手前の接近位置, 70%（頭打ち）]。加速度は直近 1 s の傾き（`fit_slope`）。終わる条件・G ガバナは変更なし
  - 案2（0→cap の基準車速を PI で追う）は採らなかった。発進直後の遅れを取り戻す I の部分が 0.2G を超えて踏みやすいこと、60 km/h から先は 70% でも 0.2G が出ず I が溜まり続けることが理由
  - **`coast_accel_rate_gain` = 100 %/s per G（半分データ由来・半分仮の値）**: 実機ログから感度は 30 km/h で約 0.004G/%・60 km/h 以上で 0.002〜0.003G/%（速いほど小さい）、踏んでから G が出るまでの遅れは約 0.5〜1 s。
    応答の時定数（1/(100×感度) ≒ 2〜3 s）が遅れの 2 倍以上になる値。大きいほど早く届くが 0.2G を超えやすく、小さいと今回のように届かない。`PatternLoopConfig` のみ（yaml には出さない）
  - 実機ログから作った簡易モデル（感度が車速で下がる・遅れ 0.5〜1 s）での見積もり: cap 到達 約 34 s、G の最大 0.21〜0.26G（gain 100）。60 km/h 付近から 70% で頭打ち。実機の値ではない
  - `coast_accel_release_above_g`・`coast_accel_step_mm`・`coast_accel_dwell_s` は削除（`decel_stop` セクション自体は停車前の緩減速で使い続ける）。単体テストを書き換え、変更箇所 65 件・ruff 合格
  - 決め直し: 次の手順2 で、cap 到達時間・加速中の G の最大（0.3G 未満か）・G ガバナ作動・開度の軌跡（60 km/h あたりから 70% で頭打ちか）を見る。60 s の打ち切りは、35 s 前後で届けば短くできる

## 段5b の実機結果（手順2 再走 `drive_log_real_20260926_113953.csv`。Claude 確認 2026-09-27）と段6

モデルは 2-2 で書き戻し済み（`test_vehicle_20260926_123231.pkl`）。

| 項目 | 結果 |
|---|---|
| 網羅（WLTP + US06） | 格子外 0 s。穴 9 セル＝47 s（3%）。開度固定＋惰行の行だけだと 12 セル＝約 70 s。すべて \|a\|>3 km/h/s の強い加減速の列（④と同じ構造） |
| コーストダウン前の加速（段5b-G） | 33 s で 144 km/h に到達、最大 0.23G（狙いどおり） |
| **上限 0.4G 超え** | 25 km/h ステーションの強い減速（助走 34.6 km/h・ブレーキ 0→34% を一気に）で 0.42G（1 s 傾き）/0.47G（0.4 s）を 0.2〜0.6 s。G ガバナは「超えてから」なので構造的に一度は超える（踏んで 0.6 s 後に作動）。停車復帰（134 km/h）は G ガバナ 8 s で 0.30〜0.35G |
| 34% の出どころ | 25 km/h 中心の感度と「試した最大の 2 倍」。踏んだ 34.6 km/h ではブレーキが 2 倍以上効く（2-2 のゲイン 25 km/h 0.30 → 35 km/h 0.71） |
| ブレーキは踏むほど急に効く | 25 km/h: 12→16% で 0.21、16→24% で 0.34 km/h/s/%（比 1.6）。0.2G からの直線外挿は危険側 |
| ブレーキ学習 R² 0.496 | 誤差の 78% が 7 km/h 未満（クリープ域は開度 8.5〜18.6% のどれでも同じ減速で止まる＝開度が決まらない）。20 km/h 以上は学習 MAE 0.76 だが区間を抜いた検証は 3.5〜5.8（データ 107 s・37 区間）。**判断は手順3 で行う（ユーザー決定 2026-09-27）** |

### 段6 の決定（ユーザー 2026-09-27）

- 上限 G は **Must**。2 段構え: 門① 最初に G 校正（0.2G 狙いのブレーキ減速）で「上限 G に届く開度」を車速ごとに予測、門② 走行中に超えそうなら緩める（踏み込みのランプ・先読み・踏み増し停止）
- ④ は **通し掃引**（車速ごとに開度を切り替えて一気に通り、複数の車速帯を 1 回で測る）
- 順番: 先に実装 → 手順2 → 手順3。6a（門①）→ 6b（門②）→ 6c（掃引）→ 6d（実機）
- 計画: `/home/raspi5_16gb/.claude/plans/pasted-content-id-dac4-docs-problem-pro-enchanted-nygaard.md`

### 段6a（門①: G 校正と上限開度マップ）の実装と確認（Claude 確認 2026-09-27）

**実装**
- 新規 `tests/research/g_limit.py`（`GLimitMap`）: 車速帯（10 km/h）ごとの (開度, 加速度) の点と、惰行の減速（開度=不感帯の点）から、`learning.g_cap_g` に届く開度を予測する。
  帯内の点の範囲内なら内挿、範囲外なら「一番効いた点」と「開度が一番低い点」の割線で外挿（近い 2 点の割線は加速度の雑音で傾きが不安定）。
  実測で「届かなかった」開度は下限にする。点が無い帯は、そのペダルが効きやすい側（ブレーキは速い側・アクセルは遅い側）の帯を使う（予測開度が小さくなる安全側）
- 新パターン `G_CALIB`（`pattern_drive.build_patterns` でコーストダウンの直後・格子の前に 1 本）: G 比例加速で cap まで → 惰行せず 0.2G 狙いの G 比例ブレーキ減速（新フェーズ `CALIB_BRAKE`。
  手順2 終了時の緩減速 `decelerate_to_stop` と同じ刻み方: 0.5 mm・1 s 待ち・0.18G 未満で踏み増し・0.3G 超で戻す。`decel_stop` の値を共用）→ 5 km/h 以下で終了（停車復帰へ）
- 点の集め方: CALIB_BRAKE は 1 s ごとに (実開度, 減速)、コーストダウンの惰行は開始 1.5 s 後から減速（基準点）、G 比例加速は開度が 1.5 s 以上動かずに目標 ±0.02G にいる間の (開度, 加速度)、
  格子ステップは終了時の (開度, 実測 a)。G 校正が終わると車速帯ごとの上限開度の表を表示し、走行後にも更新後の表を表示する
- `GridPlanner`: `cap_fn` で DECEL_BRAKE/STOP/ACCEL/LAUNCH の開度を頭打ちにする（`Step.capped`）。**上限を引く車速はステップが通りうる帯の中でそのペダルが一番効く端**
  （ブレーキ = 帯の上端、アクセル = 帯の下端）。20260926 の 0.47G は、25 km/h 中心の効きで決めた 34% を 34.6 km/h から踏んだのが原因だった。
  頭打ちで狙いのセルに届かなかったステップは判定「上限G」にして、残りの強い狙い（同じ向き）を捨てる（通し掃引が担当）
- config `learning.g_cap_g` = 0.3（**仮の値**。根拠: ブレーキは踏むほど急に効く（実測 比 1.6）ので割線外挿は開度を大きく見積もる。0.3G と予測した開度は実際 約 0.36G。
  vehicle.max_decel_g 0.4 との余裕。手順2 の G 校正・格子の実測点で決め直す）。検証: 0 < g_cap_g < max_decel_g

**確認**
- 単体テスト: 変更箇所（g_limit・grid_planner・pattern_loop・pattern_loop_grid・grid_patterns・pattern_drive・wltp_grid・grid_settle・no_src_import）226 件合格。
  `test_research_config.py` の 3 件（`test_pedal_search_*`・`test_stop_brake_floor_*`）は実機走行が書き換えた yaml を見る既知の失敗で今回の変更と無関係。`ruff check tests/research` 合格
- スタブ通し（`grid_stub_run --full --stations 15,55 --gain-scale 5`、876 s）: パターン 17 本（コーストダウン → G 校正 → 格子 2 ステーション → 発進停車 → クリープ）が完走。
  G_CALIB は 134.6 → 4.5 km/h、ブレーキ 0〜10.2%、減速の最大 0.206G。G 校正後の上限開度表が出て、走行後に更新された（アクセルは G 校正で点が取れず「—」＝上限なし、格子の実測から 15.7%）。
  スタブは強い狙いが小さく頭打ち・「上限G」判定は出ていない（単体テストで確認）。スタブは車両特性が別物なので上限の数値（ブレーキ 11.6% 一律など）は判断しない

**6a 時点の残り**: 門②（緩める制御）・停車復帰の G 比例化・通し掃引は 6b・6c で実装（下記）

### 段6b・6c（門②・通し掃引）の実装と確認（Claude 確認 2026-09-27）

**6b: 走行中に超えそうなら緩める（門②）**（`pattern_loop.py`）
- **踏み込みのランプ**: 開度固定のステップ（HOLD_STEP）は、踏むペダルを `step_lag_s`（0.5 s。傾きの当てはめから除く頭の区間と同じ）かけて上げる。始点はステップを始めたときの開度、
  もう一方のペダルは即座に離す。20260926 は 0→34% を一気に出して、指令から G の山まで約 0.6 s の遅れの間に超えていた
- **先読みガバナー**: 見込み = 今の G + G の増える速さ（直近 0.5 s の傾き）× `gov_lead_s` 0.6 s。見込みか今が上限（0.4G × 0.98）以上なら従来どおり 1 周期ごとに 2% 下げる、
  見込みが上限 × `gov_soft_frac` 0.85（≒ 0.33G）以上なら踏み増しを止める（その開度で頭打ち）、今が上限 × 0.7 未満なら 0.5%/周期で戻す。
  `gov_lead_s` はデータ由来（20260926 の 890.7→891.3 s: ブレーキ指令から G の山まで 0.6 s）、`gov_soft_frac` は**仮の値**（g_cap_g 0.3G と上限 0.4G の間。実機の G の最大で決め直す）
- **停車復帰（DRIVE_BRAKE）の G 比例化**: 5 km/h（`coast_down_stop_speed_kmh`）を超える車速から入ったときは、G 校正と同じ G 比例ブレーキ（0.5 mm・1 s・0.2G 狙い）で減速し、
  5 km/h 以下になったら従来のランプ（今の開度から停車保持開度へ）に切り替える。G 校正が済んでいれば、その車速で 0.2G が出る開度から始める。
  最高速超えのブレーキも同じ経路（固定 30% をやめた）。134 km/h から停車保持開度へ一気に踏んで 0.30〜0.35G だった件の対策
- 計画との差: G ガバナーは CRUISE_HOLD には適用しない（PI がレート 1 %/s で穏やか。従来どおり）。判定は指令開度基準（実開度の PNOW は使っていない）

**6c: 通し掃引**（新規 `coverage_live.py`・`sweep_planner.py`、`pattern_loop.py` の `GridSweepPattern`・フェーズ `SWEEP_UP`/`SWEEP`）
- **走行中の網羅カウンタ**（`LiveCoverage`）: 全周期の (時刻, 車速) から、事後の `wltp_grid.data_cells` と同じ定義（v0・1.0 s 先との差・停車除外・行の重み = 周期）で車速 × 加速度の各セルの秒数を数える。
  事後の数え方と一致することをテストで確認（範囲を揃えて完全一致）
- **掃引の計画**（`plan_sweeps`）: 掃引のパターンに入ったときに穴（モード ≥ 2.5 s かつデータ < 2.0 s）を数え、加速度の列ごとに 1 本（穴の一番低い〜一番高い車速ビンの間。穴でないビンも通る）。
  減速は高い車速から、加速は低い車速から。定速の列は作らない。狙いの加速度 = そのセルのモードの平均を上限 G（`g_cap_g` 0.3G）で頭打ち。需要の大きい列から
- **開度**: セルごとに「そのセルの車速で狙いの加速度が出る開度」を G 校正・格子ステップの実測（`GLimitMap`）から求め、上限 G の予測（帯の中で一番効く端）で頭打ち。
  予測できないセルは隣で埋め、全部できなければその掃引をとばす
- **走行**: 開始車速まで G 比例で加速（`SWEEP_UP`。減速は最初のセルの上端 +1 km/h、加速は下端 −2 km/h。停車からの加速掃引は直接）→ `SWEEP`（今の車速のセルの開度へ、セルが変わったら `step_lag_s` かけてランプ。
  先読みガバナー・G ガバナー適用）→ 終わりの車速で停車復帰（G 比例）。1 回が済むごとに対象セルの穴を数え直し、残っていて回数が `grid_sweep_max_passes` 未満なら同じ掃引をもう 1 回（開度は実測が増えた分で引き直す）。
  穴が無い・回数 0・開度を予測できないときは何もしない。1 行ログ: 「掃引 減速 −9〜−7 km/h/s、車速 20〜70 km/h: 1 回目 対象セルのデータ 12.0s → 23.6s、残りの穴 0 セル」
- config `learning.grid_sweep_max_passes` = 6（**仮の値**: 1 回で 1 セル約 1 s（10 km/h ÷ 約 9 km/h/s）しか取れず、穴の基準 2 s に 2〜3 回、余裕 2 倍。手順2 の実機で「何回で埋まったか」を見て決め直す。0 で掃引なし）
- 掃引はパターン列の格子ステップ・発進停車の後、クリープ発進の前（`build_patterns`）

**確認**
- 単体テスト: 変更箇所 256 件合格（新規 `test_research_sweep.py` 17 件: 計画・カウンタと事後の一致・掃引の走行（減速・加速・回数打ち切り）、先読みガバナー、ステップのランプ、停車復帰の G 比例、config 検証）。
  `test_research_config.py` の 3 件は既知の失敗（実機走行が書き換えた yaml を見るテスト）。`ruff check tests/research` 合格
- スタブ通し（`grid_stub_run --full --stations 15,55 --gain-scale 5 --sweep-max-passes 1`、1195 s）: 18 本（コースト ×2 → G 校正 → 格子 2 ステーション → 発進停車 → 通し掃引 → クリープ）が完走。
  掃引は穴 35 セル（WLTP 730 s）から 5 本作られて全部走り、対象セルのデータは 41→130 s・7→44 s・12→24 s・1.4→3.3 s と増え、残りの穴が 0 になった掃引は 2 本（残り 3 本は回数 1 では穴が残る。スタブは 2 ステーションだけで穴が多く、強い加減速も弱い）。
  フェーズ別の |G| の最大（0.4 s 傾き）は DRIVE_ACCEL 0.318・SWEEP_UP 0.318・SWEEP 0.304・BRAKE_HOLD 0.280・DRIVE_BRAKE 0.246・CALIB_BRAKE 0.244・HOLD_STEP 0.186 で全て 0.4G 未満。
  スタブは車両特性が別物（応答約 1/5）なので上限の数値・掃引の埋まり方は実機の目安にしない

### 段6d 実機（ユーザーが実施）

```bash
# 単体テスト（変更箇所のみ）
.venv/bin/python -m pytest tests/research/test_research_sweep.py tests/research/test_research_g_limit.py \
  tests/research/test_research_grid_planner.py tests/research/test_research_pattern_loop.py \
  tests/research/test_research_pattern_loop_grid.py tests/research/test_research_grid_patterns.py \
  tests/research/test_research_pattern_drive.py tests/research/test_research_wltp_grid.py -q
.venv/bin/ruff check tests/research
# 手順1 → 走行前チェック → 手順2（実機。config へ書き戻す。所要は前回 約 53 分 + G 校正・掃引の分。timeout_s は 7200 s）
.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"
# 結果を Claude と確認したあと、手順3（段2 の基準 075149 とイベント単位で比べる。ブレーキの判断もここ）
.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"
```

見るもの（手順2）:
- **上限 G**: 全行の |G|（0.4 s 傾き）の最大が 0.4G 未満か、ガバナー作動の行（CSV の `governor_active`）がどのフェーズか。フェーズ別の最大は `drive_log_real_*.csv` から前回と同じ方法で出す（Claude が確認）
- **G 校正**: 「G 校正の結果」の表（車速帯ごとの上限開度）が出るか。ブレーキが 0.2G に届く開度の車速依存（前回は 25 km/h で約 13〜16%）、走行後の表との違い
- **格子ステップ**: 判定「上限G」の数（強い狙いを頭打ちにした数）、打切りの数
- **通し掃引**: 「通し掃引: 穴 N セル → 掃引 M 本」、各掃引の「1 回目 データ a → b s、残りの穴」。何回で埋まったか・埋まらなかった掃引（`grid_sweep_max_passes` 6 の決め直し）、所要時間
- **網羅**: 走行後に `.venv/bin/python -m tests.research.wltp_grid tests/research/results/drive_log_real_<新>.csv` で、強い加減速の穴（前回 9 セル・47 s）がどれだけ減ったか
- 前回まで通りの確認: コーストダウン（33 s で 144 km/h・最大 0.23G）、停車ステップ、クリープ域

### 段6d 手順2 の実機結果（`drive_log_real_20260927_065502.csv`。Claude 確認 2026-09-27）

走行 約 3035 s（掃引を含む）。モデルは 2-2 で書き戻し済み（`test_vehicle_20260927_074535.pkl`）。

**網羅（WLTP + US06。穴 = モード ≥ 2.5 s かつ データ < 2 s）**

| 数え方 | 今回 | 前回（113953） |
|---|---|---|
| 全行 | 3 セル・16 s（1%） | 9 セル・47 s |
| 測定区間だけ（HOLD_STEP・SWEEP・COAST・CALIB_BRAKE） | 5 セル・31 s（2%） | 12 セル・約 70 s |

測定区間だけで残る穴: 0〜10 km/h × +7〜+14（8 s / 0.4 s）、0〜10 × −7〜−3（7 / 1.3）、0〜10 × +0.5〜+1.5（8 / 0。クリープ発進は測定区間に入れていないため。全行では 53 s）、
10〜20 × −14〜−7（4 / 0.9）、120〜130 × +1.5〜+3（4 / 1.9）。
穴ではないが薄い列: 加速 +1.5〜+3 km/h/s は測定区間が各車速で 2.6〜4 s（モードは 16〜32 s 使う）。

**上限 G（0.4 s 傾き）**

- **0.4G 超えは 0 行**。最大 0.378G（1033 s、助走 15 km/h からの強い加速。アクセル 6→43.5% を 0.4 s で踏んだ。上限開度表は 10〜20 km/h で「0.3G は 44.8%」と予測 → 実際は 0.38G で、予測が甘い）
- 0.3G 超えは 16 回: 格子ステップ 3・停車復帰の G 比例ブレーキ（134/120 km/h から）0.31〜0.35G・G 比例加速 0.31〜0.33G・掃引 0.31〜0.32G
- 停車復帰 DRIVE_BRAKE の最大 0.346G（1 秒待ちの刻みで戻すのが遅れる）
- **`governor_active` = 1 は「ガバナーが指令を削った印」であって 0.4G 超えではない**（見込み 0.33G で踏み増し停止・戻し中も 1）。作動 618 行の実際の G: 0.2G 未満 370・0.2〜0.3G 230・0.3〜0.4G 18・0.4G 以上 0
- 0.1 s ごとの差分で出る 0.4G 超え 93 点は、行間隔が 0.02〜0.05 s に詰まった見かけの値（35.12→35.75 km/h を 0.019 s → 0.94G、0.4 s 傾きでは 0.18G）

**気づき（今は直さない）**

- 停車からの加速掃引で 0〜10 km/h が 0.05〜0.15G しか出ない（狙い +9.4 km/h/s ≒ 0.27G）。踏み出し直後に先読みガバナーが踏み増し停止に入り、戻しが 0.5%/周期しかないため。4 回走っても +7〜+14 の穴が埋まらない
- 上限開度表の低速アクセル予測が甘い（上記 1033 s）

**ユーザー判断（2026-09-27）**: 0.4G の超過は governor_active の読み違いで、超えていないので現状で OK。手順3 へ進む。

## 段7: 手順2 の計測効率化（2026-09-27）

目標 1800 s（065502 は 約 3035 s）。条件: 穴が増える案・G ガバナ（0.4G）を超える案は不採用。
計画: `/home/raspi5_16gb/.claude/plans/docs-problem-problemreport-20260925-md-2-floofy-balloon.md`

**065502 の時間の内訳**: 格子ステップ 2251 s のうち、ステップ前の待ちが 1654 s（109 回・中央 13.9 s）。
待ちのうち ±1 km/h に入るまでの「戻り」が 1310 s、入ってからの 3 s は 343 s。ステップ本体は 330 s。
→ 時間の大半は「ステップでずれた車速を、PI（1 %/s のレート制限）でゆっくり戻す」ところ。

**検証した事実（065502）**
- 格子の惰行ステップの実測 a は、コーストダウンの同じ車速の値と ±0.1 km/h/s 以内（13 ステーション全部。例: 42 km/h −2.24 vs −2.25）。コーストダウン 1 本目と 2 本目も ±0.07 以内
- クリープ域ブレーキ保持 ×8 は毎回クリープ発進（0→4.9 km/h・両ペダル 0%）から始まる。`creep_curve.py` はフェーズではなく行の条件で拾うので、単独のクリープ発進 ×3 と同じデータ
- 網羅表: 惰行ステップ・単独クリープ発進 ×3・コーストダウン 2 本目の行を抜いても穴は 3 セル 16 s のまま
- 格子の PI 保持の行を全部抜くと穴 6 セル 126 s（定速・緩い加減速の列を支えている）→ 3 s の落ち着き窓は残す

**ユーザー決定（2026-09-27）**
- 採用: ① 格子の惰行ステップ廃止（a_coast はコーストダウンで集めた惰行の減速から）② 単独クリープ発進 ×3 廃止（`creep_launch_count` 0）
  ③ ステップ後の戻りを速く（新フェーズ `GRID_RETURN`）④ コーストダウン 2 → 1 本
- 不採用: ⑤ 掃引の「データが増えない繰り返し」の打ち切り（停車からの加速掃引の件と一緒に後で）、
  ⑥ ステーションの最後に中心へ戻らず次へ（戻りの行を抜くと 90〜100 km/h × +1.5〜+3 が 1.7 s で穴になる）
- 見積もり: 約 1950 s（③が 1 km/h/s 相当しか効かなければ約 2150 s）

**③ GRID_RETURN の中身**: ステップの後（と助走へ移るとき）、PI の目標から `grid_return_switch_kmh` より離れていれば、
遅いときはアクセル = u0 + (`grid_return_accel_kmhs` − a_hold) ÷ g_accel（[u0, 最大開度]・上限 G の予測で頭打ち）、速いときは不感帯（惰行）の固定開度で戻る。
目標の ±switch に入ったら従来の PI（開度・積分 = u0）→ ±1 km/h に 3 s で次へ。先読みガバナー・G ガバナーを適用。落ち着き待ちの打ち切り（40 s）は戻りの間も進む

| 値 | 現在 | データの根拠 | 何に効くか | いつ決めるか |
|---|---|---|---|---|
| `grid_return_accel_kmhs` | 2.0 km/h/s | 半分データ由来: 薄い +1.5〜+3 km/h/s の列（各車速 2〜4 s）の中なので戻りの行がこの列を埋める。約 0.06G | 戻りの時間・網羅 | 次の実機の手順2（戻りの秒数と網羅表） |
| `grid_return_switch_kmh` | 1.5 km/h | **仮の値**: 2 km/h/s × 遅れ 0.6 s（`gov_lead_s` の実測）≒ 1.2 km/h に余裕 | 切り替え後の行き過ぎ | 同上（切り替え後 ±1 に入るまでの時間） |

**実装・確認（Claude 2026-09-27）**
- 変更: `learning_patterns.py`（COAST_DOWN_COUNT 1）、`config.py`・yaml（creep_launch_count 0、新 2 キーと検証）、`g_limit.py`（`coast_decel_kmhs`）、
  `grid_planner.py`（`coast_fn`。取れなければ従来の惰行ステップへフォールバック）、`grid_patterns.py`、`pattern_loop.py`（`GRID_RETURN`）、`pattern_drive.py`（docstring）
- 単体テスト: 変更箇所 246 件合格（`test_research_config.py` の既知 3 件を除く）。`ruff check tests/research` 合格
- スタブ通し（`grid_stub_run --full --stations 15,55 --gain-scale 5 --sweep-max-passes 1`、`drive_log_stub_20260927_152503.csv`、1040 s）: 14 本が完走し 2-2 まで通った。
  コーストダウン 1 本・格子に「惰行」の行なし・クリープはブレーキ保持 8 本のみ。GRID_RETURN 21 回（計 67 s）→ CRUISE_HOLD へ切り替わった。
  中心の待ち 中央 8.9 s（段4b のスタブ 15.2 s）。|G|（0.4 s 傾き）の最大は GRID_RETURN 0.069・全体 0.304 で 0.4G 未満。惰行カーブ・クリープカーブは同定できた。
  スタブは応答が実車の約 1/5 なので、時間・G の数値は実車の目安にしない

### 段7 実機（ユーザーが実施）

```bash
.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"
.venv/bin/python -m tests.research.wltp_grid tests/research/results/drive_log_real_<新>.csv
```

見るもの: 所要時間（見積もり 約 1950 s）、GRID_RETURN と CRUISE_HOLD の秒数、切り替え後の行き過ぎ、|G| 最大 < 0.4G、
網羅表の穴が 065502（3 セル 16 s）から増えていないこと、2-2 の惰行カーブ・クリープカーブが同定できたこと

### 段7 実機結果（`drive_log_real_20260928_035806.csv`）で見つかった不具合と段7c（2026-09-28）

**症状（ユーザー報告）**: 15 km/h ステーションで合わせ込みに時間がかかりすぎる（アクセル 8.1% で長時間キープ）。G 校正の結果表でアクセル上限が全帯「—」。

**原因（CSV 解析で確認）**

1. **GRID_RETURN に打ち切りが無い（バグ）**: 15→25 km/h の戻り（強い減速の助走）で、8.11%（釣り合い車速 21.2 km/h）のまま 457.9〜648 s（190 s）動かなくなり、ユーザーが停止。打ち切り 40 s の判定は CRUISE_HOLD 側にしか無く、GRID_RETURN では永久に待ち続けていた。
2. **アクセル感度の初期値が実測の 1/8**: 最初のステーションではアクセルのステップ（感度を測る）がブレーキより後ろに並ぶため、戻りの時点では仮定値 2.0 km/h/s/% のまま（実測は約 0.26）。開度の上乗せが小さすぎた。
3. **ブレーキのステップ 3 本が空振り**: 開度が不感帯を 0.07〜0.29% 超えただけで、実測は惰行と同じ −2.00 km/h/s（効いていない）。感度も仮定値のままで、1 回ごとに倍にしかならず 1 回 約20 s かかった。
4. **G 校正の加速でアクセルの点が 1 つも取れない**: 記録条件が「開度が止まっている・±0.02G」のみで、40〜110 km/h は G が目標に届かず開度が動き続け、条件を満たす瞬間が無かった。

**修正（実装は Sonnet サブエージェントでなく本セッションで実施。ユーザー決定はすべて推奨案）**

- `pattern_loop.py`: `_advance_grid_hold` の打ち切り処理を `_grid_handle_settle_timeout` に切り出し、`_advance_grid_return` でも判定するように（①のバグ修正）。GRID_RETURN の開度は、踏み始めは従来の式、以降は毎周期 `_step_grid_return` が実測の加速度で刻み足す（②の対策）。G 校正の加速（`_step_coast_accel`）は、開度が止まっていなくても遅れを見込んだ窓の振れが小さければ 1 s に 1 点記録する（④の対策）。コーストダウンの終了を固定 5.0 km/h からクリープ平衡車速（`feedforward.creep_speed_kmh`）基準に変更（ユーザー追加指摘: 5.0 が普遍的な値ではない）。
- `grid_planner.py`・`g_limit.py`: `GLimitMap.strongest_point` を追加し、`GridPlanner` に `gain_fn` を渡して、そのステーションでまだ測っていないペダルの最初の感度を glimit の実測点から引くように（③の対策）。以降は従来どおり実測で上書き。
- `pattern_drive.py`: `learning.grid_settle_tol_kmh` をコーストダウンの終了判定へ渡す配線を追加。
- 新しい調整値は `coast_accel_record_max_change_pct`（**仮の値** 2.0%。根拠・決め時はコード内コメント参照）のみ。他は既存の値・式を共用。
  **段7d（下記）でこの値は不要になり削除した。実機 050944 で見つかった不具合の修正のため。**

**確認**

- 単体テスト: 変更・追加した箇所（`test_research_pattern_loop_grid.py`・`test_research_pattern_loop.py`・`test_research_grid_planner.py`・`test_research_g_limit.py`・`test_research_pattern_drive.py`・`test_research_wltp_grid.py`・`test_research_grid_patterns.py`・`test_research_grid_settle.py`・`test_research_sweep.py`）合計 244 件合格（`test_research_config.py` の既知 3 件は無関係）。`ruff check tests/research` 合格
- スタブ通し（`grid_stub_run --full --stations 15,55 --gain-scale 5 --sweep-max-passes 1 --ref-csv results/drive_log_real_20260925_075149.csv`）: コーストダウン → G 校正 → 格子ステップ 15・55 km/h → 発進停車 → 通し掃引 → クリープ域ブレーキ保持（8/14 パターン目）まで、190 s のような固着なく進行（ユーザー指示で以降は打ち切り）。GRID_RETURN は複数回とも数秒〜十数秒で目標へ収束。
  G 校正の結果表はアクセル上限が全帯埋まった（0〜10 km/h だけ外挿が不安定な値。実機で確認）。スタブは応答特性が別物なので数値は判断材料にしない

### 段7c 実機（ユーザーが実施）

```bash
.venv/bin/python -m pytest tests/research/test_research_pattern_loop_grid.py tests/research/test_research_pattern_loop.py \
  tests/research/test_research_grid_planner.py tests/research/test_research_g_limit.py -q
.venv/bin/ruff check tests/research
.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"
.venv/bin/python -m tests.research.wltp_grid tests/research/results/drive_log_real_<新>.csv
```

見るもの: 15 km/h ステーションで長時間の固着が無いこと・所要時間、G 校正の結果表（アクセル上限が埋まっているか・0〜10 km/h 帯の値が妥当か）、
GRID_RETURN の秒数と打ち切りが出ないか、|G| 最大 < 0.4G、網羅表の穴が 065502（3 セル 16 s）から増えていないこと

### 段7 実機結果2（`drive_log_real_20260928_050944.csv`）で見つかった不具合と段7d（2026-09-28）

**症状（ユーザー報告）**: G 校正の結果表のアクセル上限がおかしい（0〜10 km/h で 227.3%、110 km/h〜で 100% 超え、70〜110 km/h が 60.5% で平ら）。

**原因（CSV を同じ計算方法で再現し、表と数%以内で一致させて確認。ユーザーと議論して確定）**

| 帯 | 症状 | 原因 |
|---|---|---|
| 0〜10 km/h | 227.3% | **バグ（段7c）**: コーストダウン加速の最後の指令 70% が `_ca_opening` に残ったまま、次の G 校正の踏み始め（接近ランプ中）に開度の履歴へ積まれ、踏み始め直後に (70%, 低加速度) という偽の点になっていた |
| 110 km/h〜 | 100% 超え | 機構上限近く（70%）まで踏んでも 0.09〜0.14G しか出ず、0.3G まで直線で延ばすと 100% を超える（＝「届かない」の意味。上限としては無害） |
| 70〜110 km/h | 60.5% で平ら | 開度が約 3.5%/s で上がり続け、記録条件「1 s の振れ ≤ 2%」を満たさず点が 0 個。60〜70 km/h の値を借用していた |
| 10〜30 km/h | 49.9%（格子ステップの実測から延ばすと約 42.5%） | 指令開度で記録。低速の踏み込みは指令 16%/s に対し実アクチュエータ約 8%/s しか追従できず、実開度が指令より 5〜8% 遅れていた（危険側） |

**修正（ユーザー決定 2026-09-28。実装は本セッション＝Sonnet）**

- `pattern_loop.py`: G 校正の加速の記録条件を「開度が止まっている／振れが小さい」からやめ、実開度（`monitor_accel` の位置から。指令ではなく実アクチュエータの位置）を遅れ `gov_lead_s`（0.6 s。既存値の流用）ぶん過去へずらした窓で平均し、今の G と対にして 1 s に 1 点記録する方式に統一（`_ca_lagged_opening`）。振れの条件は無くしたので、開度が動き続けていても 70〜110 km/h を含め全帯で点が取れる。`_enter_phase` で開度の履歴も空にする（前のパターンの値が残らないように）。段7c で足した仮の調整値 `coast_accel_record_max_change_pct` は削除（不要になった）。
- `research_types.py`: `position_to_opening`（`opening_to_position` の逆関数）を追加。`_execute_one_cycle` で毎周期、実開度を保存する。
- `g_limit.py`: `GLimitMap.table` で予測開度が 100% を超える帯は「100%(届かない)」と表示（`cap_pct` の値・頭打ちとしての動きは変えない）。

**確認**

- 単体テスト: `test_research_pattern_loop.py`・`test_research_g_limit.py`・`test_research_pattern_loop_grid.py`・`test_research_grid_planner.py` 合計 152 件合格。`ruff check tests/research` 合格
- スタブは実施していない（ユーザー指示。実機ですぐ確認）
- 050944 の CSV を新しい方法で再計算した期待値: 0〜10: 19.1 / 10〜20: 34.9 / 20〜30: 40.7 / 30〜40: 50.6 / 40〜50: 52.4 / 50〜60: 59.1 / 60〜70: 59.7 / 70〜80: 70.1 / 80〜90: 77.3 / 90〜100: 88.7 / 100 km/h〜: 100%(届かない)

### 段7d 実機（ユーザーが実施）

```bash
.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"
.venv/bin/python -m tests.research.wltp_grid tests/research/results/drive_log_real_<新>.csv
```

見るもの: G 校正の結果表のアクセル上限が、上の期待値と近い形（車速とともに単調に上がる・110 km/h〜は「100%(届かない)」・0〜10 km/h が数十%）になっているか

### 段7d 実機結果と手順3 比較・クローズ（`drive_log_real_20260928_062254.csv` / `070218.csv`。Claude 確認 2026-09-28）

**手順2（062254）**

- **G 校正の結果表**: 走行開始直後（t=358.5s）の予測は 0〜10 km/h 43.8%・10〜20 33.1%・…・100〜110 99.8%・110 km/h〜「100%(届かない)」で、
  車速とともにおおむね単調に上がり 110 km/h〜が「届かない」になる形になった（段7d の期待どおり）。0〜10 km/h が偽の点（227.3%）に戻っていないことを確認（段7c のバグは再発せず）。
  走行後（格子ステップ・掃引のデータが増えた t=1964.1s）の表も同じ形を保った（30〜130 km/h 帯で数 % の見直し、110〜160 km/h は「届かない」または外挿域として残存）
- **所要時間**: 全パターン完走 1750s（目標 1800s 以内）。065502（段6d、約 3035s）・035806（段7、固着で中断）・050944（段7c、時間は改善したが表がおかしかった）と比べて、段7 の目的（計測効率化）を達成
- **落ち着き判定の集計**: 待ち時間の合計 1017s（走行時間の 58%）。中心 109 回（打切り 0、待ち中央 5.8s）、助走 14 回（打切り 0、待ち中央 13.4s）。GRID_RETURN の固着（035806 で 190s）は発生せず
- **モデル作成（2-2）**: 逆モデル アクセル R²=0.968・ブレーキ R²=0.829。惰行カーブ・クリープカーブ・停止下限・ペダルゲインを推定して config へ書き戻し済み（`test_vehicle_20260928_065534.pkl`）

**手順3（070218。FF のみ、`report20260928_RunFF.md`）を段2 の基準（075149）と比較**

| KPI | 基準 075149 | 今回 070218 | 差 |
|---|---|---|---|
| 最大逸脱 | 3.21 km/h | 3.41 km/h | +0.20 |
| 偏差 p95 | 0.86 km/h | 0.90 km/h | +0.04 |
| 符号反転（最大 / 5s） | 3 回 | 3 回 | 0 |
| ペダル往復（全体） | 1.84 回/s | 1.08 回/s | −0.76（改善） |
| ペダル往復（60s 最大） | 4.49 回/s | 2.30 回/s | −2.19（改善） |

- 最大逸脱・p95 の差は [[run-to-run-variability-warmup]] で確認済みのばらつき幅（1 本目の暖機ずれ・イベント単位で |偏差|>1.0 合計が本ごとに 29〜47s ばらつく）に収まる
- ペダル往復（滑らかさ）は全体・60s 最大とも大きく改善

**ユーザー判断（2026-09-28）**: 段7（計測効率化・手順2 の不具合修正）後の手順2・手順3 は、基準と比べて KPI が悪化していない。本問題をクローズする。

## 段4b の結果（Claude 確認）

- **追加・変更**:
  - `modes.coverage_mode_names`（config・yaml）。`wltp_grid.combined_cell_stats`（合成）・`outside_seconds`（格子の外の秒数）・`coverage_stats_from_modes`・`load_coverage_stats`。
    `main.step2_pattern_drive` は 2-0 の前に全モードを読んで合成し、モードごとの「格子外 N 秒」を表示する（読めないモードがあれば走る前に止まる）。
    `wltp_grid` CLI の網羅表も合成に切り替え、`grid_stub_run` も同じ
  - `learning.grid_hole_wltp_min_s` を**削除**（旧キーは ConfigError）。最小秒数は `LearningSection.grid_target_min_s`（= 窓 − 頭の除外）から自動。
    穴の判定（網羅表・`grid_stub_run`）も同じ値を使う
  - `grid_accel_edges_kmhs` を `[-14, -7, -3, -1.5, -0.5, 0.5, 1.5, 3, 7, 14]` に
  - 発進・停車の狙いは `plan_launch` のまま（入力が合成集計になっただけ。config に狙いは持たない）
  - 助走: `learning.grid_step_min_fit_s`（新キー）、`GridPlanner.is_strong`・`peek_approach_kmh`・`next_step(u0_pct)`、`Step.approach_kmh/base_pct`、
    `pattern_loop` の助走（PI の目標を助走の車速へ → 落ち着いたらその窓の開度を u0' に踏む → 帯の反対側で終了 → 中心へ戻る。落ち着かなければそのステップだけとばす）。
    格子の 1 行表示に「助走 N km/h」が付く
  - **落ち着き判定の集計**（`grid_settle.py`。**計画からの変更**: 計画では CSV からの CLI だったが、助走かどうか・PI の目標は CSV に無いので、`PatternLoop` が走行中に記録して
    2-1 の終わりに表で出す形にした。CLI は無し）: 1 回ごとに 待ち時間・許容幅内にいた割合・落ち着いた窓の開度の平均と σ・打ち切りか、まとめに待ち時間の合計（走行時間の何 %）・中心と助走の別、
    ステップの傾きの当てはめ（点数・残差）
- **スタブ通し**（`grid_stub_run --full --stations 15 --gain-scale 5`、WLTP + US06 の合成。走行 1139s＝19.0 分。スタブは応答が遅く実車の所要は見積もれない）:
  - 走行の最初に「狙うモード（格子外 0s ×2）」、15 km/h ステーションは減速 4 列・加速 4 列、発進・停車は発進 4 列・停車 4 列（狙い +0.96/+2.23/+4.3/**+9.39**、−0.98/−2.15/−4.4/**−7.98**）
  - 助走が動いた: 強い減速 −8.0 は「助走 25km/h」から（実測 −6.94 でやり直し → 2 回目 −7.89 OK）、強い加速 +9.34 は「助走 10km/h」から（実測 +10.40 OK）。
    発進・停車は +9.39 → 実測 +11.94、−7.98 → −7.00（やり直し）
  - **実装中に見つけた不具合と修正**: 最初の走行では、15 km/h ステーションで −4.4 km/h/s の減速ステップを踏むと車速が 0 まで落ち、「低速まで落ちたら残りをやめて停車復帰」の処理で
    後ろの強い減速（−8）と加速（+9.3 を含む全部）が測られずに終わっていた。低速まで落ちても残りを捨てず、開度を u0 に戻して PI でステーションへ戻る形に直した（テスト追加）。
    修正後は上のとおり加速の列まで測れた
  - 走行の終わりに「落ち着き判定の集計」が出た（スタブの値なので判断しない）: 待ち 19 回・合計 283s（走行時間の 25%）、中心 待ち 中央 15.2s／最大 34.8s、助走 待ち 17.6s（10.4〜17.6s）、
    落ち着いた窓の開度の σ は中央 0.02%／最大 0.16%。ステップの傾きの当てはめは 19 本、点数 中央 26／最小 12、車速の残差 σ 中央 0.07／最大 1.78 km/h
  - 2-2 のモデル作成まで通った（YAML は更新せず）。網羅マップは 1 ステーションだけなので穴だらけ（85%）。網羅の評価は段5
- **単体テスト**: 変更箇所のテスト（`test_research_wltp_grid`・`grid_planner`・`grid_settle`（新規）・`grid_patterns`・`pattern_loop_grid`・`pattern_loop`・`config`・`pattern_drive`・`no_src_import`）190 件合格
  （既知の `test_stop_brake_floor_settings_are_validated` 1 件のみ除く）。`ruff check tests/research` 合格

## 段2 の結果（Claude 確認）

- 基準（重みなし）: 手順2 `drive_log_real_20260925_070629.csv`、手順3 `drive_log_real_20260925_075149.csv`（最大逸脱 3.21 km/h・p95 0.86・ペダル往復 1.84 回/s）
- 修理後の 2-0 実測: 不感帯 アクセル 5.37% / ブレーキ 7.26%、停車保持 18.63%
- 網羅表（新 CSV、合算）: 穴 4 セル＝WLTP 101s（6%）。20〜30 km/h・30〜40 km/h・60〜70 km/h の加速側と、60〜70 km/h の緩い減速。50 km/h 以上の強い加速（WLTP ≒ 0s）に学習データが 30〜40s ずつある
- 追加: `tests/research/sample_weight.py`（重み = clip(WLTP 占有率 ÷ 学習データ占有率, w_min, w_max) → ペダルごとに平均 1。格子外・停車の行は 1）、config `learning.sample_weight_enabled/min/max`（既定 false / 0.2 / 5.0）、`relearn --weight {config,on,off} --ref-csv`、pkl に `sample_weight` キー、metrics に `mae_wltp`（WLTP で重み付けした MAE）
- テスト合格（既知の stop_brake_floor の 1 件を除く）、`ruff check` 合格
- dry-run（同じ CSV）:

| | アクセル MAE | アクセル MAE_WLTP | ブレーキ MAE | ブレーキ MAE_WLTP |
|---|---|---|---|---|
| 重みなし | 1.92 | 2.01 | 2.52 | 2.04 |
| 重みあり | 2.13 | 1.93 | 2.49 | 1.92 |

  MAE_WLTP は重みありの学習目的そのものなので、下がるのは当然。**採否は手順3 の実機比較で決める**
- 重みの張り付き: アクセルは 29% が下限、ブレーキは 59% が下限（WLTP で使わない領域のデータが多い）
- 未実施: CV（`model_analysis.py`・`kaizen.py`）への重みの組み込み（kaizen 側は段4 で削除予定のため）、ペダル別の網羅

### 手順3 比較（重みなし 075149 vs 重みあり 122437）

- 重みあり: pkl `test_vehicle_20260925_122203.pkl`（pkl 内 `sample_weight.enabled=True`、metrics が dry-run と一致 → 重みありで走ったことを確認）。
  手順3 `drive_log_real_20260925_122437.csv`、レポート `results/report20260925_RunFF_2.md`。どちらの走行も暖機済み

| | 最大逸脱 [km/h] | p95 [km/h] | 符号反転 [回/5s] | 1.0 超え | ペダル往復 全体 / 60s 最大 [回/s] | 実車速 帯RMS [km/h] |
|---|---|---|---|---|---|---|
| 重みなし | 3.21 | 0.86 | 3 | 49 回・67.0s | 1.84 / 4.49 | 0.018 |
| 重みあり | 3.04 | 0.86 | 4 | 47 回・68.7s | 1.43 / 2.69 | 0.015 |

イベント単位（基準時刻をそろえ、同じ状態が 2s 以上続く 170 イベント。|偏差| 平均の差が ±0.05 km/h を超えたら良化／悪化）:

| 状態 | 本数 | 良化 | 悪化 | 同等 | 平均偏差 なし → あり [km/h] |
|---|---|---|---|---|---|
| 加速 | 62 | 14 | 24 | 24 | +0.11 → −0.13 |
| 定速 | 37 | 3 | 8 | 26 | −0.04 → −0.06 |
| 減速 | 62 | 11 | 6 | 45 | −0.07 → −0.01 |
| 停車 | 9 | 1 | 0 | 8 | +0.52 → +0.50 |

- 悪化は 40 km/h 未満の加速に集中（平均偏差 +0.2 → −0.1〜−0.26。実車速が基準より遅い側へ）
- 良化は 80 km/h 以上（|偏差| > 0.4 の秒 9.3s → 0.1s）と 20〜40 km/h の減速
- 同じ場面のアクセル指令は重みありが 0.2〜0.6% 深い
- **結論（ユーザー判断 2026-09-25）: 重みあり／なしで結果の優位性はない**。差は走行ごとのばらつきの幅に収まる → 段2 完了
- 現状: config の `model_path` は重みありの pkl（`test_vehicle_20260925_122203.pkl`）のまま、`sample_weight_enabled` は false。
  重み付けのコード（`sample_weight.py`・`relearn --weight`）は既定 off で残っている

## 段1 の結果（Claude 確認）

- 追加: `tests/research/wltp_grid.py`、`test_research_wltp_grid.py`（12 件合格）、config `learning.grid_*` 4 キー（`config.py`・`config_testVehicle.yaml`）
- `ruff check` 合格。`test_research_no_src_import` 合格
- 9/24 の手順2 CSV（修理前。表が出るかの確認のみ）で実行:
  - WLTP（停車を除く）1573s。学習データはアクセル 683s / ブレーキ 302s / 惰行 175s
  - 穴は 16 セル＝WLTP の 209s（13%）。**中速（30〜100 km/h）の緩い減速（−1.5〜−0.5 km/h/s）と、50〜70 km/h の定速**に集中
- **ユーザー確認で見つかった修正（2026-09-25。実装済み）**:
  1. 加速度の列の境界を `[-7,-3,-1.5,-0.5,0.5,1.5,3,7]` に変更（±0.5 を「定速」の 1 列に）。
     定速階段（CRUISE_HOLD 293s）の実測加速度は PI のハンチングで ±1 km/h/s ほど揺れ、±0.1 に入る行は 13%、±0.5 に入る行は 63% だった。
     WLTP 側は揺れないので、細い定速の列だと食い違って数えていた
  2. 学習データの秒を小数 1 桁で表示（「1.5s」が「2」と出ながら穴と判定される食い違いを解消）。穴のしきい値が仮置きであることを出力に注記
  3. 修正後の 9/24 CSV: 穴 12 セル＝WLTP 167s（11%）。定速の穴は消え、**中速（30〜100 km/h）の緩い減速（−1.5〜−0.5 km/h/s）と、その一段強い減速（−3〜−1.5）**に集中
- 段2 の基準走行の後に足す: ペダル別の網羅（惰行カーブから「WLTP がそのセルで使うペダル」を決め、同じペダルのデータだけを数える）。現在の表はアクセル/ブレーキ/惰行の合算
- 注意: `test_research_config.py::test_stop_brake_floor_settings_are_validated` が失敗する。原因は今回の変更ではなく、
  作業ツリーの `config_testVehicle.yaml` の `feedforward.stop_brake_floor_offset_pct` が 0.5（HEAD は 6.0、テストは 8.0 を期待）になっていること。ユーザーの調整中の値と思われるので触っていない

## 進捗ログ

| 日付 | 内容 |
|---|---|
| 2026-09-25 | 議論・計画作成・ユーザー承認。段0 完了。段1 実装・Claude 確認済み |
| 2026-09-25 | ユーザー確認で「定速の列が細すぎる・表示の丸め」を指摘 → 修正・テスト合格（再確認待ち） |
| 2026-09-25 | 段1 をユーザーが確認 OK。段2 は別ターミナルで実施 |
| 2026-09-25 | 段2: 修理後の基準走行（手順2→3、重みなし）完了。重み付け実装・Claude 確認済み（重みありの手順3 待ち） |
| 2026-09-25 | 段2: 重みありの手順3（122437）完了。重みなしと比べて優位性なし（ユーザー判断）→ 段2 完了。次の段は別ターミナルで進める |
| 2026-09-25 | 段3: 格子ステップ走行を実装・スタブ確認（×1・×5 は流れ OK、×0.2 は整定せず）。ユーザー確認待ち |
| 2026-09-25 | 段3 完了（ユーザー判断）。感度 1/5 以下の対策は後回し。段4 は別ターミナルで実施 |
| 2026-09-25 | 段4: 手順2 のパターン列を格子ステップ走行に置き換え、旧パターン・旧ツール・旧 config キー・C6 を削除。単体テスト（全体）・ruff 合格、スタブ通し（15 本・2-2 まで）完走。ユーザー確認待ち |
| 2026-09-26 | 段4・段4b をユーザーが完了と判断。段5（実機）は別ターミナルで実施 |
| 2026-09-26 | 段5 で走る手順2 の流れを記載。段5 完了後に RPD へ記載して正式採用（ユーザー決定） |
| 2026-09-26 | 段5 の前の議論。狙うモードをリスト化（WLTP + US06）、5 秒のしきい値を 1 ステップの長さ（2.5 秒）から自動に、発進・停車も選んだモードから自動、加速度の列を ±14 に拡張、強い加減速は助走で測る、落ち着き判定の集計を追加（段4b）。ユーザー確認待ち |
| 2026-09-26 | 段5b: 段5 の実機データを検証（急ブレーキ・停車ステップ・u0 の偏り）→ A〜D を実装。単体テスト・ruff 合格。スタブ通し・ユーザーの実機確認待ち |
| 2026-09-26 | 段5b 追加: コーストダウン前の加速を G 上限の刻み踏みに変更（E）、加速の打ち切りを 30 → 60 s（F、仮値）。単体テスト・ruff 合格。段5 の手順2・3 の実機確認はユーザーが別ターミナルで実施 |
| 2026-09-26 | 段5b 追加: 実機（110427）で刻み踏みは 60 s で 83 km/h・最大 0.06G と遅すぎた。コーストダウン前の加速を「G の余裕に比例して踏む」方式に置換（G、rate_gain=100 は半分仮値）。単体テスト・ruff 合格。実機確認待ち |
| 2026-09-27 | 段5b 実機（113953）を検証: 網羅 穴 9 セル 47 s、0.4G 超え 1 件（0.47G）。上限 G 対策（門①②）と通し掃引を段6 として計画・ユーザー承認。ブレーキ学習の判断は手順3 |
| 2026-09-27 | 段6a: G 校正パターン（G_CALIB）・上限開度マップ（g_limit.py）・格子ステップの開度の上限を実装。単体テスト 226 件・ruff 合格、スタブ通し完走。ユーザー確認待ち（次は 6b） |
| 2026-09-27 | 段6b・6c: 先読みガバナー・踏み込みのランプ・停車復帰の G 比例化・通し掃引を実装。単体テスト 256 件・ruff 合格、スタブ通し完走（|G| 最大 0.318G、掃引 5 本）。実機の手順2→3 はユーザーが実施 |
| 2026-09-27 | 段6d 手順2 実機（065502）を検証: 網羅 穴 3 セル 16 s（測定区間のみ 5 セル 31 s）、0.4G 超え 0 行（最大 0.378G）。governor_active は超過の印ではない。停車加速掃引の 0〜10 km/h が埋まらない件は未対応。ユーザー判断で手順3 へ |
| 2026-09-27 | 段7: 手順2 の計測効率化を議論（065502 の内訳: 戻り 1310 s が最大）。①惰行ステップ廃止 ②単独クリープ発進廃止 ③GRID_RETURN ④コーストダウン1本 を実装。単体テスト 246 件・ruff 合格、スタブ通し完走（1040 s）。実機の手順2 はユーザーが実施 |
| 2026-09-28 | 段7 実機（035806）: 15 km/h ステーションで GRID_RETURN が 190 s 固着・G 校正のアクセル上限が全帯「—」。原因4件（GRID_RETURN 打ち切り漏れのバグ・感度初期値が実測の1/8・ブレーキの空振り・G校正の記録条件）を特定し、段7c として修正（GRID_RETURN の打ち切り共通化・実測補正、glimit からの感度の種まき、G校正の緩やかな記録、コーストダウン終了をクリープ平衡基準に）。単体テスト 244 件・ruff 合格、スタブ通し 8/14 パターンまで固着なく進行（ユーザー指示で打ち切り）。実機の手順2 はユーザーが実施 |
| 2026-09-28 | 段7 実機2（050944）: G 校正のアクセル上限がおかしい（0〜10 km/h 227.3%・110 km/h〜100%超え・70〜110 km/h 平ら）。原因4件（前パターンの開度が残る偽の点のバグ・0.3Gに届かない・記録条件の振れ判定で点が0個・指令開度で記録して危険側）を CSV 再計算で確認し、段7d として修正（実開度＋遅れずらしの平均で常時記録、100%超えは「届かない」と表示）。単体テスト 152 件・ruff 合格（スタブは実施せず、ユーザー指示で実機へ）。実機の手順2 はユーザーが実施 |
| 2026-09-28 | 段7d 実機（062254）: G 校正表が期待どおりの形（単調増加・110 km/h〜「届かない」・偽の点なし）、所要 1750s（目標 1800s）。手順3（070218）を段2 の基準（075149）と比較: 最大逸脱 3.21→3.41・p95 0.86→0.90（ばらつき幅内）、ペダル往復 全体 1.84→1.08・60s 最大 4.49→2.30（改善）。KPI 悪化なし（ユーザー判断）→ **本問題クローズ** |
