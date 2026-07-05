# 設計書

## アーキテクチャ概要

変更は `src/domain/pid_tuning.py`（Tuner）と `src/domain/control/pid.py`（微分 LPF）の2ファイルに
閉じる。Tuner の公開 API は不変のため、`robot_controller.run_pid_tuning_session`・Web API・UI は
無変更。

```
run_pid_tuning_session（無変更, robot_controller.py:933-964）
   │ while (cand := tuner.next_candidate()) is not None:
   │     set_gains → 走行 → tuning_cost → tuner.report(cand, cost)
   ▼
CoordinateDescentTuner（変更: ラウンド一括生成 → 巡回座標降下）
   baseline → kp± → ki± → kd± → kp± → …（各座標で採用即・次座標へ）

PIDController.update（変更: 微分項に1次LPF）
   d_raw = (error − prev_error)/dt → d_filt += (d_raw − d_filt)·dt/(τf + dt)
```

## 現状の問題（調査結果の記録）

`CoordinateDescentTuner`（pid_tuning.py:309-385）の現実装:

- `_gen_round()`（:341-349）が `[kp+, kp−, ki+, ki−, kd+, kd−]` の6候補を一括生成
- `report()`（:364-373）は改善時に `self._pending = []` で**残候補を全破棄**して再センタリング
- ステップ幅は `step × (|現在値| + _BASE)`、`_BASE = {"kp": 1.0, "ki": 0.1, "kd": 0.1}`（:322）

これにより Kd が変更されない3要因:

1. **順序**: kd 候補はリスト末尾。kp/ki のどれかが改善する度にラウンド破棄されるため、
   SIMC 初期値が粗い序盤（kp/ki で改善が出やすい）は kd 候補が走行前に必ず捨てられる。
2. **予算**: `max_runs=15` = baseline(1) + 6候補ラウンド × 2強。kd 候補（各ラウンド5・6番目）
   に到達する前に予算が尽きる。
3. **スケール**: kd=0 起点の候補は `0.3 × (0 + 0.1) = 0.03` のみ。マイナス側は
   `max(0, −0.03) = 0` で best と同一（1走行が無駄）。実機ノイズに対し kd=0.03 の効果は
   測定不能で、評価されても best_cost を下回らない。

加えて `PIDController.update`（pid.py:68）の微分は生の後退差分。CAN 車速の量子化（±1km/h 級）が
dt=50ms で ±20km/h/s の微分スパイクになるため、フィルタなしでは意味のある Kd はハンチング
（reversal KPI 悪化）でコスト棄却され続ける。**Tuner を直しても LPF がないと Kd は「評価される
が常に負ける」状態になる**ため、両方を1作業で行う。

## コンポーネント設計

### 1. `CoordinateDescentTuner`（`src/domain/pid_tuning.py` 変更）

**方式**: ラウンド一括生成をやめ、巡回座標降下（cyclic coordinate descent）にする。

- 座標列 `("kp", "ki", "kd")` を巡回。各座標に入る時点の best を中心に `+step` 候補・
  `−step` 候補を**遅延生成**する（採用済みの改善が次座標の中心に反映される）。
- 各座標の評価: まず `+` 候補。改善（`cost < best_cost`）なら採用して**次の座標へ**。
  改善しなければ `−` 候補を試し、改善なら採用して次の座標へ、だめなら次の座標へ。
- 候補値は `max(0, cur ± step × (|cur| + _BASE[p]))`。クランプ後の値が現在の best の値と
  一致する候補は**生成せずスキップ**（kd=0 の `−` 側などの無駄走行排除）。
- 1巡（kp→ki→kd）で一度も改善がなければ `step *= 0.5`。`step < min_step_frac` または
  `runs >= max_runs` で `next_candidate()` が None（既存の停止則と同一）。
- `_BASE` を `{"kp": 1.0, "ki": 0.1, "kd": 0.5}` に変更（kd 初回候補 = 0.3×0.5 = 0.15）。
- 公開 API・コンストラクタ引数・`report(gains, cost)` の呼び出し契約は不変。
  ベースライン測定（初回に initial を評価）も既存どおり。

**走行遍歴の保証**: baseline(1) → kp(≤2) → ki(≤2) → kd(≤2) となり、遅くとも7走行目までに
kd 候補が必ず走行される。max_runs=15 で約2巡+α を確保。

**実装スケッチ**（内部状態の持ち方は実装者に委ねるが、契約はこのとおり）:

```python
_PARAMS = ("kp", "ki", "kd")
_BASE = {"kp": 1.0, "ki": 0.1, "kd": 0.5}

# 内部状態: _param_idx（現座標）, _directions（現座標の残方向 [+1.0, -1.0]）,
#           _improved_this_cycle（巡回内改善フラグ）, _pending（次に返す候補 0..1 個）
# next_candidate():
#   runs >= max_runs → None
#   ベースライン未評価なら initial を返す
#   現座標・現方向から候補を生成。クランプで best と同値なら方向/座標を進めて再試行
#   全座標を一巡して improved なし → step 半減、min_step 未満なら None
# report(gains, cost):
#   runs += 1
#   cost < best_cost → best 更新・improved フラグ・次の座標へ進む（残方向は破棄）
#   それ以外 → 同座標の次方向へ（尽きたら次の座標へ）
```

### 2. `PIDController`（`src/domain/control/pid.py` 変更）

- 微分項に1次 LPF を追加:
  `d_raw = (error − prev_error) / dt_eff`
  `self._d_filt += (d_raw − self._d_filt) × dt_eff / (DERIV_FILTER_TAU_S + dt_eff)`
  出力は `kd × self._d_filt`。
- `DERIV_FILTER_TAU_S = 0.2`（モジュール定数）。公称 dt=50ms の4倍で、量子化スパイクの
  1サンプル影響を約 dt/(τf+dt) = 0.2 倍に抑制しつつ、FOPDT 同定の τ（秒オーダー）より
  十分速く位相遅れは実用上無視できる。
- `reset()` で `_d_filt = 0.0` もクリア（`set_gains()` は reset 経由なので自動で効く）。
- kd=0 のプロファイルでは微分項が出力に乗らないため挙動不変（既存テスト無修正で通ること）。

### 3. `robot_controller.py` / Web API / UI

無変更。`run_pid_tuning_session`（robot_controller.py:933）は Tuner の公開 API のみに依存。

## エラーハンドリング戦略

- 既存踏襲。Tuner はハード非依存の純粋ロジックのままで、走行失敗（INVALID_COST）は
  「改善なし」として扱われ次候補へ進む（現行と同じ）。
- 非常停止時の `PidTuningAborted` → `finally` で原点復帰、という既存経路に変更なし。

## テスト戦略

### ユニットテスト（`tests/unit/test_pid_tuning.py` 変更・追加）

既存 `TestCoordinateDescentTuner` 3件（test_converges_on_convex_cost / test_stops_at_max_runs /
test_keeps_best_gains）は新遍歴でも成立するはずだが、候補順に依存するアサーションがあれば調整。

追加:
- `test_kd_candidate_evaluated_within_budget`: kp/ki が毎回改善する単調コストでも、
  max_runs=15 内（遅くとも7走行目まで）に kd≠best.kd の候補が `next_candidate()` から出る
- `test_kd_improves_when_beneficial`: kd のみ改善が効く凸コスト（モック）で `best.kd > 0` になる
- `test_no_duplicate_clamped_candidates`: initial kd=0 のとき、best と同値の候補が返らない
- `test_step_halves_after_full_cycle_without_improvement`: 1巡改善なしでステップ半減→
  min_step 未満で None

### ユニットテスト（`tests/unit/test_pid.py` 変更・追加）

- 既存テストが無修正で通ること（kd=0 の互換性確認を兼ねる）
- `test_derivative_lpf_attenuates_spike`: 1サンプルの誤差ジャンプに対する微分項出力が
  未フィルタ理論値の約 0.2 倍に減衰する
- `test_reset_clears_derivative_filter`: reset 後の初回微分がフィルタ残留の影響を受けない

## 依存ライブラリ

新規追加なし。

## ディレクトリ構造

```
src/domain/pid_tuning.py       （変更: CoordinateDescentTuner 巡回化・_BASE["kd"]）
src/domain/control/pid.py      （変更: 微分1次LPF）
tests/unit/test_pid_tuning.py  （変更: Tuner テスト調整+追加）
tests/unit/test_pid.py         （変更: 微分LPFテスト追加）
```

## 実装の順序

1. `pid.py` 微分 LPF + テスト（独立・影響ゼロなので先行）
2. `CoordinateDescentTuner` 巡回化 + テスト
3. 品質チェック（pytest / ruff）

## セキュリティ・安全考慮事項

- 走行の安全機構（規定パターンの上限G/最高車速厳守、hard 違反ペナルティ、非常停止経路、
  max_runs 予算）は一切変更しない。
- kd の探索は 0 起点 + コスト採否のため、悪化する Kd は採用されない（自己防衛的）。

## 将来の拡張性

- SIMC PID 形（τD=θ/2 相当）による Kd 解析初期値は、本作業で LPF が入った後に導入すると
  安全（20260620 design.md「将来の拡張性」の残項目）。
