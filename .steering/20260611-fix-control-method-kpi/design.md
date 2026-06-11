# 設計書

## アーキテクチャ概要

現行の制御経路は「FF が (accel, brake) ペアを出力 → DriveLoop が PID を符号分割して加算 → 事後のアクセル優先排他で消去」という構成で、ペダル調停ロジックが FF・DriveLoop の 2 層に分散している。
本作業では **符号付き単一努力量(effort)アーキテクチャ** に再編する。

```
                 ┌────────────────────────── DriveLoop (50ms) ──────────────────────────┐
 基準車速 ──┬──▶ FeedforwardController.predict_effort() ──▶ ff_effort [%] (+加速/−制動)  │
            │                                                      │                     │
 実車速 ────┼──▶ PIDController.update(ref, act, dt, sat_flags) ──▶ pid_u [%] (±制限付き) │
            │                                                      ▼                     │
            │                                    effort = ff_effort + pid_u              │
            │                                                      │                     │
            │    PedalArbiter.arbitrate(effort, dt)                ▼                     │
            │      ・切替ヒステリシス / アクセル再踏込ディレイ                          │
            │      ・不感帯逆補償 / レートリミット / 最大開度クランプ                   │
            │      ・構造的同時踏み禁止(常に一方のみ非ゼロ)                           │
            │      ・飽和フラグ → 次サイクルの PID 条件付き積分へ                       │
            │                  │                                                         │
            │                  ▼ (accel_opening, brake_opening)                          │
            │            _opening_to_position → アクチュエータ                          │
            └──▶ KPIMonitor.update(ref, act) … P95 / 最大偏差 / 符号反転を常時集計 ──────┘
```

- FF は「基準軌跡のみの関数」という性質を維持(純粋フィードフォワード)。出力を符号付き効果量に変更し、レジーム境界をブレンドで連続化する。
- PID は誤差補正のみを担い、出力制限・条件付き積分・計測 dt 対応を持つ。
- **ペダルへの写像は PedalArbiter のみが行う**。「同時踏みなし」はこの層の構造的性質となり、事後の消去ルール(現 drive_loop.py:205)は削除する。

## コンポーネント設計

### 1. PedalArbiter(新規: `src/domain/control/pedal_arbiter.py`)

**責務**:
- 符号付き努力量 → (アクセル開度, ブレーキ開度) への写像。常にどちらか一方のみ非ゼロ。
- 振動抑制 KPI(符号反転 ≤1 回/5 秒)を機構として支えるヒステリシス・ディレイ・レートリミット。
- 不感帯逆補償(最終指令が物理死帯 (0, deadband) に入らないことを保証)。
- 最大開度クランプと飽和フラグの提供(PID アンチワインドアップ用)。

**インターフェース**:
```python
@dataclass
class ArbiterOutput:
    accel_opening: float   # [%] 0 または deadband 以上
    brake_opening: float   # [%] 同上。accel と排他
    saturated_high: bool   # 正方向の要求が上限・ディレイ・レート制限で削られた
    saturated_low: bool    # 負方向の要求が上限・レート制限で削られた

class PedalArbiter:
    def __init__(self, params: FeedforwardParams,
                 max_accel_opening: float, max_brake_opening: float) -> None: ...
    def reset(self) -> None: ...
    def arbitrate(self, effort: float, dt: float) -> ArbiterOutput: ...
```

**arbitrate のロジック(順序が仕様)**:
1. **ペダル選択(ヒステリシス)**: `h = params.switch_hysteresis_pct`。`effort > +h` → ACCEL、`effort < -h` → BRAKE、それ以外 → COAST(両方 0)。±h の帯内で符号がチャタついてもペダル反転しない。
2. **アクセル再踏込ディレイ**: ブレーキが非ゼロだった時刻から `accel_reengage_dwell_s` 以内は ACCEL 選択を COAST に落とす(`saturated_high=True`)。**ブレーキ側にディレイは設けない**(減速権限を遅延させない安全要件)。
3. **不感帯逆補償**: 選択ペダルの不感帯 `db` に対し `u = |effort|` を `u < db/2 → 0`、`db/2 ≤ u < db → db`、`それ以外 → u` に写像。指令が物理死帯内 (0, db) に落ちない。
4. **レートリミット**: 選択ペダルの開度変化を `±rate_limit × dt` に制限。**非選択ペダルは即座に 0**(解放は制限しない。両ペダル同時非ゼロの過渡を作らないため)。
5. **クランプ**: `[0, max_opening]`。クランプ・レート制限・ディレイで要求より小さい出力になった場合に方向別の飽和フラグを立てる(ヒステリシス帯内の COAST は飽和ではない)。

**実装の要点**:
- 内部時刻は `arbitrate` 呼び出しごとの `dt` 累積で管理(イベントループ時刻に依存せずユニットテスト可能)。
- ブレーキ解放後のディレイ起点は「ブレーキが最後に非ゼロだった内部時刻」。

### 2. FeedforwardController の連続化(`src/domain/control/feedforward.py`)

**変更点**:
- `predict()` → `predict_effort(v0, future_speeds) -> float`(符号付き [%]、+加速/−制動)に変更。
- **停車保持**(レビュー #5): 判定を「全先読み点 ≤ 0.5km/h」から「`v0 ≤ 0.5 かつ future_speeds[0](=0.5 秒先) ≤ 0.5`」へ変更。発進 0.5 秒前まで保持ブレーキ(`-stop_brake_opening_pct`)を維持する。
- アクセル・ブレーキ両モデルを常に評価し、レジーム別の補正を適用:
  - **クリープ則**(レビュー #8): `v0 < creep_speed かつ 0 ≤ desired_accel ≤ creep_rate` のときのみ `accel_pred = 0`。abs() を廃止し、低速減速時のブレーキ予測は残す。
  - **惰行則**(レビュー #8): `dv_regime < 0 かつ v0 ≥ creep_speed かつ −desired_accel ≤ engine_brake` のとき `brake_pred = 0`。クリープ速度未満では適用しない(ペダルオフで加速する領域)。
- **レジームブレンド**(レビュー #9): `REGIME_BLEND_BAND_KMH = 0.5`(モジュール定数)。
  `w = clip((dv_regime + band) / (2*band), 0, 1)`、`effort = w * accel_pred − (1−w) * brake_pred`。
  dv_1.0 = 0 近傍で出力が連続になり、巡航うねりでの FF ドロップアウトを除去する。
- **不感帯スナップを削除**(レビュー #11): 不感帯処理は PedalArbiter(FF+PID 合成後)に一本化。
- `unload_model()` を新設(レビュー #4): `_accel_model = _brake_model = None`。

### 3. PIDController の強化(`src/domain/control/pid.py`)

```python
def __init__(self, kp, ki, kd, dt=0.05, output_limit=100.0) -> None: ...
def set_output_limit(self, limit: float) -> None: ...
def update(self, setpoint, measurement, dt=None, *,
           saturated_high=False, saturated_low=False) -> float: ...
```

- **計測 dt**(レビュー #12): `dt` 引数(None なら公称値)。`[0.5×公称, 4×公称]` にクランプし、サイクルスキップ時の微分スパイク・積分過小評価を防ぐ。
- **条件付き積分**(レビュー #3): `(error > 0 and saturated_high)` または `(error < 0 and saturated_low)` のとき積分を停止(飽和方向へ巻き上げない)。
- **積分量クランプ**: `ki > 0` のとき `|integral| ≤ output_limit / ki`(I 項単独で出力上限を超えない)。
- **出力制限**: 戻り値を `±output_limit` にクランプ。limit はプロファイルの `pid_output_limit_pct` を `set_output_limit` で反映。

### 4. KPIMonitor(新規: `src/domain/control/kpi_monitor.py`)

**責務**(レビュー #7): プライマリー KPI の実行時計測。

- `update(ref_kmh, actual_kmh, now_s)`: 毎サイクル呼び出し。
  - 偏差 `dev = actual − ref` の絶対値最大を追跡。
  - 固定幅ヒストグラム(幅 0.01km/h、0〜10km/h + オーバーフロー)で P95 を算出可能にする(10h × 20Hz = 72 万点でもメモリ一定)。
  - 符号反転: ノイズフロア `±0.05km/h` を超えた符号のみ採用し、反転時刻を 5 秒窓 deque で保持。任意 5 秒窓の最大反転回数を追跡。
  - `|dev| > 1.0km/h` でハード上限違反: 違反突入時に 1 回 warning ログ(解除ヒステリシス 0.9km/h)、違反回数をカウント。
- `summary() -> dict`: `{n, max_abs_deviation_kmh, p95_kmh, reversal_max_per_5s, hard_limit_violations}`。
- 走行終了時に DriveLoop 経由でサマリを取得し、RobotController が INFO ログ + `last_kpi_summary` 属性で公開する(DB 保存・GUI 表示はスコープ外)。

### 5. DriveLoop の統合(`src/domain/control/drive_loop.py`)

- **effort 経路**: `ff_effort(has_model 時) + pid_u` → `arbiter.arbitrate(effort, dt)`。旧コード(符号分割・排他・max クランプ)を削除。飽和フラグを保持し次サイクルの `pid.update` に渡す。
- **dt 計測**: 前回 PID 更新からの `loop.time()` 差を計測して `pid.update` と `arbitrate` に渡す(初回は公称値)。
- **緊急停止ヘルパー**: 6 箇所の `stop(); await on_emergency(); return` を `await self._abort_emergency()` に集約。
- **ウォッチドッグ**(レビュー #16): `_schedule_next_cycle` で連続スキップを数え、`連続スキップ時間 ≥ WEDGED_CYCLE_TIMEOUT_S(1.0s)` で error ログ + 停止 + 非常停止タスク起動。サイクル起動成功でリセット。
- **stop_and_join(timeout_s=2.0)**(レビュー #6): `stop()` 後、進行中サイクルタスクの完了を待ち、タイムアウト時はキャンセルして回収する。呼び出し側(emergency_stop 等)はこれを await してから home_return を開始する。
- **ログ保留上限**(レビュー #13): `MAX_PENDING_LOG_TASKS = 100`。超過時は新規ログをスキップ(超過遷移時に 1 回 warning、復帰時に info)。
- **KPI**: 偏差判定と同じ場所で `kpi.update(ref, actual, loop.time())`。`kpi_summary` プロパティを公開。
- **スナップショット鮮度**: `RealtimeSnapshot` に `captured_at: float`(イベントループ時刻)を追加し、サイクルで設定する。

### 6. RobotController(`src/app/robot_controller.py`)

- `_apply_profile_to_control_stack(profile)` を抽出: PID ゲイン + `set_output_limit` / stop_config / `ff.unload_model()` → `set_params` → 条件付き `load_model`(失敗は warning、has_model False のまま継続)。`select_profile` と `refresh_active_profile` が共用。
- `refresh_active_profile(profile) -> bool`(レビュー #10): アクティブプロファイルと同一 ID かつ走行系状態(RUNNING/MANUAL/CALIBRATING/PRE_CHECK)でない場合に in-memory プロファイルと制御スタックを更新。
- **緊急ディスパッチ統一**(レビュー #15): DriveLoop の `on_emergency` を `_dispatch_emergency` に変更。`safety_monitor.trigger_emergency()` を呼び、ディスパッチ後も EMERGENCY に遷移していなければ `emergency_stop()` をフォールバック実行(コールバック未登録の DI 構成でも安全)。
- **stop_and_join の徹底**(レビュー #6): `emergency_stop` / `stop` / `stop_auto_drive` / `stop_manual` / `shutdown` の全てで `await drive_loop.stop_and_join()` してから home_return。
- **テレメトリ鮮度**(レビュー #16): `get_realtime_data` はキャッシュ済みスナップショットの age が `SNAPSHOT_MAX_AGE_S(0.5s)` を超えたらキャッシュを無視してハードウェア読み取りへフォールバック(凍結値による障害マスキング防止)。
- 走行終了時(`stop_auto_drive` / `stop` / `emergency_stop`)に `drive_loop.kpi_summary` を INFO ログ + `last_kpi_summary` に保存してから DriveLoop を破棄。

### 7. CANReader(`src/infra/can_reader.py`)

- 鮮度上限をコンストラクタ引数 `max_speed_age_s`(既定 0.2 秒)に変更(レビュー #2)。
- `settings.can.max_speed_age_s`(既定 0.2)を追加し factory から注入。0.2 秒 = 4 制御周期。5km/h/s の減速でも盲目期間中の偏差成長は 1.0km/h 以内に収まる。

### 8. WebSocket broadcast(`src/web/ws.py`)

- クライアントへの送信を `asyncio.gather` + クライアント毎 `wait_for(0.5s)` の並列送信に変更(レビュー #14)。タイムアウト・例外のクライアントは切断扱い。

### 9. FeedforwardParams 追加定数(`src/models/profile.py`)

```python
switch_hysteresis_pct: float = 0.5    # ペダル切替ヒステリシス半幅
accel_reengage_dwell_s: float = 0.3   # ブレーキ後のアクセル再踏込ディレイ
accel_rate_limit_pct_s: float = 200.0 # アクセル開度レートリミット
brake_rate_limit_pct_s: float = 300.0 # ブレーキ開度レートリミット
pid_output_limit_pct: float = 50.0    # PID 出力権限上限
```

- JSONB 永続化のため SQL マイグレーション不要(`_ffp_from_value` が欠損キーをデフォルト補完)。
- `profile_repository.py` の from/to JSON、`web/schemas.py` の `FeedforwardParamsSchema` を拡張。
- `estimate_dynamics_params` は `dataclasses.replace` のため新フィールドは自動的に保持される。

## データフロー

### 自動走行 1 サイクル(50ms)
```
1. elapsed → ref_speed / future_speeds(変更なし)
2. read_speed()(鮮度 0.2s 保証。超過は TimeoutError → _abort_emergency)
3. ff_effort = has_model ? predict_effort(ref, future) : 0.0
4. dt = loop.time() − 前回更新時刻(クランプ付き)
5. pid_u = pid.update(ref, actual, dt, sat_high, sat_low)
6. out = arbiter.arbitrate(ff_effort + pid_u, dt) → 開度・飽和フラグ
7. 開度 → 位置 → アクチュエータ(変更なし)
8. 過電流・逸脱チェック(変更なし) + kpi.update(ref, actual)
9. スナップショット(captured_at 付き)・ログ(保留上限付き)
```

### 非常停止(内部起因)
```
DriveLoop(過電流/逸脱/CAN断/例外) → _abort_emergency
  → controller._dispatch_emergency → safety_monitor.trigger_emergency
  → 登録コールバック(controller.emergency_stop ほか通知系)
  → emergency_stop: stop_and_join(飛行中指令の完了/回収) → home_return
```

## エラーハンドリング戦略

- 既存方針を踏襲: 制御サイクル内の例外は捕捉して非常停止に倒す。新規コンポーネント(Arbiter/KPIMonitor)は純粋計算であり例外を送出しない設計とする(防御的クランプのみ)。
- `stop_and_join` のタイムアウト(2 秒)は pymodbus のリトライ上限(約 12 秒)より短く、非常停止の遅延を抑えつつ通常ケース(数 ms)では指令完了を待つ。タイムアウト時はタスクをキャンセルし、例外は既存の done callback が回収する。

## テスト戦略

### ユニットテスト(新規)
- `test_pedal_arbiter.py`: 排他性(全象限で同時非ゼロなし)、ヒステリシス帯でのペダル維持/COAST、再踏込ディレイ、不感帯写像(0/db/u)、レートリミット、飽和フラグ、reset。
- `test_kpi_monitor.py`: 既知系列での P95・最大偏差・5 秒窓反転数・ハード違反カウントとログ。

### ユニットテスト(更新)
- `test_pid.py`: 条件付き積分・出力制限・積分クランプ・計測 dt・dt クランプ。
- `test_feedforward.py`: predict_effort の符号・停車保持(0.5s 先読み)・クリープ/惰行の新条件・ブレンド連続性・unload_model。
- `test_drive_loop.py`: effort 経路(FF ブレーキ+正 PID でブレーキ維持)、ウォッチドッグ、stop_and_join、ログ保留上限、KPI サマリ、captured_at。
- `test_robot_controller.py`: unload/refresh、_dispatch_emergency(trigger 経由+フォールバック)、stop_and_join 呼び出し順(home_return より前)、get_realtime_data 鮮度フォールバック、last_kpi_summary。
- `test_web_drive.py`: /learning/train 後の refresh_active_profile 反映。
- `infra/test_can_reader.py`: max_speed_age_s=0.2 の鮮度判定。
- `test_ws_broadcast.py`: ストールクライアントが他クライアント配信を阻害しない・切断される。
- `test_factory.py` / `infra/test_profile_repository.py`: 新定数の配線・JSONB ラウンドトリップ(欠損キーのデフォルト補完)。

## 依存ライブラリ

追加なし(標準ライブラリ + 既存の numpy/sklearn/asyncpg/fastapi の範囲)。

## ディレクトリ構造

```
src/domain/control/
├── drive_loop.py      # 変更: effort 経路・watchdog・stop_and_join・KPI・ログ上限
├── feedforward.py     # 変更: predict_effort・ブレンド・停車保持・unload
├── pid.py             # 変更: dt・条件付き積分・出力制限
├── pedal_arbiter.py   # 新規
└── kpi_monitor.py     # 新規
src/models/profile.py        # 変更: FeedforwardParams 追加定数
src/models/system_state.py   # 変更: RealtimeSnapshot.captured_at
src/app/robot_controller.py  # 変更: 整合・緊急経路・鮮度・KPI ログ
src/app/factory.py           # 変更: max_speed_age_s 配線
src/infra/can_reader.py      # 変更: 鮮度上限
src/infra/settings.py        # 変更: can.max_speed_age_s
src/infra/profile_repository.py  # 変更: 新定数の JSONB 変換
src/web/ws.py                # 変更: 並列送信
src/web/routers/drive.py     # 変更: train 後の refresh
src/web/schemas.py           # 変更: FeedforwardParamsSchema 拡張
```

## 実装の順序

1. モデル層(FeedforwardParams / RealtimeSnapshot)と永続化・スキーマ
2. 制御コア(PID → PedalArbiter → FeedforwardController → KPIMonitor)+ 各ユニットテスト
3. DriveLoop 統合 + テスト
4. アプリ/インフラ層(RobotController / CANReader / factory / drive.py / ws.py)+ テスト
5. 品質チェック(ruff / pytest 全件)
6. functional-design.md の制御セクション更新・振り返り

## セキュリティ考慮事項

- 変更なし(ローカルネットワーク限定・モデル pkl は信頼済みファイルのみという既存前提を維持)。

## パフォーマンス考慮事項

- Arbiter / KPIMonitor は加減算と配列インデックスのみで 50ms 予算への影響は無視できる。
- KPIMonitor のヒストグラムは固定長(約 1000 int)で 10 時間走行でもメモリ一定。
- FF は両モデル評価になるが Ridge.predict ×2(数百 µs)で予算内。

## 将来の拡張性

- KPI サマリの DB 保存・GUI 表示は `kpi_summary` dict をそのまま永続化すれば追加可能。
- ブレンドバンド・ノイズフロア等を将来プロファイル定数に昇格する場合も FeedforwardParams の JSONB 方式で互換に追加できる。
