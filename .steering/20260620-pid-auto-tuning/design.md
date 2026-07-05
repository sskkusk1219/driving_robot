# 設計書

## アーキテクチャ概要

ドメイン層に純粋ロジックの `pid_tuning` モジュールを新設し、既存の制御スタック・ログ・KPI 集計・
自動走行オーケストレーションを再利用する。新たなプラント/制御コードは書かない。

```
[学習運転ログ] ──identify_fopdt──> FOPDT(k,τ,l) ──compute_pid_gains_simc──> PIDGains
       │                                                                      │
       └─ /learning/train に統合（estimate_dynamics_params の隣）─────────────┤
                                                                              ▼
                                              profile.pid_gains 更新 → DB保存 → refresh_active_profile

[閉ループ絞り込み]
  build_tuning_trajectory(profile) → DrivingMode（メモリ上）
       │
  RobotController.run_pid_tuning_session():
    CoordinateDescentTuner（純粋ロジック）
       │  各反復:
       ├─ _pid.set_gains(候補ゲイン)
       ├─ start_auto_drive(mode, profile) → 完了待ち → last_kpi_summary
       └─ tuning_cost(kpi_summary) を Tuner へ返す
    最良ゲイン → profile_repo.update + refresh_active_profile
```

## コンポーネント設計

### 1. `src/domain/pid_tuning.py`（新規・純粋ロジック）

**責務**:
- FOPDT 同定（学習ログ→K/τ/L）
- SIMC 則による PID ゲイン算出
- 規定パターン（チューニング軌跡）の生成
- KPI サマリのコスト化
- 座標降下チューナー（ハード非依存・注入式）

**実装の要点**:
- `estimate_dynamics_params`（model_training.py）と同じ堅牢性方針: サンプル不足は None を返し既存値を保持
- `_group_by_session` / `_estimate_offsets` の手法を踏襲（必要なら同モジュールから流用）
- 単位整合: FOPDT.k は km/h per %（誤差 km/h → 出力 % の PID と一致）
- `CoordinateDescentTuner` は run 実行をコールバック注入にし、ハード無しで単体テスト可能にする

主な API:
```python
@dataclass(frozen=True)
class FOPDT:
    k: float    # 定常ゲイン [km/h / %]
    tau: float  # 時定数 [s]
    l: float    # むだ時間 [s]

def identify_fopdt(logs: list[DriveLog], profile: VehicleProfile) -> FOPDT | None
def compute_pid_gains_simc(fopdt: FOPDT, profile: VehicleProfile,
                           tau_c_factor: float = 0.5) -> PIDGains
def build_tuning_trajectory(profile: VehicleProfile) -> DrivingMode
def tuning_cost(kpi_summary: dict[str, float]) -> float

class CoordinateDescentTuner:
    def __init__(self, initial: PIDGains, *, max_runs: int = 15,
                 init_step_frac: float = 0.3, min_step_frac: float = 0.05): ...
    def next_candidate(self) -> PIDGains | None   # None で停止
    def report(self, gains: PIDGains, cost: float) -> None
    @property
    def best(self) -> PIDGains
```

### 2. `src/web/routers/drive.py`（変更）

**責務**:
- `/learning/train` にモデルベース適合を統合
- `/drive/pid-tune/validate`・`/drive/pid-tune/refine` エンドポイント追加

**実装の要点**:
- 同定・算出は CPU 同期処理のため `asyncio.to_thread` で実行（既存 train と同様）
- 状態ガードは既存 train エンドポイント（drive.py:377-388）と同じパターン
- None（サンプル不足）の場合は既存ゲイン維持・`pid_auto_tuned=False`

### 3. `src/app/robot_controller.py`（変更）

**責務**:
- `run_pid_validation(profile)`: 規定パターン1回走行 → `last_kpi_summary` 取得
- `run_pid_tuning_session(profile, max_runs)`: Tuner 駆動の反復走行と最良ゲイン保存

**実装の要点**:
- `start_auto_drive(mode_id="pid-tune", mode=..., profile=...)` でメモリ上モードを走行
- 状態機械（PRE_CHECK/RUNNING/READY, home_return, 非常停止）は既存経路に乗せる
- 各反復間で `_pid.set_gains` し、走行完了後 `stop_auto_drive` が `_record_kpi_summary` 済み
- hard 違反多発試行は棄却して前ゲインへ戻す

### 4. `src/web/schemas.py`（変更）

- `TrainModelResponse` に `pid_gains: PIDGainsSchema | None`、`pid_auto_tuned: bool` を追加
- `PidTuneRequest`（profile_id, max_runs?）、`PidTuneResponse`（kpi_summary, cost, pid_gains）追加

### 5. UI（`learning.js` / `profiles.js`）（変更）

- `learning.js`: 学習結果表示に算出 Kp/Ki/Kd を併記（自動算出バッジ）
- `profiles.js`: 「PID自動適合（検証／絞り込み）」ボタン。実行中は進捗・反復ごとのゲインとコスト表示。
  ナビロック/run gate は既存 auto-drive 機構を流用。手動編集フォームは温存。

## データフロー

### モデルベース適合（学習時）
```
1. POST /learning/train（既存）
2. train_inverse_model → estimate_dynamics_params（既存）
3. identify_fopdt(logs, profile) → FOPDT | None
4. None でなければ compute_pid_gains_simc → profile.pid_gains 更新
5. profile_repo.update → refresh_active_profile（既存経路）
6. 応答に pid_gains / pid_auto_tuned を含めて UI 表示
```

### 反復絞り込み
```
1. POST /drive/pid-tune/refine（profile_id, max_runs）
2. build_tuning_trajectory(profile) でモード生成
3. CoordinateDescentTuner 初期化（初期 = profile.pid_gains）
4. ループ: next_candidate → set_gains → start_auto_drive → 完了 → last_kpi_summary
          → tuning_cost → tuner.report
5. tuner.best を profile に保存 → refresh_active_profile
6. 応答に最良ゲイン・コスト推移を返す
```

## エラーハンドリング戦略

### カスタムエラークラス
- 既存 `LearningDataError`（learning_drive.py）を流用（サンプル不足等）。新規例外は最小限。

### エラーハンドリングパターン
- 同定不能（サンプル不足）→ None 返却、ゲイン維持、UI に「自動適合できず（既存値維持）」
- 走行中の非常停止 → 既存 emergency 経路で停止、絞り込みセッションを中断し前ゲインへ復帰
- 不正状態のAPI呼び出し → HTTP 409（既存パターン）

## テスト戦略

### ユニットテスト（`tests/unit/test_pid_tuning.py` 新規）
- `identify_fopdt`: 合成ログから K/τ/L 復元、サンプル不足→None
- `compute_pid_gains_simc`: 期待値・単調性（tau_c_factor 大でゲイン低下）、クランプ
- `tuning_cost`: hard 違反支配、KPI 良化で減少
- `CoordinateDescentTuner`: 既知の凸コスト（モック run）で最適点近傍に収束・max_runs で停止
- `build_tuning_trajectory`: max_speed 内・単調な時間軸・accel/hold/brake を含む

### 統合テスト（`tests/integration/test_web_api.py` 追記）
- `/learning/train` 応答に `pid_gains` が乗る
- `/drive/pid-tune/validate`・`/refine` の状態ガード（走行中409）と正常応答（スタブ制御）

## 依存ライブラリ

新規追加なし（numpy / sklearn は既存）。

## ディレクトリ構造

```
src/domain/pid_tuning.py            （新規）
src/web/routers/drive.py            （変更: train 統合 + 2 エンドポイント）
src/app/robot_controller.py         （変更: validation/tuning セッション）
src/web/schemas.py                  （変更: スキーマ追加）
src/web/static/js/screens/learning.js   （変更: 算出ゲイン表示）
src/web/static/js/screens/profiles.js   （変更: 自動適合ボタン）
tests/unit/test_pid_tuning.py       （新規）
tests/integration/test_web_api.py   （変更: 追記）
```

## 実装の順序

1. Phase 1: `pid_tuning.py`（FOPDT 同定 + SIMC）+ 単体テスト → `/learning/train` 統合 + スキーマ + learning.js
2. Phase 2: `build_tuning_trajectory` + `tuning_cost` + `run_pid_validation` + `/validate` + UI
3. Phase 3: `CoordinateDescentTuner` + `run_pid_tuning_session` + `/refine` + UI 進捗
4. 品質チェック（pytest / ruff / 型）

## セキュリティ考慮事項

- 自動走行を伴うため、既存の走行前チェック（car_speed==0）・状態ガードを必ず経由する。
- `max_runs` と time budget でリソース・ハード摩耗を制限する。

## パフォーマンス考慮事項

- 同定・算出は `to_thread` で 50ms 制御ループ・WS 配信・非常停止コールバックをブロックしない。
- 絞り込みは実機を最大15回走行。time budget と hard 違反棄却で安全側に制御。

## 将来の拡張性

- ブレーキ側の独立同定、Kd の解析導入、速度域別ゲインスケジューリングへ拡張可能な構造とする。
- `tuning_cost` の重みは将来の KPI 変更に追従できるよう `kpi_monitor` 定数で正規化する。
- 閉ループでの Kd 導入（`CoordinateDescentTuner` の巡回座標降下化 + PID 微分項への1次LPF）は
  `.steering/20260702-pid-kd-tuning` で実装済み。SIMC 解析算出（τD=θ/2 相当）による Kd 初期値は
  LPF 導入後の残項目として引き続き未実装。
</content>
