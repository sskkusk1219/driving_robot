# 設計書

## アーキテクチャ概要

現行の「20Hzで FF+PID+ILC を毎サイクル合成しペダルを連続変調する」構成を、
「走行開始時にペダル操作計画を立て、実行時は計画に沿って保持し、小さく滑らかに直す」構成に
再編する。**KPIは学習サイクル内の検証走行（VERIFY）で合格させ、サイクル完了後の最初の
自動運転から満足する**（ILCはKPIの前提にしない）。

```
[学習サイクル]
学習運転 → stage1学習 → REFINE_1 → stage2学習 → REFINE_2
  → VERIFY: 検証専用パターン（登録全モードを包絡・約300s・自動生成）を
    ILC無効・プラン+トリム構成で走行 → KPI判定 → 合格で完了
    （不合格: 走行データを加えてモデル再学習＋プラン再構築 → 再走行、上限5本）

[走行開始時・オフライン]                       [実行時 20Hz]
基準軌跡全体                                   effort = plan.effort_at(t)
  │                                                    + ilc.effort_at(t)   ※任意強化
  ├─ フェーズ分割 ──────────────┐                       + trim(dev, phase)
  │  DRIVE/COAST/BRAKE/STOP_HOLD │                        │
  └─ FFモデル一括評価 ─ ローパス ┴→ PedalPlan ──────→ フェーズ権限クランプ
     （滑らかな名目effort）                               │
                                                   PedalArbiter（不変）→ ペダル
[反復・走行完了後]
drive_logs の残差 → ILC学習（既存機構そのまま・モード別の任意強化）
```

- **PedalPlan** が人間スタイルを構造的に保証: クリープ発進（ペダルなし）・エンジンブレーキ優先・
  必要時のみブレーキ・停止中ブレーキ保持・不要切替はプラン段階で排除
- **TrimController** が「開度なるべく一定」を保証: 凍結帯/低速PI/速い補正層の3層。
  **低速PIは検証走行でp95≤0.2を出せる水準に校正する**（ILCに頼らない）
- **VERIFY** が「学習サイクル完了＝KPI合格」を保証（本番走行を収束の場にしない）
- **ILC（既存）** はモード別の任意強化（本番自動走行の正常完了で従来どおり学習・適用）
- 旧 B-8 の吸収: B-8-1(PID不感帯)→凍結帯、B-8-2(ブレーキ抑制)→フェーズ権限、
  B-8-3(scale上限1.5)→速い補正層、B-8-4(シーソー計測)→本設計 5 節（不要切替に再定義）

## コンポーネント設計

### 1. PedalPlan / PedalPlanner（新規 `src/domain/control/pedal_plan.py`・純ドメイン）

**データ構造**:
```python
class PlanPhase(Enum):
    DRIVE = "drive"        # アクセルで駆動（緩減速の調整踏みを含む）
    COAST = "coast"        # ペダルなし（クリープ発進・クリープ保持・エンジンブレーキ減速）
    BRAKE = "brake"        # ブレーキで減速
    STOP_HOLD = "stop"     # 停車保持（stop_brake_opening_pct を保持）

@dataclass
class PedalPlan:
    dt_s: float                 # 0.1s グリッド（ILCTable と同じ）
    efforts: list[float]        # 名目 effort [%]（符号付き、FF/ILC と同単位）
    phases: list[PlanPhase]
    # effort_at(t): np.interp + 端点クランプ（ILCController.effort_at と同型）
    # phase_at(t): グリッド最近傍
```

**PedalPlanner.build(mode, ff, params, *, dt_s=0.1) -> PedalPlan**
（ff: FeedforwardController ロード済み、params: FeedforwardParams）

1. **再サンプルと必要加速度**: reference_speed を dt_s グリッドへ線形補間 → `a_req = gradient`
   を 1s 移動平均で平滑化（人間は瞬時勾配でなく傾向で踏む）
2. **フェーズ分類**（車両定数は自動同定済みの FeedforwardParams を使用）:
   - `a_coast(v) = +creep_rate_kmhs (v < creep_speed_kmh) / −engine_brake_decel_kmhs (それ以外)`
   - `v_ref < 0.5km/h` → STOP_HOLD
   - `a_req > a_coast(v) + MARGIN` → DRIVE、`a_req < a_coast(v) − MARGIN` → COAST でなく BRAKE、
     間 → COAST（`PHASE_MARGIN_KMHS = 0.15`）
   - クリープ発進は「v<creep_speed かつ creep で足りる」＝ COAST に自然に分類される
3. **micro-phase マージ**: `MIN_PHASE_S = 1.0` 未満のフェーズは長い方の隣接フェーズへ吸収
   （STOP_HOLD は除外＝常に保持）。基準軌跡の微小うねり由来のサブ秒フリッカを消す。
   **実装時の是正**: 当初 2.0s としたが、実機シーソー（0.6〜1s 周期）は閉ループ PID 過補正
   由来で基準軌跡には無いため（トリム＋ゲイン低下で対処）、基準から導く正当な短時間ブレーキ
   （US06 のハードブレーキ等）を潰さないよう 1.0s に下げた（2.0s は US06 の生 4.1回/min を
   実ブレーキごと 1.7回/min に吸収。1.0s では 2.5回/min で保存、全12モード机上確認 2026-07-11）
4. **名目 effort**:
   - 全グリッドで FF を一括評価: `ff.predict_effort(ref(t), futures(t), pasts(t))`
     （drive_loop の now-frame 呼び出しと同形。モデル未ロード時は 0 系列）
   - ゼロ位相ローパスで平滑化: `PLAN_LOWPASS_HZ = 0.25`。**ilc.py の `_zero_phase_lowpass` を
     公開関数 `zero_phase_lowpass` に改名して再利用**（ilc 内の呼び出しも追従）
   - フェーズ整合クランプ（ローパスの滲み対策）: DRIVE→`max(effort, 0)`、BRAKE→`min(effort, 0)`、
     COAST→`0.0`、STOP_HOLD→`−max(stop_brake_opening_pct, brake_deadband_pct)`
5. 計算量: 3230点×FF評価はプロファイル選択済みモデルでミリ秒〜数十ms オーダー。走行開始処理内
   （非リアルタイム）で同期実行可

### 2. TrimController（新規 `src/domain/control/trim.py`）

DriveLoop 内で PIDController の役割を置き換える（PIDController クラス自体は速い補正層の
内部部品として存続）。

**インターフェース**:
```python
class TrimController:
    def __init__(self, fast_pid: PIDController) -> None: ...
    def update(self, ref, actual, dt, *, phase: PlanPhase,
               saturated_high: bool, saturated_low: bool, gain_scale: float) -> float: ...
    def reset(self) -> None: ...
```

**3層構造**（モジュール定数。座標降下の対象にはしない）:
- `HOLD_BAND_KMH = 0.1`: |dev| ≤ 0.1 → **出力凍結**（前回のトリム出力を保持。積分も凍結）。
  「偏差±0.1以内ならペダル操作しない」のユーザー要求の実装
- `0.1 < |dev| < FAST_ENGAGE_KMH(=0.5)`: **低速PI** — 内部PI（ゲインは FOPDT から SIMC 級を
  基準に机上校正した固定値）の出力を `TRIM_RATE_PCT_S = 2.0` [%/s] でスルーレート制限し
  `TRIM_STEP_PCT = 0.25` [%] で量子化（変化が量子未満なら前回値保持）。「少し踏み足す/緩める」
  を機構化。**注意: ILC なしで p95≤0.2 を出す必要があるため過度に遅くしない**。FF残差
  （モデルMAE≈2%開度）を d、レート r とすると追従遅れ由来の偏差 ≈ d²/(2·r·g) — d=2%・
  r=2%/s・g=3.5%/(km/h/s) で ≈0.29km/h。VERIFY のモデル再学習で d が縮むことと合わせ、
  r と PI ゲインは VERIFY 実機結果で最終校正する（tasklist P-3 に校正タスク）
- `|dev| ≥ 0.5`: **速い補正層** — profile の kp/ki/kd（座標降下の適合対象のまま）×
  gain_scale（`_GAIN_SCALE_MAX = 1.5` に引き下げ＝B-8-3 移管）で PIDController.update を実行。
  離脱ヒステリシス `FAST_RELEASE_KMH = 0.3`（0.5 で介入、0.3 を切るまで維持＝層チャタ防止）
- **バンプレス切替**: 速い補正層から低速層へ戻るとき、速い層の最終出力を低速層の保持値に
  引き継ぐ（出力段差を作らない）。凍結帯⇄低速層は保持値ベースなので構造的に連続
- アンチワインドアップ: 調停器の飽和フラグを層別に伝播（現行 PID と同じ条件付き積分）

### 3. DriveLoop 統合（`src/domain/control/drive_loop.py`）

- コンストラクタ: `plan: PedalPlan | None = None` を追加。`pid: PIDController` は
  `trim: TrimController` に置換（**plan=None なら従来経路（FF毎サイクル評価＋PID直結）に
  フォールバック**し完全回帰＝モデル未ロード・ブートストラップ時の互換維持）
- サイクル処理（plan あり時）:
  1. `ref_speed`（now-frame、KPI/ログ/WS 用）は現行どおり
  2. `base = plan.effort_at(elapsed_s) + (ilc.effort_at(elapsed_s) if ilc else 0)`
     （**FF の毎サイクル評価は行わない** — プランに焼き込み済み）
  3. `phase = plan.phase_at(elapsed_s)`、`trim_u = trim.update(ref_pid, actual, dt, phase=...,
     gain_scale=...)`（pid_preview_s の前倒しは現行どおり速い補正層の基準にのみ適用）
  4. **フェーズ権限クランプ**: `effort = base + trim_u` に対し、速い補正層が非アクティブなら
     - DRIVE: `effort = max(effort, 0)`（ブレーキに落ちない）
     - COAST: `effort = 0`（踏まない）
     - BRAKE: `effort = min(effort, 0)`（アクセルに跳ねない）
     - STOP_HOLD: `effort = plan値のまま`（トリム無効＝停車保持はプランが支配）
     速い補正層アクティブ（|dev|≥0.5 発生中）は無権限クランプ＝KPI max≤1.0 の安全網。
     クランプで削った方向は飽和フラグとして trim へ返す（積分停止）
  5. arbitrate 以降（同時踏み禁止・レート制限・解放レート・安全チェック・KPI・ログ）は不変
- `_GAIN_SCALE_MAX: 3.0 → 1.5`（docstring に B-8 実測根拠: scale3.0×kp3.91=SIMC適正の7倍で
  1Hz リミットサイクル）

### 4. アプリ層配線（`src/app/robot_controller.py`, `src/app/factory.py`）

- `start_auto_drive` / `_run_tuning_drive`: プロファイル適用（モデルロード）後に
  `plan = PedalPlanner.build(mode, self._ff, profile.feedforward_params)` を生成し DriveLoop へ
  注入。チューニング走行も同じ経路（適合走行も人間的操作で行い、そのログが stage2 モデルと
  ILC の前提分布になる）
- モデル未ロード（FF なし）時は plan=None（従来経路）
- factory: TrimController の構築（PIDController を内包）と注入

### 5. 不要切替の計測・コスト（B-8-4 移管・再定義）

問題の本質は「軌跡が要求しない不要なペダル切替」（2026-07-10 実測: 軌跡要求1.3回/min に対し
39.4回/min）。総切替数のしきい値をモードごとに設定するのではなく、**基準軌跡から要求切替を
自動計算し、その差分（不要切替）を観測する。設定は一切不要**。人間的な操作が実現できていれば
不要切替≈0 になるため、合否ゲートではなく観測指標＋チューニングコスト項とする。

- `src/domain/control/kpi_monitor.py::update` に任意引数 `brake_opening: float | None = None`。
  アクセルON⇔ブレーキON の交互踏み（異種ペダルが 2 秒以内に ON）を `pedal_switch_count` /
  `pedal_switch_per_min` として summary に追加（accel_on_count と同パターン・None 後方互換）
- `src/domain/control/drive_loop.py`: `kpi.update(..., brake_opening=self._current_brake_opening)`
- `src/domain/pid_tuning.py::tuning_cost += PEDAL_SEESAW_WEIGHT × pedal_switch_per_min`。
  重みは実機2セッション（e3773d9c=39.4回/min・2af4c09b=18.8回/min）で机上校正（初期候補 0.5、
  既存支配項を上書きしない。チューニング軌跡上の要求切替はゲイン候補間で共通の定数オフセット
  なので降下方向を歪めない）
- `scripts/analyze_session.py`: ログ内の基準軌跡（ref_speed_kmh 列）から**要求切替**を
  プランナーと同じ分類器（pedal_plan.py のフェーズ分類関数を import）で自動計算し、
  `不要切替 = 実測交互踏み − 要求切替` [回/min] を表示（目安 ≈0。ゲート判定は KPI 3項目のみ）
- 新規 `scripts/preview_pedal_plan.py`: 全登録モードについてプラン生成し、フェーズ内訳・
  プラン切替回数 vs 軌跡要求・effort 変化レートを表出力（机上検証用）

### 6. 学習サイクル VERIFY フェーズ（新規）と適合の関係

**目的**: 「学習サイクル完了＝以降の自動運転は初回から KPI 満足」を機構で保証する。
対象モードそのもののリハーサルはしない（12_AMA=87分級でサイクルが破綻）。ILC 無効で
検証することで、任意の登録モードの初回走行（モード別 ILC テーブルなし）の成績を予測する。

- **検証専用パターン**: `src/domain/pid_tuning.py::build_verification_trajectory(modes,
  profile, budget_s=300.0) -> DrivingMode`（`build_tuning_trajectory_from_mode` の隣に新設）。
  DB 登録済み全モードから包絡特性を抽出して約300秒に合成する。ユーザー選択なし・自動生成:
  - 最高速度: `cap = min(max(モード最高速度), profile.max_speed)` への到達・巡航保持（≥10s）
  - 巡航帯: 全モードの巡航速度（|a_req|<0.2 が3s以上続く速度）の代表点で保持。
    **cap を超える巡航点は自動除外**（例: max_speed=100 なら 113/131 巡航が落ち、
    最高速巡航100・総時間約226秒に短縮される — 2026-07-11 机上確認済み）
  - 加減速率: 全モードのランプ率分布の低率/中率/高率（p10/p50/p90、≤0.8×max_decel_g）を混在
  - 停止: 完全停止→再発進を 1 回以上含む（クリープ発進・停止保持の検証）
  - 安全包絡（≤max_speed・≤0.8×max_decel_g・0始端0終端）は既存規定パターンと同じ保証
  - **パターンは DB に登録・保存しない**（VERIFY 実行時にその場で生成）。生成器は 1 つで、
    プロファイル・登録モードが変われば自動追従するため、プロファイルごとのパターン管理は
    発生しない。KPI 保証の範囲はプロファイル包絡内（mode最高速度 ≤ max_speed）のモード
  - 区間構成のドラフト（max_speed=140・現12モード時: 総278s・最高131・要求切替1.5回/min・
    DRIVE78%/BRAKE18%/COAST2%/STOP3%）は `verification_pattern_draft.csv` としてユーザー
    確認済み（2026-07-11 構成承認）。実装はこの構成を包絡統計から再現すること
  - **検討記録: profile.max_speed の廃止（一律140km/h化）は不採用** — max_speed は学習運転の
    予測的cap離脱（B-7-5）・規定パターン・ゲインスケジュール構築の安全上限を兼ねており、
    140未満しか出せない/出してはいけない車両で140を指令する危険がある。パターン一本化の
    目的は「登録不要の自動生成」で既に満たされる。なお自動走行開始時に mode最高速度 >
    profile.max_speed を弾くガードは現状存在しない（本件スコープ外の既知事項として記録）
- **学習サイクルへの組み込み**（`src/app/learning_cycle.py`）: REFINE_2 の後に新フェーズ
  `VERIFY`。ループ（最大 `verify_runs_max = 5`、`src/infra/settings.py::LearningSettings` に追加）:
  1. 検証パターンを自動走行相当で実行（プラン+トリム・**ILC 無効**・
     disable_deviation_check=True で完走保証・drive_logs 記録）
  2. KPIMonitor summary で判定: p95≤0.2 / max≤1.0 / 反転≤1回/5s → **合格なら早期終了**
  3. 不合格: 検証走行ログを学習データに加えてモデル再学習（既存 stage2 学習経路を再利用。
     検証走行は本番同分布のデータなので FF 残差が縮む）→ ゲインスケジュール・プラン再構築
     → 次の検証走行
  4. 上限到達で未達なら **WARNING 付きで完了**（最良走行の KPI を記録。走行自体は可能、
     ユーザーが再サイクル・データ確認を判断）
  - **安全不変条件の維持**: 既存サイクルの「学習運転終了〜適合完了まで停車保持ブレーキで
    静止し続ける」不変条件を VERIFY にも延長する（REFINE_2→VERIFY 境界・検証走行間・
    再学習中も停車保持。`release_on_finish=False` パターンを踏襲し、解放は COMPLETED の
    原点復帰時のみ）
- **進捗表示**: `CycleProgressSchema` / `learning.js` に VERIFY フェーズ・走行番号・各走行の
  KPI 値（p95/max/反転/不要切替）を追加
- **座標降下**: 適合対象は速い補正層の kp/ki/kd/pid_preview_s のまま（TuningParams/機構不変）。
  低速トリム・凍結帯・プラン定数は固定（適合次元を増やさない）。tuning_cost は 5 節の項を追加
- FOPDT 同定・SIMC 初期値・ゲインスケジュール構築（B-6 是正版）は不変。scale 上限のみ 1.5
- ILC: 機構・テーブル・学習トリガー（本番自動走行の正常完了のみ）は不変。**KPI の前提から
  外れ、モード別の任意強化**（2 回目以降の同一モード走行をさらに滑らか・高精度にする）

## データフロー（自動走行 1 サイクル、20Hz）

```
1. ref_speed = ref(elapsed)                     # KPI/ログ/WS 用 now-frame
2. base   = plan.effort_at(elapsed) + ilc.effort_at(elapsed)
3. phase  = plan.phase_at(elapsed)
4. trim_u = trim.update(ref(elapsed+pid_preview), actual, dt, phase, sat, gain_scale)
5. effort = phase_clamp(base + trim_u, phase, fast_active)   # フェーズ権限
6. arbitrate(effort, dt) → ペダル開度 → サーボ                 # 調停器不変
7. KPIMonitor.update(ref, actual, now, accel_opening, brake_opening)
```

## エラーハンドリング戦略

- プラン生成失敗（モデル未ロード・軌跡空）→ plan=None で従来経路にフォールバック（走行は継続）
- STOP_HOLD 中もウォッチドッグ・過電流・逸脱チェックは現行どおり
- 速い補正層は常時アームされており、フェーズ権限・凍結帯はいずれも |dev|≥0.5 で無効化される
  （滑らかさは性能目標、偏差抑制は安全網、の優先順位を機構で保証）

## テスト戦略

### ユニットテスト
- `tests/unit/test_pedal_plan.py`（新規）: 合成軌跡でフェーズ分類（クリープ発進=COAST・
  緩減速=DRIVE・急減速=BRAKE・停止=STOP_HOLD）、micro-phase マージ（2s 未満吸収・STOP_HOLD
  保持）、フェーズ整合クランプ、effort_at/phase_at 補間・端点、モデル未ロードで effort=0
- `tests/unit/test_trim.py`（新規）: 凍結帯で出力不変、低速層のレート制限・量子化、
  速い層の介入(0.5)/離脱(0.3)ヒステリシス、バンプレス切替（層遷移で出力段差なし）、
  飽和フラグでの積分停止、reset
- `tests/unit/test_drive_loop.py`: plan あり経路（FF 非呼び出し・plan+ilc+trim 合成・
  フェーズ権限クランプ 4 種・STOP_HOLD でトリム無効）、plan=None で従来経路完全回帰、
  `_GAIN_SCALE_MAX=1.5` 更新
- `tests/unit/test_kpi_monitor.py` / `test_pid_tuning.py`: 交互踏みカウント（2秒窓・None 互換）、
  コスト項の単調性・重み、`build_verification_trajectory` の安全包絡（≤max_speed・
  ≤0.8×max_decel_g・0始端0終端・予算内・全モード最高速/巡航帯の包含・停止≥1回）
- `tests/unit/test_learning_cycle.py`: VERIFY フェーズの遷移（合格で早期終了・不合格で
  再学習ループ・上限5本で WARNING 完了・ILC 無効で実行）
- `test_ilc.py`: `zero_phase_lowpass` 公開化のリネーム追従（挙動回帰）

### 机上検証（実機前ゲート）
- `scripts/preview_pedal_plan.py` を全12モードで実行: プラン切替回数 = 軌跡要求（±1回/走行）、
  フェーズ時間内訳が要求スタイルと整合（発進部に COAST、減速の大半が COAST→BRAKE の順）
- 検証パターンを同スクリプトで出力し、登録全モードの速度域・ランプ率の包絡を確認

### 実機検証（ユーザー実施）
- requirements.md 受け入れ条件のとおり: 学習サイクル（VERIFY 含む）が KPI 合格で完了 →
  **最初の自動運転（登録モード）で p95≤0.2 / max≤1.0 / 反転≤1回/5s** → ペダル挙動目視・
  不要切替≈0。ILC テーブルは事前にリセット（2026-07-10 の悪走行由来テーブルは無効）

## 依存ライブラリ

新規追加なし（numpy 既存範囲）

## ディレクトリ構造

```
src/domain/control/pedal_plan.py   (新規: PlanPhase/PedalPlan/PedalPlanner/フェーズ分類関数)
src/domain/control/trim.py         (新規: TrimController)
scripts/preview_pedal_plan.py      (新規: 全モード机上検証・検証パターン確認)
tests/unit/test_pedal_plan.py      (新規)
tests/unit/test_trim.py            (新規)
既存変更: src/domain/control/{drive_loop,kpi_monitor,ilc}.py,
          src/domain/pid_tuning.py (build_verification_trajectory・コスト項),
          src/app/{robot_controller,learning_cycle,factory}.py, src/infra/settings.py,
          src/web/schemas.py, src/web/static/js/screens/learning.js（VERIFY進捗）,
          scripts/analyze_session.py
```

## 実装の順序

1. pedal_plan.py（＋ilc.py の zero_phase_lowpass 公開化）＋テスト
2. preview_pedal_plan.py で全12モード机上検証（プラン切替=軌跡要求を確認してから先へ進む）
3. trim.py ＋テスト
4. drive_loop 統合（plan/trim/フェーズ権限/scale1.5）＋テスト
5. robot_controller/factory 配線＋テスト
6. kpi_monitor/tuning_cost/analyze_session（不要切替）＋テスト
7. build_verification_trajectory ＋ learning_cycle VERIFY フェーズ ＋ WebUI進捗 ＋テスト
8. 品質チェック → ドキュメント更新 → 実機検証（ユーザー）

## セキュリティ考慮事項

- 新APIなし。既存の走行開始/停止経路のみ

## パフォーマンス考慮事項

- 20Hz サイクル内の追加コストは plan/ilc の線形補間＋trim の算術のみ（FF 毎サイクル評価が
  消えるため現行より軽い）
- プラン生成は走行開始時に一度（数千点の FF 一括評価、非リアルタイム）

## 将来の拡張性

- WebUI へのフェーズ表示・プラン可視化（PedalPlan をシリアライズするだけで可能）
- ILC 収束後にプランへ焼き込み（plan+ilc の合成を新プランとして保存）すれば初回走行から
  高精度化できる（今回はスコープ外）
- 低速トリムのレート/量子は将来プロファイルパラメータ化可能（今回はモジュール定数）
