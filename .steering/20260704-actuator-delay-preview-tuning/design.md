# 設計書

## アーキテクチャ概要

既存レイヤ構成(domain / app / infra / web)を維持し、新規コンポーネントは追加しない。変更は「制御時間フレームの分離」「チューナーのパラメータ一般化」「プロファイルのフィールド追加」の3点に集約する。

```
学習運転(開ループ) → drive_logs
  → TRAINING_1: train_and_apply(update_pid_gains=True)
       ├ identify_fopdt(logs) → FOPDT(k, τ, θ)
       ├ compute_pid_gains_simc(θフル) → pid_gains 初期値   ※変更なし
       ├ dynamics_params ← (k, τ, θ, preview_time_s=clamp(θ, 0..3))   ← 新規
       └ profile_repo.update() + controller.refresh_active_profile()
  → REFINE_1/2: run_pid_tuning_session
       ├ CoordinateDescentTuner(TuningParams(kp, ki, kd, preview_time_s))  ← 次元追加
       ├ 各走行: 候補previewをreplaceしたプロファイルで DriveLoop を構築
       └ best → _persist_best_params(gains+preview) → profile_repo.update
  → 通常自動走行: DriveLoop が
       ref_ctrl = _ref_speed_at(elapsed + profile.dynamics_params.preview_time_s)
       を FF・PID の基準に使用(KPI/逸脱/ログは elapsed 基準のまま)
```

## 設計判断(確定事項と根拠)

### 判断1: preview は FF・PID の両方に適用(制御時間フレームの一本化)

- むだ時間θの系では「今出す指令の効果は t+θ に現れる」ため、FFとPIDが同じ前倒し目標に向かって動くのが整合的。
- FFのみ前倒しにすると、ランプ中にPID(現在基準)がFFの先行分を「誤差」とみなして打ち消し、補償が減殺される。
- KPI/逸脱は now-frame のまま(ダイナモの追従仕様)。preview が過大なら now-frame KPI が悪化するため、**座標降下のコストが preview を実効遅れへ自己調整する**(コスト関数変更不要)。

### 判断2: 永続化は新JSONBカラム `dynamics_params`(FeedforwardParams拡張は不採用)

- FeedforwardParams はユーザー編集可能な「車両物理定数」でUIフォームに露出しており、FOPDT同定値という「学習成果メタデータ」と意味論が異なる。
- 既存バグ: `profiles.py` の `_ffp_to_schema`/`_ffp_from_schema` が11フィールド中6つしかマッピングせず、PUTで調停定数がリセットされる。ここにpreviewを足すと「学習サイクルで適合したpreviewがUI編集で消える」事故が起きる。新カラムなら独立に守れる。このマッピングバグ自体も本作業で `fields()` ベースの汎用変換に修正する。

### 判断3: SIMCはθフル維持、自動適合は「θ初期値 → 座標降下で実走絞り込み」

- preview は基準の位相を進めるだけで、外乱に対するフィードバックループのむだ時間は残る。θを割引いてゲインを上げると安定余裕を失うため、SIMC計算は変更しない。
- θ初期値(ほぼ最適近傍)から始めるため、座標降下1巡でも意味がある。

## コンポーネント設計

### 1. DynamicsParams (src/models/profile.py)

**責務**: 学習成果としての動特性パラメータの保持。

```python
@dataclass
class DynamicsParams:
    """FOPDT同定と適合走行で得た動特性パラメータ(学習成果メタデータ)。

    preview_time_s は基準軌跡の時間シフト量[s]。制御用の基準サンプリングを
    この秒数だけ前倒しし、アクチュエータ〜車両系のむだ時間を補償する。
    0.0 で従来動作(前倒しなし)。
    """
    preview_time_s: float = 0.0
    fopdt_k: float | None = None      # 定常ゲイン [km/h / %]
    fopdt_tau: float | None = None    # 時定数 [s]
    fopdt_theta: float | None = None  # むだ時間 [s]
```

- `VehicleProfile.dynamics_params: DynamicsParams = field(default_factory=DynamicsParams)` を末尾に追加(デフォルト付きなので既存の全構築箇所は無変更で通る)。

### 2. DriveLoop の制御フレーム分離 (src/domain/control/drive_loop.py)

**責務**: 制御用参照(t_ctrl)と評価用参照(now-frame)の分離。

**実装の要点** (`_execute_one_cycle`, 345-410行付近):
- コンストラクタで `self._preview_s = max(0.0, profile.dynamics_params.preview_time_s)` をキャッシュ。
- `t_ctrl = elapsed_s + self._preview_s`
- **t_ctrl 基準(制御)**: `ref_speed_ctrl = _ref_speed_at(t_ctrl)`、`future_speeds = [_ref_speed_at(t_ctrl + h) for h in ff.horizons]`、`past_speeds = [_ref_speed_at(t_ctrl - h) ...]`、`ff.predict_effort(ref_speed_ctrl, ...)`、`pid.update(ref_speed_ctrl, actual_speed, ...)`
- **now-frame 基準(評価・不変)**: `ref_speed = _ref_speed_at(elapsed_s)` → `_last_ref_speed`、KPI、逸脱判定、DriveLogData.ref_speed_kmh、WS表示、完了判定(`elapsed_s >= total_duration`)、pause凍結
- `_ref_speed_at` の終端クランプにより t_ctrl が末尾超過しても安全。
- 停車ショートサーキット(feedforward.py:145)の保持ブレーキ判定が preview 秒早まるのは意図通り(実応答はθ遅れて基準に着地する)。

### 3. TuningParams と CoordinateDescentTuner の一般化 (src/domain/pid_tuning.py)

**責務**: 4パラメータ(kp, ki, kd, preview_time_s)の巡回座標降下。

**実装の要点**:
- `@dataclass TuningParams: kp, ki, kd, preview_time_s`(`PIDGains` は既存のまま維持し、変換ヘルパー `gains` プロパティ等を用意)
- `_PARAMS = ("kp", "ki", "kd", "preview_time_s")`、`_BASE["preview_time_s"] = 0.5`
- 定数 `PREVIEW_MIN_S = 0.0`, `PREVIEW_MAX_S = L_MAX_S (=3.0)`
- 現状の `max(0.0, ...)` 下限クランプを per-param クランプ `_CLAMP: dict[str, tuple[float, float]]` に置換(kp/ki/kd は `(0.0, inf)`、preview は `(0.0, 3.0)`)。クランプ後にbestと同値になる候補をスキップする既存ロジックはそのまま効く。
- `initial_preview_from_fopdt(fopdt: FOPDT) -> float`: `min(max(fopdt.theta, 0.0), PREVIEW_MAX_S)`

### 4. 配線 (src/app/training_service.py, robot_controller.py, learning_cycle.py)

**training_service.train_and_apply** (FOPDT同定ブロック 107-115行):
- `update_pid_gains=True` かつ `fopdt is not None` のとき `profile.dynamics_params = DynamicsParams(preview_time_s=initial_preview_from_fopdt(fopdt), fopdt_k=..., fopdt_tau=..., fopdt_theta=...)`
- `update_pid_gains=False`(TRAINING_2)では既存値保持(1段目の適合結果を消さない)
- `TrainResult` に `dynamics_params` を追加

**robot_controller.run_pid_tuning_session** (982-1032行):
- `CoordinateDescentTuner(TuningParams(kp, ki, kd, preview=profile.dynamics_params.preview_time_s))` で初期化
- 候補ごとに `profile_run = dataclasses.replace(profile, dynamics_params=replace(profile.dynamics_params, preview_time_s=cand.preview_time_s))` を走行に渡す(DriveLoopは走行ごとに再構築されるため自然に効く)
- history の各エントリに `preview_time_s` を追加。戻り値を `tuple[TuningParams, list[dict]]` へ変更(呼び出し元: learning_cycle と drive ルーターの2箇所)

**learning_cycle**:
- `_persist_best_gains` → `_persist_best_params(profile, best: TuningParams)`: `profile.pid_gains` と `profile.dynamics_params.preview_time_s` を更新して `profile_repo.update` + `refresh_active_profile`
- `learning_cycles.detail` JSONB に `stageN_preview_time_s`・`fopdt` を追記(スキーマレスなのでDDL変更不要)

### 5. 永続化 (scripts/setup_db.py, src/infra/profile_repository.py)

- DDL: `ALTER TABLE vehicle_profiles ADD COLUMN IF NOT EXISTS dynamics_params JSONB`(feedforward_params の既存前例と同型、冪等)
- `_dyn_from_value` / `_dyn_to_json`: `_ffp_from_value`/`_ffp_to_json` と同じ `fields()` ベースの汎用実装。NULL/欠損キーはデフォルト補完
- `_row_to_profile`・`create`(INSERT列)・`update`(SET列)に組み込み

### 6. Web API / UI (src/web/schemas.py, routers/profiles.py, static/js)

- `DynamicsParamsSchema`(4フィールド、デフォルト付き)を `ProfileResponse` に必須、`ProfileUpdateRequest` に optional で追加
- **併修**: `_ffp_to_schema`/`_ffp_from_schema` を `fields()` ベースへ書き換え(調停定数リセットバグ解消)
- `TrainModelResponse` に dynamics_params、PID適合系レスポンスの history に preview を露出
- profiles.js: 「先読み補償時間 [s]」number入力(0〜3)+ FOPDT同定値の読み取り専用表示(未同定は「—」)。既存フォーム定義パターン踏襲
- learning.js: サイクル進捗/完了表示に best preview を追加(最小限)

## データフロー

### 通常自動走行(適合済みプロファイル)
```
1. select_profile → DriveLoop(profile) 構築、preview_s キャッシュ
2. 50ms周期: t_ctrl = elapsed + preview_s
3. FF・PID とも t_ctrl 基準の目標速度で努力量を計算
4. KPI・逸脱・ログは elapsed 基準の ref_speed で評価(従来通り)
```

### 学習サイクル
```
1. 学習走行(開ループ) → drive_logs
2. TRAINING_1: FOPDT同定 → SIMCゲイン + dynamics_params(preview=θ) → プロファイル保存
3. REFINE_1: 4次元座標降下(候補previewはreplaceで注入) → best保存
4. TRAINING_2: モデル再学習のみ(dynamics_params は保持)
5. REFINE_2: 同上 → 最終best保存
```

## エラーハンドリング戦略

- FOPDT同定不能(`identify_fopdt` が None)時: dynamics_params は既存値を保持(previewだけ0に戻さない)。従来のSIMCスキップと同じ流儀。
- DB既存行の dynamics_params が NULL: リポジトリのデシリアライザがデフォルト(preview=0.0)補完 → 従来動作。
- preview の異常値: DriveLoop 側で `max(0.0, ...)`、チューナー側で [0, 3.0] クランプの二重防御。

## テスト戦略

### ユニットテスト
- `tests/unit/test_models.py`: DynamicsParams デフォルト・VehicleProfile 後方互換構築
- `tests/unit/test_drive_loop.py`: preview>0 で PID/FF の受信基準が前倒し / KPI・逸脱・ログは now-frame / preview=0 回帰 / 終端クランプ
- `tests/unit/test_pid_tuning.py`: 4次元巡回、previewクランプ[0,3]、クランプ同値スキップ、凸コストで収束、予算内でpreview候補が評価される
- `tests/unit/test_training_service.py`: FOPDT成功時の初期化 / update_pid_gains=False で保持 / 同定不能時に保持
- `tests/unit/test_learning_cycle.py`: `_persist_best_params` が gains+preview を update へ渡し refresh される
- `tests/unit/test_robot_controller.py`: 候補previewを差し替えたプロファイルで走行する
- `tests/unit/infra/test_profile_repository.py`: dynamics_params ラウンドトリップ・NULL行デフォルト補完

### 統合テスト
- ProfileResponse に dynamics_params が含まれる / PUT で preview 更新 / PUT で FFP 調停定数が保持される(バグ修正検証)

## 依存ライブラリ

追加なし。

## ディレクトリ構造

```
変更のみ(新規ファイルなし):
  src/models/profile.py                     DynamicsParams 追加
  scripts/setup_db.py                       ALTER TABLE 追加
  src/infra/profile_repository.py           シリアライズ/CRUD組込み
  src/domain/control/drive_loop.py          制御フレーム分離
  src/domain/pid_tuning.py                  TuningParams / 4次元チューナー
  src/app/training_service.py               FOPDT→dynamics_params
  src/app/robot_controller.py               run_pid_tuning_session 一般化
  src/app/learning_cycle.py                 _persist_best_params
  src/web/schemas.py                        DynamicsParamsSchema
  src/web/routers/profiles.py               スキーマ組込み + FFPバグ修正
  src/web/routers/drive.py                  戻り値型変更への追随
  src/web/static/js/screens/profiles.js     preview入力 + FOPDT表示
  src/web/static/js/screens/learning.js     best preview 表示
  config/settings.toml.example              refine_runs_stage1 = 14
```

## 実装の順序

1. models → setup_db → profile_repository(永続化基盤)+ 単体テスト
2. drive_loop の制御フレーム分離 + 単体テスト
3. pid_tuning の TuningParams 一般化 + 単体テスト
4. training_service / robot_controller / learning_cycle の配線 + 単体テスト
5. web schemas/routers/JS + 統合テスト
6. 実機検証(setup_db 実行 → 学習サイクル1回 → KPI比較)

## セキュリティ考慮事項

- 変更なし(新規外部入力は preview の PUT のみ。スキーマで数値型・サーバ側クランプで防御)。

## パフォーマンス考慮事項

- `_ref_speed_at` は bisect O(log n) で、呼び出し回数は不変(基準時刻がシフトするだけ)。50ms周期への影響なし。

## リスクと対処

| リスク | 対処 |
|---|---|
| 停車ショートサーキットの保持ブレーキ判定が preview 秒早まる | むだ時間補償として意図通り。過大なら now-frame KPI コストが縮める。テストで挙動確認 |
| 通常走行で preview 過大時に now-frame 逸脱判定が発火 | 仕様通りの安全網として許容(適合走行は従来通り逸脱判定無効) |
| pause 中の保持目標が `ref(paused_elapsed + preview)` になる | 凍結値が一定である性質は不変で実害なし(挙動差として記録) |
| 座標降下の次元増で走行予算逼迫 | θ初期値で最適近傍から開始 + refine_runs_stage1=14(1巡=最大8評価) |
| プロファイル PUT で適合値が消える | 新カラム分離 + FFPマッピングバグ修正 |
| 既存DB互換 | ADD COLUMN IF NOT EXISTS + NULL→デフォルト補完。preview=0 で従来動作に完全一致 |

## 将来の拡張性

- `DynamicsParams` は速度域別 preview(ゲインスケジューリング)やブレーキ側独立θへの拡張余地を持つ(JSONBなのでフィールド追加はマイグレーション不要)
- アクチュエータ単体の遅れ計測(サーボステップ応答試験)を導入する場合も、結果の格納先として同カラムを使える
