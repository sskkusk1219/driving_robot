# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

### タスクスキップが許可される唯一のケース
技術的理由（実装方針変更・アーキテクチャ変更・依存関係変更）のみ。スキップ時は理由を明記:
`- [x] ~~タスク名~~（実装方針変更により不要: 具体的な技術的理由）`

### 実機検証ゲートについて
- P-9 実機検証はユーザーがシャシダイナモで実施する。実装セッションは「実装＋単体テスト＋
  机上検証まで」を完了させる

---

## P-1: ペダルプラン ドメイン層（Opus実装。設計は design.md 1節）

- [x] `src/domain/control/ilc.py`: `_zero_phase_lowpass` → 公開関数 `zero_phase_lowpass` に改名
      （ilc 内の呼び出し・`tests/unit/test_ilc.py` の参照を追従。挙動不変）
- [x] `src/domain/control/pedal_plan.py` 新規: `PlanPhase`（DRIVE/COAST/BRAKE/STOP_HOLD）、
      `PedalPlan`（dt_s=0.1・efforts・phases・effort_at=np.interp端点クランプ・phase_at=最近傍）
- [x] フェーズ分類関数（analyze_session からも import する純関数）: a_req 1s平滑、
      `a_coast(v)`（creep_rate / −engine_brake、FeedforwardParams の同定値使用）、
      `PHASE_MARGIN_KMHS=0.15`、v<0.5 → STOP_HOLD
- [x] micro-phase マージ: `MIN_PHASE_S=2.0` 未満を長い隣接フェーズへ吸収（STOP_HOLD は保持）
- [x] `PedalPlanner.build(mode, ff, params)`: FF 一括評価（未ロード時 0 系列）→
      `zero_phase_lowpass`（`PLAN_LOWPASS_HZ=0.25`）→ フェーズ整合クランプ
      （DRIVE≥0 / BRAKE≤0 / COAST=0 / STOP_HOLD=−max(stop_brake_opening_pct, brake_deadband_pct)）
- [x] `tests/unit/test_pedal_plan.py`: フェーズ分類（クリープ発進=COAST・緩減速=DRIVE・
      急減速=BRAKE・停止=STOP_HOLD）、マージ、クランプ、補間・端点、モデル未ロード

## P-2: 全モード机上検証（P-1 完了直後に実施・以降の前提）

- [x] `scripts/preview_pedal_plan.py` 新規: 全登録モード（DB）についてプラン生成し、
      フェーズ時間内訳・プラン切替回数 vs 軌跡要求切替・effort 変化レート最大値を表出力
- [x] 全12モードで「プラン切替回数 = 軌跡要求（±1回/走行）」を確認（NG ならマージ規則を是正
      してから先へ進む）。参考要求値: WLTP_ExHi 1.3/min・US06 3.7/min・HWFET 1.0/min

## P-3: トリム制御（design.md 2節）

- [x] `src/domain/control/trim.py` 新規: `TrimController`（fast_pid: PIDController を内包）。
      定数 `HOLD_BAND_KMH=0.1` / `FAST_ENGAGE_KMH=0.5` / `FAST_RELEASE_KMH=0.3` /
      `TRIM_RATE_PCT_S=2.0` / `TRIM_STEP_PCT=0.25`
- [x] 凍結帯: |dev|≤0.1 で前回トリム出力を保持（積分凍結）
- [x] 低速PI: SIMC 級を基準に机上校正した固定ゲイン＋スルーレート制限＋量子化。
      **校正条件: ILC なしで p95≤0.2 を出せる帯域を確保**（design.md 2節の d²/(2·r·g) 評価を
      FOPDT シミュレーションで机上確認し、定数をコメントに根拠付きで記録）
- [x] 速い補正層: profile の kp/ki/kd × gain_scale で PIDController.update。介入0.5/離脱0.3の
      ヒステリシス。`is_fast_active` プロパティ（drive_loop のフェーズ権限判定用）
- [x] バンプレス切替（速い層→低速層で出力段差なし）・飽和フラグの層別伝播・reset
- [x] `tests/unit/test_trim.py`: 3層遷移・レート制限・量子化・ヒステリシス・バンプレス・
      アンチワインドアップ・reset・FOPDTシミュレーションで残差2%開度時に偏差<0.2km/h 収束

## P-4: DriveLoop 統合（design.md 3節）

- [x] `src/domain/control/drive_loop.py`: コンストラクタに `plan: PedalPlan | None = None`、
      `pid` → `trim: TrimController` 置換。**plan=None は従来経路（FF毎サイクル＋fast_pid直結）
      で完全回帰**
- [x] plan あり経路: FF 毎サイクル評価を廃し `base = plan.effort_at + ilc.effort_at`、
      `trim.update(..., phase=plan.phase_at(elapsed))`、フェーズ権限クランプ
      （DRIVE≥0 / COAST=0 / BRAKE≤0 / STOP_HOLD=プラン値のままトリム無効。
      速い層アクティブ時は無権限クランプ）。クランプで削った方向を飽和フラグで trim へ返す
- [x] `_GAIN_SCALE_MAX: 3.0 → 1.5`（B-8-3 移管。docstring に実測根拠を記録）
- [x] `tests/unit/test_drive_loop.py`: plan 経路（FF非呼出・合成・権限クランプ4種・STOP_HOLD）、
      plan=None 回帰、scale クランプ 1.5

## P-5: アプリ層配線（design.md 4節）

- [x] `src/app/robot_controller.py`: `start_auto_drive` / `_run_tuning_drive` でプロファイル適用後に
      `PedalPlanner.build(...)`（モデル未ロードは None）を DriveLoop へ注入
- [x] `src/app/factory.py` ほか DriveLoop 構築箇所: TrimController 構築・注入（stubs 含む）
- [x] `tests/unit/test_robot_controller.py`: auto/tuning でプラン生成・注入、モデル無しで None

## P-6: 不要切替の計測・コスト（B-8-4 移管・再定義、design.md 5節）

- [x] `src/domain/control/kpi_monitor.py::update` に `brake_opening: float | None = None` 追加、
      交互踏み（異種ペダルが2秒以内にON）を `pedal_switch_count` / `pedal_switch_per_min` として
      summary に追加（None 後方互換）
- [x] `src/domain/control/drive_loop.py`: `kpi.update(..., brake_opening=...)` 配線
- [x] `src/domain/pid_tuning.py::tuning_cost += PEDAL_SEESAW_WEIGHT × pedal_switch_per_min`
      （実機 e3773d9c=39.4/min・2af4c09b=18.8/min で机上校正、初期候補 0.5）
- [x] `scripts/analyze_session.py`: ログの ref_speed_kmh 列から**軌跡要求切替**を pedal_plan の
      分類関数で自動計算し「**不要切替 = 実測交互踏み − 要求切替** [回/min]（目安≈0）」を表示。
      **モードごとの設定は作らない**（合否ゲートは KPI 3項目のみ）
- [x] `tests/unit/test_kpi_monitor.py`（交互踏み・2秒窓・None互換）／
      `test_pid_tuning.py::TestTuningCost`（項の単調性・重み・欠損0）

## P-7: 学習サイクル VERIFY フェーズ（design.md 6節）

- [x] `src/domain/pid_tuning.py::build_verification_trajectory(modes, profile, budget_s=300)`:
      登録全モードの包絡（最高速度巡航≥10s・巡航代表点3〜4・ランプ率p10/p50/p90・完全停止→
      再発進≥1回）を約300秒に合成。安全包絡（≤max_speed・≤0.8×max_decel_g・0始端0終端）保証
- [x] `src/infra/settings.py::LearningSettings`: `verify_runs_max: int = 5` 追加
- [x] `src/app/learning_cycle.py`: REFINE_2 後に VERIFY フェーズ — 検証パターン走行（プラン+
      トリム・**ILC無効**・disable_deviation_check=True）→ KPI判定（p95≤0.2/max≤1.0/反転≤1）
      → 合格で早期終了 / 不合格で検証ログを加えてモデル再学習＋プラン・スケジュール再構築 →
      再走行 / 上限で WARNING 完了（最良 KPI を detail に記録）。**停車保持の安全不変条件を
      VERIFY 全体（フェーズ境界・走行間・再学習中）に延長し、解放は COMPLETED 時のみ**
- [x] `src/web/schemas.py::CycleProgressSchema` / `learning.js`: VERIFY フェーズ・走行番号・
      各走行 KPI（p95/max/反転/不要切替）の進捗表示
- [x] `scripts/preview_pedal_plan.py` に検証パターンの出力（速度域・ランプ率の包絡確認）を追加
- [x] `tests/unit/test_pid_tuning.py`: 検証パターンの安全包絡・全モード最高速/巡航帯包含・停止
      ≥1回・予算内・**cap クリップ（max_speed=100 で cap 超の巡航点が除外され最高速巡航=100・
      全点≤100 になる）**／`test_learning_cycle.py`: VERIFY 遷移（合格早期終了・再学習ループ・
      上限 WARNING・ILC無効）

## P-8: 品質チェック

- [x] `pytest tests/unit` ＋ `tests/integration/test_web_api.py` 全緑（1152 passed）
- [x] `ruff check` 通過、`mypy` 新規エラーなし（残2件 robot_controller:728/app.py:100 は既存）

## P-9: 実機検証（ユーザー実施）

- [ ] **ILCテーブルをリセット**（WebUIのリセット、または `DELETE FROM ilc_tables`。
      現テーブルは 2026-07-10 の悪走行 p95=1.34 から学習済みで新構成では無効）
- [ ] 学習サイクル実行 → VERIFY フェーズが KPI 合格で完了することを確認
      （不合格上限到達なら WARNING 内容・検証走行ログを確認して再サイクル）
- [ ] **サイクル完了後の最初の自動運転（登録モード、例: WLTP_ExHi）で
      p95≤0.2 / max≤1.0 / 反転≤1回/5s を満足**（`python -m scripts.analyze_session --latest auto`）
- [ ] ペダル挙動の目視: 開度なるべく一定・クリープ発進・エンジンブレーキ優先・停止中ブレーキ
      保持。不要切替 ≈0（analyze_session の観測指標）
- [ ] 旧ステアリング C-6 の残項目はこの結果をもって完了扱い（ILC はモード別の任意強化として
      2回目以降の走行で従来どおり学習・適用される）

## ドキュメント更新

- [x] `docs/architecture.md`: 制御構成を plan+ilc+trim に更新（effort合成・フェーズ権限・
      VERIFY フェーズ・ディレクトリツリーに pedal_plan.py/trim.py）
- [x] `docs/functional-design.md`: 自動走行フローにペダルプラン生成・TrimController・
      フェーズ権限・学習サイクルの VERIFY フェーズを反映
- [x] `docs/glossary.md`: ペダルプラン・トリム制御・フェーズ権限・不要切替・検証走行 の用語追加
- [ ] 実装後の振り返り（このファイル下部に記録）

---

## 実装後の振り返り

### 実装完了日
- P-1〜P-8（ドメイン〜アプリ〜WebUI〜品質チェック・ドキュメント）: 2026-07-11 実装完了
- P-9 実機検証（ILCリセット→学習サイクル→初回自動運転で KPI 確認）はユーザー実施待ち

### 計画と異なった点
- **MIN_PHASE_S: 2.0 → 1.0**: 全12モードの机上検証で、2.0s は US06 等の正当な短時間ハードブレーキ
  （生 4.1回/min → 1.7回/min）まで吸収すると判明。実機シーソー（0.6〜1s）は閉ループ PID 過補正由来で
  基準軌跡には存在しない（トリム＋ゲイン低下で対処）ため、基準から導く正当なブレーキを保存する 1.0s に
  下げた。design.md 1節に是正記録。
- **トリム低速 PI ゲインの机上校正**: TRIM_SLOW_KP=2.0/KI=1.2/RATE=3.0 を FOPDT(k=2.16,τ=2.08,θ=0.30)
  シミュレーションで選定（FF 残差 2% 開度に対し定常偏差 0.03km/h・整定後ペダル変更 0 回）。当初案の
  1.5/1.0/2.0 は定常 0.22km/h で目標 0.2 ギリギリだったため強化した。
- **プランなし経路は「速い補正層 PID 直結」で完全回帰**: トリム 3 層を通すと注入 PID と別のトリム内部
  ゲインが効いてしまい旧テストの FF+PID 合成の前提が崩れるため、plan=None では `trim.fast_pid` を直結。
- **budget_s を機能させる後処理を追加**: 検証パターンが予算超過時に保持時間を圧縮（最高速巡航≥10s・
  停止≥4s は床）。当初は未使用パラメータだった。

### 新たに必要になったタスク
- `run_verification_drive`（RobotController 公開メソッド）: `_run_tuning_drive` を VERIFY 用に薄くラップ
  （ILC 無効・停車保持維持）。
- ModeRepoProtocol への `list_all` 追加（VERIFY が登録全モードを列挙するため）。
- analyze_session の profile 定数フェッチ（不要切替の軌跡要求切替をプランナーと同じ分類器で計算するため）。

### 学んだこと
- **ペダルの滑らかさは「計画」で作るのが機構的に確実**: 毎サイクルの閉ループ合成でヒステリシスや
  レート制限を積んでも「開度なるべく一定・調整は時々」にはならない。走行前に操作骨格（プラン）を決め、
  実行時は小さく直すだけ、という人間の運転構造をそのまま写すと不要切替が 39回/min → 軌跡要求レベルに落ちる
  （机上）。
- **速度形 PI はバンプレス切替に有利**: 出力を「前回値＋Δ」で積むと層をまたいでも段差が出ず、凍結帯・
  低速層・速い層の遷移が自然に連続する。量子化は内部連続値を保持することで積分作用を落とさずサーボ微振動
  だけ消せる。
- **基準軌跡の物理床とマージ設定の分離**: フェーズ分類（生）は物理的に必要な切替床とほぼ一致し、マージは
  滑らかさのための別ノブ。両者を混同すると US06 の実ブレーキを潰す。マージは「基準の微小うねり由来の
  サブ秒フリッカ除去」に留め、閉ループ由来のシーソーはトリム＋ゲインで対処、と役割を分ける。
- **KPI 達成を学習サイクル内で完結させる価値**: 本番走行を収束の場にしない（ユーザー方針）と、VERIFY で
  「初回自動運転から KPI 満足」を保証できる。ILC は前提から任意強化に降格し、設計がシンプルになった。

### 次回への改善提案
- VERIFY の合否は現状「単走行で 3 項目同時達成」。実機で FF 残差が帯により大きい場合、低速トリムの
  レート/ゲインを VERIFY 実績で再校正する余地（design.md P-3 の校正条件に沿って）。
- ILC 寄与を可視化するため将来 DriveLogData に plan/trim/ilc の内訳列を追加すると実機分解分析が容易
  （今回は DEBUG ログに留めた）。
- ペダルプランの実効果（フェーズ列・名目 effort）は現状 preview_pedal_plan.py の机上出力のみ。WebUI へ
  プラン可視化・VERIFY 各走行の KPI 履歴グラフを足すと実機ゲートの判断が速くなる。
