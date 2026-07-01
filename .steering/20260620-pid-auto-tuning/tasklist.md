# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

### タスクスキップが許可される唯一のケース
以下の技術的理由に該当する場合のみスキップ可能:
- 実装方針の変更により、機能自体が不要になった
- アーキテクチャ変更により、別の実装方法に置き換わった
- 依存関係の変更により、タスクが実行不可能になった

スキップ時は必ず理由を明記:
```markdown
- [x] ~~タスク名~~（実装方針変更により不要: 具体的な技術的理由）
```

---

## フェーズ1: モデルベース解析適合（A）

- [x] `src/domain/pid_tuning.py` を新規作成（FOPDT 同定 + SIMC）
  - [x] `FOPDT` dataclass（k[km/h/%], tau[s], l[s]）を定義
  - [x] `identify_fopdt(logs, profile) -> FOPDT | None` を実装（accel 保持区間抽出・K/τ/L 集約・サンプル不足None）
  - [x] `compute_pid_gains_simc(fopdt, profile, tau_c_factor=0.5) -> PIDGains` を実装（SIMC則・クランプ）
- [x] `tests/unit/test_pid_tuning.py` を新規作成（Phase1分）
  - [x] `identify_fopdt`: 合成ログから K/τ/L 復元
  - [x] `identify_fopdt`: サンプル不足→None
  - [x] `compute_pid_gains_simc`: 期待値・単調性・クランプ
- [x] `src/web/schemas.py` に `TrainModelResponse.pid_gains` / `pid_auto_tuned` を追加
- [x] `src/web/routers/drive.py` の `train_learning_model` にモデルベース適合を統合
  - [x] `identify_fopdt`→`compute_pid_gains_simc` を `to_thread` で実行
  - [x] 算出できたら `profile.pid_gains` 更新（None なら維持）、応答に反映
- [x] `tests/integration/test_web_api.py` に train 応答の pid_gains テストを追加
- [x] `src/web/static/js/screens/learning.js` に算出 Kp/Ki/Kd 表示（自動算出バッジ）
- [x] Phase1 の単体・統合テストが通ることを確認

## フェーズ2: 閉ループ検証（単発走行）

- [x] `src/domain/pid_tuning.py` に `build_tuning_trajectory(profile) -> DrivingMode` を実装
- [x] `src/domain/pid_tuning.py` に `tuning_cost(kpi_summary) -> float` を実装（kpi_monitor 定数で正規化）
- [x] `tests/unit/test_pid_tuning.py` に Phase2 テスト追加
  - [x] `build_tuning_trajectory`: max_speed 内・単調時間軸・accel/hold/brake を含む
  - [x] `tuning_cost`: hard 違反支配・KPI 良化で減少
- [x] `src/app/robot_controller.py` に `run_pid_validation(profile)` を実装
- [x] `src/web/schemas.py` に `PidTuneRequest` / `PidTuneResponse` を追加
- [x] `src/web/routers/drive.py` に `POST /drive/pid-tune/validate` を実装（状態ガード付き）
- [x] `tests/integration/test_web_api.py` に validate エンドポイントのテストを追加（KPI/コスト/404/409）
- [x] `src/web/static/js/screens/profiles.js` に「PID自動適合（検証）」ボタン・KPI表示を追加
- [x] Phase2 の単体・統合テストが通ることを確認

## フェーズ3: 反復絞り込み（座標降下）

- [x] `src/domain/pid_tuning.py` に `CoordinateDescentTuner` を実装（注入式・ハード非依存）
- [x] `tests/unit/test_pid_tuning.py` に `CoordinateDescentTuner` テスト追加（凸コストで収束・max_runs停止）
- [x] `src/app/robot_controller.py` に `run_pid_tuning_session(profile, max_runs)` を実装
  - [x] 各反復で set_gains→走行→last_kpi_summary→tuning_cost→report
  - [x] hard 違反多発試行の棄却（tuning_cost が hard 違反に 100x ペナルティ→座標降下が best に採用せず前ゲイン維持）
  - [x] 非常停止時は PidTuningAborted で中断（_run_tuning_drive）
  - [x] 最良ゲインを profile_repo.update + refresh_active_profile で保存（router 側）
- [x] `src/web/routers/drive.py` に `POST /drive/pid-tune/refine` を実装（状態ガード付き）
- [x] `tests/unit/test_robot_controller.py` に orchestration テスト追加（READY/calibration ガード・反復・非常停止中断）
- [x] `src/web/static/js/screens/profiles.js` に「絞り込み」ボタン・最良ゲイン/コスト表示
- [x] `tests/integration/test_web_api.py` に `/validate`・`/refine` の状態ガード/正常応答テスト追加
- [x] Phase3 の単体・統合テストが通ることを確認

## フェーズ4: 品質チェックと修正

- [x] すべてのテストが通ることを確認
  - [x] `.venv/bin/pytest tests/unit/test_pid_tuning.py tests/integration/test_web_api.py`（22 passed）
  - [x] `.venv/bin/pytest -m "not hardware"`（687 passed）。hardware マーク 1 件は実 CAN バス必須で本変更と無関係
- [x] リント・型エラーがないことを確認
  - [x] `.venv/bin/ruff check src tests`（All checks passed）
  - [x] `.venv/bin/mypy`（変更 4 ファイルは issue なし。既存 2 エラーは未変更ファイル）
- [ ] ~~実機/手動確認（Playwright MCP・実車走行）~~（オペレーター実施項目: 検証/絞り込みは実車を走行させるため、
      アシスタントが自律実行すべきでない。下記手順でオペレーターが実機確認する）
  - 学習→train で自動算出 PID が表示・保存されること
  - 「検証（1回走行）」で KPI サマリが表示されること
  - 「絞り込み（反復走行）」で最終 p95 が初期より悪化しないこと

## フェーズ5: ドキュメント更新

- [x] `docs/functional-design.md` に PID 自動適合機能を追記
- [x] 実装後の振り返り（このファイルの下部に記録）

## フェーズ6: フォローアップ（2026-06-21 学習運転フローへのステップ4統合）

- [x] `profiles.js`: プロファイル編集の「PID自動適合」赤枠パネル（検証/絞り込みボタン）を削除
- [x] `learning.js`: 学習完了後そのまま自動で `/pid-tune/refine`（max_runs=15）まで一気通貫実行
  - [x] `busyRef` でステップ4の反復走行中の学習再トリガーを抑止
  - [x] 学習ページに基準パターン（ref 線）を描画しない（`showModeAxis:false` で既に未描画。実車速のみ）
  - [x] 規定パターンの主要速度＋最大減速G を文字情報で表示
- [x] `pid_tuning.build_tuning_trajectory`: 減速区間を `max_decel_g` から算出し上限G/最高車速を厳守
  - [x] `G_TO_KMHS`/`DECEL_MARGIN`/`MIN_RATE_KMHS` 定数追加、減速レート ≤ 上限G を保証
  - [x] 単体テストに G 厳守・最高車速以内の parametrized アサート追加
- [x] `docs/functional-design.md` を 4 ステップフローへ更新
- [x] esbuild で `learning.js` の JSX をトランスパイル検証、`node --check` で `profiles.js` 検証
- [ ] ~~実機確認（オペレーター実施）~~: 学習→モデル作成→PID初期値→規定パターン走行でPID最適化が自動連続で
      走ること、上限G/最高車速を超えないこと、編集画面に赤枠が無いこと

## フェーズ7: フォローアップ（2026-07-01 学習→適合の橋渡し停止・適合中の逸脱停止無効化）

学習運転終了後に自動で PID 適合へ切り替わる際、車両が停止しきれておらず、適合の規定パターン
（0km/h 始点）との追従誤差で逸脱非常停止がかかってしまう不具合への対応。

- [x] `DriveLoop` に `disable_deviation_check`（既定 False）を追加し、True のとき逸脱判定・
      逸脱非常停止をスキップ（過電流・CAN 断・ウォッチドッグ等の安全網は維持、KPI 集計は継続）
- [x] `robot_controller._build_and_start_drive_loop` に `disable_deviation_check` を配線
- [x] `_run_tuning_drive` で `disable_deviation_check=True` を渡す（適合走行のみ逸脱停止を無効化）
- [x] `stop_learning_drive` を `_apply_brake_hold` 即時適用から `_decelerate_to_stop`（~0.1G 緩減速 +
      停止確認 → 停車保持ブレーキ）へ変更し、停止確認後に READY へ遷移
- [x] テスト追加
  - [x] `test_drive_loop.py`: `disable_deviation_check=True` で逸脱しても非常停止しない・check_deviation 未呼出
  - [x] `test_robot_controller.py`: 既停止時は即停車保持・READY、転動時は減速ループ後に停車保持
  - [x] `test_robot_controller.py`: `_run_tuning_drive` が `disable_deviation_check=True` を渡す
- [x] `docs/functional-design.md` に橋渡し停止と適合中の逸脱停止無効化を追記
- [x] 品質チェック（pytest -m "not hardware" 709 passed / ruff All checks passed / mypy 対象 2 ファイル Success）
- [ ] ~~実機確認（オペレーター実施）~~: 学習→適合の自動切替で車両が ~0.1G で停止後に適合が走り、
      適合中に逸脱非常停止がかからないこと

---

## 実装後の振り返り

### 実装完了日
2026-06-20

### 計画と実績の差分

**計画と異なった点**:
- FOPDT のむだ時間記号を当初 `l` としたが ruff E741（曖昧な変数名）に抵触したため `theta` に統一した。
- 「hard 違反多発試行の棄却・前ゲイン復帰」を明示的なロジックではなく `tuning_cost` の 100x ペナルティ
  ＋座標降下が悪コスト候補を best に採用しない性質で実現した（実装が単純化し、副作用も少ない）。
- 最良ゲインの永続化はコントローラではなく router 側に置いた（コントローラが profile_repo を持たない
  既存設計に合わせ、`train` エンドポイントと同じ「コントローラは制御スタックへ反映・router が DB 保存」
  の責務分担を踏襲）。

**新たに必要になったタスク**:
- `PIDGainsSchema` が `TrainModelResponse` より後に定義されていたため、参照可能にする並び替えが必要だった。
- 閉ループ走行の完了待ちのため、コントローラに `asyncio.Event`（`_drive_complete`）を追加し
  `stop_auto_drive`／`emergency_stop` で set する仕組みを足した。

### 学んだこと

**技術的な学び**:
- 既存の自動走行経路（`start_auto_drive`＋`last_kpi_summary`）をそのまま再利用することで、走行前チェック・
  非常停止・home_return・セッション記録といった安全機構を新規実装せずに閉ループ適合へ流用できた。
- 学習運転が既に開ループのステップ応答を記録しているため、追加走行ゼロで FOPDT 同定→SIMC 初期ゲインが
  得られる。閉ループ絞り込みは初期値からの微調整に徹することでハード稼働を最小化できる。
- 座標降下チューナーを走行実行から切り離す（注入式）ことで、ハード無しに収束挙動を単体テストできた。

**プロセス上の改善点**:
- ドメインの純粋ロジック（同定・SIMC・コスト・チューナー）を先に実装・テストし、ハード結合部
  （コントローラ・エンドポイント）を後段に積み上げる順序が、テスト容易性と安全性の両面で有効だった。
- `git stash` を検証目的で実行して全変更を退避してしまうミスがあった。`git stash pop` で復旧したが、
  作業ツリーに未コミット変更がある状態での stash は避けるべき。

### 次回への改善提案
- 実機での検証/絞り込みはオペレーター実施が必須（実車走行のためアシスタントは自律実行しない）。
  実機確認後、ブレーキ側プラントの独立同定や Kd の解析導入を検討する。
- 絞り込みの進捗を WebSocket で反復ごとにストリームすると UX が向上する（現状は完了時に履歴一括返却）。
