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

## フェーズ1: PID 微分項の1次ローパスフィルタ（`src/domain/control/pid.py`）

- [x] `DERIV_FILTER_TAU_S = 0.2` をモジュール定数として追加
- [x] `PIDController` にフィルタ状態 `_d_filt` を追加し、`update()` の微分項を
      `d_filt += (d_raw − d_filt) × dt_eff / (τf + dt_eff)` に変更（出力は `kd × d_filt`）
- [x] `reset()` で `_d_filt = 0.0` をクリア
- [x] テスト追加（`tests/unit/test_pid.py`）
  - [x] ~~既存テストが無修正で全通過~~（kd=0 のテストは無修正で通過を確認。
        kd≠0 で微分の生値を直接検証していた `TestPIDControllerDerivative` 3件と
        `test_measured_dt_used_for_derivative` はフィルタ後の値に更新が必要だった。
        design.md の互換性保証は明示的に「kd=0 のとき出力が従来と完全一致」の範囲であり、
        kd≠0 で生の後退差分を検証するテストは LPF 導入と両立不可能なため期待値を更新）
  - [x] `test_derivative_lpf_attenuates_spike`: 1サンプル誤差ジャンプの微分項が
        未フィルタ理論値の約 0.2 倍に減衰
  - [x] `test_reset_clears_derivative_filter`: reset でフィルタ残留が消える

## フェーズ2: CoordinateDescentTuner の巡回座標降下化（`src/domain/pid_tuning.py`）

- [x] `_BASE` を `{"kp": 1.0, "ki": 0.1, "kd": 0.5}` に変更
- [x] ラウンド一括生成（`_gen_round` + 改善時 `_pending = []` 全破棄）を廃止し、
      kp→ki→kd 巡回の遅延候補生成に変更（design.md「実装スケッチ」参照）
  - [x] 各座標: `+` 候補 → 改善なら採用して次座標へ / 改善なしなら `−` 候補
  - [x] クランプ後の値が best と同値になる候補は生成せずスキップ
  - [x] 1巡改善なしでステップ半減、`min_step_frac` 未満 or `max_runs` で None（既存停止則）
  - [x] 公開 API（`next_candidate`/`report`/`best`/`best_cost`/`runs`）不変・
        ベースライン初回評価も既存どおり
- [x] `robot_controller.py` が無変更で動くことを確認（run_pid_tuning_session のループ契約）
- [x] テスト調整・追加（`tests/unit/test_pid_tuning.py`）
  - [x] 既存 `TestCoordinateDescentTuner` 3件を新遍歴に合わせて調整（成立するなら無修正）
        → 無修正で成立（候補順に依存するアサーションがなかったため）
  - [x] `test_kd_candidate_evaluated_within_budget`: kp/ki が改善し続けても
        遅くとも7走行目までに kd 候補が出る
  - [x] `test_kd_improves_when_beneficial`: kd のみ効く凸コストで `best.kd > 0`
  - [x] `test_no_duplicate_clamped_candidates`: kd=0 初期で best と同値の候補が返らない
  - [x] `test_step_halves_after_full_cycle_without_improvement`

## フェーズ3: 品質チェック

- [x] すべてのテストが通ることを確認
  - [x] `.venv/bin/python -m pytest tests/unit/ -x`（740 passed）
- [x] リントエラーがないことを確認
  - [x] `.venv/bin/ruff check src tests`（All checks passed!）

## フェーズ4: ドキュメント更新

- [x] `.steering/20260620-pid-auto-tuning/design.md` の「将来の拡張性」に、
      閉ループでの Kd 導入が本作業（20260702-pid-kd-tuning）で実装された旨を追記
- [x] 実装後の振り返り（このファイルの下部に記録）

## フェーズ5: 実機検証（ユーザー実施）

- [ ] `/drive/pid-tune/refine` を実行し、反復履歴（history）に kd≠0 の候補が現れることを
      WebUI で確認（採用されるかはコスト次第で、評価されていれば OK）

---

## 実装後の振り返り

### 実装完了日
2026-07-02

### 計画と実績の差分

**計画と異なった点**:
- `tests/unit/test_pid.py` の既存テストのうち `TestPIDControllerDerivative` 3件
  （test_d_first_step / test_d_constant_error / test_d_decreasing_error）と
  `TestPIDMeasuredDt.test_measured_dt_used_for_derivative` は kd=1.0 で生の後退差分値
  （200.0 / 0.0 / -100.0 / -50.0 等）を直接検証していたため、LPF 導入後は期待値の更新が
  必要だった。design.md/requirements.md の互換性保証は明示的に「kd=0 のとき出力が従来と
  完全一致」の範囲であり、kd≠0 で生微分値を検証するテストは LPF と原理的に両立しない。
  各テストをフィルタ後の値（40.0 / 32.0 / 12.0 / 10.0）に更新し、計算根拠をコメントで残した。
  `test_reset_clears_state` には `_d_filt == 0.0` の確認も追加した。
- `CoordinateDescentTuner` の実装は design.md の実装スケッチをほぼそのまま採用。
  `_baseline_done` フラグでベースライン評価を座標巡回のカウントから明確に分離する点が
  設計スケッチには無かった実装判断（ベースライン report 時に param_idx を進めてしまうと
  最初の実座標が kp ではなく ki から始まってしまうバグになるため）。

**新たに必要になったタスク**:
- なし（設計どおりの2ファイル変更で完結）

### 学んだこと

**技術的な学び**:
- 1次LPFを微分項に挟むと、たとえ kd の値そのものは変えなくても「誤差が一定に戻った直後の
  出力」はフィルタ残留の影響で即座にゼロへ戻らない（指数減衰する）。生の後退差分を直接
  検証するテストは、フィルタ導入のような挙動変更と本質的に非両立になるため、フィルタ済み
  出力かフィルタ状態（kd=0 の場合は無関係）のどちらを保証したいかをテスト設計時に明確に
  分けておく必要がある。
- 巡回座標降下でベースライン測定を「座標の1つ」として扱うと、座標順序がずれる罠がある。
  ベースラインは常に別状態として扱い、座標カウンタに影響させないことで
  「kp から必ず開始する」という契約を守れた。

**プロセス上の改善点**:
- 各コンポーネント変更ごとに小さくテストを回し（pid.py 先行 → pid_tuning.py）、都度
  `pytest`/`ruff` を実行したことで、依存のない2ファイル変更が最後まで独立性を保てた。

### 次回への改善提案
- 次にブレーキ側の独立同定や速度域別ゲインスケジューリングに着手する際は、本作業で
  導入した `DERIV_FILTER_TAU_S` がブレーキ側 FOPDT の時定数と整合するか再検証すること。
- `compute_pid_gains_simc` に SIMC PID 形（τD=θ/2 相当）で Kd 解析初期値を導入する場合、
  今回の LPF がある前提で安全に導入できる（design.md 将来拡張性に追記済み）。
