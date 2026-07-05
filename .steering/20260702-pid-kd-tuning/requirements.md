# 要求内容

## 概要

PID自動適合（.steering/20260620-pid-auto-tuning）の閉ループ絞り込みで、Kd が実際に評価・採用される
ようにする。あわせて PID 微分項にローパスフィルタを追加し、Kd 導入時の CAN 車速量子化ノイズによる
微分の暴れを抑える。

## 背景

「PID適合を行っても Kd がいつも変更されない」という問題の調査（2026-07-02, Fable 実施）で、
以下の構造的原因が特定された:

1. **学習時（SIMC）**: `compute_pid_gains_simc`（`src/domain/pid_tuning.py:189-221`）は
   設計として常に `kd=0.0` を返す。これは CAN 車速の量子化ノイズ対策で意図どおり（変更しない）。
   design.md（20260620）は「Kd は必要なら閉ループ絞り込みで導入」としている。
2. **閉ループ絞り込み**: `CoordinateDescentTuner` は kp/ki/kd の3軸を探索する設計だが、
   3つの要因が重なり Kd 候補がほぼ走行されない（詳細は design.md 参照）:
   - 候補順が `[kp+, kp−, ki+, ki−, kd+, kd−]` 固定で、改善が出ると `report()` が
     ラウンド全破棄（再センタリング）→ 末尾の kd 候補は走行前に捨てられる
   - `max_runs=15` では kd 候補到達前に予算が尽きる
   - kd=0 起点のステップは `0.3 × (0 + _BASE 0.1) = 0.03` と極小。マイナス側候補は
     `max(0, −0.03) = 0` で best と同一の無駄走行
3. **PID 実装**: `PIDController`（`src/domain/control/pid.py:68`）の微分項は生の後退差分で
   フィルタなし。dt=50ms で CAN 量子化 1km/h が 20km/h/s のスパイクとなり、フィルタなしでは
   意味のある Kd はコスト（reversal KPI 悪化）で棄却され続ける。

つまり「Kd を導入するはずの閉ループ絞り込み」が実装上機能していない。

## 実装対象の機能

### 1. CoordinateDescentTuner の巡回座標降下化

- kp → ki → kd の順に各座標を確実に1回ずつ評価する巡回方式に変更
- 改善時はラウンド破棄ではなく「採用して次の座標へ」進む
- クランプで best と同値になる候補は生成しない（無駄走行の排除）
- kd の探索基準ステップを拡大（`_BASE["kd"]: 0.1 → 0.5`）

### 2. PID 微分項の1次ローパスフィルタ

- 後退差分微分に1次 LPF（時定数 0.2s）を追加
- kd=0 の既存プロファイルには挙動影響なし

## 受け入れ条件

### CoordinateDescentTuner
- [ ] baseline + kp(≤2走行) + ki(≤2走行) の後、遅くとも7走行目までに kd 候補が走行される
- [ ] kp/ki が改善し続けるコスト形状でも、max_runs=15 内に kd 候補が評価される
- [ ] kd のみ改善が効く凸コストで kd が実際に更新される
- [ ] クランプにより best と同値になる候補（kd=0 の − 側等）が生成されない
- [ ] 公開 API（`next_candidate`/`report`/`best`/`best_cost`/`runs`）は不変で
      `robot_controller.py` は無変更
- [ ] 停止則（改善なし1巡でステップ半減、min_step_frac 未満 or max_runs で停止）は既存踏襲

### PID 微分 LPF
- [ ] 微分項スパイク（1サンプルの誤差ジャンプ）が未フィルタ比で大幅に減衰する
- [ ] kd=0 のとき出力が従来と完全一致（既存テストが無修正で通る）
- [ ] `reset()` / `set_gains()` でフィルタ状態もクリアされる

## 成功指標

- 実機の `/drive/pid-tune/refine` の反復履歴（history）に kd≠0 の候補が現れ、
  採否がコストで決まる（＝Kd が「評価されない」のではなく「評価の結果」になる）

## スコープ外

以下はこのフェーズでは実装しません:

- `compute_pid_gains_simc` での Kd 解析算出（SIMC PID 形 τD=θ/2 相当）。学習時の kd=0
  初期化は設計どおり維持
- `run_pid_tuning_session` / Web API / UI の変更（Tuner の公開 API 不変のため不要）
- 速度域別ゲインスケジューリング

## 参照ドキュメント

- `.steering/20260620-pid-auto-tuning/design.md` - PID自動適合の全体設計（本作業はその続き）
- `docs/functional-design.md` - 機能設計書
- `docs/architecture.md` - アーキテクチャ設計書
