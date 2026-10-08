"""手順 2-1 の閉ループパターン走行を実行する、研究用の自前状態機械。

ProblemReport_20260910 の遵守事項「`/src` の本番環境のコードを実行しないこと。実行環境は
すべて `tests/` に記載すること」に対応する。以前は `src.domain.control.learning_loop.LearningLoop`
をサブクラス化してそのまま実行していたが、それは「引用」ではなく本番コードの実行に当たるため、
アルゴリズムだけを移植した自前実装に置き換えた。

引用元（アルゴリズムを移植。クラス自体は import・実行しない）:
    src/domain/control/learning_loop.py … フェーズ遷移（MEASURE/DRIVE_ACCEL/COAST/DRIVE_BRAKE/
        BRAKE_HOLD/CRUISE_TRIM）、包絡線ガバナ（上限G厳守）、ランプ、オーバースピード回復、
        非常停止判定（CAN取得失敗・アクチュエータ通信失敗・過電流）のロジック。
    src/domain/safety_monitor.py … `check_overcurrent`（`current_ma > limit` の比較のみ）。

本番との違い（研究用にシンプルにしたところ）:
    - `CycleLoopBase`（本番の共通基盤）が持つ絶対時刻グリッドスケジューリング・
      ウォッチドッグ（サイクル未完了の連続監視）・DBログ書き込みバックログ管理は持たない。
      研究ハーネスは単発の短い走行のみを行うため、固定周期の単純な asyncio ループで足りる。
    - `SafetyMonitor` クラスは使わず、過電流しきい値との比較を直接行う（同じロジック）。
    - `RealtimeSnapshot`（WebSocket 配信用のキャッシュ）は持たない。研究ハーネスは配信しない。
    - サンプルごとのログフックは `on_sample` コールバックとして最初から持ち、
      本番の `_enqueue_log_write` をサブクラスでオーバーライドする形は取らない。

2026-09-13 A6（KAIZEN 表5-5 順3。report20260912_KAIZEN_process2,3.md）で本番から変えたところ:
    - ガバナーの解除: 本番は頭打ちをフェーズが変わるまで戻さず、実機では DRIVE_BRAKE が 8〜10%
      （不感帯未満）・DRIVE_ACCEL が 19〜22% に張り付いて「打ち切りに張り付く」原因になっていた。
      加減速が上限 × gov_release_frac を下回ったら gov_raise_step_pct ずつ戻し、
      指令に届いたら解除する。
    - 停車復帰を全運転パターンの後に置く: ACCEL_SWEEP だけでなく BRAKE_HOLD・COAST_DOWN・
      CRUISE_TRIM も、終わって停車していなければ DRIVE_BRAKE で停車させてから次へ進む
      （次の段を必ず停車から始める）。目標は ACCEL_SWEEP がリセットブレーキ、他は停車保持開度。
    - DRIVE_BRAKE の打ち切りは「過ぎたら次へ進む」ではなく
      「過ぎても停車しなければ中断する安全上限」。
    - ブレーキのランプはフェーズに入ったときの開度から始める
      （BRAKE_HOLD の保持開度から 0% に抜けない）。
    - 走行中の安全網（axis_safety.py: アラーム確認・電流ゼロの継続）を入れ、
      中断理由を abort_reason に残す。

2026-09-13 A2・A5（KAIZEN 表5-5 順4）で足したところ:
    - 目標車速つきパターン `SpeedTargetPattern`: 加速を cap ではなく accel_target_kmh で終える
      （BRAKE_HOLD を 60 km/h から保持する段に使う。src の LearningPattern は変えない）。

2026-09-13 A3・A4（KAIZEN 表5-5 順5）で足したところ:
    - トリム階段 `TrimStairPattern`（kind は CRUISE_TRIM のまま）: accel_target_kmh まで加速したら、
      trim_steps_pct をこの順に step_hold_s ずつ保持し、終わったら停車復帰する。
      cap に達したら次の（低い）段へ下げ、最後の段は cap に達しても保持を続ける
      （最高速を超えたら今と同じ回復に入る）。通常の CRUISE_TRIM は今と同じく cap で終える。
    - A4（20 km/h から高ブレーキで停車）は SpeedTargetPattern の BRAKE_HOLD をそのまま使う。

2026-09-13 A7（KAIZEN 表5-5 順6）で変えたところ:
    - 各軸の電流の読み取り（CNOW）を、0x9000 から 14 レジスタのまとめ読み（`read_monitor`）に
      替えた。実位置・ステータスも同じ 1 回で取れる。過電流・安全網の判定は電流だけを見る（中身は
      変えない）。
    - `on_sample` に両軸のモニターと、その周期の処理時間 cycle_ms（車速を読む直前から
      まとめ読みの後まで）を足して渡す。

2026-09-14 定速階段（段2）で足したところ:
    背景（docs/memo.md「応答遅れの実測 → 高速域の偏りの原因 → 定速階段＋走行抵抗式（C6）の計画」）:
    手順2に「定速保持」の状態が 50 km/h 以上で 1 行も無く、FF モデルの定速開度の予測が
    60〜80 km/h で浅く（遅れ）・130 km/h で深く（出すぎ）なる原因になっていた。手順3では
    指令→実開度の遅れがほぼ無い（|実−指令| p95 0.06〜0.07%）と実測済みなので、階段の開度は
    低ゲイン・レート制限の緩い PI で「実開度 ≈ 指令」を保ったまま定常開度を記録すればよい。
    - トリム階段と同じ流儀の `CruiseStairPattern`（kind は CRUISE_TRIM のまま）を追加:
      `hold_speeds_kmh`（昇順。例 30〜130 km/h を 10 km/h 刻み）へ、専用フェーズ
      `_Phase.CRUISE_HOLD` で 1 本ずつ弱い PI 保持する。DRIVE_ACCEL は最初の車速
      （`hold_speeds_kmh[0]`）で終える（`_accel_target_kmh`）。
    - CRUISE_HOLD の開度は速度偏差 [km/h] → 開度 [%] の PI（`_cruise_hold_opening`）。
      アンチワインドアップ（[不感帯, max_accel_opening] で頭打ちの方向へはこれ以上積分しない）・
      1 周期あたりの変化量を `max_rate_pct_per_s × interval_s` に制限（実開度が指令に追従できる
      速さに保つ）。初期開度は「アクセル不感帯 + initial_offset_pct」（DRIVE_ACCEL の加速用開度
      は定速に対して大きすぎ、そこから始めるとレート制限で最初の車速に間に合わないため）。
      積分・開度は車速をまたいでも引き継ぎ、段の変わり目で跳躍させない。
    - 前進判定（`_advance_cruise_hold`）: |車速−目標| ≤ settle_tol_kmh が settle_s 続いたら
      保持タイマーを開始し、hold_s 経過で次の車速へ（積分は引き継ぐ・settle/hold タイマーだけ
      リセット）。step_timeout_s を過ぎたら保持できていなくても次へ進む。最後の車速の後は
      `_finish_pattern`（停車復帰）。最高速超え・低速側（coast_down_stop_speed_kmh 以下）は
      トリム階段と同じ回復（前者は DRIVE_BRAKE の速度超過回復、後者は停車復帰）。
    - `_move_duration` は CRUISE_HOLD も DRIVE_ACCEL と同じ短い一手（pedal_step_time_s）にし、
      実開度が指令に追従できるようにする。G ガバナー（`_update_governor`）は CRUISE_HOLD に
      適用しない（緩い PI・レート制限そのものが穏やかなので、ガバナーによる頭打ちは不要と判断）。

2026-09-17 クリープ発進・クリープ域ブレーキ保持（段1。ProblemReport_20260916 課題#2）で足したところ:
    背景（docs/Problem/ProblemReport_20260916.md）: 本番のクリープ加速率 creep_rate_kmhs=0.19 が
    実測クリープ加速（中央値 2.39 km/h/s）と 12 倍ずれていた。原因は「ペダルオフ・低速・加速中」の
    全サンプルの中央値を取る既存推定（手順2がクリープ安定まで待つため母集団が定常側に偏る）。
    - `CreepLaunchPattern`（kind は CREEP_SETTLE を流用）を追加: 専用フェーズ `_Phase.CREEP_LAUNCH`
      で両ペダル 0%（完全解放）を指令し、`target_kmh` 到達または `timeout_s` の打ち切りまでクリープ
      のみで自走させる（`_advance_creep_launch`）。到達後は `hold_after` で分岐:
      False（クリープ発進）は通常の停車復帰（`_finish_pattern` → `_Phase.DRIVE_BRAKE`）、
      True（クリープ域ブレーキ保持）は `brake_opening`（不感帯 + offset）を保持する
      既存の `_Phase.BRAKE_HOLD` へ直接入る。
    - `_move_duration` は変更しない（CREEP_LAUNCH は COAST と同じく既定の pedal_release_time_s）。
      G ガバナー（`_update_governor`）も適用しない（両ペダル 0% で開度指令が無いため不要）。

2026-09-19 低開度階段（段3-1。ProblemReport_20260919 候補(c)）で足したところ:
    背景（docs/Problem/ProblemReport_20260919.md 6章）: 手順2 に「不感帯の直上を細かく刻んで
    一定保持する」パターンが 1 本も無く、5〜24 km/h × 不感帯 +0〜2% の学習行が 5.4 秒しか
    無かった（既存の ACCEL_DEADBAND_PROBE は無ランプ・昇順で停車に戻らずつながっており、
    開度と車速が一体で動くため定常判定をほとんど通らない）。
    - `LowOpenStairPattern`（`TrimStairPattern` を継承。kind は CRUISE_TRIM のまま）を追加:
      停車から `accel_target_kmh`（= 1 段目と同じ開度で終える低い車速）まで加速し、
      `trim_steps_pct` を `step_hold_s` ずつ一定保持して、その開度固有の平衡車速へ収束させる。
      `TrimStairPattern` の機構（`_command_openings` の CRUISE_TRIM 分岐・
      `_phase_after_accel`・`_finish_pattern` 等）をそのまま使うが、低速域が運転域の
      真ん中に来るため、低速終了の判定だけ `min_speed_kmh`（既定 2.0）に差し替える
      （`_advance_trim_stair`。トリム階段の `coast_down_stop_speed_kmh`=5.0 のままだと
      1 段目で即終了してしまう）。

2026-09-25 段4（ProblemReport_20260925）で削除したところ:
    手順2 のパターン列を格子ステップ走行に置き換えたため、上記の履歴のうち次の機構はコードから
    消した（履歴は経緯として残す）: 目標車速つき `SpeedTargetPattern`・トリム階段・定速階段・
    低開度階段（`TrimStairPattern`・`CruiseStairPattern`・`LowOpenStairPattern`）と、それらの
    前進判定・PI。ACCEL_SWEEP・BRAKE_HOLD（固定ブレーキ保持）・CRUISE_TRIM・不感帯プローブの各
    `PatternKind` も無い。残るのは COAST_DOWN・クリープ発進（CreepLaunchPattern。ブレーキ保持は
    `_Phase.BRAKE_HOLD`）・格子ステップ走行。`_Phase.CRUISE_HOLD` は格子ステーションの PI 保持が
    使う（CSV の phase 名を段3 と揃えるため名前は据え置き）。

2026-09-28 段7c（ProblemReport_20260925 段7。実機 20260928_035806 で見つかった不具合の修正）で
変えたところ:
    背景: 15 km/h ステーションで、強い減速の助走のため 25 km/h へ GRID_RETURN に入ったが、
    アクセル指令 8.11%（釣り合う車速 21.2 km/h）で 190 s 止まった。打ち切りが効かないバグと、
    感度の初期値（未測定のアクセルの感度を仮定の 2.0 のまま使う）が浅すぎたことが重なった。
    - GRID_RETURN の打ち切り（バグ修正）: `_advance_grid_hold` の打ち切り処理を
      `_grid_handle_settle_timeout` に切り出し、`_advance_grid_return` でも
      `now − _grid_settle_started ≥ settle_timeout_s` を判定する（今までは CRUISE_HOLD 側にしか
      無く、GRID_RETURN では永久に待ち続けていた）。
    - GRID_RETURN の開度: 踏み始めは従来の式（u0 + (狙い − a_hold) / g_accel）のまま、以降は毎周期
      `_step_grid_return` が直近 `coast_accel_slope_window_s` の実測加速度と
      `grid_return_accel_kmhs` の差に応じて開度を刻み足す（G 校正の加速 `_step_coast_accel` と
      同じ式。新しい調整値は増やさない）。
    - G 校正の加速（`_step_coast_accel`）: 開度が目標 G ± 押し込み余裕で止まっている瞬間しか
      点を記録していなかったため、40〜110 km/h 帯では開度が動き続けてアクセルの点が 1 つも
      取れず、上限開度表のアクセル列が全帯「—」になっていた。開度がゆっくり動いている
      （`gov_lead_s` 前の窓の振れが `coast_accel_record_max_change_pct` 以下）間も 1 s に 1 点
      記録するようにした（`_ca_slow_move_opening`）。**段7d で「振れが小さい間だけ」の条件は
      やめ、実開度＋遅れずらしの平均で常に記録する方式に置き換えた（下記）**。
    - 格子ステップの最初の感度: そのステーションでまだ測っていないペダルの最初の開度は、感度の
      仮定値（2.0）ではなく `glimit`（上限 G の予測マップ）にあるその車速帯の一番効いた実測点から
      引く（`GridPlanner` の `gain_fn`。`GLimitMap.strongest_point`）。HOLD・COAST（惰行）の実測
      直後に 1 度だけ種をまき、以降は従来どおり実測で更新する。発進・停車セル（`begin_launch`）は
      今のまま（直前のステーションの感度を引き継ぐ）。
    - コーストダウンの終了: `coast_down_stop_speed_kmh`（固定 5.0）ではなく、クリープ平衡車速
      （`feedforward.creep_speed_kmh`。2-0 実測）を基準にする。惰行は平衡へ漸近するだけで
      ちょうど届く保証がないため、平衡以下に達するか、クリープの影響が残る車速
      （平衡 + `grid_settle_tol_kmh`）以下で車速の傾きが `creep_launch_settle_kmhs` 未満の状態が
      `creep_launch_settle_s` 続いたら終える（`_coast_creep_settled`。停車ステップの
      `_grid_creep_settled` と同じ判定・同じ調整値、`_grid_samples` の代わりに毎周期の
      `_speed_hist` を使う）。`coast_down_stop_speed_kmh` の他の使い道（G 校正のブレーキ減速の
      終了・停車復帰の切替・格子ステップの底・発進セル）はブレーキを踏む区間でクリープに
      左右されないため変えない。

2026-09-28 段7d（ProblemReport_20260925 段7。実機 20260928_050944 で見つかった不具合の修正）で
変えたところ:
    背景: 段7c 後の G 校正の上限開度表で、アクセル上限が 0〜10 km/h で 227.3%（バグ）、
    70〜110 km/h が 60.5% で平ら（点が取れず借用）、10〜30 km/h が実測より高め（危険側）だった。
    - 偽の点（0〜10 km/h の 227.3%）: 前のパターン（コーストダウン加速）の指令開度 70% が
      `_ca_opening` に残ったまま、次の G 校正の踏み始め（接近ランプ中）に開度の履歴へ積まれ、
      踏み始め直後に (70%, 実測は低加速度) という偽の点になっていた。実開度で記録するように
      すれば、接近ランプ中の実際の位置（0% 付近）が積まれるので直る。`_enter_phase` でも
      `_ca_opening_hist` を空にする（保険）。
    - 記録は「開度が止まっている／振れが小さい」の条件をやめ、実開度（`monitor_accel` から。
      指令ではなく、低速の踏み込みで指令より遅れる実際の位置）を、遅れ `gov_lead_s`（0.6 s。
      ブレーキ指令から G の山までの遅れの実測値の流用）ぶん過去へずらした窓で平均し、今の G と
      対にして 1 s に 1 点記録する（`_ca_lagged_opening`）。開度が動き続けていても対応関係は
      崩れないため、70〜110 km/h も含め全帯で点が取れる。
    - 上限開度表（`GLimitMap.table`）: 予測開度が 100% を超える帯は数値の代わりに
      「100%(届かない)」と表示する（実際に機構の上限 G まで踏めないという意味。上限としての
      動きは変えない）。

2026-10 段2k 段E（ProblemReport_20261001）で削除したところ:
    手順2 のパターン列は G ランプ走行（`GRampPattern`）に置き換わったため、格子ステップ走行・
    通し掃引・G 校正の機構（`GridStationPattern`・`GridLaunchPattern`・`GridSweepPattern`、
    フェーズ CRUISE_HOLD・HOLD_STEP・GRID_RETURN・CALIB_BRAKE・SWEEP_UP・SWEEP、`PatternKind` の
    GRID_STEP・GRID_LAUNCH・G_CALIB・GRID_SWEEP、grid_planner.py ほか）をコードから消した。
    上の段7c・7d の履歴のうち、これらに触れる箇所は経緯として残してある。残るのは
    COAST_DOWN・クリープ発進・クリープ域ブレーキ保持・G ランプ走行・停車復帰（高い車速からは
    `_step_gprop_brake` の G 比例ブレーキ）。

2026-10-08 段2l-6（ProblemReport_20261001）で変えたところ:
    不感帯の測定をやめ、手で測った「ペダルに触れた位置」min（vehicle.min_*_opening_pct。
    プロファイルの feedforward_params.accel_deadband_pct・brake_deadband_pct に入る。名前は
    不感帯のまま、意味は u の原点）を待機位置かつ u の原点にした。
    - 待機位置: 「不感帯 − standby_margin_pct」をやめ min そのもの（`pedal_standby`）。ラッチ後の
      ブレーキは u ≤ 0 でブレーキ ＝ min ＋ |u|（下限 min・u の上限 0）。
    - 「踏んでいない」の判定に幅 PEDAL_OFF_TOL_PCT（待機位置がしきい値ちょうどになり、実位置の
      ずれで行が割れるため）。G ランプの実測点を貯める条件に使う。
    - G ランプの最初の開度の見積もりは外挿しない（`_glimit_open_for(extrapolate=False)`）。
    - RAMP_UP の発進（`_rp_launch`）: 停車保持開度から止まった開度（2-0 の stop_confirm_pct）まで
      戻し、そこから毎周期「実測 G ＜ 狙い（保持マージンの外）」のときだけブレーキを下げる。
      単調に下がり、踏み直さない。min に着いたらアクセルの u 制御（u ≥ 0）に渡す。
    - RAMP_DOWN は 5 km/h（coast_down_stop_speed_kmh）ではなく停車（STOP_SPEED_KMH）まで続ける。

2026-10-08 段2l-6b（ProblemReport_20261001。ユーザー決定）で変えたところ:
    コーストダウンの停車復帰（DRIVE_BRAKE）の境目を車ごとの値にした。クリープ車速は車によって
    違うので、固定の `coast_down_stop_speed_kmh`（5.0）ではなく「クリープ車速 + 落ち着きの許容幅」
    （`_creep_ceiling_kmh(grid_settle_tol_kmh)`）より上は G 比例ブレーキ、以下（クリープ域）は
    今のブレーキ開度から停車保持開度まで一定の速さで踏み上げる（`_db_creep_press`。速さは
    `_step_gprop_brake` の 1 刻み/待ちと同じ。単調に上げ、途中で下げない）。次のパターンへ進むのは
    停車かつ停車保持開度に届いてから、`brake_stop_timeout_s` も届いてから数える。G ランプなど
    他のパターンの停車復帰・速度超過の回復（overspeed_recovery）は従来どおり。
"""

from __future__ import annotations

import asyncio
import statistics
import time
from collections import deque
from collections.abc import Awaitable, Callable, Sequence
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Protocol

import numpy as np

from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet
from tests.research.g_limit import PEDAL_ACCEL, PEDAL_BRAKE, GLimitMap
from tests.research.research_types import (
    ACTUATOR_PULSE_MAX,
    G_TO_KMHS,
    PEDAL_OFF_TOL_PCT,
    VEHICLE_STOP_SPEED_KMH,
    DriveLogData,
    LearningPattern,
    PatternKind,
    VehicleProfile,
    clamp_opening,
    enforce_pedal_exclusion,
    opening_to_position,
    position_to_opening,
)


def fit_slope(times_s: Sequence[float], speeds_kmh: Sequence[float]) -> float:
    """車速 [km/h] の時間 [s] に対する最小二乗の傾き [km/h/s]。2 点未満・時間幅 0 なら 0。"""
    if len(times_s) < 2 or times_s[-1] - times_s[0] <= 0.0:
        return 0.0
    slope, _ = np.polyfit(np.asarray(times_s, dtype=float), np.asarray(speeds_kmh, dtype=float), 1)
    return float(slope)


PATTERN_LOOP_INTERVAL_S: float = 0.1  # 100ms 周期（drive_logs の記録間隔に一致。本番と同じ）
STOP_SPEED_KMH: float = VEHICLE_STOP_SPEED_KMH
# G ランプ 1 区間（加速または減速）の打ち切り = 見込み秒数（上端 ÷ 狙いの加速度）× 係数 + 余裕。
# 仮値（データの根拠なし）。狙いの G に届かず上端に着けない・クリープ域まで下りられない場合の
# 安全網で、通常の走行には効かない。2l の実機で実際の所要秒数を見て決め直す
RAMP_LEG_TIMEOUT_FACTOR: float = 3.0
RAMP_LEG_TIMEOUT_MARGIN_S: float = 10.0


@dataclass(frozen=True)
class CreepLaunchPattern(LearningPattern):
    """クリープ発進・クリープ域ブレーキ保持（段1。ProblemReport_20260916 課題#2。研究側の追加）。

    両ペダル 0%（完全解放）でクリープ自走し、専用フェーズ `_Phase.CREEP_LAUNCH` で `target_kmh`
    到達または `timeout_s` の打ち切りを待つ。到達後の扱いは `hold_after`:
      - False（クリープ発進）: 通常の停車復帰（`_Phase.DRIVE_BRAKE`、停車保持開度）。
      - True（クリープ域ブレーキ保持）: `brake_opening`（不感帯 + offset。呼び出し側で
        max_brake_opening にクランプ済み）を保持して停車させる（既存の `_Phase.BRAKE_HOLD`）。

    kind は CREEP_SETTLE を流用する（両ペダル解放でクリープを測る、という意味は共通）。CSV の
    pattern 列表示にのみ影響し、`_advance` の CREEP_SETTLE 専用分岐は `_Phase.MEASURE` でしか
    見ない（本パターンの初期フェーズは CREEP_LAUNCH）ため衝突しない。
    """

    target_kmh: float = 0.0
    timeout_s: float = 20.0
    hold_after: bool = False


@dataclass(frozen=True)
class GRampPattern(LearningPattern):
    """G ランプ走行 1 本（段2k 段B。kind は G_RAMP）。

    停車から `+target_g` を狙って `top_kmh` まで加速（RAMP_UP）→ `-target_g` を狙って減速
    （RAMP_DOWN）→ クリープ域まで下りたら通常の停車復帰。開度は固定値を持たず、走行中に
    実測の G を見て「効く開度 u」を動かして作る（`accel_opening` 等は使わない）。
    """

    target_g: float = 0.0  # 狙いの G（正。加速は +、減速は − に使う）
    top_kmh: float = 0.0  # 上端の車速 [km/h]


@dataclass(frozen=True)
class RampResult:
    """G ランプ 1 本の結果（段ごとの表示用。`PatternLoop.ramp_results`）。"""

    pattern_idx: int
    target_g: float
    top_kmh: float
    up_s: float  # RAMP_UP の所要秒数
    down_s: float  # RAMP_DOWN の所要秒数
    accel_g_median: float | None  # 加速側の実測 G の中央値（取れなければ None）
    decel_g_median: float | None  # 減速側の実測 G の中央値（正。取れなければ None）
    max_speed_kmh: float  # 到達した最高車速


class _Phase(Enum):
    MEASURE = auto()
    DRIVE_ACCEL = auto()
    COAST = auto()
    DRIVE_BRAKE = auto()
    BRAKE_HOLD = auto()
    CREEP_LAUNCH = auto()  # クリープ発進・クリープ域ブレーキ保持（段1）。両ペダル 0% で自走
    RAMP_UP = auto()  # G ランプ走行（段2k 段B）: 狙いの G で上端の車速まで加速
    RAMP_DOWN = auto()  # G ランプ走行（段2k 段B）: 同じ G で減速してクリープ域まで下りる
    DONE = auto()


@dataclass
class PatternLoopConfig:
    """開ループ実行のタイミング・ランプ。`LearningLoopConfig` と同じフィールド名。

    既定値も本番と同じ。ただし A6 で accel_full_range_timeout_s（20 → 60s）・brake_stop_timeout_s
    （10 → 60s、意味も「停車しなければ中断する安全上限」）を変え、gov_release_frac・
    gov_raise_step_pct（ガバナーの解除）を足した。
    """

    brake_stop_timeout_s: float = field(default=60.0)
    brake_hold_timeout_s: float = field(default=20.0)
    accel_ramp_time_s: float = field(default=1.5)
    brake_ramp_time_s: float = field(default=1.5)
    pedal_release_time_s: float = field(default=0.4)
    pedal_step_time_s: float = field(default=0.2)
    skip_consecutive_required: int = field(default=2)
    g_smoothing_window_s: float = field(default=0.4)
    g_limit_frac: float = field(default=0.98)
    gov_reduce_step_pct: float = field(default=2.0)
    gov_release_frac: float = field(default=0.7)  # 加減速が上限のこの割合を下回ったら頭打ちを戻す
    gov_raise_step_pct: float = field(default=0.5)  # 戻すときの 1 周期あたりの量 [%]
    accel_speed_cap_frac: float = field(default=0.98)
    accel_full_range_timeout_s: float = field(default=60.0)
    overspeed_lead_s: float = field(default=1.2)
    overspeed_recovery_brake_pct: float = field(default=30.0)
    coast_down_stop_speed_kmh: float = field(default=5.0)
    coast_timeout_s: float = field(default=90.0)
    # 段7c: コーストダウンの終了判定に使う「落ち着いた」許容幅（learning.grid_settle_tol_kmh から
    # pattern_drive.py が渡す。既定値は config.LearningSection と揃えている）
    grid_settle_tol_kmh: float = field(default=1.0)
    creep_settle_min_s: float = field(default=13.0)
    creep_settle_stable_tol_kmhs: float = field(default=0.3)
    creep_settle_stable_duration_s: float = field(default=2.0)
    creep_settle_timeout_s: float = field(default=25.0)
    # クリープ発進（段1b。CreepLaunchPattern）の平衡到達判定。learning.creep_launch_settle_* から
    # pattern_drive.py が渡す（既定値はここと config.py の LearningSection とで揃えている）
    creep_launch_settle_kmhs: float = field(default=0.1)
    creep_launch_settle_s: float = field(default=2.0)
    # 2026-09-17（誤判定修正）: 停車保持解放直後は車速0・傾き0で
    # abs(accel_kmhs) < creep_launch_settle_kmhs が最初から成立してしまい、車が動き出す前に
    # 「平衡到達」と誤判定していた。車速がこの値以上になるまで安定カウントを積まない
    # （本質的なガード）。learning.creep_launch_min_speed_kmh から渡す
    creep_launch_min_speed_kmh: float = field(default=1.0)
    # elapsed がこの値以上になるまで平衡到達で終了しない（_advance_creep_settle の
    # creep_settle_min_s と同じ流儀の最短時間ガード）。learning.creep_launch_settle_min_s から渡す
    creep_launch_settle_min_s: float = field(default=5.0)
    # コーストダウンの加速（DRIVE_ACCEL）。目標 G との差に比例した速さでアクセルを踏み進める
    # （G に余裕があればぐっと踏み、目標に近づくほどゆっくり、超えたら戻す）。
    # 上限は pattern.accel_opening（COAST_DOWN_ACCEL_PCT。頭打ち）。
    # target/margin/slope_window/approach_margin は decel_stop セクションと同じ値
    # （pattern_drive.py が渡す。既定値は config.DecelStopSection と揃えている）
    coast_accel_target_g: float = field(default=0.2)
    coast_accel_press_margin_g: float = field(default=0.02)  # 目標 ± これ の間は開度を保持
    coast_accel_slope_window_s: float = field(default=1.0)  # 加速度（最小二乗の傾き）の窓 [s]
    coast_accel_approach_margin_pct: float = field(default=1.0)  # min − これ まで先に上げる
    # 踏む速さ [%/s] = これ ×（目標 G − 今の G）。半分データ由来・半分仮の値:
    # 感度 0.004〜0.005 G/%・遅れ約 1s（実機 110427）から、応答の時定数（1/(これ×感度) ≒ 2〜2.5s）
    # が遅れの 2 倍以上になる値。大きいほど早く届くが 0.2G を超えやすい。実機で決め直す
    coast_accel_rate_gain: float = field(default=100.0)
    # 高い車速からの停車復帰の G 比例ブレーキ（DRIVE_BRAKE。`_step_gprop_brake`）。
    # stop_decel.decelerate_to_stop（手順2 終了時の緩減速）と同じ刻み方:
    # dwell_s ごとに直近 slope_window_s の減速 G を見て、目標 − press_margin 未満なら
    # step_mm 踏み増し、release_above_g 超なら step_mm 戻す。目標・余裕・窓・接近位置は
    # coast_accel_* を共用（decel_stop と同じ値）。pattern_drive.py が decel_stop から渡す
    calib_release_above_g: float = field(default=0.3)
    calib_step_mm: float = field(default=0.5)
    calib_dwell_s: float = field(default=1.0)
    # 惰行に入ってからこの時間は減速を数えない（ペダルを離してから車が応える遅れ。実機で
    # 指令→G の山まで約 0.6s、離す動作 0.4s）。惰行の減速 → 上限 G の予測の基準点に使う
    coast_measure_delay_s: float = field(default=1.5)
    # 段6b・門②: 上限 G を先読みして緩める。G の見込み = 今の G + G の増える速さ × gov_lead_s。
    # gov_lead_s 0.6 s は実機（20260926 の 890.7→891.3 s）で「ブレーキ指令から G の山まで」の遅れ。
    # 見込みが上限 × gov_soft_frac に届いたら踏み増しを止め、上限（× g_limit_frac）に届いたら
    # 従来どおり 1 周期ごとに下げる。gov_soft_frac 0.85 は**仮の値**（0.85 × 0.4G × 0.98 ≒ 0.33G）。
    # 実機の G の最大で決め直す
    gov_lead_s: float = field(default=0.6)
    gov_soft_frac: float = field(default=0.85)
    # 待機位置を使うか。待機位置 ＝ ペダルに触れた位置 min（プロファイルの
    # feedforward_params.*_deadband_pct。名前は不感帯のまま値は u の原点）。(1) G ランプの減速で
    # ブレーキの領域に入ったあと、ブレーキの下限を min にする（0% へ戻さない。2026-10-07 段2l-3）。
    # (2) 周期の出力で両ペダルの指令に min を下限として掛ける（使わない側を 0% にせず待機位置に
    # 置く。手順3 の mode_drive と同じ形。2026-10-07 段2l-4）。False なら (2) は掛けない
    # （今までどおり 0%）。2026-10-08 段2l-6: 旧 standby_margin_pct（不感帯 − 余裕）を置き換えた
    pedal_standby: bool = field(default=False)
    # G ランプの発進の最初のブレーキ開度 = 2-0 で測った止まった開度（stop_confirm_pct）。
    # None なら停車保持開度から始める。2026-10-08 段2l-6
    stop_confirm_pct: float | None = field(default=None)
    # コーストダウン加速の車速の上端 [km/h]（None なら車両の最高速 × accel_speed_cap_frac のまま）。
    # 選んだモードの最高車速 + 余裕（learning.ramp_top_margin_kmh）。2026-10-07 段2l-4
    top_speed_cap_kmh: float | None = field(default=None)
    gov_rate_window_s: float = field(default=0.5)  # G の増える速さを見る窓 [s]
    # G ランプ走行（段2k 段B。GRampPattern）。learning.ramp_* から pattern_drive.py が渡す。
    # 効く開度 u の動かす速さ [%/s] = ゲイン ×（狙いの G − 実測の G）のゲインの上限（感度が測れて
    # いないときはこの値）。半分データ由来: コーストダウン加速（coast_accel_rate_gain）と同じ値。
    # 前の車で感度 0.004〜0.005 G/%・遅れ約 1 s から決めた。2026-10-07 段2l-2 で意味を「上限」に変更
    ramp_rate_gain: float = field(default=100.0)
    # G のずれを何秒かけて埋めるか。ゲイン ＝ min(ramp_rate_gain, 1 ÷（感度[G/%] × これ））。
    # 2026-10-07 段2l-2
    ramp_settle_time_s: float = field(default=2.0)
    # 狙いとの差がこの幅以内なら u を動かさない。幅 = max(狙いの G × frac, min_g)。仮値
    # （今の固定 0.02G は 0.02G の段では使えないため、狙いに比例させ下限を置いた）。
    # 開度の細かい往復に効く。2l の実機走行後に決め直す
    ramp_hold_margin_frac: float = field(default=0.10)
    ramp_hold_margin_min_g: float = field(default=0.005)


class ActuatorDriverProtocol(Protocol):
    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None: ...

    async def read_monitor(self) -> AxisMonitor: ...

    async def is_alarm_active(self) -> bool: ...


class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...


# on_sample(データ, パターン番号, 段, ガバナー作動, アラーム(アクセル), アラーム(ブレーキ),
#           モニター(アクセル), モニター(ブレーキ), 周期の処理時間 [ms])
OnSample = Callable[
    [DriveLogData, int, str, bool, bool | None, bool | None, AxisMonitor, AxisMonitor, float],
    None,
]


class PatternLoop:
    """開度パターンを開ループ実行し、1 サイクルごとに `on_sample` でログへ渡す研究用ループ。

    `src/domain/control/learning_loop.py::LearningLoop` のフェーズ遷移・ガバナ・非常停止判定を
    移植したもの（アルゴリズムは変更しない）。
    """

    def __init__(
        self,
        *,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        profile: VehicleProfile,
        patterns: list[LearningPattern],
        overcurrent_limit_ma: float,
        on_complete: Callable[[], Awaitable[None]],
        on_emergency: Callable[[], Awaitable[None]],
        on_sample: OnSample,
        config: PatternLoopConfig | None = None,
        interval_s: float = PATTERN_LOOP_INTERVAL_S,
    ) -> None:
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._profile = profile
        self._patterns = patterns
        self._overcurrent_limit_ma = overcurrent_limit_ma
        self._on_complete = on_complete
        self._on_emergency = on_emergency
        self._on_sample = on_sample
        self._config = config if config is not None else PatternLoopConfig()
        self._interval_s = interval_s

        self._max_decel_kmhs = max(profile.max_decel_g * G_TO_KMHS, 0.0)
        self._g_limit_kmhs = self._max_decel_kmhs * self._config.g_limit_frac
        self._accel_speed_cap = profile.max_speed * self._config.accel_speed_cap_frac
        if self._config.top_speed_cap_kmh is not None:  # 2026-10-07 段2l-4
            self._accel_speed_cap = min(self._accel_speed_cap, self._config.top_speed_cap_kmh)

        self._running = False
        self._cycle_task: asyncio.Task[None] | None = None
        self.abort_reason: str | None = None  # 非常停止した理由（正常終了なら None）
        # 状態機械（同期）が決めた中断。周期の終わりで処理する
        self._pending_abort: str | None = None
        self._safety = AxisSafetyNet(interval_s)
        self._cycles = 0
        self._started_at: float | None = None

        self._pattern_idx = 0
        self._phase = self._initial_phase(0)
        self._phase_started_at: float | None = None
        self._prev_speed: float | None = None
        self._prev_time: float | None = None
        self._skip_count = 0
        self._stable_count = 0
        self._accel_gov_cap: float | None = None
        self._brake_gov_cap: float | None = None
        self._overspeed_recovery = False
        # コーストダウン加速の G 比例制御（DRIVE_ACCEL）。_enter_phase で作り直す
        self._ca_t0: float | None = None  # 接近ランプの開始時刻。None ならまだ始めていない
        self._ca_approach_pct = 0.0
        self._ca_opening = 0.0  # 接近後の指令開度（ガバナー適用前）
        self._ca_last_t: float | None = None  # 直近に開度を更新した時刻
        self._ca_hist: deque[tuple[float, float]] = deque()  # 加速度の窓 (時刻, 車速)
        # 段7d: アクセルの点を記録するための実開度の履歴 (時刻, 開度)。_ca_lagged_opening が
        # 遅れを見込んだ窓の代表開度を作るのに使う
        self._ca_opening_hist: deque[tuple[float, float]] = deque()
        # 段7d: 毎周期の実開度（monitor から。_execute_one_cycle で更新）。モニタが無い経路
        # （_advance 直叩きのテスト等）では None のまま（_step_coast_accel が _ca_opening で代用）
        self._accel_actual_pct: float | None = None
        # 高い車速からの停車復帰の G 比例ブレーキ（DRIVE_BRAKE）の状態。_enter_phase で作り直す
        self._cb_t0: float | None = None  # 接近ランプの開始時刻
        self._cb_approach_pct = 0.0
        self._cb_opening = 0.0  # 接近後の指令開度（ガバナー適用前）
        self._cb_next_t: float | None = None  # 次に開度を決める時刻。None ならまだ接近中
        self._cb_hist: deque[tuple[float, float]] = deque()  # 減速度の窓 (時刻, 車速)
        # 上限 G に届く開度の予測マップ（段6a）。コーストダウン・G ランプの実測で育てる
    # （G ランプの最初の開度の見積もりに使う）
        self.glimit = GLimitMap()
        # 停車復帰（DRIVE_BRAKE）を G 比例で行うか（高い車速から入ったとき。_enter_phase で決める）
        self._db_gprop = False
        # コーストダウンの停車復帰のクリープ域で、停車保持開度まで徐々に踏むか（段2l-6b）
        self._db_creep_press = False
        self._db_press_reached_at: float | None = None  # 停車保持開度に届いた時刻
        self._cb_start_pct = 0.0  # G 比例ブレーキの最初の開度（接近ランプの目標）
        # G ランプ走行（段2k 段B）。RAMP_UP に入るたびに _enter_phase が作り直す
        self._rp_u: float | None = None  # 効く開度 u [%]（None なら RAMP_UP の最初の見積もり前）
        # この減速の区間で一度でもブレーキ側（u ＜ 0）に入ったか（2026-10-07 段2l-2）。立つと
        # アクセルへ戻らない（惰行の減速は車速が下がるほど弱くなるので、戻る必要がない）
        self._rp_brake_latched = False
        # 発進の段階（2026-10-08 段2l-6）: RAMP_UP の頭でブレーキを止まった開度から min まで
        # 単調に離す。その間 u は None のまま。min に着いたら False にして u を見積もる
        self._rp_launch = False
        self._rp_launch_brake_pct = 0.0  # 発進の段階のブレーキ指令（ガバナー適用前）
        self._rp_last_t: float | None = None  # 直近に u を更新した時刻
        self._rp_hist: deque[tuple[float, float]] = deque()  # 加速度の窓 (時刻, 車速)
        # 実開度の履歴 (時刻, アクセル, ブレーキ)。遅れを見込んだ窓の代表開度を作る
        self._rp_open_hist: deque[tuple[float, float, float]] = deque()
        self._rp_record_t = -1e9  # 上限 G マップに最後に足した時刻（1 s に 1 点）
        self._rp_up_started = 0.0
        self._rp_down_started = 0.0
        self._rp_max_speed = 0.0
        self._rp_up_g: list[float] = []  # 加速側の実測 G（符号つき。窓が溜まってから）
        self._rp_down_g: list[float] = []  # 減速側の実測 G（符号つき）
        self.ramp_results: list[RampResult] = []  # G ランプ 1 本ごとの結果
        self._brake_actual_pct: float | None = None  # 毎周期のブレーキ実開度（モニターから）
        self._accel_hist: deque[tuple[float, float]] = deque()  # (時刻, 平滑化した加速度)
        self._accel_rate = 0.0  # 加速度の変化の速さ [km/h/s²]（先読みガバナー用）
        self._ca_record_t = -1e9  # 加速側の点を最後に足した時刻（1 s に 1 点）
        self._speed_hist: deque[tuple[float, float]] = deque()
        self._last_speed = 0.0  # 定速階段の PI が使う直近の車速（_execute_one_cycle で更新）
        self._accel_pos_cmd = 0
        self._brake_pos_cmd = 0
        self._current_accel_opening = 0.0
        self._current_brake_opening = 0.0
        self._accel_request = 0.0  # ガバナーをかける前の指令開度（解除の判定に使う）
        self._brake_request = 0.0
        self._brake_ramp_from = 0.0  # ブレーキのランプを始める開度（フェーズに入ったときの開度）
        # 停車復帰の目標開度 = 停車保持開度
        self._stop_return_brake_pct = min(
            self._profile.feedforward_params.stop_brake_opening_pct,
            self._profile.max_brake_opening,
        )

        calib = self._profile.calibration
        if calib is not None:
            self._accel_pos_cmd = calib.accel_zero_pos
            self._brake_pos_cmd = opening_to_position(
                self._stop_return_brake_pct, calib.brake_zero_pos, calib.brake_full_pos
            )
        if self._phase is _Phase.RAMP_UP:  # 最初が G ランプのとき（_enter_phase を通らない）
            self._ramp_launch_begin()

    # ── ライフサイクル ─────────────────────────────────────────────
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._cycle_task = asyncio.ensure_future(self._run())

    def stop(self) -> None:
        self._running = False

    async def stop_and_join(self, timeout_s: float = 2.0) -> None:
        self.stop()
        task = self._cycle_task
        if task is None or task.done() or task is asyncio.current_task():
            return
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout_s)
        except TimeoutError:
            task.cancel()
        except Exception:
            pass

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        next_tick = loop.time()
        try:
            while self._running:
                await self._execute_one_cycle()
                if not self._running:
                    return
                next_tick += self._interval_s
                delay = max(0.0, next_tick - loop.time())
                await asyncio.sleep(delay)
        except Exception as exc:
            await self._abort_emergency(f"パターン走行ループで例外（{type(exc).__name__}: {exc}）")

    # ── 1 サイクル ─────────────────────────────────────────────────
    async def _execute_one_cycle(self) -> None:
        if not self._running:
            return

        loop = asyncio.get_running_loop()
        now = loop.time()
        cycle_started = time.perf_counter()

        if self._pattern_idx >= len(self._patterns):
            self._running = False
            await self._on_complete()
            return

        pattern = self._patterns[self._pattern_idx]
        if self._phase_started_at is None:
            self._phase_started_at = now
        if self._started_at is None:
            self._started_at = now

        try:
            speed = await self._can_reader.read_speed()
        except Exception as exc:
            await self._abort_emergency(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）")
            return

        self._last_speed = speed
        self._speed_hist.append((now, speed))
        accel_kmhs = self._smoothed_accel(now)
        self._update_accel_rate(now, accel_kmhs)

        self._update_governor(accel_kmhs)
        accel_opening, brake_opening = self._command_openings(pattern, now)
        accel_opening, brake_opening = enforce_pedal_exclusion(accel_opening, brake_opening)
        self._current_accel_opening = accel_opening
        self._current_brake_opening = brake_opening
        # 2026-10-07 段2l-4: 出力だけ待機位置を下限にする（手順3 mode_drive と同じ形）。内部状態
        # （_current_*・ガバナー・停車復帰）は掛ける前の値のまま。
        # pedal_standby が False なら掛けない。
        # 2026-10-08 段2l-6: 待機位置は min（ペダルに触れた位置）そのもの
        if self._config.pedal_standby:
            ff = self._profile.feedforward_params
            accel_opening = max(accel_opening, ff.accel_deadband_pct)
            brake_opening = max(brake_opening, ff.brake_deadband_pct)

        calib = self._profile.calibration
        if calib is None:
            await self._abort_emergency("車両プロファイルにキャリブレーションがありません")
            return

        accel_pos = opening_to_position(accel_opening, calib.accel_zero_pos, calib.accel_full_pos)
        brake_pos = opening_to_position(brake_opening, calib.brake_zero_pos, calib.brake_full_pos)

        if not self._running:
            return

        try:
            monitor_accel, monitor_brake = await self._drive_or_sense(accel_pos, brake_pos)
        except Exception as exc:
            await self._abort_emergency(
                f"アクチュエータ通信に失敗しました（{type(exc).__name__}: {exc}）"
            )
            return

        cycle_ms = 1000.0 * (time.perf_counter() - cycle_started)
        accel_current, brake_current = monitor_accel.current_ma, monitor_brake.current_ma
        # 段7d: 上限 G の予測マップ用に、指令ではなく実際の開度を積む（低速の踏み込みは遅れる）
        self._accel_actual_pct = position_to_opening(
            monitor_accel.position_pulse, calib.accel_zero_pos, calib.accel_full_pos
        )
        self._brake_actual_pct = position_to_opening(
            monitor_brake.position_pulse, calib.brake_zero_pos, calib.brake_full_pos
        )
        t = now - self._started_at
        for label, current in (("アクセル", accel_current), ("ブレーキ", brake_current)):
            if current > self._overcurrent_limit_ma:
                await self._abort_emergency(
                    f"{label}軸が過電流です"
                    f"（{current:.0f} mA > {self._overcurrent_limit_ma:.0f} mA、t={t:.1f}s）"
                )
                return
        alarm_accel, alarm_brake = await self._safety.poll_alarms(
            self._cycles, self._accel_driver, self._brake_driver
        )
        self._cycles += 1
        reason = self._safety.check(
            t=t, accel_pos=accel_pos, brake_pos=brake_pos,
            accel_current=accel_current, brake_current=brake_current,
            alarm_accel=alarm_accel, alarm_brake=alarm_brake,
        )
        if reason is not None:
            await self._abort_emergency(reason)
            return

        self._on_sample(
            DriveLogData(
                ref_speed_kmh=None,
                actual_speed_kmh=speed,
                accel_opening=accel_opening,
                brake_opening=brake_opening,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=accel_current,
                brake_current=brake_current,
            ),
            self._pattern_idx,
            self._phase.name,
            self._governor_limiting(),
            alarm_accel,
            alarm_brake,
            monitor_accel,
            monitor_brake,
            cycle_ms,
        )

        self._advance(pattern, speed, accel_kmhs, now)
        if self._pending_abort is not None:
            await self._abort_emergency(self._pending_abort)
            return

        self._prev_speed = speed
        self._prev_time = now

    # ── 状態機械 ───────────────────────────────────────────────────
    def _initial_phase(self, idx: int) -> _Phase:
        if idx >= len(self._patterns):
            return _Phase.DONE
        if isinstance(self._patterns[idx], CreepLaunchPattern):
            return _Phase.CREEP_LAUNCH
        if isinstance(self._patterns[idx], GRampPattern):
            return _Phase.RAMP_UP  # 停車保持から。発進の段階でブレーキを離し、min で u の制御へ
        if self._patterns[idx].kind is PatternKind.COAST_DOWN:
            return _Phase.DRIVE_ACCEL
        return _Phase.MEASURE

    def _command_openings(self, pattern: LearningPattern, now: float) -> tuple[float, float]:
        assert self._phase_started_at is not None
        elapsed = max(0.0, now - self._phase_started_at)
        if self._phase in (_Phase.RAMP_UP, _Phase.RAMP_DOWN):
            return self._ramp_command()
        if self._phase is _Phase.DRIVE_ACCEL:
            ramped = self._coast_accel_opening(pattern, now)
            self._accel_request = ramped
            if self._accel_gov_cap is not None:
                ramped = min(ramped, self._accel_gov_cap)
            return clamp_opening(ramped, self._profile.max_accel_opening), 0.0
        if self._phase is _Phase.COAST:
            return 0.0, 0.0
        if self._phase is _Phase.CREEP_LAUNCH:
            return 0.0, 0.0  # 両ペダル完全解放（クリープのみで自走）
        if self._phase is _Phase.DRIVE_BRAKE and self._db_gprop:
            opening = self._calib_brake_opening(now)
            self._brake_request = opening
            if self._brake_gov_cap is not None:
                opening = min(opening, self._brake_gov_cap)
            return 0.0, clamp_opening(opening, self._profile.max_brake_opening)
        if self._phase is _Phase.DRIVE_BRAKE and self._db_creep_press:
            # 段2l-6b: 今の開度から停車保持開度まで一定の速さで踏み上げる（下げない）
            rate = self._stop_return_press_rate_pctps()
            assert rate is not None
            target_brake = self._stop_return_brake_pct
            ramped = min(target_brake, self._brake_ramp_from + rate * elapsed)
            ramped = max(ramped, min(self._brake_ramp_from, target_brake))
            self._brake_request = ramped
            if self._brake_gov_cap is not None:
                ramped = min(ramped, self._brake_gov_cap)
            return 0.0, clamp_opening(ramped, self._profile.max_brake_opening)
        if self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD):
            target_brake = pattern.brake_opening
            if self._phase is _Phase.DRIVE_BRAKE:
                target_brake = self._stop_return_brake_pct  # 停車復帰（A6）
                if self._overspeed_recovery:
                    target_brake = max(target_brake, self._config.overspeed_recovery_brake_pct)
            start = self._brake_ramp_from
            frac = self._ramp_fraction(elapsed, self._config.brake_ramp_time_s)
            ramped = start + (target_brake - start) * frac
            self._brake_request = ramped
            if self._brake_gov_cap is not None:
                ramped = min(ramped, self._brake_gov_cap)
            return 0.0, clamp_opening(ramped, self._profile.max_brake_opening)
        return pattern.accel_opening, pattern.brake_opening

    @staticmethod
    def _ramp_fraction(elapsed: float, ramp_time_s: float) -> float:
        if ramp_time_s <= 0.0:
            return 1.0
        return min(1.0, max(0.0, elapsed) / ramp_time_s)

    def _advance(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, now: float
    ) -> None:
        assert self._phase_started_at is not None
        elapsed = now - self._phase_started_at

        if self._phase is _Phase.DRIVE_ACCEL:
            self._advance_drive_accel(pattern, speed, accel_kmhs, elapsed, now)
            return
        if self._phase is _Phase.COAST:
            self._advance_coast(speed, accel_kmhs, elapsed, now)
            return
        if self._phase is _Phase.CREEP_LAUNCH:
            self._advance_creep_launch(pattern, speed, accel_kmhs, elapsed, now)
            return
        if self._phase in (_Phase.RAMP_UP, _Phase.RAMP_DOWN):
            assert isinstance(pattern, GRampPattern)
            self._advance_ramp(pattern, speed, elapsed, now)
            return
        if self._phase is _Phase.DRIVE_BRAKE:
            if self._db_gprop:
                self._step_gprop_brake(speed, now)
                if speed <= self._stop_return_low_speed_kmh():
                    # 低速まで下りた。あとは停車保持開度へ（今の開度から。コーストダウンは
                    # 徐々に踏む、それ以外は従来どおり brake_ramp_time_s のランプ）
                    self._db_gprop = False
                    self._db_creep_press = self._creep_press_applies()
                    self._brake_ramp_from = self._current_brake_opening
                    self._phase_started_at = now
                    elapsed = 0.0
            if self._db_creep_press:
                # 段2l-6b: 停車かつ停車保持開度に届いてから次へ。時間切れも届いてから数える
                # （踏み上げに 30 秒以上かかるため、届く前には中断しない）
                if (
                    self._db_press_reached_at is None
                    and self._brake_request >= self._stop_return_brake_pct - 1e-9
                ):
                    self._db_press_reached_at = now
                if self._db_press_reached_at is None:
                    return
                if speed <= STOP_SPEED_KMH:
                    self._advance_pattern(now)
                    return
                elapsed = now - self._db_press_reached_at
            if speed <= STOP_SPEED_KMH:
                self._advance_pattern(now)
            elif elapsed >= self._config.brake_stop_timeout_s:
                self._pending_abort = (
                    f"停車復帰（DRIVE_BRAKE）で brake_stop_timeout_s="
                    f"{self._config.brake_stop_timeout_s:g}s 以内に停車しません"
                    f"（パターン {self._pattern_idx + 1}:{pattern.kind.name}、"
                    f"車速 {speed:.1f} km/h、"
                    f"ブレーキ {self._current_brake_opening:.2f}%）"
                )
            return
        if self._phase is _Phase.BRAKE_HOLD:
            if speed <= STOP_SPEED_KMH:
                self._advance_pattern(now)
            elif elapsed >= self._config.brake_hold_timeout_s:
                self._finish_pattern(speed, now)
            return
        if self._phase is _Phase.MEASURE and pattern.kind is PatternKind.CREEP_SETTLE:
            self._advance_creep_settle(elapsed, accel_kmhs, now)
            return
        if self._phase is _Phase.MEASURE:
            if elapsed >= pattern.hold_duration_s:
                self._advance_pattern(now)
            return

    def _coast_accel_opening(self, pattern: LearningPattern, now: float) -> float:
        """コーストダウン加速の指令開度（ガバナー適用前）。

        最初は 0 から「min − approach_margin」まで accel_ramp_time_s で上げる（遊びの中では
        加速が出ない）。以降の開度は `_step_coast_accel` が G との差に応じて動かす。
        """
        cfg = self._config
        if self._ca_t0 is None:
            ff = self._profile.feedforward_params
            self._ca_approach_pct = min(
                pattern.accel_opening,
                max(0.0, ff.accel_deadband_pct - cfg.coast_accel_approach_margin_pct),
            )
            self._ca_t0 = now
        elapsed = now - self._ca_t0
        if elapsed < cfg.accel_ramp_time_s:
            return self._ca_approach_pct * self._ramp_fraction(elapsed, cfg.accel_ramp_time_s)
        if self._ca_last_t is None:  # 接近が済んだ。ここから G 比例で踏み進める
            self._ca_opening = self._ca_approach_pct
            self._ca_last_t = now
        return self._ca_opening

    def _step_coast_accel(self, pattern: LearningPattern, speed: float, now: float) -> None:
        """毎周期、直近 slope_window_s の加速度 G から開度を動かす。

        踏む速さ [%/s] = rate_gain ×（目標 G − 今の G）。目標 ± press_margin_g の間は保持。
        範囲は [接近位置, pattern.accel_opening]（G が高ければ同じ式で戻る）。
        """
        cfg = self._config
        hist = self._ca_hist
        hist.append((now, speed))
        while hist and now - hist[0][0] > cfg.coast_accel_slope_window_s:
            hist.popleft()
        # 段7d: 実開度の履歴（_ca_lagged_opening が遅れを見込んだ窓の代表開度を作るのに使う）。
        # モニタが無い経路（_advance 直叩きのテスト等）は _accel_actual_pct が None のまま
        # なので、この phase の指令値 _ca_opening で代用する
        actual = self._accel_actual_pct if self._accel_actual_pct is not None else self._ca_opening
        self._ca_opening_hist.append((now, actual))
        max_age = cfg.coast_accel_slope_window_s + cfg.gov_lead_s + 0.5
        while self._ca_opening_hist and now - self._ca_opening_hist[0][0] > max_age:
            self._ca_opening_hist.popleft()
        if self._ca_last_t is None:
            return  # 接近ランプ中
        dt = now - self._ca_last_t
        self._ca_last_t = now
        accel_g = fit_slope([t for t, _ in hist], [v for _, v in hist]) / G_TO_KMHS
        margin = cfg.coast_accel_target_g - accel_g
        # 記録は 1 s に 1 点。開度が動いていても（40〜110 km/h は G が目標に届かず開度が
        # 上がり続け、以前の「止まっている間しか記録しない」条件では 1 点も取れなかった）、
        # 遅れ gov_lead_s（＝指令から G の山までの遅れ。ガバナーの先読みと同じ値）ぶん過去へ
        # ずらした窓の実開度の平均を、今の G と対にして記録する（段7d）。振れの条件は無い
        # （実開度で記録するので、指令が動いていても対応関係は崩れない）
        lagged = self._ca_lagged_opening(now)
        if lagged is not None and now - self._ca_record_t >= 1.0:
            self._ca_record_t = now
            mean_v = sum(v for _, v in hist) / len(hist)
            self.glimit.add(PEDAL_ACCEL, mean_v, lagged, accel_g * G_TO_KMHS)
        if abs(margin) <= cfg.coast_accel_press_margin_g:
            return
        opening = self._ca_opening + cfg.coast_accel_rate_gain * margin * dt
        opening = min(pattern.accel_opening, max(self._ca_approach_pct, opening))
        self._ca_opening = opening

    def _ca_lagged_opening(self, now: float) -> float | None:
        """遅れ gov_lead_s を見込んだ窓 `[now − gov_lead_s − slope_window_s, now − gov_lead_s]`

        の実開度の平均（段7d。窓に 2 点未満なら None）。今の G はこの窓の開度が作ったもの、
        という遅れの見込み（`gov_lead_s` はブレーキ指令から G の山までの遅れの実測値の流用）。
        """
        cfg = self._config
        lo = now - cfg.gov_lead_s - cfg.coast_accel_slope_window_s
        hi = now - cfg.gov_lead_s
        openings = [o for t, o in self._ca_opening_hist if lo <= t <= hi]
        if len(openings) < 2:
            return None
        return sum(openings) / len(openings)

    def _advance_drive_accel(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, elapsed: float, now: float
    ) -> None:
        cfg = self._config
        self._step_coast_accel(pattern, speed, now)
        if speed > self._profile.max_speed:
            self._skip_count += 1
        else:
            self._skip_count = 0
        if self._skip_count >= cfg.skip_consecutive_required:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return

        # コーストダウン: cap の手前（加速度 × 先読み）まで上げたら惰行へ。
        # accel_full_range_timeout_s は「加速に使う時間の上限」で、cap 未達でもここで惰行へ進む
        # （目標 G が出ないほどペダルが利かない場合に、加速だけで走り続けないための打ち切り）
        exit_speed = self._accel_speed_cap - max(0.0, accel_kmhs) * cfg.overspeed_lead_s
        if speed >= exit_speed or elapsed >= cfg.accel_full_range_timeout_s:
            self._enter_phase(_Phase.COAST, now)

    # ── G ランプ走行（段2k 段B）─────────────────────────────────────
    def _ramp_u_limits(self) -> tuple[float, float]:
        """効く開度 u の範囲 (下限＝ブレーキ側, 上限＝アクセル側)。

        2026-10-07 段2l-2: RAMP_UP は下限 0（ブレーキ側へ行かない。クリープ域で狙いより強く加速
        しても両ペダル 0 のままクリープに任せる）。RAMP_DOWN で一度ブレーキ側に入ったあと
        （`_rp_brake_latched`）は上限 0（アクセルへ戻らない。惰行の減速は車速が下がるほど弱く
        なるので、一度ブレーキが要る領域に入ったらその先でアクセルが要ることはない。ユーザー決定）。

        2026-10-07 段2l-3: ラッチ後の上限を待機位置に取り、u が 0 をまたぐたびにブレーキが
        0% ↔ 不感帯を飛ばないようにした（飛ぶとペダルの移動（約 0.7 秒）の間に G が変わらず u が
        積み上がって踏みすぎ、往復した。実機 151515）。
        2026-10-08 段2l-6: 待機位置が min そのものになったので、ラッチ後の上限は素直に 0
        （ブレーキ ＝ min ＋ |u|。下限 min）。
        """
        ff = self._profile.feedforward_params
        lo = -max(0.0, self._profile.max_brake_opening - ff.brake_deadband_pct)
        hi = max(0.0, self._profile.max_accel_opening - ff.accel_deadband_pct)
        if self._phase is _Phase.RAMP_UP:
            lo = 0.0
        elif self._phase is _Phase.RAMP_DOWN and self._rp_brake_latched:
            hi = 0.0
        return lo, hi

    def _ramp_on_brake(self) -> bool:
        """G ランプの u がブレーキ側を指すか（RAMP_DOWN のラッチ後、または u ＜ 0）。"""
        if self._phase is _Phase.RAMP_DOWN and self._rp_brake_latched:
            return True
        return self._rp_u is not None and self._rp_u < 0.0

    def _ramp_estimate_u(self, speed: float, target_g: float) -> float | None:
        """`speed` で `target_g`（符号つき。加速 +・減速 −）を出すと予測される u。無ければ None。

        実測の（車速・開度・加速度）を持つ `self.glimit` から引く。減速で狙いが惰行より緩い
        （その車速帯の惰行の減速の中央値より小さい）ときはアクセルを少し踏んだ側（u ＞ 0）、
        それ以外の減速はブレーキ側（u ＜ 0）の予測を使う。

        2026-10-08 段2l-6: 狙いが貯めた点のどれより強いときは割線の外挿をせず、いちばん効いた
        点の開度を返す（`extrapolate=False`）。補間は変わらない。
        """
        ff = self._profile.feedforward_params
        target_kmhs = target_g * G_TO_KMHS
        if target_kmhs >= 0.0:
            opening = self._glimit_open_for(PEDAL_ACCEL, speed, target_kmhs, extrapolate=False)
            return None if opening is None else opening - ff.accel_deadband_pct
        coast = self.glimit.coast_decel_kmhs(speed)
        if coast is not None and -target_kmhs < coast:
            # 惰行より緩い減速: アクセル側の負の加速度として引く（惰行の点も使う）
            opening = self._glimit_open_for(PEDAL_ACCEL, speed, target_kmhs, extrapolate=False)
            return None if opening is None else opening - ff.accel_deadband_pct
        opening = self._glimit_open_for(PEDAL_BRAKE, speed, -target_kmhs, extrapolate=False)
        return None if opening is None else -(opening - ff.brake_deadband_pct)

    def _ramp_command(self) -> tuple[float, float]:
        """G ランプの指令開度 (アクセル, ブレーキ)。u ＞ 0 はアクセル ＝ min ＋ u、
        u ＜ 0 はブレーキ ＝ min ＋ |u|、u ＝ 0 は両方 0（待機位置の下限は出力側で掛ける。
        `fwd_data.effective_opening` の逆）。

        2026-10-07 段2l-3: RAMP_DOWN のラッチ後（ブレーキの領域）は u の符号に関係なく
        アクセル ＝ 0、ブレーキ ＝ min − u（u は 0 以下なのでブレーキ ≧ min。0% へ戻さない）。
        0% ↔ 不感帯の飛びでペダルの移動 約 0.7 秒の間に踏みすぎて往復した（実機 151515）。

        2026-10-08 段2l-6: RAMP_UP の発進の段階（`_rp_launch`）は u を使わず、ブレーキ指令
        `_rp_launch_brake_pct`（止まった開度から min へ単調に下がる）だけを出す。アクセルは 0
        （待機位置の下限で min）。
        """
        ff = self._profile.feedforward_params
        if self._rp_launch:
            accel = 0.0
            brake = self._rp_launch_brake_pct
        else:
            if self._rp_u is None:  # RAMP_UP の発進の後: 実測から見積もる。無ければ u ＝ 0 から
                self._rp_u = self._ramp_initial_u(self._last_speed)
            u = self._rp_u
            if self._phase is _Phase.RAMP_DOWN and self._rp_brake_latched:
                accel = 0.0
                brake = ff.brake_deadband_pct - u
            else:
                accel = ff.accel_deadband_pct + u if u > 0.0 else 0.0
                brake = ff.brake_deadband_pct - u if u < 0.0 else 0.0
        self._accel_request = accel
        self._brake_request = brake
        if accel <= 0.0:
            self._accel_gov_cap = None  # 使っていないペダルの古い頭打ちは持ち越さない
        if brake <= 0.0:
            self._brake_gov_cap = None
        if self._accel_gov_cap is not None:
            accel = min(accel, self._accel_gov_cap)
        if self._brake_gov_cap is not None:
            brake = min(brake, self._brake_gov_cap)
        return (
            clamp_opening(accel, self._profile.max_accel_opening),
            clamp_opening(brake, self._profile.max_brake_opening),
        )

    def _ramp_initial_u(self, speed: float) -> float:
        """今のフェーズの最初の u。実測から見積もり、範囲に収める（見積もれなければ 0）。"""
        est = self._ramp_estimate_u(speed, self._ramp_target_g())
        lo, hi = self._ramp_u_limits()
        return 0.0 if est is None else min(hi, max(lo, est))

    def _ramp_launch_begin(self) -> None:
        """RAMP_UP の発進の段階を始める。ブレーキを止まった開度（無ければ停車保持開度）に置く。

        今のブレーキ位置がそれより低いとき（前の G ランプの減速で、止まった開度に届く前に停車した
        など）は、踏み直さずに今の位置から始める。
        """
        self._rp_launch = True
        start = self._config.stop_confirm_pct
        if start is None:
            start = self._stop_return_brake_pct
        calib = self._profile.calibration
        if calib is not None:
            start = min(
                start,
                position_to_opening(
                    self._brake_pos_cmd, calib.brake_zero_pos, calib.brake_full_pos
                ),
            )
        floor = self._profile.feedforward_params.brake_deadband_pct
        self._rp_launch_brake_pct = max(floor, min(start, self._profile.max_brake_opening))

    def _ramp_target_g(self) -> float:
        """今のフェーズの狙いの G（符号つき）。"""
        pattern = self._patterns[self._pattern_idx]
        assert isinstance(pattern, GRampPattern)
        return pattern.target_g if self._phase is _Phase.RAMP_UP else -pattern.target_g

    def _ramp_lagged_openings(self, now: float) -> tuple[float, float] | None:
        """遅れ gov_lead_s を見込んだ窓の実開度の平均 (アクセル, ブレーキ)。2 点未満なら None。"""
        cfg = self._config
        lo = now - cfg.gov_lead_s - cfg.coast_accel_slope_window_s
        hi = now - cfg.gov_lead_s
        rows = [(a, b) for t, a, b in self._rp_open_hist if lo <= t <= hi]
        if len(rows) < 2:
            return None
        return sum(a for a, _ in rows) / len(rows), sum(b for _, b in rows) / len(rows)

    def _ramp_gain(
        self, speed: float, actual_a: float, actual_b: float, *, on_brake: bool | None = None
    ) -> float:
        """u を動かす速さのゲイン [%/s per G]。

        ゲイン ＝ min(ramp_rate_gain, 1 ÷（感度[G/%] × ramp_settle_time_s））。感度は上限 G の
        予測マップの実測（今使っているペダルの今の開度付近。`sensitivity_kmhs_per_pct`）。
        測れていない（None）か 0 以下ならゲインは ramp_rate_gain。ブレーキはアクセルより開度
        あたりの効きが数倍大きいので、同じゲインでは往復する（2026-10-07 段2l-2）。
        `on_brake` を渡すとブレーキ側・アクセル側をそれで決める（発進の段階は u が無いので True）。
        """
        cfg = self._config
        ff = self._profile.feedforward_params
        if self._ramp_on_brake() if on_brake is None else on_brake:
            sens = self.glimit.sensitivity_kmhs_per_pct(
                PEDAL_BRAKE, speed, actual_b, ff.brake_deadband_pct
            )
        else:
            sens = self.glimit.sensitivity_kmhs_per_pct(
                PEDAL_ACCEL, speed, actual_a, ff.accel_deadband_pct
            )
        gain = cfg.ramp_rate_gain
        if sens is not None and sens > 0.0:
            gain = min(gain, 1.0 / ((sens / G_TO_KMHS) * cfg.ramp_settle_time_s))
        return gain

    def _step_ramp(self, pattern: GRampPattern, speed: float, now: float) -> float | None:
        """毎周期、直近 slope_window_s の加速度 G から u を動かす。戻り値は実測 G（窓前は None）。

        u の動かす速さ [%/s] = ゲイン ×（狙いの G − 実測の G）。ゲインは `_ramp_gain`
        （実測の感度から決め、ramp_rate_gain を上限とする）。差が保持幅
        max(狙い × ramp_hold_margin_frac, ramp_hold_margin_min_g) 以内なら動かさない。実測は
        上限 G の予測マップにも足す（アクセル側・ブレーキ側それぞれ。`_step_coast_accel` と同じ）。
        """
        cfg = self._config
        hist = self._rp_hist
        hist.append((now, speed))
        while hist and now - hist[0][0] > cfg.coast_accel_slope_window_s:
            hist.popleft()
        actual_a = self._accel_actual_pct if self._accel_actual_pct is not None else (
            self._current_accel_opening
        )
        actual_b = self._brake_actual_pct if self._brake_actual_pct is not None else (
            self._current_brake_opening
        )
        self._rp_open_hist.append((now, actual_a, actual_b))
        max_age = cfg.coast_accel_slope_window_s + cfg.gov_lead_s + 0.5
        while self._rp_open_hist and now - self._rp_open_hist[0][0] > max_age:
            self._rp_open_hist.popleft()
        last_t = self._rp_last_t
        self._rp_last_t = now
        # 発進の段階は u が無い（min に着いて渡すとき見積もる）。それ以外は u が出来てから
        if last_t is None or (self._rp_u is None and not self._rp_launch) or len(hist) < 3:
            return None
        dt = now - last_t
        accel_kmhs = fit_slope([t for t, _ in hist], [v for _, v in hist])
        measured_g = accel_kmhs / G_TO_KMHS
        lagged = self._ramp_lagged_openings(now)
        if lagged is not None and now - self._rp_record_t >= 1.0:
            lag_a, lag_b = lagged
            mean_v = sum(v for _, v in hist) / len(hist)
            # 2026-10-07 段2l-4: 両ペダルが待機位置にいるので「0 超・0.05 以下」では判定できない。
            # 2026-10-08 段2l-6: 待機位置が min なので、自分が min ＋ 幅 より上・反対側が
            # min ＋ 幅 以下
            # （PEDAL_OFF_TOL_PCT。実位置のずれで行が割れないよう幅を持たせる）
            ffp = self._profile.feedforward_params
            a_floor = ffp.accel_deadband_pct + PEDAL_OFF_TOL_PCT
            b_floor = ffp.brake_deadband_pct + PEDAL_OFF_TOL_PCT
            if lag_a > a_floor and lag_b <= b_floor:
                self._rp_record_t = now
                self.glimit.add(PEDAL_ACCEL, mean_v, lag_a, accel_kmhs)
            elif lag_b > b_floor and lag_a <= a_floor:
                # 発進の段階は加速中でブレーキの減速は負になる。減速が出た点だけ貯める
                if not self._rp_launch or -accel_kmhs > 0.0:
                    self._rp_record_t = now
                    self.glimit.add(PEDAL_BRAKE, mean_v, lag_b, -accel_kmhs)
        target = self._ramp_target_g()
        err = target - measured_g
        margin = max(pattern.target_g * cfg.ramp_hold_margin_frac, cfg.ramp_hold_margin_min_g)
        if self._rp_launch:
            self._step_ramp_launch(speed, err, margin, dt, actual_a, actual_b)
            return measured_g
        assert self._rp_u is not None
        if abs(err) > margin:
            new_u = self._rp_u + self._ramp_gain(speed, actual_a, actual_b) * err * dt
            # ガバナーで削られている間は、削られているペダルをさらに踏み増す向きへ積まない
            # （ラッチ後は u が減る向きが踏み増し。それ以外は |u| が増える向き）
            if self._phase is _Phase.RAMP_DOWN and self._rp_brake_latched:
                pressing_more = new_u < self._rp_u
            else:
                pressing_more = abs(new_u) > abs(self._rp_u)
            if self._governor_limiting() and pressing_more:
                new_u = self._rp_u
            lo, hi = self._ramp_u_limits()
            self._rp_u = min(hi, max(lo, new_u))
            if (
                self._phase is _Phase.RAMP_DOWN
                and not self._rp_brake_latched
                and self._rp_u < 0.0
            ):
                self._rp_brake_latched = True
                # ラッチした瞬間は待機位置（min）から踏み始める（0% から min まで空走させない）
                self._rp_u = self._ramp_u_limits()[1]
        return measured_g

    def _step_ramp_launch(
        self, speed: float, err: float, margin: float, dt: float, actual_a: float, actual_b: float
    ) -> None:
        """RAMP_UP の発進の段階の毎周期の処理（2026-10-08 段2l-6）。

        実測 G が狙いより margin を超えて低いときだけ、ブレーキを `ramp_gain × 不足 × dt` 下げる。
        狙い以上ならその位置で止めて待つ。上げる方向へは動かさない（単調に下がり、踏み直さない）。
        ブレーキが min に着いたら発進の段階を終える（u の見積もりは次の指令で `_ramp_command` が
        行い、アクセルの u 制御（u ≥ 0）に渡す。以後その RAMP_UP ではブレーキに戻らない）。
        """
        floor = self._profile.feedforward_params.brake_deadband_pct
        if err > margin:
            gain = self._ramp_gain(speed, actual_a, actual_b, on_brake=True)
            self._rp_launch_brake_pct = max(floor, self._rp_launch_brake_pct - gain * err * dt)
        if self._rp_launch_brake_pct <= floor:
            self._rp_launch = False

    def _advance_ramp(
        self, pattern: GRampPattern, speed: float, elapsed: float, now: float
    ) -> None:
        """G ランプの前進判定。

        RAMP_UP: 車速が上端（`top_kmh`。最高速側の安全のため accel_speed_cap − 加速度 ×
        overspeed_lead_s でも頭打ち）に着いたら RAMP_DOWN へ。RAMP_DOWN: 車速が停車
        （`STOP_SPEED_KMH` 以下）になったら `_finish_pattern` へ渡す（2026-10-08 段2l-6:
        旧 coast_down_stop_speed_kmh（5 km/h）から変更。0 km/h まで測る）。
        どちらの区間も、見込み秒数 × RAMP_LEG_TIMEOUT_FACTOR ＋ RAMP_LEG_TIMEOUT_MARGIN_S で打ち切り
        （狙いの G に届かないときの安全網。RAMP_UP は RAMP_DOWN へ、RAMP_DOWN は停車復帰へ進む）。
        """
        cfg = self._config
        measured_g = self._step_ramp(pattern, speed, now)
        self._rp_max_speed = max(self._rp_max_speed, speed)
        if measured_g is not None and elapsed >= cfg.coast_accel_slope_window_s:
            (self._rp_up_g if self._phase is _Phase.RAMP_UP else self._rp_down_g).append(measured_g)
        if speed > self._profile.max_speed:
            self._skip_count += 1
        else:
            self._skip_count = 0
        if self._skip_count >= cfg.skip_consecutive_required:
            self._ramp_record_result(pattern, now)
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        leg_s = max(pattern.top_kmh, 0.0) / (max(pattern.target_g, 1e-6) * G_TO_KMHS)  # 0 km/h まで
        timeout_s = leg_s * RAMP_LEG_TIMEOUT_FACTOR + RAMP_LEG_TIMEOUT_MARGIN_S
        if self._phase is _Phase.RAMP_UP:
            hist = self._rp_hist
            accel_kmhs = (
                fit_slope([t for t, _ in hist], [v for _, v in hist]) if len(hist) >= 3 else 0.0
            )
            exit_speed = min(
                pattern.top_kmh, self._accel_speed_cap - max(0.0, accel_kmhs) * cfg.overspeed_lead_s
            )
            if speed >= exit_speed or elapsed >= timeout_s:
                self._ramp_switch_down(speed, now)
            return
        if speed <= STOP_SPEED_KMH or elapsed >= timeout_s:
            self._ramp_record_result(pattern, now)
            self._finish_pattern(speed, now)

    def _ramp_switch_down(self, speed: float, now: float) -> None:
        """RAMP_UP → RAMP_DOWN。最初の u は実測から見積もる（無ければ今の u から続ける）。"""
        self._rp_down_started = now
        self._enter_phase(_Phase.RAMP_DOWN, now)  # u は引き継ぐ（RAMP_UP のときだけ作り直す）
        self._rp_brake_latched = False  # 見積もりの前に下ろす（上限 0 で見積もりを潰さない）
        est = self._ramp_estimate_u(speed, self._ramp_target_g())
        if est is not None:
            lo, hi = self._ramp_u_limits()
            self._rp_u = min(hi, max(lo, est))
            if est < 0.0:
                self._rp_brake_latched = True

    def _ramp_record_result(self, pattern: GRampPattern, now: float) -> None:
        """この G ランプの結果（所要秒数・実測 G の中央値・最高車速）を記録する。"""
        if self._phase is _Phase.RAMP_UP:
            up_s, down_s = now - self._rp_up_started, 0.0
        else:
            up_s = self._rp_down_started - self._rp_up_started
            down_s = now - self._rp_down_started
        self.ramp_results.append(
            RampResult(
                pattern_idx=self._pattern_idx,
                target_g=pattern.target_g,
                top_kmh=pattern.top_kmh,
                up_s=up_s,
                down_s=down_s,
                accel_g_median=statistics.median(self._rp_up_g) if self._rp_up_g else None,
                decel_g_median=-statistics.median(self._rp_down_g) if self._rp_down_g else None,
                max_speed_kmh=self._rp_max_speed,
            )
        )

    def _calib_brake_opening(self, now: float) -> float:
        """高い車速からの停車復帰の G 比例ブレーキ開度（ガバナー適用前）。

        最初は 0 から「min − approach_margin」まで brake_ramp_time_s で上げる（遊びの中では
        減速が出ない）。以降の開度は `_step_gprop_brake` が減速 G との差に応じて刻む。
        上限は停車保持開度（停車復帰で毎回踏んでいる実績のある開度）。
        """
        cfg = self._config
        if self._cb_t0 is None:
            ff = self._profile.feedforward_params
            self._cb_approach_pct = min(
                self._stop_return_brake_pct,
                max(0.0, ff.brake_deadband_pct - cfg.coast_accel_approach_margin_pct),
            )
            # コーストダウン・G ランプの実測があれば、その車速で目標 G が出る開度から始める
            # （無ければ min の手前から刻んで上げる）
            self._cb_start_pct = self._cb_approach_pct
            predicted = self._glimit_open_for(
                PEDAL_BRAKE, self._last_speed, cfg.coast_accel_target_g * G_TO_KMHS
            )
            if predicted is not None:
                self._cb_start_pct = min(
                    self._stop_return_brake_pct, max(self._cb_approach_pct, predicted)
                )
            self._cb_t0 = now
        elapsed = now - self._cb_t0
        if elapsed < cfg.brake_ramp_time_s:
            return self._cb_start_pct * self._ramp_fraction(elapsed, cfg.brake_ramp_time_s)
        if self._cb_next_t is None:  # 接近が済んだ。ここから G の刻みで踏み進める
            self._cb_opening = self._cb_start_pct
            self._cb_next_t = now + cfg.calib_dwell_s
        return self._cb_opening

    def _step_gprop_brake(self, speed: float, now: float) -> None:
        """G 比例ブレーキ減速の毎周期の処理（高い車速からの停車復帰）。

        dwell_s ごとに直近 slope_window_s の減速 G から開度を 1 刻み動かし、そのときの
        (開度, 減速) を上限 G の予測マップに足す。

        刻みは 0.5mm（約 0.5%）で、1 刻みの G の変化は 0.003〜0.007G と小さい。指令から G が
        出るまでの遅れがあっても、毎回の点は準定常とみなせる。
        """
        cfg = self._config
        hist = self._cb_hist
        hist.append((now, speed))
        while hist and now - hist[0][0] > cfg.coast_accel_slope_window_s:
            hist.popleft()
        if self._cb_next_t is None or now < self._cb_next_t or len(hist) < 3:
            return
        self._cb_next_t = now + cfg.calib_dwell_s
        decel_kmhs = -fit_slope([t for t, _ in hist], [v for _, v in hist])
        mean_v = sum(v for _, v in hist) / len(hist)
        # 記録するのは実際に出ていた開度（ガバナーで削られていればその値）
        self.glimit.add(PEDAL_BRAKE, mean_v, self._current_brake_opening, decel_kmhs)
        step_pct = cfg.calib_step_mm * 100.0 * 100.0 / ACTUATOR_PULSE_MAX
        press_below = (cfg.coast_accel_target_g - cfg.coast_accel_press_margin_g) * G_TO_KMHS
        if decel_kmhs > cfg.calib_release_above_g * G_TO_KMHS:
            self._cb_opening = max(self._cb_approach_pct, self._cb_opening - step_pct)
        elif decel_kmhs < press_below:
            self._cb_opening = min(self._stop_return_brake_pct, self._cb_opening + step_pct)

    def _glimit_open_for(
        self, pedal: str, speed_kmh: float, target_kmhs: float, *, extrapolate: bool = True
    ) -> float | None:
        """そのペダルが `target_kmhs`（正）を出すと予測される開度（測れていなければ None）。

        `extrapolate=False` は G ランプの最初の開度の見積もり用（狙いが貯めた点のどれより強い
        ときは、いちばん効いた点の開度を返す。2026-10-08 段2l-6）。
        """
        ff = self._profile.feedforward_params
        db = ff.brake_deadband_pct if pedal == PEDAL_BRAKE else ff.accel_deadband_pct
        return self.glimit.cap_pct(pedal, speed_kmh, target_kmhs, db, extrapolate=extrapolate)

    def _advance_coast(self, speed: float, accel_kmhs: float, elapsed: float, now: float) -> None:
        """コーストダウンの惰行の前進判定（段7c: 終了をクリープ平衡車速から決める）。

        以前は固定 `coast_down_stop_speed_kmh`（5.0）で終えていたが、クリープ平衡がそれより
        高い車両では惰行だけで届かず `coast_timeout_s`（90s）まで待ち、低い車両では惰行カーブの
        低速端のデータを取りこぼしていた。惰行は平衡へ漸近するだけでちょうど届く保証がないため、
        平衡以下に達するか、クリープの影響が残る車速（平衡 + `grid_settle_tol_kmh`）以下で
        車速の傾きが落ち着いたら終える（`_coast_creep_settled`）。`coast_down_stop_speed_kmh` は
        他の使い道（G ランプの減速の終了・G ランプなどの停車復帰の切替・超過回復）のまま残す。
        コーストダウンの停車復帰の境目は車ごと（`_stop_return_low_speed_kmh`。2026-10-08
        段2l-6b）で、クリープ域は停車保持開度まで徐々に踏む。
        """
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        if elapsed >= cfg.coast_measure_delay_s and accel_kmhs < 0.0:
            self.glimit.add_coast(speed, -accel_kmhs)  # 上限 G の予測の基準（開度=min の点）
        if elapsed >= cfg.coast_timeout_s:
            self._finish_pattern(speed, now)
            return
        creep = self._profile.feedforward_params.creep_speed_kmh
        if speed <= creep:
            self._finish_pattern(speed, now)
            return
        ceiling = self._creep_ceiling_kmh(cfg.grid_settle_tol_kmh)
        if speed <= ceiling and self._coast_creep_settled(now):
            self._finish_pattern(speed, now)

    def _coast_creep_settled(self, now: float) -> bool:
        """コーストダウンの惰行が、クリープ平衡近くで車速の傾きが落ち着いたか（段7c）。

        クリープ発進の平衡到達と同じ調整値（`creep_launch_settle_*`）で、毎周期の `_speed_hist`
        の傾きを見る。
        """
        cfg = self._config
        window_s = cfg.creep_launch_settle_s
        recent = [(t, v) for t, v in self._speed_hist if t >= now - window_s]
        if len(recent) < 3 or self._speed_hist[0][0] > now - window_s:
            return False
        slope = fit_slope([t for t, _ in recent], [v for _, v in recent])
        return abs(slope) < cfg.creep_launch_settle_kmhs

    def _advance_creep_launch(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, elapsed: float, now: float
    ) -> None:
        """クリープ発進・クリープ域ブレーキ保持（段1b）の前進判定。

        2026-09-17（ProblemReport_20260916 ユーザー決定）: 終了条件を「target_kmh 到達」から
        「平衡到達（加速が止まった）」に変えた。車速の傾きが creep_launch_settle_kmhs 未満の
        状態が creep_launch_settle_s 続いたら平衡到達とみなす（_advance_creep_settle と同じ
        流儀）。target_kmh は安全上限として残す（クリープでこれ以上の車速にはならないはずなので、
        達したら平衡を待たず打ち切る）。timeout_s は従来どおりの打ち切り。

        2026-09-17（誤判定修正）: 停車保持を解放した直後は車速 0・傾き 0 で
        abs(accel_kmhs) < creep_launch_settle_kmhs が最初から成立してしまい、車が動き出す前に
        「平衡到達」と誤判定して即終了していた（実機では解放から動き出しまで約 0.3〜0.4s、その
        間の傾きは 0.17 km/h/s 程度でしきい値 0.1 に近く、確実に誤判定する）。
        _advance_creep_settle の settled 判定（creep_settle_min_s による最短時間ガード）に
        揃えて、2 つのガードを足す:
          - 車速ゲート: speed が creep_launch_min_speed_kmh 以上になるまで安定カウントを
            積まない（動き出す前を「平衡」に含めない、本質的なガード）。
          - 最短時間ガード: elapsed が creep_launch_settle_min_s 以上になるまで終了しない。
        到達後は hold_after=True なら BRAKE_HOLD（brake_opening を保持して停車）、
        False なら通常の停車復帰（_finish_pattern）へ進む。
        """
        assert isinstance(pattern, CreepLaunchPattern)
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        moving = speed >= cfg.creep_launch_min_speed_kmh
        if moving and abs(accel_kmhs) < cfg.creep_launch_settle_kmhs:
            self._stable_count += 1
        else:
            self._stable_count = 0
        stable_long = self._stable_count * self._interval_s >= cfg.creep_launch_settle_s
        settled = elapsed >= cfg.creep_launch_settle_min_s and stable_long
        if settled or speed >= pattern.target_kmh or elapsed >= pattern.timeout_s:
            self._stable_count = 0
            if pattern.hold_after:
                self._enter_phase(_Phase.BRAKE_HOLD, now)
            else:
                self._finish_pattern(speed, now)

    def _advance_creep_settle(self, elapsed: float, accel_kmhs: float, now: float) -> None:
        cfg = self._config
        if abs(accel_kmhs) < cfg.creep_settle_stable_tol_kmhs:
            self._stable_count += 1
        else:
            self._stable_count = 0
        stable_long = self._stable_count * self._interval_s >= cfg.creep_settle_stable_duration_s
        settled = elapsed >= cfg.creep_settle_min_s and stable_long
        if settled or elapsed >= cfg.creep_settle_timeout_s:
            self._stable_count = 0
            self._advance_pattern(now)

    def _creep_press_applies(self) -> bool:
        """停車復帰を「徐々に踏む」にするか（段2l-6b）。コーストダウンで、超過回復でなく、
        踏む速さが決まる（calib_dwell_s > 0）とき。"""
        return (
            self._pattern_idx < len(self._patterns)
            and self._patterns[self._pattern_idx].kind is PatternKind.COAST_DOWN
            and not self._overspeed_recovery
            and self._stop_return_press_rate_pctps() is not None
        )

    def _stop_return_low_speed_kmh(self) -> float:
        """停車復帰で G 比例ブレーキからクリープ域の踏み方へ切り替える車速（段2l-6b）。

        コーストダウンはクリープ車速 + 落ち着きの許容幅（車ごと）。それ以外・超過回復は固定の
        `coast_down_stop_speed_kmh`。
        """
        cfg = self._config
        if (
            self._pattern_idx < len(self._patterns)
            and self._patterns[self._pattern_idx].kind is PatternKind.COAST_DOWN
            and not self._overspeed_recovery
        ):
            return self._creep_ceiling_kmh(cfg.grid_settle_tol_kmh)
        return cfg.coast_down_stop_speed_kmh

    def _stop_return_press_rate_pctps(self) -> float | None:
        """クリープ域で停車保持開度へ踏み上げる速さ [%/s]。`_step_gprop_brake` の 1 刻み/待ちと
        同じ。calib_dwell_s が 0 以下なら None（従来のランプに落とす）。"""
        cfg = self._config
        if cfg.calib_dwell_s <= 0.0:
            return None
        return cfg.calib_step_mm * 100.0 * 100.0 / ACTUATOR_PULSE_MAX / cfg.calib_dwell_s

    def _creep_ceiling_kmh(self, settle_tol_kmh: float) -> float:
        """クリープの影響が残る車速の上端 = クリープ平衡速度 + 落ち着きの許容幅。

        コーストダウンの終了判定（`_advance_coast`）が使う。
        """
        return self._profile.feedforward_params.creep_speed_kmh + settle_tol_kmh

    def _finish_pattern(self, speed: float, now: float) -> None:
        """運転パターンの計測が終わった。停車していなければ停車復帰（DRIVE_BRAKE）に入る（A6）。"""
        if speed > STOP_SPEED_KMH:
            self._enter_phase(_Phase.DRIVE_BRAKE, now)
        else:
            self._advance_pattern(now)

    def _advance_pattern(self, now: float) -> None:
        self._pattern_idx += 1
        self._enter_phase(self._initial_phase(self._pattern_idx), now)

    def _enter_phase(
        self, phase: _Phase, now: float, *, overspeed_recovery: bool = False
    ) -> None:
        self._phase = phase
        self._phase_started_at = now
        self._skip_count = 0
        self._stable_count = 0
        self._accel_gov_cap = None
        self._brake_gov_cap = None
        self._ca_t0 = None
        self._ca_last_t = None
        self._ca_hist.clear()
        self._ca_opening_hist.clear()  # 段7d: 前のパターンの開度を次のパターンに持ち込まない
        self._cb_t0 = None
        self._cb_next_t = None
        self._cb_hist.clear()
        # 停車復帰は、低速（クリープ域）からでなければ G 比例で減速する（段6b）。一気に停車保持
        # 開度へ踏むと、高い車速ではブレーキがよく効いて上限 G を超える（20260926: 134 km/h で
        # 0.30〜0.35G）
        self._overspeed_recovery = overspeed_recovery  # 下の停車復帰の判定が見る
        self._db_gprop = (
            phase is _Phase.DRIVE_BRAKE and self._last_speed > self._stop_return_low_speed_kmh()
        )
        # コーストダウンのクリープ域からの停車復帰は徐々に踏む（段2l-6b）
        self._db_creep_press = (
            phase is _Phase.DRIVE_BRAKE and not self._db_gprop and self._creep_press_applies()
        )
        self._db_press_reached_at = None
        self._rp_hist.clear()
        self._rp_open_hist.clear()
        self._rp_last_t = None
        self._rp_record_t = -1e9
        self._rp_launch = False  # 発進の段階は RAMP_UP の頭だけ（下で始める）
        if phase is _Phase.RAMP_UP:  # G ランプ 1 本の始まり: 状態を作り直す
            self._rp_u = None
            self._rp_brake_latched = False
            self._ramp_launch_begin()  # 2026-10-08 段2l-6: 発進の段階から始める
            self._rp_up_started = now
            self._rp_down_started = now
            self._rp_max_speed = 0.0
            self._rp_up_g = []
            self._rp_down_g = []
        self._brake_ramp_from = self._current_brake_opening

    def _update_governor(self, accel_kmhs: float) -> None:
        """G ガバナー（段6b で先読みつきに）。加速（DRIVE_ACCEL）・ブレーキ（DRIVE_BRAKE・
        BRAKE_HOLD）・G ランプ（RAMP_UP・RAMP_DOWN）に適用する。
        """
        if self._g_limit_kmhs <= 0.0:
            return
        rate = self._accel_rate
        if self._phase in (_Phase.DRIVE_ACCEL,):
            self._accel_gov_cap = self._next_gov_cap(
                self._accel_gov_cap, accel_kmhs, rate, self._current_accel_opening,
                self._accel_request,
            )
        elif self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD):
            self._brake_gov_cap = self._next_gov_cap(
                self._brake_gov_cap, -accel_kmhs, -rate, self._current_brake_opening,
                self._brake_request,
            )
        elif self._phase in (_Phase.RAMP_UP, _Phase.RAMP_DOWN):
            # G ランプは今使っているペダルだけ頭打ちする
            accel_on, brake_on = self._accel_request > 0.0, self._brake_request > 0.0
            if accel_on:
                self._accel_gov_cap = self._next_gov_cap(
                    self._accel_gov_cap, accel_kmhs, rate, self._current_accel_opening,
                    self._accel_request,
                )
            if brake_on:
                self._brake_gov_cap = self._next_gov_cap(
                    self._brake_gov_cap, -accel_kmhs, -rate, self._current_brake_opening,
                    self._brake_request,
                )

    def _next_gov_cap(
        self, cap: float | None, pedal_accel_kmhs: float, pedal_rate: float, current: float,
        request: float,
    ) -> float | None:
        """頭打ちの次の値（先読みつき）。

        `pedal_accel_kmhs` はそのペダルが出す向きの加速度（ブレーキなら減速度）、`pedal_rate` はその
        増える速さ。見込み = 今 + 増える速さ × gov_lead_s（増えていないときは今のまま）。

        見込み or 今が上限以上 … 最初は現在開度で頭打ち、以降 1 周期ごとに
                                        gov_reduce_step_pct 下げる
        見込みが上限 × gov_soft_frac 以上 … 踏み増しを止める（現在開度で頭打ち。すでにあれば保持）
        今が上限 × gov_release_frac 未満（かつ見込みも soft 未満） … gov_raise_step_pct ずつ戻し、
                                        指令（request）に届いたら解除（A6）
        その間 … 保持
        """
        cfg = self._config
        limit = self._g_limit_kmhs
        predicted = pedal_accel_kmhs + max(0.0, pedal_rate) * cfg.gov_lead_s
        if predicted >= limit or pedal_accel_kmhs >= limit:
            if cap is None:
                return current
            return max(0.0, cap - cfg.gov_reduce_step_pct)
        if predicted >= limit * cfg.gov_soft_frac:
            return current if cap is None else cap
        if cap is None:
            return None
        if pedal_accel_kmhs < limit * cfg.gov_release_frac:
            raised = cap + cfg.gov_raise_step_pct
            return None if raised >= request else raised
        return cap

    def _governor_limiting(self) -> bool:
        """この周期の指令がガバナーで削られたか（CSV の governor_active 列）。"""
        if self._phase is _Phase.DRIVE_ACCEL and self._accel_gov_cap is not None:
            return self._accel_gov_cap < self._accel_request
        if self._phase in (_Phase.RAMP_UP, _Phase.RAMP_DOWN):
            accel_limited = (
                self._accel_gov_cap is not None and self._accel_gov_cap < self._accel_request
            )
            brake_limited = (
                self._brake_gov_cap is not None and self._brake_gov_cap < self._brake_request
            )
            return accel_limited or brake_limited
        braking = self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD)
        if braking and self._brake_gov_cap is not None:
            return self._brake_gov_cap < self._brake_request
        return False

    def _update_accel_rate(self, now: float, accel_kmhs: float) -> None:
        """加速度の変化の速さ（直近 gov_rate_window_s の最小二乗の傾き）。先読みガバナー用。"""
        hist = self._accel_hist
        hist.append((now, accel_kmhs))
        while hist and now - hist[0][0] > self._config.gov_rate_window_s:
            hist.popleft()
        self._accel_rate = (
            fit_slope([t for t, _ in hist], [a for _, a in hist]) if len(hist) >= 3 else 0.0
        )

    def _smoothed_accel(self, now: float) -> float:
        hist = self._speed_hist
        win = self._config.g_smoothing_window_s
        while len(hist) >= 2 and now - hist[0][0] > win:
            hist.popleft()
        if len(hist) >= 2:
            t0, v0 = hist[0]
            t1, v1 = hist[-1]
            if t1 - t0 > 0.0:
                return (v1 - v0) / (t1 - t0)
        return self._measured_accel()

    def _measured_accel(self) -> float:
        if self._prev_speed is None or self._prev_time is None:
            return 0.0
        cur = self._speed_hist[-1][1] if self._speed_hist else self._prev_speed
        dt = self._speed_hist[-1][0] - self._prev_time if self._speed_hist else 0.0
        if dt <= 0.0:
            return 0.0
        return (cur - self._prev_speed) / dt

    # ── 補助 ───────────────────────────────────────────────────────
    def _move_duration(self) -> float:
        if self._phase in (
            _Phase.DRIVE_ACCEL, _Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD,
            _Phase.RAMP_UP, _Phase.RAMP_DOWN,
        ):
            return self._config.pedal_step_time_s
        return self._config.pedal_release_time_s

    async def _abort_emergency(self, reason: str) -> None:
        self._running = False
        if self.abort_reason is None:
            self.abort_reason = reason
        await self._on_emergency()

    async def _drive_or_sense(
        self, accel_pos: int, brake_pos: int
    ) -> tuple[AxisMonitor, AxisMonitor]:
        """位置が変わった軸だけ指令し、両軸をまとめ読みする（A7。以前は電流だけ読んでいた）。"""
        dur = self._move_duration()
        a_changed = accel_pos != self._accel_pos_cmd
        b_changed = brake_pos != self._brake_pos_cmd

        async def accel_axis() -> AxisMonitor:
            if a_changed:
                return await self._drive_axis_timed(
                    self._accel_driver, accel_pos, self._accel_pos_cmd, dur
                )
            return await self._accel_driver.read_monitor()

        async def brake_axis() -> AxisMonitor:
            if b_changed:
                return await self._drive_axis_timed(
                    self._brake_driver, brake_pos, self._brake_pos_cmd, dur
                )
            return await self._brake_driver.read_monitor()

        a, b = await asyncio.gather(accel_axis(), brake_axis())
        if a_changed:
            self._accel_pos_cmd = accel_pos
        if b_changed:
            self._brake_pos_cmd = brake_pos
        return a, b

    async def _drive_axis_timed(
        self,
        driver: ActuatorDriverProtocol,
        target_pos: int,
        current_pos: int,
        duration_s: float,
    ) -> AxisMonitor:
        await driver.move_to_position_timed(target_pos, current_pos, duration_s)
        return await driver.read_monitor()


__all__ = [
    "PATTERN_LOOP_INTERVAL_S",
    "STOP_SPEED_KMH",
    "ActuatorDriverProtocol",
    "CANReaderProtocol",
    "OnSample",
    "PatternLoop",
    "CreepLaunchPattern",
    "GRampPattern",
    "RampResult",
    "PatternLoopConfig",
]
