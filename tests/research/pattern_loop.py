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

2026-09-25 格子ステップ走行（ProblemReport_20260925 段3）で足したところ:
    背景: 手順2 の開度は「不感帯 + 固定の%」で車両に合わせて手で決めた値だった。車両が変わっても
    WLTP の「車速 × 加速度」の格子を測れるように、開度を車両ごとに自動で決める新パターンを足す
    （既存パターンは変えない。手順2 への組み込み・旧パターンの削除は段4）。決め方・打ち切りは
    `grid_planner.py`、ここは走行の状態機械だけ持つ。
    - `GridStationPattern`（車速ステーション 1 つ。kind は GRID_STEP）: 専用の PI（`pi_hold_step`。
      P・I ゲインは感度 g_accel で正規化、レート上限は実開度が追従できる速さの固定値）で
      車速を保ち、落ち着いたら定常開度 u0（落ち着いた窓の指令の平均）を決めて、プランナーが返す
      ステップを `_Phase.HOLD_STEP`（開度固定・ランプなし）で 1 つずつ測る。ステップが終わる
      たびに開度を u0 に戻して PI へ戻り、全部終わったら**止まらず**次のステーションへ
      （PI の状態を引き継ぐ）。最後のステーションの後は停車復帰。
    - `GridLaunchPattern`（kind は GRID_LAUNCH）: 停車から固定アクセルで launch_end_kmh まで
      （LAUNCH）→ そのまま固定ブレーキで停車まで（STOP）を、プランナーの狙いぶん繰り返す。
      停車の後は停車復帰（DRIVE_BRAKE）を挟んで次の発進へ。
    - HOLD_STEP 中も G ガバナー（安全網）は効く。作動したステップは「強すぎ」として打ち切り扱い。
    - 強いステップの助走（段4b）: 中心から踏むと帯を出るまでに傾きを測る時間が足りない狙い
      （`GridPlanner.is_strong`）は、帯の手前の端（`peek_approach_kmh`）へ PI で下がって（上がって）
      落ち着いてから踏む。落ち着いた窓の開度を基準 u0' として開度を決め、ステップは帯の反対側へ
      出たら終える。助走で落ち着かなければそのステップだけとばす。終わったらステーションの
      中心へ戻る。
    - PI 状態（`_grid_opening`・`_grid_integral`）は `_enter_phase` の `keep_grid=True` で引き継ぐ。
      ブレーキ復帰・最高速超えの回復などを通ったら作り直す。

2026-09-25 段4（ProblemReport_20260925）で削除したところ:
    手順2 のパターン列を格子ステップ走行に置き換えたため、上記の履歴のうち次の機構はコードから
    消した（履歴は経緯として残す）: 目標車速つき `SpeedTargetPattern`・トリム階段・定速階段・
    低開度階段（`TrimStairPattern`・`CruiseStairPattern`・`LowOpenStairPattern`）と、それらの
    前進判定・PI。ACCEL_SWEEP・BRAKE_HOLD（固定ブレーキ保持）・CRUISE_TRIM・不感帯プローブの各
    `PatternKind` も無い。残るのは COAST_DOWN・クリープ発進（CreepLaunchPattern。ブレーキ保持は
    `_Phase.BRAKE_HOLD`）・格子ステップ走行。`_Phase.CRUISE_HOLD` は格子ステーションの PI 保持が
    使う（CSV の phase 名を段3 と揃えるため名前は据え置き）。

2026-09-27 段7b（ProblemReport_20260925 段7。手順2 の計測効率化）で足したところ:
    背景: 実測（065502、約3035s）で、ステップの後にステーションの目標（中心・助走の車速）へ
    PI（CRUISE_HOLD）だけで戻す区間が待ち時間の最大要因だった（低ゲイン・レート制限で意図的に
    穏やかにしているため遅い）。
    - 新フェーズ `_Phase.GRID_RETURN`: 固定開度で目標へ速く戻る。目標より遅ければ
      `grid_return_accel_kmhs` 狙いのアクセル（`u0 + (狙い − a_hold) / g_accel`。範囲
      [u0, 最大開度]、上限 G の予測（`_glimit_cap`）で頭打ち）、速ければ不感帯（惰行）。
      先読みガバナー・G ガバナーは HOLD_STEP・SWEEP と同じく適用する（安全網）。
    - 入る場所（`_grid_begin_return`）: ステップの後（`_advance_grid_step`）と、助走の目標を
      セットしたとき（`_advance_grid_hold`）。条件は「ステーション開始済み（u0 あり）かつ
      |PI の目標 − 車速| > `grid_return_switch_kmh`」。満たさなければ従来どおり直接 CRUISE_HOLD
      （`_grid_switch_to_cruise_hold`。開度・積分 = u0）。
    - 抜ける（`_advance_grid_return`）: |目標 − 車速| ≤ `grid_return_switch_kmh` で
      `_grid_switch_to_cruise_hold` へ（CRUISE_HOLD の落ち着き判定に戻す）。落ち着き待ちの
      打ち切りタイマー（`_grid_settle_started`）は `_grid_begin_return` で戻り試行の頭でしか
      リセットしないため、GRID_RETURN の間も進み続ける（打ち切り 40s の扱い自体は変えない）。
      最高車速超えは従来どおり `_Phase.DRIVE_BRAKE`（残りのステップは捨てる）。

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
"""

from __future__ import annotations

import asyncio
import statistics
import time
from collections import deque
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Protocol

import numpy as np

from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet
from tests.research.coverage_live import LiveCoverage
from tests.research.g_limit import PEDAL_ACCEL, PEDAL_BRAKE, GLimitMap
from tests.research.grid_planner import (
    GridPlanner,
    GridSettings,
    LaunchPlan,
    StationPlan,
    Step,
    StepKind,
    StepResult,
    fit_slope,
    pi_hold_step,
)
from tests.research.grid_settle import SettleRecord
from tests.research.research_types import (
    ACTUATOR_PULSE_MAX,
    G_TO_KMHS,
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
from tests.research.sweep_planner import SweepPlan, plan_sweeps

PATTERN_LOOP_INTERVAL_S: float = 0.1  # 100ms 周期（drive_logs の記録間隔に一致。本番と同じ）
STOP_SPEED_KMH: float = VEHICLE_STOP_SPEED_KMH


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
class GridStationPattern(LearningPattern):
    """格子ステップ走行の車速ステーション 1 つ（段3。kind は GRID_STEP）。

    `plan` が狙う加速度の列、`settings` が調整値。開度は走行中にプランナーが決める
    （`accel_opening` 等の固定開度は使わない）。
    """

    plan: StationPlan | None = None
    settings: GridSettings = GridSettings()


@dataclass(frozen=True)
class GridLaunchPattern(LearningPattern):
    """格子ステップ走行の発進・停車セル（段3。kind は GRID_LAUNCH）。"""

    plan: LaunchPlan | None = None
    settings: GridSettings = GridSettings()


@dataclass(frozen=True)
class GridSweepPattern(LearningPattern):
    """通し掃引（段6c。kind は GRID_SWEEP）。走行中に網羅の穴を見て、掃引を作って走る。

    `wltp_seconds`/`wltp_mean` はモード（複数なら合成）の車速 × 加速度の集計、`speed_edges`/
    `accel_edges` は格子の境界、`min_s` は「狙う最小秒数」、`data_max_s` は「穴」の判定
    （データ秒がこれ未満）、`max_passes` は 1 本の掃引の最大回数（0 なら何もしない）。
    """

    wltp_seconds: np.ndarray | None = field(default=None, compare=False)
    wltp_mean: np.ndarray | None = field(default=None, compare=False)
    speed_edges: tuple[float, ...] = ()
    accel_edges: tuple[float, ...] = ()
    min_s: float = 2.5
    data_max_s: float = 2.0
    max_passes: int = 6
    settings: GridSettings = GridSettings()


class _Phase(Enum):
    MEASURE = auto()
    DRIVE_ACCEL = auto()
    COAST = auto()
    DRIVE_BRAKE = auto()
    BRAKE_HOLD = auto()
    CRUISE_HOLD = auto()  # 格子ステップ走行の PI 保持（段3）。旧定速階段のフェーズ名を引き継ぐ
    CREEP_LAUNCH = auto()  # クリープ発進・クリープ域ブレーキ保持（段1）。両ペダル 0% で自走
    HOLD_STEP = auto()  # 格子ステップ走行（段3）。開度固定・ランプなしで加速度を測る
    GRID_RETURN = auto()  # 格子ステップ走行（段7b）: ステップの後、固定開度で目標へ速く戻る
    CALIB_BRAKE = auto()  # G 校正（段6a）: 0.2G 狙いの G 比例ブレーキ減速。各車速の開度と減速を記録
    SWEEP_UP = auto()  # 通し掃引（段6c）の助走: G 比例の加速で掃引の開始車速まで上げる
    SWEEP = auto()  # 通し掃引（段6c）: 車速のセルごとに開度を切り替えながら通り抜ける
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
    # pattern_drive.py が渡す。既定値は GridSettings.settle_tol_kmh と揃えている）
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
    coast_accel_approach_margin_pct: float = field(default=1.0)  # 不感帯 − これ まで先に上げる
    # 踏む速さ [%/s] = これ ×（目標 G − 今の G）。半分データ由来・半分仮の値:
    # 感度 0.004〜0.005 G/%・遅れ約 1s（実機 110427）から、応答の時定数（1/(これ×感度) ≒ 2〜2.5s）
    # が遅れの 2 倍以上になる値。大きいほど早く届くが 0.2G を超えやすい。実機で決め直す
    coast_accel_rate_gain: float = field(default=100.0)
    # G 校正のブレーキ減速（CALIB_BRAKE）。stop_decel.decelerate_to_stop（手順2 終了時の緩減速）と
    # 同じ刻み方: dwell_s ごとに直近 slope_window_s の減速 G を見て、目標 − press_margin 未満なら
    # step_mm 踏み増し、release_above_g 超なら step_mm 戻す。目標・余裕・窓・接近位置は
    # coast_accel_* を共用（decel_stop と同じ値）。pattern_drive.py が decel_stop から渡す
    calib_release_above_g: float = field(default=0.3)
    calib_step_mm: float = field(default=0.5)
    calib_dwell_s: float = field(default=1.0)
    # 上限 G の予測の目標 [G]（learning.g_cap_g から渡す）。ここまでなら踏んでよいと予測した開度で
    # 格子ステップの開度を頭打ちにする。既定は config.LearningSection.g_cap_g と揃えている
    g_cap_g: float = field(default=0.3)
    # 惰行に入ってからこの時間は減速を数えない（ペダルを離してから車が応える遅れ。実機で
    # 指令→G の山まで約 0.6s、離す動作 0.4s）。惰行の減速 → 上限 G の予測の基準点に使う
    coast_measure_delay_s: float = field(default=1.5)
    # 段6b・門②: 上限 G を先読みして緩める。G の見込み = 今の G + G の増える速さ × gov_lead_s。
    # gov_lead_s 0.6 s は実機（20260926 の 890.7→891.3 s）で「ブレーキ指令から G の山まで」の遅れ。
    # 見込みが上限 × gov_soft_frac に届いたら踏み増しを止め、上限（× g_limit_frac）に届いたら
    # 従来どおり 1 周期ごとに下げる。gov_soft_frac 0.85 は**仮の値**（0.85 × 0.4G × 0.98 ≒ 0.33G。
    # g_cap_g 0.3G と上限 0.4G の間）。実機の G の最大で決め直す
    gov_lead_s: float = field(default=0.6)
    gov_soft_frac: float = field(default=0.85)
    gov_rate_window_s: float = field(default=0.5)  # G の増える速さを見る窓 [s]


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
        on_grid_result: Callable[[StepResult], None] | None = None,
        on_grid_settle: Callable[[SettleRecord], None] | None = None,
        on_calib_done: Callable[[list[str]], None] | None = None,
        on_sweep_log: Callable[[str], None] | None = None,
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
        self._on_grid_result = on_grid_result
        self._on_grid_settle = on_grid_settle
        # G 校正が終わったとき、上限開度の表（行のリスト）を渡す
        self._on_calib_done = on_calib_done
        self._on_sweep_log = on_sweep_log  # 通し掃引の 1 行ログ
        self.grid_settles: list[SettleRecord] = []  # 落ち着き待ちの記録（grid_settle の集計用）

        self._max_decel_kmhs = max(profile.max_decel_g * G_TO_KMHS, 0.0)
        self._g_limit_kmhs = self._max_decel_kmhs * self._config.g_limit_frac
        self._accel_speed_cap = profile.max_speed * self._config.accel_speed_cap_frac

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
        # 段7b・7c: GRID_RETURN の刻み足し（_step_grid_return）の状態。_grid_begin_return で作り直す
        self._gr_opening = 0.0  # 指令開度（ガバナー適用前）
        self._gr_last_t: float | None = None  # 直近に開度を更新した時刻
        self._gr_hist: deque[tuple[float, float]] = deque()  # 加速度の窓 (時刻, 車速)
        # G 校正のブレーキ減速（CALIB_BRAKE）の状態。_enter_phase で作り直す
        self._cb_t0: float | None = None  # 接近ランプの開始時刻
        self._cb_approach_pct = 0.0
        self._cb_opening = 0.0  # 接近後の指令開度（ガバナー適用前）
        self._cb_next_t: float | None = None  # 次に開度を決める時刻。None ならまだ接近中
        self._cb_hist: deque[tuple[float, float]] = deque()  # 減速度の窓 (時刻, 車速)
        # 上限 G に届く開度の予測マップ（段6a）。G 校正・惰行・格子ステップの実測で育てる
        self.glimit = GLimitMap()
        # 停車復帰（DRIVE_BRAKE）を G 比例で行うか（高い車速から入ったとき。_enter_phase で決める）
        self._db_gprop = False
        self._cb_start_pct = 0.0  # G 比例ブレーキの最初の開度（接近ランプの目標）
        # 通し掃引（段6c）。網羅カウンタは掃引のパターンがあるときだけ作る（全周期の車速を数える）
        self.coverage: LiveCoverage | None = None
        sweeps = [p for p in patterns if isinstance(p, GridSweepPattern)]
        if sweeps and sweeps[0].speed_edges:
            self.coverage = LiveCoverage(list(sweeps[0].speed_edges), list(sweeps[0].accel_edges))
        self._sw_seen_idx = -1
        self._sw_plans: list[SweepPlan] = []  # まだ走っていない掃引（先頭が今の掃引）
        self._sw_passes = 0  # 今の掃引を走った回数
        # 今の掃引のセルごとの (アクセル, ブレーキ)
        self._sw_openings: list[tuple[float, float]] = []
        self._sw_from = (0.0, 0.0)  # セルが変わったときのランプの始点
        self._sw_cell = -1  # 今のセル（plan.cells の添字）
        self._sw_ramp_t = 0.0
        self._sw_cmd = (0.0, 0.0)  # 今の指令開度 (アクセル, ブレーキ)
        self._sw_before = 0.0  # 掃引の前の、対象セルのデータ秒の合計（ログ用）
        self._accel_hist: deque[tuple[float, float]] = deque()  # (時刻, 平滑化した加速度)
        self._accel_rate = 0.0  # 加速度の変化の速さ [km/h/s²]（先読みガバナー用）
        self._step_from = (0.0, 0.0)  # 開度固定ステップを始めたときの (アクセル, ブレーキ) 開度
        self._ca_record_t = -1e9  # 加速側の点を最後に足した時刻（1 s に 1 点）
        self._speed_hist: deque[tuple[float, float]] = deque()
        self._last_speed = 0.0  # 定速階段の PI が使う直近の車速（_execute_one_cycle で更新）
        # 格子ステップ走行（段3）。グリッドのパターンがあるときだけプランナーを持つ
        self._grid: GridPlanner | None = None
        self._grid_opening: float | None = None  # 専用 PI の開度（None なら未初期化）
        self._grid_integral = 0.0
        self._grid_seen_idx = -1  # 入場処理を済ませたパターン番号
        self._grid_station_begun = False  # begin_station を呼んだか（u0 を決めたか）
        self._grid_settle_started = 0.0  # 落ち着くのを待ち始めた時刻（打ち切り用）
        self._grid_settle_since: float | None = None
        self._grid_window_speeds: list[tuple[float, float]] = []  # 落ち着いた窓の (時刻, 車速)
        self._grid_approach_kmh: float | None = None  # 助走中の PI の目標車速（None なら中心）
        self._grid_window: list[float] = []  # 連続して許容幅内だった間に送ったアクセル開度
        self._grid_wait_cycles = 0  # 落ち着き待ちの周期数と、そのうち許容幅内だった周期数
        self._grid_inside_cycles = 0
        self._grid_step: Step | None = None
        self._grid_samples: list[tuple[float, float]] = []  # (時刻, 車速)
        self._grid_governed = False
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

        grid_patterns = [
            p for p in patterns if isinstance(p, GridStationPattern | GridLaunchPattern)
        ]
        if grid_patterns:
            ff = profile.feedforward_params
            self._grid = GridPlanner(
                grid_patterns[0].settings,
                accel_deadband_pct=ff.accel_deadband_pct,
                brake_deadband_pct=ff.brake_deadband_pct,
                max_accel_pct=profile.max_accel_opening,
                max_brake_pct=profile.max_brake_opening,
                # 減速の助走の上限。落ち着き待ちの許容幅が最高車速を超えないよう手前にする
                max_speed_kmh=profile.max_speed - 2.0 * grid_patterns[0].settings.settle_tol_kmh,
                stop_brake_pct=ff.stop_brake_opening_pct,
                cap_fn=self._glimit_cap,
                # 惰行はコーストダウンの実測から引く（段7a）。測れない車速帯は None
                # （GridPlanner が従来どおり惰行ステップにフォールバックする）
                coast_fn=lambda v: (d := self.glimit.coast_decel_kmhs(v)) and -d,
                # そのステーションでまだ測っていないペダルの最初の感度を、glimit の実測点から
                # 引く（段7c。仮定値 2.0 が実測と 8 倍ずれ、GRID_RETURN の開度が浅すぎた対策）
                gain_fn=self.glimit.strongest_point,
            )

        calib = self._profile.calibration
        if calib is not None:
            self._accel_pos_cmd = calib.accel_zero_pos
            self._brake_pos_cmd = opening_to_position(
                self._stop_return_brake_pct, calib.brake_zero_pos, calib.brake_full_pos
            )

    @property
    def grid_planner(self) -> GridPlanner | None:
        """格子ステップ走行のプランナー（測定結果 `results`・感度 g を持つ）。無ければ None。"""
        return self._grid

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
        if self._grid_enter_pattern(pattern, now):
            return  # 狙いが無いグリッドのパターンをとばした。次の周期でやり直す
        if self._sweep_enter_pattern(pattern, now):
            return  # 穴が無く掃引をとばした。次の周期でやり直す

        try:
            speed = await self._can_reader.read_speed()
        except Exception as exc:
            await self._abort_emergency(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）")
            return

        self._last_speed = speed
        if self.coverage is not None:
            self.coverage.push(now, speed)
        self._speed_hist.append((now, speed))
        accel_kmhs = self._smoothed_accel(now)
        self._update_accel_rate(now, accel_kmhs)

        self._update_governor(accel_kmhs)
        accel_opening, brake_opening = self._command_openings(pattern, now)
        accel_opening, brake_opening = enforce_pedal_exclusion(accel_opening, brake_opening)
        self._current_accel_opening = accel_opening
        self._current_brake_opening = brake_opening

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
        # 段7d: G 校正の上限開度表用に、指令ではなく実際の開度を積む（低速の踏み込みは遅れる）
        self._accel_actual_pct = position_to_opening(
            monitor_accel.position_pulse, calib.accel_zero_pos, calib.accel_full_pos
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
        if isinstance(self._patterns[idx], GridStationPattern):
            return _Phase.CRUISE_HOLD  # 停車からも専用の PI で上げる（固定開度の加速は使わない）
        if isinstance(self._patterns[idx], GridLaunchPattern):
            return _Phase.HOLD_STEP
        if isinstance(self._patterns[idx], GridSweepPattern):
            return _Phase.SWEEP_UP  # 入場処理（_sweep_enter_pattern）が掃引を作って始める
        if self._patterns[idx].kind in (PatternKind.COAST_DOWN, PatternKind.G_CALIB):
            return _Phase.DRIVE_ACCEL
        return _Phase.MEASURE

    def _command_openings(self, pattern: LearningPattern, now: float) -> tuple[float, float]:
        assert self._phase_started_at is not None
        elapsed = max(0.0, now - self._phase_started_at)
        if self._phase is _Phase.SWEEP:
            return self._sweep_command(now)
        if self._phase in (_Phase.DRIVE_ACCEL, _Phase.SWEEP_UP):
            ramped = self._coast_accel_opening(pattern, now)
            self._accel_request = ramped
            if self._accel_gov_cap is not None:
                ramped = min(ramped, self._accel_gov_cap)
            return clamp_opening(ramped, self._profile.max_accel_opening), 0.0
        if self._phase is _Phase.COAST:
            return 0.0, 0.0
        if self._phase is _Phase.CREEP_LAUNCH:
            return 0.0, 0.0  # 両ペダル完全解放（クリープのみで自走）
        if self._phase is _Phase.CRUISE_HOLD:
            assert isinstance(pattern, GridStationPattern)
            return self._grid_hold_opening(pattern), 0.0
        if self._phase is _Phase.GRID_RETURN:
            assert isinstance(pattern, GridStationPattern)
            opening = self._grid_return_opening(pattern)
            self._accel_request = opening
            if self._accel_gov_cap is not None:
                opening = min(opening, self._accel_gov_cap)
            return clamp_opening(opening, self._profile.max_accel_opening), 0.0
        if self._phase is _Phase.HOLD_STEP:
            assert isinstance(pattern, GridStationPattern | GridLaunchPattern)
            return self._grid_step_openings(now, pattern.settings.step_lag_s)
        if self._phase is _Phase.CALIB_BRAKE or (
            self._phase is _Phase.DRIVE_BRAKE and self._db_gprop
        ):
            opening = self._calib_brake_opening(now)
            self._brake_request = opening
            if self._brake_gov_cap is not None:
                opening = min(opening, self._brake_gov_cap)
            return 0.0, clamp_opening(opening, self._profile.max_brake_opening)
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
        if self._phase is _Phase.CALIB_BRAKE:
            self._advance_calib_brake(speed, elapsed, now)
            return
        if self._phase is _Phase.SWEEP_UP:
            assert isinstance(pattern, GridSweepPattern)
            self._advance_sweep_up(pattern, speed, elapsed, now)
            return
        if self._phase is _Phase.SWEEP:
            assert isinstance(pattern, GridSweepPattern)
            self._advance_sweep(pattern, speed, elapsed, now)
            return
        if self._phase is _Phase.CREEP_LAUNCH:
            self._advance_creep_launch(pattern, speed, accel_kmhs, elapsed, now)
            return
        if self._phase is _Phase.HOLD_STEP:
            self._advance_grid_step(pattern, speed, now)
            return
        if self._phase is _Phase.DRIVE_BRAKE:
            if self._db_gprop:
                self._step_gprop_brake(speed, now)
                if speed <= self._config.coast_down_stop_speed_kmh:
                    # 低速まで下りた。あとは従来どおり停車保持開度へランプ（今の開度から）
                    self._db_gprop = False
                    self._brake_ramp_from = self._current_brake_opening
                    self._phase_started_at = now
                    elapsed = 0.0
            if speed <= STOP_SPEED_KMH:
                if isinstance(pattern, GridLaunchPattern):
                    self._grid_launch_continue(now)  # 停車復帰が済んだ。次の発進へ
                elif isinstance(pattern, GridSweepPattern):
                    self._sweep_pass_done(pattern, now)  # 掃引 1 回が済んだ。次の回・次の掃引へ
                else:
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
        if self._phase is _Phase.CRUISE_HOLD:
            assert isinstance(pattern, GridStationPattern)
            self._advance_grid_hold(pattern, speed, now)
            return
        if self._phase is _Phase.GRID_RETURN:
            assert isinstance(pattern, GridStationPattern)
            self._advance_grid_return(pattern, speed, now)
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

        最初は 0 から「不感帯 − approach_margin」まで accel_ramp_time_s で上げる（遊びの中では
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
            if pattern.kind is PatternKind.G_CALIB:
                self._enter_phase(_Phase.CALIB_BRAKE, now)  # 惰行せず、cap からそのまま校正の減速へ
            else:
                self._enter_phase(_Phase.COAST, now)

    def _calib_brake_opening(self, now: float) -> float:
        """G 校正のブレーキ開度（ガバナー適用前）。

        最初は 0 から「不感帯 − approach_margin」まで brake_ramp_time_s で上げる（遊びの中では
        減速が出ない）。以降の開度は `_advance_calib_brake` が減速 G との差に応じて刻む。
        上限は停車保持開度（停車復帰で毎回踏んでいる実績のある開度）。
        """
        cfg = self._config
        if self._cb_t0 is None:
            ff = self._profile.feedforward_params
            self._cb_approach_pct = min(
                self._stop_return_brake_pct,
                max(0.0, ff.brake_deadband_pct - cfg.coast_accel_approach_margin_pct),
            )
            # G 校正が済んでいれば、その車速で目標 G が出る開度から始める（校正の結果の使いどころ。
            # 無ければ不感帯の手前から刻んで上げる）
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

    def _advance_calib_brake(self, speed: float, elapsed: float, now: float) -> None:
        """G 校正の減速。低速まで下りたら終わる。刻みは `_step_gprop_brake`。"""
        cfg = self._config
        self._step_gprop_brake(speed, now)
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        if speed <= cfg.coast_down_stop_speed_kmh or elapsed >= cfg.brake_stop_timeout_s:
            self._finish_pattern(speed, now)

    def _step_gprop_brake(self, speed: float, now: float) -> None:
        """G 比例ブレーキ減速の毎周期の処理（G 校正・高い車速からの停車復帰で共用）。

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

    def _glimit_open_for(self, pedal: str, speed_kmh: float, target_kmhs: float) -> float | None:
        """そのペダルが `target_kmhs`（正）を出すと予測される開度（測れていなければ None）。"""
        ff = self._profile.feedforward_params
        db = ff.brake_deadband_pct if pedal == PEDAL_BRAKE else ff.accel_deadband_pct
        return self.glimit.cap_pct(pedal, speed_kmh, target_kmhs, db)

    def _glimit_cap(self, pedal: str, speed_kmh: float) -> float | None:
        """格子ステップの開度の上限（上限 G に届くと予測される開度）。測れていなければ None。"""
        ff = self._profile.feedforward_params
        db = ff.brake_deadband_pct if pedal == PEDAL_BRAKE else ff.accel_deadband_pct
        return self.glimit.cap_pct(pedal, speed_kmh, self._config.g_cap_g * G_TO_KMHS, db)

    def _grid_feed_glimit(self, step: Step, a_meas: float, now: float, s: GridSettings) -> None:
        """測れたステップの (開度, 加速度) を上限 G の予測マップに足す。"""
        assert self._phase_started_at is not None
        t0 = self._phase_started_at + s.step_lag_s
        vs = [v for t, v in self._grid_samples if t >= t0] or [v for _, v in self._grid_samples]
        if not vs:
            return
        mean_v = sum(vs) / len(vs)
        if step.kind is StepKind.COAST:
            if a_meas < 0.0:
                self.glimit.add_coast(mean_v, -a_meas)
        elif step.brake_pct > 0.0:
            self.glimit.add(PEDAL_BRAKE, mean_v, step.brake_pct, -a_meas)
        elif step.accel_pct > 0.0 and step.kind is not StepKind.HOLD:
            self.glimit.add(PEDAL_ACCEL, mean_v, step.accel_pct, a_meas)

    def _advance_coast(self, speed: float, accel_kmhs: float, elapsed: float, now: float) -> None:
        """コーストダウンの惰行の前進判定（段7c: 終了をクリープ平衡車速から決める）。

        以前は固定 `coast_down_stop_speed_kmh`（5.0）で終えていたが、クリープ平衡がそれより
        高い車両では惰行だけで届かず `coast_timeout_s`（90s）まで待ち、低い車両では惰行カーブの
        低速端のデータを取りこぼしていた。惰行は平衡へ漸近するだけでちょうど届く保証がないため、
        平衡以下に達するか、クリープの影響が残る車速（平衡 + `grid_settle_tol_kmh`）以下で
        車速の傾きが落ち着いたら終える（`_coast_creep_settled`。停車ステップの
        `_grid_creep_settled` と同じ判定・調整値）。`coast_down_stop_speed_kmh` は他の使い道
        （G 校正のブレーキ減速の終了・停車復帰の切替・格子ステップの底・発進セル）のまま残す。
        """
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        if elapsed >= cfg.coast_measure_delay_s and accel_kmhs < 0.0:
            self.glimit.add_coast(speed, -accel_kmhs)  # 上限 G の予測の基準（開度=不感帯の点）
        if elapsed >= cfg.coast_timeout_s:
            self._finish_pattern(speed, now)
            return
        creep = self._profile.feedforward_params.creep_speed_kmh
        if speed <= creep:
            self._finish_pattern(speed, now)
            return
        ceiling = self._grid_creep_ceiling_kmh(cfg.grid_settle_tol_kmh)
        if speed <= ceiling and self._coast_creep_settled(now):
            self._finish_pattern(speed, now)

    def _coast_creep_settled(self, now: float) -> bool:
        """コーストダウンの惰行が、クリープ平衡近くで車速の傾きが落ち着いたか（段7c）。

        停車ステップの `_grid_creep_settled` と同じ判定・同じ調整値（`creep_launch_settle_*`）を、
        格子ステップの記録（`_grid_samples`）ではなく毎周期の `_speed_hist`（コーストダウンは
        格子パターンの外でも走るため）から行う。
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

    # ── 格子ステップ走行（段3）───────────────────────────────────────
    def _grid_enter_pattern(self, pattern: LearningPattern, now: float) -> bool:
        """グリッドのパターンに入った最初の周期の準備。パターンをとばしたら True。"""
        if self._grid is None or self._grid_seen_idx == self._pattern_idx:
            return False
        if not isinstance(pattern, GridStationPattern | GridLaunchPattern):
            return False
        self._grid_seen_idx = self._pattern_idx
        if isinstance(pattern, GridStationPattern):
            self._grid_reset_settle(now)
            self._grid_station_begun = False
            self._grid_approach_kmh = None
            return False
        assert pattern.plan is not None
        self._grid.begin_launch(pattern.plan)
        step = self._grid.next_step()
        if step is None:
            self._advance_pattern(now)
            return True
        self._grid_start_step(step, now)
        return False

    def _grid_reset_settle(self, now: float) -> None:
        self._grid_settle_started = now
        self._grid_settle_since = None
        self._grid_window = []
        self._grid_window_speeds = []
        self._grid_wait_cycles = 0
        self._grid_inside_cycles = 0

    def _grid_record_settle(
        self, pattern: GridStationPattern, now: float, *, timed_out: bool
    ) -> None:
        """落ち着き待ちの 1 回ぶんを記録する（許容幅・待ち時間を決める材料。grid_settle）。"""
        assert pattern.plan is not None
        window = self._grid_window
        window_speeds = self._grid_window_speeds
        record = SettleRecord(
            station_kmh=pattern.plan.speed_kmh,
            target_kmh=self._grid_target_kmh(pattern),
            approach=self._grid_approach_kmh is not None,
            wait_s=now - self._grid_settle_started,
            inside_frac=self._grid_inside_cycles / max(self._grid_wait_cycles, 1),
            u_mean_pct=(
                float("nan") if timed_out or not window else statistics.fmean(window)
            ),
            u_std_pct=(
                float("nan") if timed_out or len(window) < 2 else statistics.pstdev(window)
            ),
            timed_out=timed_out,
            slope_kmhs=(
                float("nan") if timed_out or len(window_speeds) < 2
                else fit_slope([t for t, _ in window_speeds], [v for _, v in window_speeds])
            ),
        )
        self.grid_settles.append(record)
        if self._on_grid_settle is not None:
            self._on_grid_settle(record)

    def _grid_start_step(self, step: Step, now: float) -> None:
        """開度固定のステップを始める（`_enter_phase` は通さない。PI 状態を消さないため）。"""
        self._grid_step = step
        self._grid_samples = []
        self._grid_governed = False
        self._accel_gov_cap = None
        self._brake_gov_cap = None
        self._step_from = (self._current_accel_opening, self._current_brake_opening)
        self._phase = _Phase.HOLD_STEP
        self._phase_started_at = now

    def _grid_target_kmh(self, pattern: GridStationPattern) -> float:
        """PI の目標車速: 助走中は助走の車速、それ以外はステーションの中心。"""
        assert pattern.plan is not None
        if self._grid_approach_kmh is not None:
            return self._grid_approach_kmh
        return pattern.plan.speed_kmh

    def _grid_hold_opening(self, pattern: GridStationPattern) -> float:
        """ステーションの車速へ保つ専用 PI の開度。ゲインは感度 g_accel で正規化する。"""
        assert self._grid is not None and pattern.plan is not None
        s = pattern.settings
        ff = self._profile.feedforward_params
        min_pct, max_pct = ff.accel_deadband_pct, self._profile.max_accel_opening
        if self._grid_opening is None:
            self._grid_opening = min_pct  # 停車・回復の後は不感帯から。レート制限で上げていく
            self._grid_integral = min_pct
        g = self._grid.g_accel
        self._grid_opening, self._grid_integral = pi_hold_step(
            self._grid_opening, self._grid_integral,
            self._grid_target_kmh(pattern), self._last_speed,
            kp=s.hold_kp_norm / g, ki=s.hold_ki_norm / g, min_pct=min_pct, max_pct=max_pct,
            max_rate_pct_s=s.hold_max_rate_pct_s, dt=self._interval_s,
        )
        return self._grid_opening

    def _grid_return_opening(self, pattern: GridStationPattern) -> float:
        """GRID_RETURN（段7b・7c）の指令開度。PI（CRUISE_HOLD）より速く目標へ寄せる。

        目標より遅ければ `_step_grid_return` が毎周期刻み足した開度（`_gr_opening`）をそのまま
        返す。目標より速ければ不感帯（惰行）でよい（下がり過ぎない限りブレーキは踏まない）。
        """
        assert self._grid is not None and pattern.plan is not None
        ff = self._profile.feedforward_params
        if self._last_speed >= self._grid_target_kmh(pattern):
            return ff.accel_deadband_pct
        return self._gr_opening

    def _step_grid_return(self, pattern: GridStationPattern, speed: float, now: float) -> None:
        """GRID_RETURN 毎周期の開度の刻み足し（段7c）。

        直近 `coast_accel_slope_window_s` の実測加速度と `grid_return_accel_kmhs`（狙い）の差
        [G] に応じて `coast_accel_rate_gain` で踏み足す（G 校正の加速 `_step_coast_accel` と
        同じ式）。踏み始めの開度は `_grid_begin_return` が `u0 + (狙い − a_hold) / g_accel`
        （感度は段7c で glimit から種をまいた値）で決めており、ここではその後を実測で補正する。

        20260928 実機: 15→25 km/h の戻りが、感度の当て推量（仮定値 2.0 で実測の 1/8）だけの
        開度 8.11%（釣り合い車速 21.2 km/h）で 190 s 止まった。実測の加速度で補正すれば、
        狙いに届かない開度のまま止まり続けることはない。
        """
        assert self._grid is not None
        cfg = self._config
        hist = self._gr_hist
        hist.append((now, speed))
        while hist and now - hist[0][0] > cfg.coast_accel_slope_window_s:
            hist.popleft()
        if self._gr_last_t is None:
            self._gr_last_t = now
            return
        dt = now - self._gr_last_t
        self._gr_last_t = now
        if len(hist) < 3:
            return
        accel_kmhs = fit_slope([t for t, _ in hist], [v for _, v in hist])
        margin_g = (pattern.settings.grid_return_accel_kmhs - accel_kmhs) / G_TO_KMHS
        if abs(margin_g) <= cfg.coast_accel_press_margin_g:
            return
        opening = self._gr_opening + cfg.coast_accel_rate_gain * margin_g * dt
        opening = min(self._profile.max_accel_opening, max(self._grid.u0, opening))
        cap = self._glimit_cap(PEDAL_ACCEL, speed)
        if cap is not None:
            opening = min(opening, cap)
        self._gr_opening = opening

    def _grid_step_openings(self, now: float, lag_s: float) -> tuple[float, float]:
        """HOLD_STEP の指令: ステップの開度（固定）。踏むペダルは step_lag_s かけて上げる（段6b）。

        一気に出すと、指令から G の山まで約 0.6 s の遅れの間に上限 G を超える（20260926 の 0.47G）。
        step_lag_s は傾きの当てはめから除く区間（頭の 0.5 s）と同じなので、測定には影響しない。
        もう一方のペダル（0 のペダル）は即座に離す。ランプの始点はステップを始めたときの開度。
        """
        step = self._grid_step
        assert step is not None and self._phase_started_at is not None
        frac = self._ramp_fraction(now - self._phase_started_at, lag_s)
        accel, brake = step.accel_pct, step.brake_pct
        self._accel_request, self._brake_request = accel, brake
        from_accel, from_brake = self._step_from
        if accel > 0.0:
            accel = from_accel + (accel - from_accel) * frac
        if brake > 0.0:
            brake = from_brake + (brake - from_brake) * frac
        if self._accel_gov_cap is not None:
            accel = min(accel, self._accel_gov_cap)
        if self._brake_gov_cap is not None:
            brake = min(brake, self._brake_gov_cap)
        return (
            clamp_opening(accel, self._profile.max_accel_opening),
            clamp_opening(brake, self._profile.max_brake_opening),
        )

    def _grid_handle_settle_timeout(
        self, pattern: GridStationPattern, speed: float, now: float
    ) -> None:
        """落ち着き待ちの打ち切り（CRUISE_HOLD・GRID_RETURN 共通。段7c）。

        助走中なら、そのステップだけとばして中心へ戻る。中心（助走なし）ならステーションの
        残りをすべて捨てて次へ進む。以前は CRUISE_HOLD（`_advance_grid_hold`）側にしか無く、
        GRID_RETURN は釣り合い車速が目標に届かないと永久に待ち続けた（20260928 実機で 190 s）。
        """
        assert self._grid is not None
        self._grid_record_settle(pattern, now, timed_out=True)
        if self._grid_approach_kmh is not None:
            self._grid.drop_next()
            self._grid_approach_kmh = None
            self._grid_reset_settle(now)
            return
        self._grid.abort_remaining()
        self._grid_station_done(speed, now)

    def _advance_grid_hold(self, pattern: GridStationPattern, speed: float, now: float) -> None:
        """ステーションで PI が落ち着くのを待ち、落ち着いたら次のステップを始める。"""
        assert self._grid is not None and pattern.plan is not None
        s = pattern.settings
        if speed > self._profile.max_speed:
            self._grid.abort_remaining()
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        self._grid_wait_cycles += 1
        if abs(speed - self._grid_target_kmh(pattern)) <= s.settle_tol_kmh:
            self._grid_inside_cycles += 1
            if self._grid_settle_since is None:
                self._grid_settle_since = now
            self._grid_window.append(self._current_accel_opening)
            self._grid_window_speeds.append((now, speed))
        else:
            self._grid_settle_since = None
            self._grid_window = []
            self._grid_window_speeds = []
        settled = (
            self._grid_settle_since is not None and now - self._grid_settle_since >= s.settle_s
        )
        if not settled:
            if now - self._grid_settle_started >= s.settle_timeout_s:
                self._grid_handle_settle_timeout(pattern, speed, now)
            return
        window_mean = sum(self._grid_window) / len(self._grid_window)
        self._grid_record_settle(pattern, now, timed_out=False)
        if not self._grid_station_begun:
            self._grid.begin_station(pattern.plan, window_mean)
            self._grid_station_begun = True
        u0_approach: float | None = None
        if self._grid_approach_kmh is None:
            approach = self._grid.peek_approach_kmh()
            if approach is not None:  # 強いステップ: 帯の手前の端へ移ってから踏む
                self._grid_begin_return(pattern, now, approach, speed, is_approach=True)
                return
        else:
            u0_approach = window_mean  # 助走の車速で落ち着いた開度 = そこでの加速度 0 の開度 u0'
        step = self._grid.next_step(u0_approach)
        self._grid_approach_kmh = None
        if step is None:
            self._grid_station_done(speed, now)
            return
        self._grid_start_step(step, now)

    def _advance_grid_step(self, pattern: LearningPattern, speed: float, now: float) -> None:
        """開度固定のステップの終了判定。終わったら加速度を測って記録し、次へ進む。"""
        assert self._grid is not None and self._grid_step is not None
        assert isinstance(pattern, GridStationPattern | GridLaunchPattern)
        assert pattern.plan is not None
        s = pattern.settings
        step = self._grid_step
        assert self._phase_started_at is not None
        elapsed = now - self._phase_started_at
        self._grid_samples.append((now, speed))
        self._grid_governed = self._grid_governed or self._governor_limiting()
        launching = isinstance(pattern, GridLaunchPattern)
        if speed > self._profile.max_speed:
            self._grid.abort_remaining()
            self._grid_step = None
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        if step.kind is StepKind.LAUNCH:
            plan = pattern.plan
            assert isinstance(plan, LaunchPlan)
            done = speed >= plan.end_kmh or elapsed >= s.settle_timeout_s
        elif step.kind is StepKind.STOP:
            done = (
                speed <= STOP_SPEED_KMH or elapsed >= s.settle_timeout_s
                or self._grid_creep_settled(now, speed, s)
            )
        else:
            plan = pattern.plan
            assert isinstance(plan, StationPlan)
            floor = self._config.coast_down_stop_speed_kmh
            if step.approach_kmh is not None:
                assert step.target is not None
                # 助走の端から踏んだので、帯の反対側へ出たら終える（今いる側は出ていてよい）
                edge = s.step_band_max_kmh
                beyond = (
                    speed > plan.speed_kmh + edge if step.target.a_kmhs > 0.0
                    else speed < plan.speed_kmh - edge
                )
            else:
                beyond = abs(speed - plan.speed_kmh) > s.step_band_max_kmh
            done = elapsed >= s.step_window_s or beyond or speed <= floor
        if not done:
            return
        a_meas, fit_points, fit_resid = self._grid_measured_accel(step, now, s)
        self._grid_feed_glimit(step, a_meas, now, s)
        result = self._grid.record(
            step, a_meas, governed=self._grid_governed,
            fit_points=fit_points, fit_resid_kmh=fit_resid,
        )
        if self._on_grid_result is not None:
            self._on_grid_result(result)
        self._grid_step = None
        if launching:
            self._grid_launch_after_step(pattern, speed, now)
        else:
            # 低速まで落ちても残りは捨てない（段4b）。低いステーションでは強い減速のあとに車速が
            # 底まで落ちるが、続く強い加速こそ US06 などで要る狙い。GRID_RETURN・PI（段7b）で
            # ステーションの車速へ戻る（戻れなければ落ち着き待ちの打ち切りが残りをとばす）
            assert isinstance(pattern, GridStationPattern)
            self._grid_begin_return(pattern, now, plan.speed_kmh, speed, is_approach=False)

    def _grid_creep_ceiling_kmh(self, settle_tol_kmh: float) -> float:
        """クリープの影響が残る車速の上端 = クリープ平衡速度 + 落ち着きの許容幅。

        段7c: コーストダウンの終了判定（`_advance_coast`）からも使うため、`GridSettings` 全体
        ではなく `settle_tol_kmh` だけを受け取る（格子のパターンが無い走行でも使えるように）。
        """
        return self._profile.feedforward_params.creep_speed_kmh + settle_tol_kmh

    def _grid_creep_settled(self, now: float, speed: float, s: GridSettings) -> bool:
        """停車ステップがクリープ平衡に達したか（ブレーキが弱く、停まらずにクリープ速度で釣り合った）。

        車速がクリープ域で、直近 creep_launch_settle_s の傾きが creep_launch_settle_kmhs 未満。
        判定値は既存のクリープ発進の平衡到達と同じ（新しい数値は持たない）。
        """
        if speed > self._grid_creep_ceiling_kmh(s.settle_tol_kmh):
            return False
        window_s = self._config.creep_launch_settle_s
        recent = [(t, v) for t, v in self._grid_samples if t >= now - window_s]
        if len(recent) < 3 or self._grid_samples[0][0] > now - window_s:
            return False
        slope = fit_slope([t for t, _ in recent], [v for _, v in recent])
        return abs(slope) < self._config.creep_launch_settle_kmhs

    def _grid_measured_accel(
        self, step: Step, now: float, s: GridSettings
    ) -> tuple[float, int, float]:
        """ステップの加速度 [km/h/s] = 車速の最小二乗の傾き。頭の step_lag_s は除く。

        戻り値は `(傾き, 当てはめに使った点数, 車速の残差の標準偏差)`（後ろ 2 つは集計用）。
        """
        assert self._phase_started_at is not None
        t0 = self._phase_started_at + s.step_lag_s
        samples = [(t, v) for t, v in self._grid_samples if t >= t0]
        if step.kind in (StepKind.LAUNCH, StepKind.STOP):
            # クリープの影響がある低速域（停車付近・クリープ平衡）を除く
            floor = self._grid_creep_ceiling_kmh(s.settle_tol_kmh)
            samples = [(t, v) for t, v in samples if v >= floor]
        if len(samples) < 3:
            samples = self._grid_samples
        times, speeds = [t for t, _ in samples], [v for _, v in samples]
        slope = fit_slope(times, speeds)
        resid = float("nan")
        if len(samples) >= 3 and times[-1] - times[0] > 0.0:
            coef = np.polyfit(np.asarray(times, dtype=float), np.asarray(speeds, dtype=float), 1)
            fitted = np.polyval(coef, np.asarray(times, dtype=float))
            resid = float(np.std(np.asarray(speeds, dtype=float) - fitted))
        return slope, len(samples), resid

    def _grid_begin_return(
        self, pattern: GridStationPattern, now: float, target_kmh: float, speed: float,
        *, is_approach: bool,
    ) -> None:
        """新しい戻り試行を始める（段7b）: ステップの後、または助走の目標を決めたとき。

        ステーション開始済み（u0 あり）で、目標（`target_kmh`）から `grid_return_switch_kmh` より
        離れていれば `_Phase.GRID_RETURN`（固定開度で速く戻る）へ。そうでなければ従来どおり
        直接 `_grid_switch_to_cruise_hold`（PI に任せる）。落ち着き待ちの打ち切りタイマーは
        ここで一度だけリセットする（GRID_RETURN に入ってもリセットしない＝戻りの間も進む）。
        """
        assert self._grid is not None
        self._grid_reset_settle(now)
        self._grid_approach_kmh = target_kmh if is_approach else None
        # 直前のステップのガバナー頭打ちを引き継がない（`_grid_start_step` と同じ流儀）
        self._accel_gov_cap = None
        self._brake_gov_cap = None
        switch = pattern.settings.grid_return_switch_kmh
        if self._grid_station_begun and abs(target_kmh - speed) > switch:
            self._phase = _Phase.GRID_RETURN
            self._phase_started_at = now
            self._gr_hist.clear()
            self._gr_last_t = None
            # 段7c: 開度の始点は従来どおりの当て推量（u0 + (狙い − a_hold) / g_accel。g_accel は
            # ステーション開始時に glimit から種をまいた値）。以降は _step_grid_return が実測で補正
            grid = self._grid
            u = grid.u0 + max(0.0, pattern.settings.grid_return_accel_kmhs - grid.a_hold) / (
                grid.g_accel
            )
            u = min(max(u, grid.u0), self._profile.max_accel_opening)
            cap = self._glimit_cap(PEDAL_ACCEL, speed)
            self._gr_opening = u if cap is None else min(u, cap)
            return
        self._grid_switch_to_cruise_hold()

    def _grid_switch_to_cruise_hold(self) -> None:
        """開度を定常開度 u0 に戻して PI（CRUISE_HOLD）へ（積分も u0。段差は PI のレート制限が
        ならす）。GRID_RETURN の出口・戻り不要のときの両方から呼ぶ（落ち着き待ちの打ち切り
        タイマーはここでは触らない。段7b）。
        """
        assert self._grid is not None
        self._grid_opening = self._grid.u0
        self._grid_integral = self._grid.u0
        self._phase = _Phase.CRUISE_HOLD

    def _advance_grid_return(self, pattern: GridStationPattern, speed: float, now: float) -> None:
        """GRID_RETURN（段7b・7c）: 開度を刻み足しながらステーションの目標へ戻る。近づいたら PI へ。

        落ち着き待ちの打ち切り（段7c）を CRUISE_HOLD と共通の `_grid_handle_settle_timeout` で
        判定する。以前は無く、釣り合い車速が目標に届かないと GRID_RETURN から永久に抜けなかった
        （20260928 実機で 190 s）。
        """
        assert self._grid is not None
        s = pattern.settings
        if speed > self._profile.max_speed:
            self._grid.abort_remaining()
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        target = self._grid_target_kmh(pattern)
        if abs(target - speed) <= s.grid_return_switch_kmh:
            self._grid_switch_to_cruise_hold()
            return
        if now - self._grid_settle_started >= s.settle_timeout_s:
            self._grid_handle_settle_timeout(pattern, speed, now)
            return
        if speed < target:
            self._step_grid_return(pattern, speed, now)
        else:
            self._gr_hist.clear()
            self._gr_last_t = None

    def _grid_station_done(self, speed: float, now: float) -> None:
        """ステーションが終わった。次もステーションなら止まらずに移り、最後なら停車復帰。"""
        nxt = self._pattern_idx + 1
        if nxt < len(self._patterns) and isinstance(self._patterns[nxt], GridStationPattern):
            self._pattern_idx = nxt
            self._enter_phase(_Phase.CRUISE_HOLD, now, keep_grid=True)
            return
        self._finish_pattern(speed, now)

    def _grid_launch_after_step(self, pattern: LearningPattern, speed: float, now: float) -> None:
        """発進・停車セルのステップの後。発進の後は止まらずに停車ステップへ、停車の後は停車復帰。"""
        assert self._grid is not None
        assert isinstance(pattern, GridLaunchPattern) and pattern.plan is not None
        while (kind := self._grid.peek_kind()) is not None:
            if kind is StepKind.STOP:
                if speed >= pattern.plan.end_kmh - 3.0:  # 発進の直後（走行中）だけ測れる
                    step = self._grid.next_step()
                    assert step is not None
                    self._grid_start_step(step, now)
                    return
                self._grid.drop_next()  # 発進が打ち切られて走っていない: 停車は測れない
                continue
            break
        if kind is None:
            self._finish_pattern(speed, now)
        elif speed > STOP_SPEED_KMH:
            self._enter_phase(_Phase.DRIVE_BRAKE, now)  # 停車復帰 → 次の発進（終わりは _advance）
        else:
            self._grid_launch_continue(now)

    def _grid_launch_continue(self, now: float) -> None:
        """停車した状態から次の発進ステップを始める（無ければパターンを終える）。"""
        assert self._grid is not None
        step = self._grid.next_step()
        if step is None:
            self._advance_pattern(now)
            return
        self._grid_start_step(step, now)

    def _finish_pattern(self, speed: float, now: float) -> None:
        """運転パターンの計測が終わった。停車していなければ停車復帰（DRIVE_BRAKE）に入る（A6）。"""
        if speed > STOP_SPEED_KMH:
            self._enter_phase(_Phase.DRIVE_BRAKE, now)
        else:
            self._advance_pattern(now)

    # ── 通し掃引（段6c） ─────────────────────────────────────────────
    def _sweep_log(self, text: str) -> None:
        if self._on_sweep_log is not None:
            self._on_sweep_log(text)

    def _sweep_enter_pattern(self, pattern: LearningPattern, now: float) -> bool:
        """通し掃引のパターンに入った最初の周期: 網羅の穴から掃引を作る。とばしたら True。"""
        if not isinstance(pattern, GridSweepPattern) or self._sw_seen_idx == self._pattern_idx:
            return False
        self._sw_seen_idx = self._pattern_idx
        cov = self.coverage
        if (
            cov is None or pattern.wltp_seconds is None or pattern.wltp_mean is None
            or pattern.max_passes <= 0
        ):
            self._sweep_log("通し掃引: 無効（何もしません）")
            self._advance_pattern(now)
            return True
        holes = cov.holes(pattern.wltp_seconds, pattern.min_s, pattern.data_max_s)
        self._sw_plans = plan_sweeps(
            holes, pattern.wltp_mean, pattern.speed_edges, pattern.accel_edges,
            self._config.g_cap_g * G_TO_KMHS, pattern.wltp_seconds,
        )
        demand = sum(float(pattern.wltp_seconds[c]) for c in holes)
        self._sweep_log(
            f"通し掃引: 穴 {len(holes)} セル（WLTP {demand:.0f}s）"
            f" → 掃引 {len(self._sw_plans)} 本（各 最大 {pattern.max_passes} 回）"
        )
        self._sw_passes = 0
        if not self._sweep_begin(pattern, now):
            self._advance_pattern(now)
            return True
        return False

    def _sweep_open_for(self, plan: SweepPlan) -> list[tuple[float, float]] | None:
        """掃引のセルごとの (アクセル, ブレーキ) 開度。狙いの加速度が出る開度を上限 G で頭打ち。

        どこか 1 つでも予測できない（G 校正・格子の実測が無い）セルが両端にあれば、そのセルを
        外す。途中のセルは隣の開度で埋める。全部予測できなければ None。
        """
        ff = self._profile.feedforward_params
        found: list[tuple[float, float] | None] = []
        for cell in plan.cells:
            v = 0.5 * (cell.speed_lo_kmh + cell.speed_hi_kmh)
            if plan.decel:
                u = self._glimit_open_for(PEDAL_BRAKE, v, -cell.a_kmhs)
                cap = self._glimit_cap(PEDAL_BRAKE, cell.speed_hi_kmh)  # 帯の中で一番効く端
                if u is not None and cap is not None:
                    u = min(u, cap)
                found.append(
                    None if u is None
                    else (0.0, min(max(u, ff.brake_deadband_pct), self._profile.max_brake_opening))
                )
            else:
                u = self._glimit_open_for(PEDAL_ACCEL, v, cell.a_kmhs)
                cap = self._glimit_cap(PEDAL_ACCEL, cell.speed_lo_kmh)
                if u is not None and cap is not None:
                    u = min(u, cap)
                found.append(
                    None if u is None
                    else (min(max(u, ff.accel_deadband_pct), self._profile.max_accel_opening), 0.0)
                )
        if all(f is None for f in found):
            return None
        last = None
        for k, f in enumerate(found):  # 途中の欠けは前のセルの開度で埋める（先頭の欠けは後ろから）
            if f is None:
                found[k] = last
            else:
                last = f
        nxt = None
        for k in range(len(found) - 1, -1, -1):
            if found[k] is None:
                found[k] = nxt
            else:
                nxt = found[k]
        return [f for f in found if f is not None]

    def _sweep_begin(self, pattern: GridSweepPattern, now: float) -> bool:
        """先頭の掃引の 1 回目を始める。始められる掃引が無ければ False。"""
        while self._sw_plans:
            plan = self._sw_plans[0]
            openings = self._sweep_open_for(plan)
            if openings is None:
                self._sweep_log(
                    f"  掃引 {self._sweep_name(plan)}: 開度を予測できないのでとばします"
                )
                self._sw_plans.pop(0)
                continue
            self._sw_openings = openings
            self._sw_passes = 0
            self._sweep_start_pass(pattern, now)
            return True
        return False

    @staticmethod
    def _sweep_name(plan: SweepPlan) -> str:
        a = [c.a_kmhs for c in plan.cells]
        return (
            f"{'減速' if plan.decel else '加速'} {min(a):+.1f}〜{max(a):+.1f} km/h/s、車速 "
            f"{plan.cells[0].speed_lo_kmh:g}〜{plan.cells[-1].speed_hi_kmh:g} km/h"
            if not plan.decel
            else f"減速 {min(a):+.1f}〜{max(a):+.1f} km/h/s、車速 "
            f"{plan.cells[-1].speed_lo_kmh:g}〜{plan.cells[0].speed_hi_kmh:g} km/h"
        )

    def _sweep_cell_seconds(self, plan: SweepPlan) -> float:
        cov = self.coverage
        assert cov is not None
        return float(sum(cov.seconds[c.i, c.j] for c in plan.cells))

    def _sweep_start_pass(self, pattern: GridSweepPattern, now: float) -> None:
        """掃引 1 回を始める: 開始車速まで G 比例で加速（停車からの加速掃引は直接）。"""
        plan = self._sw_plans[0]
        self._sw_before = self._sweep_cell_seconds(plan)
        if plan.start_kmh - 2.0 <= self._config.coast_down_stop_speed_kmh and not plan.decel:
            self._sweep_go(now)
        else:
            self._enter_phase(_Phase.SWEEP_UP, now)

    def _sweep_up_target_kmh(self, plan: SweepPlan) -> float:
        """助走の目標車速。減速は最初のセルの上端、加速は最初のセルの下端の少し手前。

        最高車速（加速の cap）を超えないようにする。
        """
        target = plan.start_kmh + 1.0 if plan.decel else plan.start_kmh - 2.0
        return min(target, self._accel_speed_cap - 2.0)

    def _advance_sweep_up(
        self, pattern: GridSweepPattern, speed: float, elapsed: float, now: float
    ) -> None:
        self._step_coast_accel(pattern, speed, now)  # G 比例の加速（上限 G の予測の点も足す）
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        target = self._sweep_up_target_kmh(self._sw_plans[0])
        if speed >= target:
            self._sweep_go(now)
        elif elapsed >= self._config.accel_full_range_timeout_s:
            self._sweep_log("  掃引: 開始車速まで上がらない（打ち切り）")
            self._enter_phase(_Phase.DRIVE_BRAKE, now)

    def _sweep_go(self, now: float) -> None:
        self._enter_phase(_Phase.SWEEP, now)
        self._sw_cell = -1
        self._sw_from = (self._current_accel_opening, self._current_brake_opening)
        self._sw_ramp_t = now
        self._sw_cmd = self._sw_from

    def _sweep_command(self, now: float) -> tuple[float, float]:
        """掃引中の指令。今の車速のセルの開度へ、セルが変わったら step_lag_s かけて移る。

        踏むペダルだけランプさせ、もう一方は即座に離す（HOLD_STEP と同じ）。
        """
        plan = self._sw_plans[0]
        cell = plan.cell_at(self._last_speed)
        k = plan.cells.index(cell)
        pattern = self._patterns[self._pattern_idx]
        assert isinstance(pattern, GridSweepPattern)
        if k != self._sw_cell:
            self._sw_cell = k
            self._sw_from = self._sw_cmd
            self._sw_ramp_t = now
        # 開度の表は掃引のセルと同じ順（_sweep_open_for が欠けを埋めて同じ長さにする）
        target_a, target_b = self._sw_openings[min(k, len(self._sw_openings) - 1)]
        frac = self._ramp_fraction(now - self._sw_ramp_t, pattern.settings.step_lag_s)
        accel, brake = target_a, target_b
        self._accel_request, self._brake_request = accel, brake
        if accel > 0.0:
            accel = self._sw_from[0] + (accel - self._sw_from[0]) * frac
        if brake > 0.0:
            brake = self._sw_from[1] + (brake - self._sw_from[1]) * frac
        if self._accel_gov_cap is not None:
            accel = min(accel, self._accel_gov_cap)
        if self._brake_gov_cap is not None:
            brake = min(brake, self._brake_gov_cap)
        accel = clamp_opening(accel, self._profile.max_accel_opening)
        brake = clamp_opening(brake, self._profile.max_brake_opening)
        self._sw_cmd = (accel, brake)
        return accel, brake

    def _advance_sweep(
        self, pattern: GridSweepPattern, speed: float, elapsed: float, now: float
    ) -> None:
        plan = self._sw_plans[0]
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        floor = max(plan.end_kmh, STOP_SPEED_KMH)
        done = speed <= floor if plan.decel else speed >= plan.end_kmh
        if done or elapsed >= cfg.coast_timeout_s:
            # 掃引が済んだ。停車してから次の回・次の掃引へ
            self._enter_phase(_Phase.DRIVE_BRAKE, now)

    def _sweep_pass_done(self, pattern: GridSweepPattern, now: float) -> None:
        """掃引 1 回が済んで停車した。対象セルに穴が残れば同じ掃引をもう 1 回、無ければ次へ。"""
        assert self.coverage is not None and pattern.wltp_seconds is not None
        plan = self._sw_plans[0]
        self._sw_passes += 1
        after = self._sweep_cell_seconds(plan)
        left = [
            c for c in plan.cells
            if pattern.wltp_seconds[c.i, c.j] >= pattern.min_s
            and self.coverage.seconds[c.i, c.j] < pattern.data_max_s
        ]
        self._sweep_log(
            f"  掃引 {self._sweep_name(plan)}: {self._sw_passes} 回目 "
            f"対象セルのデータ {self._sw_before:.1f}s → {after:.1f}s、残りの穴 {len(left)} セル"
        )
        if left and self._sw_passes < pattern.max_passes:
            openings = self._sweep_open_for(plan)  # 実測が増えたので開度を引き直す
            if openings is not None:
                self._sw_openings = openings
            self._sweep_start_pass(pattern, now)
            return
        self._sw_plans.pop(0)
        if not self._sweep_begin(pattern, now):
            self._advance_pattern(now)

    def glimit_table(self) -> list[str]:
        """車速帯ごとの上限開度の表（今の予測。ターミナル表示用）。"""
        ff = self._profile.feedforward_params
        return self.glimit.table(
            self._config.g_cap_g * G_TO_KMHS, ff.brake_deadband_pct, ff.accel_deadband_pct,
            self._profile.max_speed,
        )

    def _advance_pattern(self, now: float) -> None:
        if (
            self._on_calib_done is not None
            and self._pattern_idx < len(self._patterns)
            and self._patterns[self._pattern_idx].kind is PatternKind.G_CALIB
        ):
            self._on_calib_done(self.glimit_table())
        self._pattern_idx += 1
        self._enter_phase(self._initial_phase(self._pattern_idx), now)

    def _enter_phase(
        self, phase: _Phase, now: float, *, overspeed_recovery: bool = False,
        keep_grid: bool = False,
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
        self._ca_opening_hist.clear()  # 段7d: 前のパターンの開度を次の G 校正に持ち込まない
        self._cb_t0 = None
        self._cb_next_t = None
        self._cb_hist.clear()
        # 停車復帰は、低速（クリープ域）からでなければ G 比例で減速する（段6b）。一気に停車保持
        # 開度へ踏むと、高い車速ではブレーキがよく効いて上限 G を超える（20260926: 134 km/h で
        # 0.30〜0.35G）
        self._db_gprop = (
            phase is _Phase.DRIVE_BRAKE
            and self._last_speed > self._config.coast_down_stop_speed_kmh
        )
        self._overspeed_recovery = overspeed_recovery
        self._brake_ramp_from = self._current_brake_opening
        if not keep_grid:  # ステーション間の移動以外は、格子の PI 状態を作り直す
            self._grid_opening = None
            self._grid_integral = 0.0
            self._grid_step = None
            self._grid_approach_kmh = None

    def _update_governor(self, accel_kmhs: float) -> None:
        """G ガバナー（段6b で先読みつきに）。CRUISE_HOLD（格子の PI 保持）には適用しない: PI 自体が
        低ゲイン・レート制限で穏やかなので、頭打ちの必要が無いと判断した。HOLD_STEP・GRID_RETURN
        （段7b。固定開度で速く戻る）には適用する。
        """
        if self._g_limit_kmhs <= 0.0:
            return
        rate = self._accel_rate
        if self._phase in (_Phase.DRIVE_ACCEL, _Phase.SWEEP_UP):
            self._accel_gov_cap = self._next_gov_cap(
                self._accel_gov_cap, accel_kmhs, rate, self._current_accel_opening,
                self._accel_request,
            )
        elif self._phase in (
            _Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD, _Phase.CALIB_BRAKE
        ):
            self._brake_gov_cap = self._next_gov_cap(
                self._brake_gov_cap, -accel_kmhs, -rate, self._current_brake_opening,
                self._brake_request,
            )
        elif self._phase in (_Phase.HOLD_STEP, _Phase.SWEEP, _Phase.GRID_RETURN):
            accel_on, brake_on = self._governed_step_pedals()
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

    def _governed_step_pedals(self) -> tuple[bool, bool]:
        """開度固定の区間（HOLD_STEP・SWEEP・GRID_RETURN）で今使っているペダル
        (アクセル, ブレーキ)。
        """
        if self._phase is _Phase.HOLD_STEP:
            step = self._grid_step
            if step is None:
                return False, False
            return step.accel_pct > 0.0, step.brake_pct > 0.0
        if self._phase is _Phase.GRID_RETURN:  # ブレーキは使わない（惰行が頭打ち）
            return self._accel_request > 0.0, False
        return self._accel_request > 0.0, self._brake_request > 0.0

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
        if self._phase in (_Phase.DRIVE_ACCEL, _Phase.SWEEP_UP) and self._accel_gov_cap is not None:
            return self._accel_gov_cap < self._accel_request
        if self._phase in (_Phase.HOLD_STEP, _Phase.SWEEP, _Phase.GRID_RETURN):
            accel_limited = (
                self._accel_gov_cap is not None and self._accel_gov_cap < self._accel_request
            )
            brake_limited = (
                self._brake_gov_cap is not None and self._brake_gov_cap < self._brake_request
            )
            return accel_limited or brake_limited
        braking = self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD, _Phase.CALIB_BRAKE)
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
            _Phase.DRIVE_ACCEL, _Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD, _Phase.CRUISE_HOLD,
            _Phase.HOLD_STEP, _Phase.CALIB_BRAKE, _Phase.SWEEP_UP, _Phase.SWEEP,
            _Phase.GRID_RETURN,
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
    "GridLaunchPattern",
    "GridStationPattern",
    "PatternLoopConfig",
]
