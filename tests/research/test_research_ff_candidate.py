"""改善案 C1（ff_candidate.py）のユニットテスト。

A1（学習行の選別）はそのペダルが効いている行だけを使うこと、B1〜B3（レジーム合成）は
停車保持・クリープ任せを保ったまま惰行カーブでペダルを選び、不感帯以上へ切り上げること、
そして `kaizen.decide_openings`（レポート 3 章の C1 と同じ判定）と一致することを確かめる。
"""

from __future__ import annotations

import pickle
from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np
import pytest

from src.domain.control.feedforward import FeedforwardController
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import DEFAULT_FEATURE_SPEC, STOP_SPEED_KMH
from src.models.drive_log import DriveLog
from src.models.profile import (
    FeedforwardParams,
    PIDGains,
    StopConfig,
    VehicleProfile,
    coast_decel_at,
)
from tests.research.cruise_curve import CruiseCurve
from tests.research.ff_candidate import (
    CANDIDATE_CLASSES,
    TRAINING_ROWS_EFFECTIVE,
    CandidateC2,
    CandidateC3,
    CandidateC4,
    CandidateC5,
    CandidateC6,
    CandidateFeedforward,
    cruise_skeleton,
    make_candidate,
    train_inverse_model_effective,
)
from tests.research.ff_params import ResearchFFParams, creep_accel_at
from tests.research.reachability import free_speeds_at

SPEEDS = (5.0, 15.0, 25.0, 35.0, 45.0, 55.0, 65.0, 75.0, 85.0, 95.0, 105.0, 115.0, 125.0, 135.0)
COAST = (1.6, 1.73, 2.58, 3.195, 3.575, 3.57, 3.11, 2.54, 1.83, 1.6, 1.6, 1.6, 1.6, 1.6)
PARAMS = FeedforwardParams(
    creep_speed_kmh=4.944,
    creep_rate_kmhs=0.155,
    stop_brake_opening_pct=30.53,
    coast_decel_speeds_kmh=SPEEDS,
    coast_decel_kmhs=COAST,
    accel_deadband_pct=10.0,
    brake_deadband_pct=13.68,
)
SPEC = DEFAULT_FEATURE_SPEC


class _Const:
    """与えた定数を返すだけの推定器（predict_effort の分岐だけを見るため）。"""

    def __init__(self, value: float) -> None:
        self.value = value

    def predict(self, x: np.ndarray) -> np.ndarray:
        return np.full(len(x), self.value, dtype=float)


def _ff(accel_pred: float, brake_pred: float, clip: float | None = None) -> CandidateFeedforward:
    ff = CandidateFeedforward()
    ff.set_params(PARAMS)
    ff._accel_model = _Const(accel_pred)  # noqa: SLF001 - テスト用の差し込み
    ff._brake_model = _Const(brake_pred)  # noqa: SLF001
    ff._speed_clip_max = clip  # noqa: SLF001
    return ff


def _points(v0: float, a: float) -> tuple[list[float], list[float]]:
    """一定加速度 a [km/h/s] の軌跡（先読み・過去）。"""
    return (
        [v0 + a * h for h in SPEC.lookahead_horizons_s],
        [v0 - a * h for h in SPEC.past_horizons_s],
    )


# ── B1〜B3: レジーム合成 ────────────────────────────────────────────


def test_unloaded_model_raises() -> None:
    ff = CandidateFeedforward()
    ff.set_params(PARAMS)
    with pytest.raises(RuntimeError):
        ff.predict_effort(50.0, *_points(50.0, 0.0))


def test_stop_regime_holds_brake() -> None:
    """停車判定は現行のまま（停車保持ブレーキ開度をそのまま出す）。"""
    ff = _ff(20.0, 20.0)
    future, past = _points(0.0, 0.0)
    assert ff.predict_effort(0.0, future, past) == pytest.approx(-PARAMS.stop_brake_opening_pct)


def test_creep_partial_demand_without_band_selects_brake() -> None:
    """段2: 旧「クリープ任せ」の固定窓（[0, creep_rate_kmhs]）は廃止し惰行帯に一本化した。

    `set_research_params` を呼ばない（帯幅 0.0）と、クリープ加速度ちょうど未満の要求はもう
    無条件では解放されず、B1 の閾値どおりブレーキ側になる（惰行域は
    `test_coast_band_in_creep_region_beyond_old_window` が別途確かめる）。
    """
    ff = _ff(20.0, 20.0)
    v0 = PARAMS.creep_speed_kmh - 0.5
    future, past = _points(v0, PARAMS.creep_rate_kmhs * 0.5)
    assert v0 > STOP_SPEED_KMH
    assert ff.predict_effort(v0, future, past) == pytest.approx(-20.0)


def test_b1_mild_decel_above_coast_selects_accel() -> None:
    """B1: 惰行より緩い減速はアクセル側（現行は dv_1.0 が負なのでブレーキ側に落ちていた）。"""
    ff = _ff(16.0, 20.0)
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)  # 約 -3.3 km/h/s
    future, past = _points(v0, coast / 2.0)  # 惰行の半分の減速 = 惰行より緩い
    assert ff.predict_effort(v0, future, past) == pytest.approx(16.0)


def test_b1_decel_stronger_than_coast_selects_brake() -> None:
    ff = _ff(16.0, 20.0)
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 1.5)  # 惰行より強い減速
    assert ff.predict_effort(v0, future, past) == pytest.approx(-20.0)


def test_b2_no_coast_taper() -> None:
    """B2: 惰行テーパを廃止したので、緩減速でもアクセル予測が絞られない。"""
    v0, accel_pred = 60.0, 16.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 0.9)  # 現行なら accel_pred × 0.1 まで絞られる領域
    assert _ff(accel_pred, 20.0).predict_effort(v0, future, past) == pytest.approx(accel_pred)
    # 現行（FeedforwardController）はここでテーパがかかることを対照として確かめる
    current = FeedforwardController()
    current.set_params(PARAMS)
    current._accel_model = _Const(accel_pred)  # noqa: SLF001
    current._brake_model = _Const(20.0)  # noqa: SLF001
    assert current.predict_effort(v0, future, past) == pytest.approx(accel_pred * 0.1, abs=0.2)


@pytest.mark.parametrize(
    ("accel_pred", "brake_pred", "a", "expected"),
    [
        (2.0, 0.0, 1.0, PARAMS.accel_deadband_pct),  # 不感帯未満のアクセル予測 → 不感帯へ
        (0.0, 2.0, -8.0, -PARAMS.brake_deadband_pct),  # 不感帯未満のブレーキ予測 → 不感帯へ
        (24.0, 0.0, 1.0, 24.0),  # 不感帯以上はそのまま
        (0.0, 30.0, -8.0, -30.0),
    ],
)
def test_b3_rounds_up_to_deadband(
    accel_pred: float, brake_pred: float, a: float, expected: float
) -> None:
    ff = _ff(accel_pred, brake_pred)
    assert ff.predict_effort(60.0, *_points(60.0, a)) == pytest.approx(expected)


def test_negative_prediction_is_clamped_then_rounded_up() -> None:
    """モデルが負を返しても 0 クランプ後に不感帯へ切り上げる（効かない指令を出さない）。"""
    assert _ff(-5.0, 0.0).predict_effort(60.0, *_points(60.0, 1.0)) == pytest.approx(
        PARAMS.accel_deadband_pct
    )


def test_speed_clip_shifts_trajectory() -> None:
    """学習域クリップは現行のまま（v0 を上限へ置き、先読み/過去を平行移動して dv を保つ）。"""
    ff = _ff(30.0, 40.0, clip=100.0)
    future, past = _points(130.0, 2.0)
    assert ff.predict_effort(130.0, future, past) == pytest.approx(30.0)  # 加速要求は保たれる


# ── 段2: 惰行帯（coast_band_kmhs） ────────────────────────────────────


def _ff_with_research(
    accel_pred: float, brake_pred: float, research: ResearchFFParams
) -> CandidateFeedforward:
    ff = _ff(accel_pred, brake_pred)
    ff.set_research_params(research)
    return ff


def test_coast_band_center_releases_both_pedals() -> None:
    """帯の中（要求 = 惰行の加速度ちょうど）はどちらのペダルも使わない。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    ff = _ff_with_research(16.0, 20.0, ResearchFFParams(coast_band_kmhs=0.5))
    future, past = _points(v0, coast)
    assert ff.predict_effort(v0, future, past) == 0.0


def test_coast_band_boundary_is_outside_band() -> None:
    """差が帯幅 ε 以上は帯の外（`<` なので境界は含まない）→ アクセル側になる。

    差をちょうど ε にすると `_points`/`build_feature_row` を往復する丸め誤差で ε をわずかに
    下回ることがあるため（浮動小数点）、境界のすぐ外側で確かめる。
    """
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    epsilon = 0.5
    ff = _ff_with_research(16.0, 20.0, ResearchFFParams(coast_band_kmhs=epsilon))
    future, past = _points(v0, coast + epsilon + 1e-6)
    assert ff.predict_effort(v0, future, past) == pytest.approx(16.0)


def test_coast_band_outside_accel_side() -> None:
    """帯の外・加速側は正（アクセル）、かつ不感帯以上。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    ff = _ff_with_research(16.0, 20.0, ResearchFFParams(coast_band_kmhs=0.5))
    future, past = _points(v0, coast + 1.0)
    effort = ff.predict_effort(v0, future, past)
    assert effort > 0.0
    assert effort >= PARAMS.accel_deadband_pct


def test_coast_band_outside_brake_side() -> None:
    """帯の外・減速側は負（ブレーキ）、かつ不感帯以上（絶対値）。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    ff = _ff_with_research(16.0, 20.0, ResearchFFParams(coast_band_kmhs=0.5))
    future, past = _points(v0, coast - 1.0)
    effort = ff.predict_effort(v0, future, past)
    assert effort < 0.0
    assert effort <= -PARAMS.brake_deadband_pct


def test_coast_band_in_creep_region_beyond_old_window() -> None:
    """クリープ域: 実測クリープ加速カーブの ±帯 内は惰行（旧「クリープ任せ」窓 0.23 の外でも）。"""
    research = ResearchFFParams(
        creep_accel_speeds_kmh=(0.5, 1.5, 2.5, 3.5, 4.5),
        creep_accel_kmhs=(1.7, 1.3, 0.91, 0.52, 0.1),
        coast_band_kmhs=0.5,
    )
    v0 = 1.0  # PARAMS.creep_speed_kmh(4.944) 未満
    creep_a = creep_accel_at(research, v0)
    assert creep_a is not None  # 同定済み（creep_accel_speeds_kmh/kmhs あり）なので None にならない
    assert creep_a > PARAMS.creep_rate_kmhs + 0.23  # 旧窓 [0, creep_rate_kmhs] の外
    ff = _ff_with_research(16.0, 20.0, research)
    future, past = _points(v0, creep_a)
    assert ff.predict_effort(v0, future, past) == 0.0


def test_no_research_params_matches_pre_stage2_pedal_choice() -> None:
    """後方互換: `set_research_params` を呼ばない候補は、段1前と同じペダル選択を返す。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    ff = _ff(16.0, 20.0)  # set_research_params は呼ばない（coast_band_kmhs は既定 0.0）
    future, past = _points(v0, coast * 1.5)  # 惰行より強い減速 → ブレーキ側
    assert ff.predict_effort(v0, future, past) == pytest.approx(-20.0)
    future, past = _points(v0, coast * 0.5)  # 惰行より緩い減速 → アクセル側
    assert ff.predict_effort(v0, future, past) == pytest.approx(16.0)


def test_stop_regime_unaffected_by_coast_band() -> None:
    """停車保持は惰行帯があっても変わらない（帯より優先される）。"""
    ff = _ff_with_research(20.0, 20.0, ResearchFFParams(coast_band_kmhs=0.5))
    future, past = _points(0.0, 0.0)
    assert ff.predict_effort(0.0, future, past) == pytest.approx(-PARAMS.stop_brake_opening_pct)


# ── 段3: 到達可能性判定（reach_horizons_s） ──────────────────────────


def test_reach_horizons_empty_matches_pre_stage3_behavior() -> None:
    """回帰: `reach_horizons_s=()`（既定）なら段3 導入前と全く同じ経路を通る（3 経路）。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    research = ResearchFFParams(coast_band_kmhs=0.5, reach_horizons_s=())

    # 惰行帯の中
    ff = _ff_with_research(16.0, 20.0, research)
    future, past = _points(v0, coast)
    assert ff.predict_effort(v0, future, past) == 0.0

    # アクセル側
    ff = _ff_with_research(16.0, 20.0, research)
    future, past = _points(v0, coast + 1.0)
    assert ff.predict_effort(v0, future, past) == pytest.approx(16.0)

    # ブレーキ側
    ff = _ff_with_research(16.0, 20.0, research)
    future, past = _points(v0, coast - 1.0)
    assert ff.predict_effort(v0, future, past) == pytest.approx(-20.0)


def test_reach_horizons_pure_coast_trajectory_selects_coast() -> None:
    """段3: 惰行のまま進んだ先の速度そのものを軌跡にすると、必ず惰行（0.0）になる。

    v0 をクリープ平衡点（creep_speed_kmh=4.944）のすぐ上に取ると、惰行の加速度は 1 秒の間に
    大きく変わる（惰行域 → クリープ域）。旧来の 1.0s 1 点近似（`reach_horizons_s=()`）は
    coast を 1 秒一定とみなすため誤差が大きく、同じ軌跡でも帯の外と誤判定してアクセル側を
    選んでしまう。これが段3 の差が実際に出る例。
    """
    v0 = 5.0
    research_on = ResearchFFParams(
        coast_band_kmhs=0.5, reach_horizons_s=(0.5, 1.0, 2.0), reach_step_s=0.05
    )
    future = list(
        free_speeds_at(
            PARAMS, research_on, v0, SPEC.lookahead_horizons_s, step_s=research_on.reach_step_s
        )
    )
    past = [v0, v0]

    ff_on = _ff_with_research(16.0, 20.0, research_on)
    assert ff_on.predict_effort(v0, future, past) == 0.0

    # 同じ future/past でも段3 無効（1.0s 1 点近似）だと帯の外に誤判定し、アクセル側を選ぶ
    research_off = ResearchFFParams(coast_band_kmhs=0.5, reach_horizons_s=())
    ff_off = _ff_with_research(16.0, 20.0, research_off)
    effort_off = ff_off.predict_effort(v0, future, past)
    assert effort_off == pytest.approx(16.0)  # 段3 なら 0.0 のところ、誤ってアクセル側になる


def test_reach_horizons_shortest_horizon_wins_over_opposite_sign() -> None:
    """段3: 遠いホライズンが逆符号でも、帯の外に出た最短ホライズンでペダルを選ぶ。"""
    v0 = 60.0
    hs = (0.5, 1.0, 2.0)
    research = ResearchFFParams(coast_band_kmhs=0.5, reach_horizons_s=hs, reach_step_s=0.05)
    v_free = free_speeds_at(PARAMS, research, v0, hs, step_s=research.reach_step_s)

    # 最短(0.5s)はアクセル側で帯の外、中間(1.0s)は帯の中、最長(2.0s)はブレーキ側で帯の外
    # （符号が逆）。最短が勝ってアクセル側になることを確かめる。
    future = [
        v_free[0] + 2.0 * hs[0],  # need(0.5s) = +2.0（帯の外・アクセル側）
        v_free[1] + 0.0 * hs[1],  # need(1.0s) = 0.0（帯の中）
        v_free[2] - 2.0 * hs[2],  # need(2.0s) = -2.0（帯の外・ブレーキ側）
        v0,  # 3.0s（reach_horizons_s に含まれないため未使用）
    ]
    past = [v0, v0]

    ff = _ff_with_research(16.0, 20.0, research)
    assert ff.predict_effort(v0, future, past) == pytest.approx(16.0)


# ── 段4改訂: クリープ域ブレーキの下限（brake_trim_max_kmh・stop_brake_floor_offset_pct） ──

# 手描きの下限（推定器のテストは test_research_stop_brake_floor.py 側）。
# PARAMS.brake_deadband_pct=13.68 + offset_pct=11.32 → floor=25.0（丸め数で検算しやすくする）
STOP_BRAKE_FLOOR: dict[str, float] = {"stop_brake_floor_offset_pct": 11.32}
FLOOR_PCT = PARAMS.brake_deadband_pct + STOP_BRAKE_FLOOR["stop_brake_floor_offset_pct"]


def test_brake_trim_disabled_matches_pre_stage4_effort_on_all_brake_paths() -> None:
    """回帰: `brake_trim_max_kmh=0.0`（既定）ならブレーキの3経路（C1モデル・C2解析式・
    C2フォールバック）とも段4 導入前と同じ effort になる。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 1.5)  # 惰行より強い減速 → ブレーキ側
    research = ResearchFFParams(brake_trim_max_kmh=0.0, **STOP_BRAKE_FLOOR)

    # 経路1: C1（モデル予測）
    ff = _ff_with_research(16.0, 20.0, research)
    assert ff.predict_effort(v0, future, past) == pytest.approx(-20.0)

    # 経路2: C2（解析式・ゲイン同定済み）
    coast_gain = -coast_decel_at(PARAMS_WITH_GAIN, v0)
    future_g, past_g = _points(v0, coast_gain * 1.5)
    c2 = _c2(16.0, 999.0, PARAMS_WITH_GAIN)
    c2.set_research_params(research)
    delta_a = coast_gain * 1.5 - coast_gain
    expected = -(-delta_a / 0.4 + PARAMS_WITH_GAIN.brake_deadband_pct)
    assert c2.predict_effort(v0, future_g, past_g) == pytest.approx(expected)

    # 経路3: C2 のフォールバック（ゲイン未同定＝PARAMS）
    c2_fallback = _c2(16.0, 20.0, PARAMS)
    c2_fallback.set_research_params(research)
    assert c2_fallback.predict_effort(v0, future, past) == pytest.approx(-20.0)


def test_brake_trim_floor_raises_opening_when_approaching_stop() -> None:
    """停止接近（ref_next=0.04・v0=1.15）でブレーキ開度が floor（不感帯 + offset）以上に
    切り上がる（17% 台で浮いていた領域を実測の停止境界まで引き上げる段4改訂 の下限）。"""
    v0 = 1.15
    a = -3.0  # 強い減速要求（ブレーキ側を選ばせる）
    future, past = _points(v0, a)
    future[0] = 0.04  # ref_next（最短ホライズン先の基準）を要求値どおりに固定する
    assert future[0] <= 0.3  # brake_trim_ref_kmh（既定 0.3）以下＝下限が効く条件
    research = ResearchFFParams(brake_trim_max_kmh=5.0, **STOP_BRAKE_FLOOR)
    ff = _ff_with_research(0.0, 5.0, research)  # brake_pred=5.0（不感帯未満・floor より低い）

    effort = ff.predict_effort(v0, future, past)

    assert effort == pytest.approx(-FLOOR_PCT)


def test_brake_trim_floor_applies_on_launch_too_same_rule_as_stop() -> None:
    """発進（ref_next=0.0・v0=0.0）でも同じ下限が掛かる（決定2: 停止側と発進側は同じ規則。
    旧仕様の `ref_far` による方向の場合分けは不要になった）。

    `predict_effort` は v0<=STOP_SPEED_KMH かつ future[0]<=STOP_SPEED_KMH を停車レジームとして
    先に処理してしまう（このトリムに到達する前に -stop_brake_opening_pct を返して終わる）ため、
    ここではトリム本体 `_apply_brake_trim` を直接呼んで境界値（v0=0.0・ref_next=0.0）を確かめる
    （停車レジームを抜けた直後の 1 周期を想定した境界値の直接検証）。
    """
    research = ResearchFFParams(brake_trim_max_kmh=5.0, **STOP_BRAKE_FLOOR)
    ff = _ff_with_research(0.0, 0.0, research)
    trimmed = ff._apply_brake_trim(5.0, 0.0, ref_next=0.0)  # noqa: SLF001 - 境界値の直接検証
    # 停止接近（test_brake_trim_floor_raises_opening_when_approaching_stop）と同じ floor
    assert trimmed == pytest.approx(FLOOR_PCT)


def test_brake_trim_releases_when_ref_next_moves_past_threshold() -> None:
    """`ref_next > brake_trim_ref_kmh` なら下限は掛からない（基準が動き出したら解放する）。"""
    research = ResearchFFParams(brake_trim_max_kmh=5.0, brake_trim_ref_kmh=0.3, **STOP_BRAKE_FLOOR)
    ff = _ff_with_research(0.0, 0.0, research)
    trimmed = ff._apply_brake_trim(5.0, 0.0, ref_next=0.31)  # noqa: SLF001 - 閾値の直接検証
    assert trimmed == pytest.approx(5.0)  # floor(20.0) より低いままで変わらない


def test_brake_trim_no_effect_when_v0_above_max() -> None:
    """`v0 > brake_trim_max_kmh` なら下限は掛からない。"""
    research = ResearchFFParams(brake_trim_max_kmh=5.0, **STOP_BRAKE_FLOOR)
    ff = _ff_with_research(0.0, 0.0, research)
    trimmed = ff._apply_brake_trim(5.0, 5.01, ref_next=0.0)  # noqa: SLF001 - 境界値の直接検証
    assert trimmed == pytest.approx(5.0)


def test_brake_trim_no_effect_when_offset_unidentified_even_if_positive() -> None:
    """`stop_brake_floor_offset_pct == 0.0`（未同定）なら `brake_trim_max_kmh` が正でも
    現行どおり。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 1.5)
    research = ResearchFFParams(brake_trim_max_kmh=100.0, stop_brake_floor_offset_pct=0.0)
    ff = _ff_with_research(16.0, 20.0, research)
    assert ff.predict_effort(v0, future, past) == pytest.approx(-20.0)


def test_brake_trim_does_not_lower_opening_already_deeper_than_floor() -> None:
    """下限は FF の開度が既に floor より深いときは何もしない（`max` なので下げない）。"""
    research = ResearchFFParams(brake_trim_max_kmh=5.0, **STOP_BRAKE_FLOOR)
    ff = _ff_with_research(0.0, 0.0, research)
    trimmed = ff._apply_brake_trim(30.0, 0.0, ref_next=0.0)  # noqa: SLF001 - floor(20.0) より深い
    assert trimmed == pytest.approx(30.0)


def test_c2_brake_trim_applies_on_top_of_analytic_formula() -> None:
    """C2 でも同じトリムが通る（解析式の結果を _apply_brake_trim に通す）。"""
    v0 = 1.0
    a = -3.0  # future[0](ref_next) が 0.3 以下 → 下限（floor）が効く
    future, past = _points(v0, a)
    assert future[0] <= 0.3
    research = ResearchFFParams(brake_trim_max_kmh=5.0, **STOP_BRAKE_FLOOR)
    c2 = _c2(0.0, 0.0, PARAMS_WITH_GAIN)  # brake_pred は使われない（解析式の分岐）
    c2.set_research_params(research)

    effort = c2.predict_effort(v0, future, past)

    coast = PARAMS_WITH_GAIN.creep_rate_kmhs  # v0 < creep_speed_kmh のクリープ側フォールバック
    delta_a = a - coast
    pre_trim = -delta_a / 0.4 + PARAMS_WITH_GAIN.brake_deadband_pct
    floor = PARAMS_WITH_GAIN.brake_deadband_pct + research.stop_brake_floor_offset_pct
    assert pre_trim < floor  # トリムが実際に効く前提
    assert effort == pytest.approx(-floor)


# ── kaizen.decide_openings（レポート 3 章の C1）との一致 ──────────────


def test_matches_kaizen_decide_openings() -> None:
    """レポート 3 章で比べた C1 の判定と、走行に使う predict_effort が一致すること。

    段2で `predict_effort` は旧「クリープ任せ」の固定窓（[0, creep_rate_kmhs]）を惰行帯
    （`coast_band_kmhs`）に一本化した（`ff_candidate.CandidateFeedforward` docstring 参照）。
    `kaizen.decide_openings` はレポート 3 章の C1 をそのまま再現するオフライン参照実装で、
    固定窓のまま変更していないため、`set_research_params` を呼ばない（帯幅 0.0）比較では
    creep_speed_kmh 未満の v0 だけ両者が食い違いうる。ここでは creep_speed_kmh 以上（惰行域）
    に絞って比較する。
    """
    from tests.research.config import load_config  # noqa: PLC0415 - 設定は重いので中で読む
    from tests.research.kaizen import decide_openings  # noqa: PLC0415

    cfg = load_config(Path("tests/research/config_testVehicle.yaml"))
    accel_pred, brake_pred = 16.0, 20.0
    ff = _ff(accel_pred, brake_pred)

    assert PARAMS.creep_speed_kmh < 6.0  # v0s の下限がクリープ域の外であることの前提
    v0s = np.array([6.0, 20.0, 60.0, 100.0, 130.0])
    accels = np.array([-8.0, -3.0, -1.0, -0.1, 0.0, 0.1, 1.0, 3.0])
    rows = [(float(v), float(a)) for v in v0s for a in accels]

    v0_arr = np.array([v for v, _ in rows])
    a_arr = np.array([a for _, a in rows])
    near = np.array([v + a * SPEC.lookahead_horizons_s[0] for v, a in rows])
    accel_ref, brake_ref = decide_openings(
        PARAMS, cfg, v0_arr, near, v0_arr, a_arr,
        np.full(len(rows), accel_pred), np.full(len(rows), brake_pred),
    )
    for i, (v, a) in enumerate(rows):
        effort = ff.predict_effort(v, *_points(v, a))
        got_accel = max(0.0, effort)
        got_brake = max(0.0, -effort)
        assert got_accel == pytest.approx(accel_ref[i]), f"v0={v} a={a}"
        assert got_brake == pytest.approx(brake_ref[i]), f"v0={v} a={a}"


# ── A1: 学習行の選別 ───────────────────────────────────────────────


def _logs(rows: list[tuple[float, float, float]]) -> list[DriveLog]:
    """(車速, アクセル開度, ブレーキ開度) の列から 0.1s 刻みのログを作る。"""
    t0 = datetime(2026, 9, 12, tzinfo=UTC)
    return [
        DriveLog(
            id=i, session_id="s1", timestamp=t0 + timedelta(seconds=0.1 * i),
            ref_speed_kmh=None, actual_speed_kmh=v,
            accel_opening=a, brake_opening=b,
            accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
        )
        for i, (v, a, b) in enumerate(rows)
    ]


def _profile() -> VehicleProfile:
    now = datetime(2026, 9, 12, tzinfo=UTC)
    return VehicleProfile(
        id="unit_test", name="unit_test", max_speed=140.0, max_decel_g=0.4,
        max_accel_opening=80.0, max_brake_opening=80.0,
        pid_gains=PIDGains(kp=0.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=9999.0),
        calibration=None, model_path=None,
        created_at=now, updated_at=now, feedforward_params=PARAMS,
    )


def _mixed_rows(n: int = 300) -> list[tuple[float, float, float]]:
    """加速（アクセル 20%）・惰行（両方 0）・減速（ブレーキ 20%）を混ぜた合成走行。"""
    rows: list[tuple[float, float, float]] = []
    v = 5.0
    for i in range(n):
        if i % 3 == 0:
            v += 0.5
            rows.append((v, 20.0, 0.0))
        elif i % 3 == 1:
            v = max(1.0, v - 0.2)
            rows.append((v, 0.0, 0.0))  # 惰行（どちらのモデルにも入らない）
        else:
            v = max(1.0, v - 0.5)
            rows.append((v, 0.0, 20.0))
    return rows


def test_a1_uses_only_effective_rows(tmp_path: Path) -> None:
    logs = _logs(_mixed_rows())
    path, metrics = train_inverse_model_effective(logs, _profile(), output_dir=str(tmp_path))
    assert Path(path).exists()
    # 惰行の行（開度 0）はどちらのモデルにも入らない = 全行の 1/3 ずつが上限
    assert 0 < metrics["accel"]["n"] <= len(logs) / 3 + 1
    assert 0 < metrics["brake"]["n"] <= len(logs) / 3 + 1
    assert "below_deadband" in metrics["accel"]
    assert "below_deadband" in metrics["brake"]


def test_a1_model_loads_into_feedforward(tmp_path: Path) -> None:
    """pkl の形式は本番と同じ（FeedforwardController.load_model がそのまま読める）。"""
    path, _ = train_inverse_model_effective(_logs(_mixed_rows()), _profile(), str(tmp_path))
    ff = CandidateFeedforward()
    ff.set_params(PARAMS)
    ff.load_model(path)
    assert ff.has_model
    with Path(path).open("rb") as f:
        payload = pickle.load(f)  # noqa: S301 - テストで作った自前のファイル
    assert payload["training_rows"] == TRAINING_ROWS_EFFECTIVE
    assert payload["deadbands_pct"] == {"accel": 10.0, "brake": 13.68}


def test_a1_raises_when_a_pedal_has_no_effective_rows(tmp_path: Path) -> None:
    """ブレーキが不感帯を超えない走行では、ブレーキモデルが作れないことを明示する。"""
    rows = [(5.0 + 0.1 * i, 20.0, 5.0) for i in range(300)]  # ブレーキは常に不感帯未満
    with pytest.raises(LearningDataError, match="ブレーキ"):
        train_inverse_model_effective(_logs(rows), _profile(), output_dir=str(tmp_path))


def test_pkl_filename_stamp_is_local_time(tmp_path: Path) -> None:
    """pkl ファイル名のスタンプは走行ログ CSV と同じローカル時刻（JST）であること。

    UTC 命名に戻ると 9 時間ずれるため、この assert が落ちて検知できる。
    """
    path, _ = train_inverse_model_effective(_logs(_mixed_rows()), _profile(), str(tmp_path))
    stamp = Path(path).stem.rsplit("_", 2)[-2:]
    trained_at = datetime.strptime("_".join(stamp), "%Y%m%d_%H%M%S")  # noqa: DTZ007 - naive比較
    assert abs(trained_at - datetime.now()) < timedelta(minutes=5)


# ── V1: C2〜C5（KAIZEN 報告書 3 章 表 3-1） ─────────────────────────


GAIN_SPEEDS = (5.0, 60.0, 130.0)
ACCEL_GAIN = (0.5, 0.5, 0.5)
BRAKE_GAIN = (0.4, 0.4, 0.4)
PARAMS_WITH_GAIN = FeedforwardParams(
    creep_speed_kmh=PARAMS.creep_speed_kmh,
    creep_rate_kmhs=PARAMS.creep_rate_kmhs,
    stop_brake_opening_pct=PARAMS.stop_brake_opening_pct,
    coast_decel_speeds_kmh=SPEEDS,
    coast_decel_kmhs=COAST,
    accel_deadband_pct=10.0,
    brake_deadband_pct=13.68,
    pedal_gain_speeds_kmh=GAIN_SPEEDS,
    accel_gain_kmhs_per_pct=ACCEL_GAIN,
    brake_gain_kmhs_per_pct=BRAKE_GAIN,
)


def test_candidate_classes_registered_with_matching_names() -> None:
    for name, cls in CANDIDATE_CLASSES.items():
        assert cls().candidate == name
    assert set(CANDIDATE_CLASSES) == {"C1", "C2", "C3", "C4", "C5", "C6"}


def test_make_candidate_unknown_name_raises() -> None:
    with pytest.raises(ValueError, match="C9"):
        make_candidate("C9")


def test_c3_is_logic_identical_to_c1_except_name() -> None:
    """C3 はロジックが C1 と同一（ずれはモデルの horizons で表現するので候補名だけ違う）。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 1.5)  # 惰行より強い減速 → ブレーキ側

    c1 = _ff(16.0, 20.0)
    c3 = CandidateC3()
    c3.set_params(PARAMS)
    c3._accel_model = c1._accel_model  # noqa: SLF001
    c3._brake_model = c1._brake_model  # noqa: SLF001

    assert c3.candidate == "C3"
    assert not c3.uses_actual_speed
    assert c3.predict_effort(v0, future, past) == pytest.approx(
        c1.predict_effort(v0, future, past)
    )


def test_c4_and_c5_flag_actual_speed_use() -> None:
    assert CandidateC4().uses_actual_speed
    assert CandidateC5().uses_actual_speed
    assert not CandidateFeedforward().uses_actual_speed
    assert not CandidateC2().uses_actual_speed
    assert not CandidateC3().uses_actual_speed


def _c2(accel_pred: float, brake_pred: float, params: FeedforwardParams) -> CandidateC2:
    ff = CandidateC2()
    ff.set_params(params)
    ff._accel_model = _Const(accel_pred)  # noqa: SLF001
    ff._brake_model = _Const(brake_pred)  # noqa: SLF001
    return ff


def test_c2_brake_uses_analytic_formula_when_gain_identified() -> None:
    v0 = 60.0
    coast = -coast_decel_at(PARAMS_WITH_GAIN, v0)  # 惰行加速度（負）
    a_req = coast * 1.5  # 惰行より強い減速 → ブレーキ側
    future, past = _points(v0, a_req)
    ff = _c2(16.0, 999.0, PARAMS_WITH_GAIN)  # brake_pred はモデル予測（使われないはず）

    effort = ff.predict_effort(v0, future, past)

    delta_a = a_req - coast
    gain = 0.4  # BRAKE_GAIN の一定値
    expected = -(-delta_a / gain + PARAMS_WITH_GAIN.brake_deadband_pct)
    assert effort == pytest.approx(expected)
    assert effort != pytest.approx(-999.0)  # モデル予測（C1 の分岐）ではない


def test_c2_falls_back_to_model_when_gain_not_identified() -> None:
    """ペダルゲイン未同定（PARAMS はゲイン曲線を持たない）は C1 と同じ挙動。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 1.5)
    c1 = _ff(16.0, 20.0)
    c2 = _c2(16.0, 20.0, PARAMS)
    assert c2.predict_effort(v0, future, past) == pytest.approx(
        c1.predict_effort(v0, future, past)
    )


# ── C6: 骨格（定速階段の実測テーブル）+ 残差 ML ─────────────────────


def _c6_curve() -> CruiseCurve:
    return CruiseCurve(
        speeds_kmh=(30.0, 60.0, 90.0), openings_pct=(12.65, 16.96, 16.82), n_rows=(6, 6, 6)
    )


def test_cruise_skeleton_adds_gain_term_when_identified() -> None:
    """骨格 = opening_at(v0) + a_req ÷ k(v0)（ゲイン同定済み）。"""
    curve = _c6_curve()
    skeleton = cruise_skeleton(curve, PARAMS_WITH_GAIN, 60.0, 1.0)
    assert skeleton == pytest.approx(16.96 + 1.0 / 0.5)  # ACCEL_GAIN の一定値 0.5


def test_cruise_skeleton_a_req_term_is_zero_when_gain_not_identified() -> None:
    """ゲイン未同定（None）・0 以下なら a_req 項は 0（実測テーブルの値だけを返す）。"""
    curve = _c6_curve()
    assert cruise_skeleton(curve, PARAMS, 60.0, 1.0) == pytest.approx(16.96)
    assert cruise_skeleton(curve, PARAMS, 60.0, -3.0) == pytest.approx(16.96)


def _sample_curve_for_training() -> CruiseCurve:
    """学習ログの車速レンジ（_mixed_rows は概ね 1〜60 km/h）を覆う小さな実測テーブル。"""
    return CruiseCurve(speeds_kmh=(10.0, 50.0), openings_pct=(12.0, 16.0), n_rows=(5, 5))


def test_train_inverse_model_effective_with_curve_writes_cruise_curve_key(tmp_path: Path) -> None:
    curve = _sample_curve_for_training()
    path, metrics = train_inverse_model_effective(
        _logs(_mixed_rows()), _profile(), output_dir=str(tmp_path), cruise_curve=curve
    )
    with Path(path).open("rb") as f:
        payload = pickle.load(f)  # noqa: S301 - テストで作った自前のファイル
    assert payload["cruise_curve"] == curve.to_dict()
    assert "below_deadband" in metrics["accel"]
    assert metrics["accel"]["n"] > 0


def test_train_inverse_model_effective_without_curve_has_no_cruise_curve_key(
    tmp_path: Path,
) -> None:
    """C1（cruise_curve 省略）の pkl には研究用キーが増えないこと。"""
    path, _ = train_inverse_model_effective(
        _logs(_mixed_rows()), _profile(), output_dir=str(tmp_path)
    )
    with Path(path).open("rb") as f:
        payload = pickle.load(f)  # noqa: S301
    assert "cruise_curve" not in payload


def test_candidate_c6_load_model_rejects_c1_pkl(tmp_path: Path) -> None:
    """C1 の pkl（cruise_curve キーなし）を C6 に読ませたら ValueError（黙って動かさない）。"""
    path, _ = train_inverse_model_effective(
        _logs(_mixed_rows()), _profile(), output_dir=str(tmp_path)
    )
    ff = CandidateC6()
    ff.set_params(PARAMS)
    with pytest.raises(ValueError, match="cruise_curve"):
        ff.load_model(path)


def test_candidate_c6_load_model_reads_cruise_curve(tmp_path: Path) -> None:
    curve = _sample_curve_for_training()
    path, _ = train_inverse_model_effective(
        _logs(_mixed_rows()), _profile(), output_dir=str(tmp_path), cruise_curve=curve
    )
    ff = CandidateC6()
    ff.set_params(PARAMS)
    ff.load_model(path)
    assert ff.has_model
    assert ff._cruise_curve == curve  # noqa: SLF001 - テスト用の確認


def _c6(
    residual: float, brake_pred: float, curve: CruiseCurve, clip: float | None = None
) -> CandidateC6:
    ff = CandidateC6()
    ff.set_params(PARAMS)
    ff._accel_model = _Const(residual)  # noqa: SLF001
    ff._brake_model = _Const(brake_pred)  # noqa: SLF001
    ff._speed_clip_max = clip  # noqa: SLF001
    ff._cruise_curve = curve  # noqa: SLF001
    return ff


def test_c6_accel_prediction_is_skeleton_plus_residual() -> None:
    """C6 のアクセル予測 = max(0, 骨格 + 残差)。B3 の不感帯切り上げは C1 と同じ。"""
    curve = _c6_curve()
    residual = 2.0
    ff = _c6(residual, brake_pred=0.0, curve=curve)
    v0 = 60.0  # 表の点そのもの（16.96%）
    future, past = _points(v0, 1.0)  # 加速要求 → アクセル側（B1）

    effort = ff.predict_effort(v0, future, past)

    expected_skeleton = 16.96  # ゲイン未同定の PARAMS なので a_req 項は 0
    assert effort == pytest.approx(max(PARAMS.accel_deadband_pct, expected_skeleton + residual))


def test_c6_accel_opening_uses_raw_v0_for_skeleton_above_speed_clip() -> None:
    """骨格の v0 は学習域クリップ前の実測値（表の外は opening_at の直線延長で外挿する）。"""
    curve = _c6_curve()
    ff = _c6(residual=0.0, brake_pred=0.0, curve=curve, clip=50.0)
    v0 = 100.0  # 表の最高速 90 を超え、かつ学習域クリップ (50) も超える
    future, past = _points(v0, 1.0)

    effort = ff.predict_effort(v0, future, past)

    high_slope = (16.82 - 16.96) / (90.0 - 60.0)
    expected_skeleton = 16.82 + high_slope * (v0 - 90.0)  # v0=100（クリップ後の 50 ではない）
    assert effort == pytest.approx(max(PARAMS.accel_deadband_pct, expected_skeleton))


def test_make_candidate_c6() -> None:
    ff = make_candidate("C6")
    assert isinstance(ff, CandidateC6)
    assert ff.candidate == "C6"
    assert not ff.uses_actual_speed
