"""研究開発用ハーネスの設定ローダ／CLI のユニットテスト。"""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import main as mainmod


def _load_default() -> cfgmod.ResearchConfig:
    return cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)


def test_default_config_loads_and_validates() -> None:
    """同梱の config_testVehicle.yaml はそのまま検証を通る。

    max_speed_kmh・max_decel_g は config_testVehicle.yaml
    （ユーザーが実機に合わせて書き換えるファイル）の値なので決め打ちしない。
    """
    cfg = _load_default()
    assert cfg.feedforward.model_path.endswith(".pkl")
    assert cfgmod.validate_config(cfg) == []


def test_arbiter_new_flags_load_and_default_false() -> None:
    """段3: 調停の個別スイッチ（微小変化の保持・再踏込ディレイ・解放レート）が読める。"""
    # YAML の値は段3で 1 つずつ切り替えるので、型だけ確かめる（既定値はクラス側で確かめる）
    a = _load_default().arbiter
    for flag in (a.enable_min_step, a.enable_reengage_dwell, a.enable_release_rate):
        assert isinstance(flag, bool)
    d = cfgmod.ArbiterSection()
    assert (d.enable_min_step, d.enable_reengage_dwell, d.enable_release_rate) == (
        False, False, False
    )


def test_log_interval_must_be_multiple_of_loop_interval() -> None:
    cfg = _load_default()
    cfg.control.log_interval_ms = 130
    problems = cfgmod.validate_config(cfg)
    assert any("整数倍" in p for p in problems)


def test_p95_above_hard_limit_is_rejected() -> None:
    cfg = _load_default()
    cfg.kpi.p95_deviation_kmh = 1.5
    problems = cfgmod.validate_config(cfg)
    assert any("max_abs_deviation_kmh" in p for p in problems)


def test_pedal_reversal_kpi_settings_are_validated() -> None:
    cfg = _load_default()
    assert not any("pedal_reversal" in p for p in cfgmod.validate_config(cfg))
    cfg.kpi.pedal_reversal_window_limit_per_s = cfg.kpi.pedal_reversal_limit_per_s - 0.1
    assert any("pedal_reversal_window_limit_per_s" in p for p in cfgmod.validate_config(cfg))
    cfg = _load_default()
    cfg.kpi.pedal_reversal_min_window_s = cfg.kpi.pedal_reversal_window_s + 1.0
    assert any("pedal_reversal_min_window_s" in p for p in cfgmod.validate_config(cfg))


def test_features_section_is_case_a_and_validated() -> None:
    """手順5-1 案A: dv_0.5 を外し、過去は素の past_speeds。設定の不整合は検証で落ちる。"""
    cfg = _load_default()
    spec = cfg.features.to_feature_spec()
    assert "dv_0.5" not in spec.feature_names()
    assert spec.feature_names()[-2:] == ["past_0.5", "past_1.0"]
    assert cfgmod.validate_config(cfg) == []

    cfg.features.use_h1 = False  # レジーム判定に使うので外せない
    assert any("use_h1" in p for p in cfgmod.validate_config(cfg))
    cfg.features.use_h1 = True
    cfg.features.p1_s = 2.0  # 過去が昇順でない
    assert any("p1_s" in p for p in cfgmod.validate_config(cfg))
    cfg.features.p1_s = 0.5
    cfg.features.h2_s = 0.8  # 先読みが昇順でない
    assert any("features" in p for p in cfgmod.validate_config(cfg))


def test_search_grid_matches_min_max_step() -> None:
    """手順6: ホライズン自動選択の探索格子。0.1刻みの浮動小数丸め誤差が出ないこと。"""
    ft = cfgmod.FeaturesSection(search_min_s=0.1, search_max_s=1.0, search_step_s=0.1)
    grid = ft.search_grid()
    assert grid == pytest.approx(tuple(round(0.1 * i, 1) for i in range(1, 11)))


def test_search_grid_excludes_below_min_horizon() -> None:
    ft = cfgmod.FeaturesSection(
        search_min_s=0.1, search_max_s=1.0, search_step_s=0.1, search_min_horizon_s=0.5
    )
    grid = ft.search_grid()
    assert min(grid) == pytest.approx(0.5)
    assert 0.3 not in [round(g, 1) for g in grid]


def test_search_grid_rejects_nonpositive_step() -> None:
    ft = cfgmod.FeaturesSection(search_step_s=0.0)
    with pytest.raises(ValueError, match="search_step_s"):
        ft.search_grid()


def test_default_config_has_horizon_search_settings_validated() -> None:
    """同梱の config_testVehicle.yaml の探索パラメータ（手順6）はそのまま検証を通る。"""
    cfg = _load_default()
    assert cfgmod.validate_config(cfg) == []
    assert cfg.features.search_max_s > cfg.features.search_min_s
    assert len(cfg.features.search_grid()) > 0


@pytest.mark.parametrize(
    ("field", "value", "match"),
    [
        ("search_min_s", 0.0, "search_min_s"),
        ("search_step_s", -0.1, "search_step_s"),
        ("search_max_horizons", 0, "search_max_horizons"),
        ("search_min_improvement", 1.0, "search_min_improvement"),
        ("search_min_improvement", -0.1, "search_min_improvement"),
        ("search_min_horizon_s", -1.0, "search_min_horizon_s"),
        ("search_cv_splits", 1, "search_cv_splits"),
    ],
)
def test_horizon_search_settings_out_of_range_are_rejected(
    field: str, value: float, match: str
) -> None:
    cfg = _load_default()
    setattr(cfg.features, field, value)
    problems = cfgmod.validate_config(cfg)
    assert any(match in p for p in problems), problems


def test_search_max_s_must_exceed_search_min_s() -> None:
    cfg = _load_default()
    cfg.features.search_max_s = cfg.features.search_min_s
    problems = cfgmod.validate_config(cfg)
    assert any("search_max_s" in p for p in problems)


def test_unknown_candidate_is_rejected() -> None:
    cfg = _load_default()
    # candidate の既定値は config_testVehicle.yaml（ユーザーが書き換えるファイル）依存なので
    # 決め打ちせず、C9・C6 を弾き C5 を通すことだけを主題にする
    assert cfg.feedforward.candidate in cfgmod.CANDIDATE_NAMES
    cfg.feedforward.candidate = "C9"
    problems = cfgmod.validate_config(cfg)
    assert any("candidate" in p for p in problems)
    cfg.feedforward.candidate = "C5"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.candidate = "C6"  # 2026-09-25 段4 で削除（骨格を定速階段にする案）
    assert any("candidate" in p for p in cfgmod.validate_config(cfg))


def test_curve_length_mismatch_is_rejected() -> None:
    cfg = _load_default()
    cfg.feedforward.coast_decel_speeds_kmh = [10.0, 20.0, 30.0]
    cfg.feedforward.coast_decel_kmhs = [1.0, 2.0]
    problems = cfgmod.validate_config(cfg)
    assert any("点数" in p for p in problems)


def test_creep_accel_curve_settings_are_validated() -> None:
    """段1（ProblemReport_20260916 課題#2）: クリープ加速カーブは空可・昇順・長さ一致・正値。

    creep_accel_speeds_kmh/creep_accel_kmhs は手順2の実機走行で実測値が
    自動保存されるため、既定値そのものは assert しない（走行のたびに壊れる）。
    ここでは検証ロジック（長さ一致・昇順・正値）だけを確かめる。
    """
    cfg = _load_default()
    assert cfg.feedforward.coast_band_kmhs == 0.5  # 段2で 0.5 を採用（ProblemReport_20260916）
    assert cfgmod.validate_config(cfg) == []

    cfg.feedforward.creep_accel_speeds_kmh = [1.0, 2.0]
    cfg.feedforward.creep_accel_kmhs = [3.0]  # 長さ不一致
    assert any("creep_accel" in p and "点数" in p for p in cfgmod.validate_config(cfg))

    cfg.feedforward.creep_accel_speeds_kmh = [1.0, 2.0]
    cfg.feedforward.creep_accel_kmhs = [3.0, -1.0]  # 負値
    assert any("creep_accel_kmhs" in p for p in cfgmod.validate_config(cfg))

    cfg.feedforward.creep_accel_speeds_kmh = [2.0, 1.0]  # 昇順でない
    cfg.feedforward.creep_accel_kmhs = [3.0, 2.0]
    assert any("creep_accel_speeds_kmh" in p for p in cfgmod.validate_config(cfg))

    cfg.feedforward.creep_accel_speeds_kmh = [1.0, 2.0]
    cfg.feedforward.creep_accel_kmhs = [3.4, 0.5]
    assert cfgmod.validate_config(cfg) == []

    cfg.feedforward.coast_band_kmhs = -0.1
    assert any("coast_band_kmhs" in p for p in cfgmod.validate_config(cfg))

    cfg.feedforward.coast_band_kmhs = 5.0  # 上限（未満でなければならない）
    assert any("coast_band_kmhs" in p for p in cfgmod.validate_config(cfg))


def test_reach_horizons_settings_are_validated() -> None:
    """段3（ProblemReport_20260916 課題#1・#3）: reach_horizons_s は正値・昇順、reach_step_s は
    0 より大きく 0.5 以下。既定（空リスト）は合格。
    """
    cfg = _load_default()
    # config_testVehicle.yaml（ユーザーが書き換えるファイル）は段3 を既に有効化しているため
    # 値そのものは決め打ちせず、検証ロジック（正値・昇順・範囲）だけを確かめる
    assert cfgmod.validate_config(cfg) == []

    cfg.feedforward.reach_horizons_s = []  # 空リスト（段3 無効）も合格
    assert cfgmod.validate_config(cfg) == []

    cfg.feedforward.reach_horizons_s = [1.0, 0.5, 2.0]  # 降順
    assert any("reach_horizons_s" in p for p in cfgmod.validate_config(cfg))

    cfg.feedforward.reach_horizons_s = [0.5, -1.0]  # 負値
    assert any("reach_horizons_s" in p for p in cfgmod.validate_config(cfg))

    cfg.feedforward.reach_horizons_s = [0.5, 1.0, 2.0]
    assert cfgmod.validate_config(cfg) == []

    cfg.feedforward.reach_step_s = 0.0
    assert any("reach_step_s" in p for p in cfgmod.validate_config(cfg))

    cfg.feedforward.reach_step_s = 0.51
    assert any("reach_step_s" in p for p in cfgmod.validate_config(cfg))

    cfg.feedforward.reach_step_s = 0.05
    assert cfgmod.validate_config(cfg) == []


def test_curve_equilibrium_endpoint_zero_is_allowed() -> None:
    """段2.5（ProblemReport_20260916）: coast_decel_kmhs の先頭・creep_accel_kmhs の末尾は
    クリープ平衡点（free_accel_at=0 の定義値）として 0.0 を許す。それ以外の位置の 0.0・
    どの位置の負値も従来どおり不合格（_validate_curve の allow_zero_at 引数）。
    """
    cfg = _load_default()

    # coast_decel_kmhs: 先頭が 0.0（許可された端点）は合格
    cfg.feedforward.coast_decel_speeds_kmh = [4.793, 10.0, 20.0]
    cfg.feedforward.coast_decel_kmhs = [0.0, 3.0, 5.0]
    assert cfgmod.validate_config(cfg) == []

    # coast_decel_kmhs: 途中に 0.0 があるのは不合格
    cfg.feedforward.coast_decel_kmhs = [1.0, 0.0, 5.0]
    assert any("coast_decel_kmhs" in p for p in cfgmod.validate_config(cfg))

    # coast_decel_kmhs: 末尾（許可端と逆）が 0.0 なのは不合格
    cfg.feedforward.coast_decel_kmhs = [1.0, 3.0, 0.0]
    assert any("coast_decel_kmhs" in p for p in cfgmod.validate_config(cfg))

    # coast_decel_kmhs: 負値はどの位置でも不合格（許可端でも）
    cfg.feedforward.coast_decel_kmhs = [-1.0, 3.0, 5.0]
    assert any("coast_decel_kmhs" in p for p in cfgmod.validate_config(cfg))

    # creep_accel_kmhs: 末尾が 0.0（許可された端点）は合格
    cfg.feedforward.coast_decel_speeds_kmh = []
    cfg.feedforward.coast_decel_kmhs = []
    cfg.feedforward.creep_accel_speeds_kmh = [1.0, 2.0, 4.793]
    cfg.feedforward.creep_accel_kmhs = [3.0, 1.5, 0.0]
    assert cfgmod.validate_config(cfg) == []

    # creep_accel_kmhs: 途中に 0.0 があるのは不合格
    cfg.feedforward.creep_accel_kmhs = [3.0, 0.0, 0.5]
    assert any("creep_accel_kmhs" in p for p in cfgmod.validate_config(cfg))

    # creep_accel_kmhs: 先頭（許可端と逆）が 0.0 なのは不合格
    cfg.feedforward.creep_accel_kmhs = [0.0, 1.5, 0.5]
    assert any("creep_accel_kmhs" in p for p in cfgmod.validate_config(cfg))

    # creep_accel_kmhs: 負値はどの位置でも不合格（許可端でも）
    cfg.feedforward.creep_accel_kmhs = [3.0, 1.5, -0.5]
    assert any("creep_accel_kmhs" in p for p in cfgmod.validate_config(cfg))


def test_decel_stop_thresholds_must_be_ordered() -> None:
    cfg = _load_default()
    cfg.decel_stop.release_above_g = 0.15  # 目標 0.2G より小さい
    assert any("release_above_g" in p for p in cfgmod.validate_config(cfg))
    cfg.decel_stop.release_above_g = 0.5  # vehicle.max_decel_g 0.4 を超える
    assert any("release_above_g" in p for p in cfgmod.validate_config(cfg))
    cfg.decel_stop.release_above_g = 0.3
    cfg.decel_stop.press_margin_g = 0.25  # 目標以上の margin
    assert any("press_margin_g" in p for p in cfgmod.validate_config(cfg))


def test_pedal_search_new_keys_are_validated() -> None:
    """段1b: onset_accel_kmhs・search_step_mm・deadband_max_pct（新設定）の検証。"""
    cfg = _load_default()
    ps = cfg.pedal_search
    assert ps.onset_accel_kmhs == 0.2
    assert ps.search_step_mm == 0.1
    assert ps.deadband_max_pct == 20.0
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    ps.onset_accel_kmhs = 0.0
    assert problems_about("onset_accel_kmhs")
    ps.onset_accel_kmhs = 0.2

    # 0.01mm = 1 pulse（PCON-CB の位置指令単位）が機械的な下限
    ps.search_step_mm = 0.01
    assert cfgmod.validate_config(cfg) == []
    ps.search_step_mm = 0.009
    assert problems_about("search_step_mm")
    ps.search_step_mm = 5.01
    assert problems_about("search_step_mm")
    ps.search_step_mm = 0.1

    ps.deadband_max_pct = 0.0
    assert problems_about("deadband_max_pct")
    ps.deadband_max_pct = min(ps.accel_max_pct, ps.brake_max_pct) + 0.1
    assert problems_about("deadband_max_pct")
    ps.deadband_max_pct = 20.0
    assert cfgmod.validate_config(cfg) == []


def test_pedal_search_creep_keys_validate() -> None:
    """2026-09-20: クリープ安定判定の新3キー（creep_window_s・creep_settle_kmhs・
    creep_settle_min_s）が validate_config を通り、範囲外は弾かれること。"""
    cfg = _load_default()
    ps = cfg.pedal_search
    # 2026-09-21 実機確認で 3.0 → 5.0 に調整（ProblemReport_20260919 13.2）
    assert ps.creep_window_s == 5.0
    assert ps.creep_settle_kmhs == pytest.approx(0.020)  # 同上調整で 0.033 → 0.020
    assert ps.creep_settle_min_s == 10.0
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    ps.creep_window_s = 0.0
    assert problems_about("creep_window_s")
    ps.creep_window_s = 3.0

    ps.creep_settle_kmhs = 0.0
    assert problems_about("creep_settle_kmhs")
    ps.creep_settle_kmhs = 0.033

    ps.creep_settle_min_s = -0.1
    assert problems_about("creep_settle_min_s")
    ps.creep_settle_min_s = 0.0
    assert cfgmod.validate_config(cfg) == []


def test_gain_min_offsets_are_validated() -> None:
    cfg = _load_default()
    cfg.learning.brake_gain_min_offset_pct = 0.0  # 正値でない
    assert any("brake_gain_min_offset_pct" in p for p in cfgmod.validate_config(cfg))


def test_sample_weight_settings_are_validated() -> None:
    """段2（ProblemReport_20260925。学習サンプルの WLTP 重み付け）: 既定は無効（後方互換）、
    範囲は 0 < min <= 1 <= max。"""
    cfg = _load_default()
    lr = cfg.learning
    assert lr.sample_weight_enabled is False
    assert (lr.sample_weight_min, lr.sample_weight_max) == (0.2, 5.0)
    assert cfgmod.validate_config(cfg) == []

    lr.sample_weight_min = 0.0  # 0 < min でなければならない
    assert any("sample_weight" in p for p in cfgmod.validate_config(cfg))

    lr.sample_weight_min = 1.5  # min <= 1 でなければならない
    assert any("sample_weight" in p for p in cfgmod.validate_config(cfg))

    lr.sample_weight_min = 0.2
    lr.sample_weight_max = 0.5  # 1 <= max でなければならない
    assert any("sample_weight" in p for p in cfgmod.validate_config(cfg))

    lr.sample_weight_min = 1.0
    lr.sample_weight_max = 1.0  # 境界（min=1=max）は合格
    assert cfgmod.validate_config(cfg) == []


def test_creep_launch_settings_are_validated() -> None:
    """段1（ProblemReport_20260916 課題#2）: クリープ発進・クリープ域ブレーキ保持の新設定。

    2026-09-17 段1b: 終了条件を target_kmh 到達から平衡到達へ変更したのに合わせ、
    creep_launch_target_kmh（安全上限）4.5→15.0・creep_launch_timeout_s 20.0→30.0 に改定、
    creep_launch_settle_kmhs / creep_launch_settle_s を新設。

    2026-09-17（誤判定修正）: 停車保持解放直後を「平衡到達」と誤判定するバグの修正に合わせ、
    creep_launch_min_speed_kmh / creep_launch_settle_min_s を新設し、
    creep_launch_timeout_s を 30.0→60.0 に改定（クリープ平衡への収束が漸近的なため）。
    """
    cfg = _load_default()
    lr = cfg.learning
    # 2026-09-27 段7a（ProblemReport_20260925 段7）: 手順2 の計測効率化のため 3→0
    assert lr.creep_launch_count == 0
    assert lr.creep_launch_target_kmh == 15.0
    assert lr.creep_launch_timeout_s == 60.0
    assert lr.creep_launch_settle_kmhs == 0.1
    assert lr.creep_launch_settle_s == 2.0
    assert lr.creep_launch_min_speed_kmh == 1.0
    assert lr.creep_launch_settle_min_s == 5.0
    # 2026-09-25 段4: 「不感帯 + frac × (停車保持開度 − 不感帯)」の frac 列（1.0 = 停車保持開度）。
    # 修理後の実測（不感帯 7.26・停車保持 18.63）で +0.6〜+11.4% になり、停止境界をはさむ
    assert lr.creep_brake_hold_fracs == [0.05, 0.15, 0.25, 0.45, 0.55, 0.7, 0.9, 1.0]
    assert lr.creep_curve_bin_kmh == 1.0
    assert lr.creep_curve_min_bin_samples == 5
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    lr.creep_launch_count = -1
    assert problems_about("creep_launch_count")
    lr.creep_launch_count = 0
    lr.creep_brake_hold_fracs = []
    assert cfgmod.validate_config(cfg) == []  # 両方 0/空でも可（足さない）

    lr.creep_launch_target_kmh = 0.0
    assert problems_about("creep_launch_target_kmh")
    lr.creep_launch_target_kmh = cfg.vehicle.max_speed_kmh
    assert problems_about("creep_launch_target_kmh")
    lr.creep_launch_target_kmh = 15.0

    lr.creep_launch_timeout_s = 0.0
    assert problems_about("creep_launch_timeout_s")
    lr.creep_launch_timeout_s = 60.0

    lr.creep_launch_settle_kmhs = 0.0
    assert problems_about("creep_launch_settle_kmhs")
    lr.creep_launch_settle_kmhs = 0.1

    lr.creep_launch_settle_s = 0.0
    assert problems_about("creep_launch_settle_s")
    lr.creep_launch_settle_s = 2.0

    lr.creep_launch_min_speed_kmh = -1.0
    assert problems_about("creep_launch_min_speed_kmh")
    lr.creep_launch_min_speed_kmh = lr.creep_launch_target_kmh  # target 未満でなければ不可
    assert problems_about("creep_launch_min_speed_kmh")
    lr.creep_launch_min_speed_kmh = 1.0

    lr.creep_launch_settle_min_s = -1.0
    assert problems_about("creep_launch_settle_min_s")
    lr.creep_launch_settle_min_s = 5.0

    lr.creep_brake_hold_fracs = [0.5, 0.2]  # 昇順でない
    assert problems_about("creep_brake_hold_fracs")
    lr.creep_brake_hold_fracs = [0.0, 0.5]  # 0 は不可（0<frac<=1.2）
    assert problems_about("creep_brake_hold_fracs")
    lr.creep_brake_hold_fracs = [0.5, 1.3]  # 停車保持開度の 1.2 倍を超える
    assert problems_about("creep_brake_hold_fracs")
    lr.creep_brake_hold_fracs = [0.2, 0.5]

    lr.creep_curve_bin_kmh = 0.0
    assert problems_about("creep_curve_bin_kmh")
    lr.creep_curve_bin_kmh = 1.0

    lr.creep_curve_min_bin_samples = 0
    assert problems_about("creep_curve_min_bin_samples")
    lr.creep_curve_min_bin_samples = 5
    assert cfgmod.validate_config(cfg) == []


def test_coast_curve_low_settings_are_validated() -> None:
    """段2.5（低速の惰行カーブを実測に合わせる。ProblemReport_20260916）: 惰行カーブの低速端
    （creep_speed_kmh〜coast_curve_low_max_kmh）を細ビンで同定し直すための新設定。
    """
    cfg = _load_default()
    lr = cfg.learning
    assert lr.coast_curve_low_bin_kmh == 1.0
    assert lr.coast_curve_low_max_kmh == 15.0
    assert lr.coast_curve_low_min_bin_samples == 5
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    lr.coast_curve_low_bin_kmh = 0.0
    assert problems_about("coast_curve_low_bin_kmh")
    lr.coast_curve_low_bin_kmh = -1.0
    assert problems_about("coast_curve_low_bin_kmh")
    lr.coast_curve_low_bin_kmh = 1.0

    # COAST_CURVE_BIN_KMH（本番の惰行カーブビン幅。10.0）未満は不可
    lr.coast_curve_low_max_kmh = 9.9
    assert problems_about("coast_curve_low_max_kmh")
    lr.coast_curve_low_max_kmh = 10.0  # 境界（本番と同じ幅）は可
    assert cfgmod.validate_config(cfg) == []
    lr.coast_curve_low_max_kmh = 15.0

    lr.coast_curve_low_min_bin_samples = 0
    assert problems_about("coast_curve_low_min_bin_samples")
    lr.coast_curve_low_min_bin_samples = 5
    assert cfgmod.validate_config(cfg) == []


def test_stop_brake_floor_settings_are_validated() -> None:
    """段4改訂（クリープ域ブレーキの下限。ProblemReport_20260916）:
    stop_brake_floor_offset_pct・brake_trim_max_kmh・brake_trim_ref_kmh、
    learning.stop_brake_floor_* の範囲検証。
    """
    cfg = _load_default()
    ff = cfg.feedforward
    lr = cfg.learning
    assert ff.brake_trim_max_kmh == 5.0  # 2026-09-19 段4 で有効化済み（0.0 なら無効）
    assert ff.stop_brake_floor_offset_pct == pytest.approx(8.0)
    assert lr.stop_brake_floor_start_tol_kmh == pytest.approx(1.5)
    assert lr.stop_brake_floor_min_float_s == pytest.approx(5.0)
    assert lr.stop_brake_floor_opening_tol_pct == pytest.approx(0.3)
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    # stop_brake_floor_offset_pct の範囲（0<=pct<20.0）
    ff.stop_brake_floor_offset_pct = -0.1
    assert problems_about("stop_brake_floor_offset_pct")
    ff.stop_brake_floor_offset_pct = 20.0  # 上限（未満でなければならない）
    assert problems_about("stop_brake_floor_offset_pct")
    ff.stop_brake_floor_offset_pct = 0.0  # 未同定は合格
    assert cfgmod.validate_config(cfg) == []
    ff.stop_brake_floor_offset_pct = 8.0

    # brake_trim_max_kmh の範囲（0<=km/h<20.0）
    ff.brake_trim_max_kmh = -0.1
    assert problems_about("brake_trim_max_kmh")
    ff.brake_trim_max_kmh = 20.0  # 上限（未満でなければならない）
    assert problems_about("brake_trim_max_kmh")
    ff.brake_trim_max_kmh = 5.0
    assert cfgmod.validate_config(cfg) == []
    ff.brake_trim_max_kmh = 0.0

    # brake_trim_ref_kmh の範囲（0<km/h<=2.0）
    ff.brake_trim_ref_kmh = 0.0
    assert problems_about("brake_trim_ref_kmh")
    ff.brake_trim_ref_kmh = 2.1
    assert problems_about("brake_trim_ref_kmh")
    ff.brake_trim_ref_kmh = 0.3
    assert cfgmod.validate_config(cfg) == []

    # learning.stop_brake_floor_* の範囲（すべて正値）
    lr.stop_brake_floor_start_tol_kmh = 0.0
    assert problems_about("stop_brake_floor_start_tol_kmh")
    lr.stop_brake_floor_start_tol_kmh = 1.5

    lr.stop_brake_floor_min_float_s = 0.0
    assert problems_about("stop_brake_floor_min_float_s")
    lr.stop_brake_floor_min_float_s = 5.0

    lr.stop_brake_floor_opening_tol_pct = 0.0
    assert problems_about("stop_brake_floor_opening_tol_pct")
    lr.stop_brake_floor_opening_tol_pct = 0.3
    assert cfgmod.validate_config(cfg) == []


def test_pedal_gain_curve_still_rejects_zero_value_after_stop_brake_removal() -> None:
    """段4改訂で stop_brake 専用の「速度グリッドに 0.0 を許す」検証ブロックを丸ごと削除した
    ことの確認: `_validate_curve` に `allow_zero_at` を渡していない pedal_gain のようなカーブは
    影響を受けず、値側の 0.0 は従来どおりどの位置でも不合格のまま
    （coast_decel/creep_accel の allow_zero_at 挙動は `test_curve_equilibrium_endpoint_zero_is_
    allowed` が別途確認する）。
    """
    cfg = _load_default()
    ff = cfg.feedforward
    ff.pedal_gain_speeds_kmh = [0.0, 1.0, 2.0]
    ff.accel_gain_kmhs_per_pct = [0.0, 1.0, 2.0]
    ff.brake_gain_kmhs_per_pct = [1.0, 1.0, 2.0]
    assert any("pedal_gain" in p for p in cfgmod.validate_config(cfg))


def test_negative_standby_margin_is_rejected() -> None:
    cfg = _load_default()
    cfg.mode_drive.standby_margin_pct = -0.5
    assert any("standby_margin_pct" in p for p in cfgmod.validate_config(cfg))
    cfg.mode_drive.standby_margin_pct = 0.0
    assert not any("standby_margin_pct" in p for p in cfgmod.validate_config(cfg))


def test_checks_section_loads_from_default_yaml() -> None:
    """既定 YAML は開発時に UPS を使わないため checks.init_ups/pre_ups だけ false。"""
    cfg = _load_default()
    assert cfg.checks.init_ups is False
    assert cfg.checks.pre_ups is False
    assert cfg.checks.init_servo_comm is True
    assert cfg.checks.init_clear_errors is True
    assert cfg.checks.init_servo_on is True
    assert cfg.checks.init_can is True
    assert cfg.checks.init_home_return is True
    assert cfg.checks.pre_communication is True
    assert cfg.checks.pre_servo_state is True
    assert cfg.checks.pre_profile is True
    assert cfg.checks.pre_actuator_position is True
    assert cfg.checks.pre_brake_stop is True
    assert cfg.checks.pre_vehicle_stopped is True
    assert cfgmod.validate_config(cfg) == []


def test_pre_ups_true_requires_init_ups_true() -> None:
    cfg = _load_default()
    cfg.checks.pre_ups = True  # init_ups は既定 YAML のまま false
    assert any("checks.init_ups" in p for p in cfgmod.validate_config(cfg))
    cfg.checks.init_ups = True
    assert not any("checks.init_ups" in p for p in cfgmod.validate_config(cfg))


def test_negative_gain_is_rejected() -> None:
    cfg = _load_default()
    cfg.pid.kp = -1.0
    assert any("pid.kp" in p for p in cfgmod.validate_config(cfg))


def test_unknown_key_raises(tmp_path: Path) -> None:
    path = tmp_path / "cfg.yaml"
    path.write_text("pid:\n  kp: 1.0\n  kq: 2.0\n", encoding="utf-8")
    with pytest.raises(cfgmod.ConfigError, match="pid.kq"):
        cfgmod.load_config(path)


def test_unknown_section_raises(tmp_path: Path) -> None:
    path = tmp_path / "cfg.yaml"
    path.write_text("nonesuch:\n  a: 1\n", encoding="utf-8")
    with pytest.raises(cfgmod.ConfigError, match="nonesuch"):
        cfgmod.load_config(path)


def test_missing_config_is_copied_from_default(tmp_path: Path) -> None:
    path = tmp_path / "sub" / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    assert path.exists()
    assert cfg.source_path == path
    # config_testVehicle.yaml の値を決め打ちせず、既定からコピーされたことだけを確認する
    assert cfg.vehicle == cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH).vehicle


def test_save_preserves_comments_and_layout(tmp_path: Path) -> None:
    """値の書き戻しでコメント・行順・インデントが壊れないこと。"""
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    before = path.read_text(encoding="utf-8").splitlines()

    changed = cfg.save(
        {
            "pid.kp": 4.762333295145048,
            "pid.ki": 0.5720877379661851,
            "feedforward.coast_decel_speeds_kmh": [5.0, 15.0, 25.0],
            "feedforward.coast_decel_kmhs": [1.6, 1.81, 2.7],
            "output.plot": False,
            "control.loop_interval_ms": 20,
            "modes.wltp_mode_name": "01_WLTP_Low,Mid,Hi,ExHi",
        }
    )
    assert len(changed) == 7
    after = path.read_text(encoding="utf-8").splitlines()
    assert len(before) == len(after)
    # コメントは残る
    assert any("比例ゲイン" in line for line in after)
    assert any("惰行カーブ 速度グリッド" in line for line in after)

    reloaded = cfgmod.load_config(path)
    assert reloaded.pid.kp == pytest.approx(4.76233, rel=1e-4)
    assert reloaded.feedforward.coast_decel_speeds_kmh == [5.0, 15.0, 25.0]
    assert reloaded.output.plot is False
    assert reloaded.control.loop_interval_ms == 20
    assert reloaded.modes.wltp_mode_name == "01_WLTP_Low,Mid,Hi,ExHi"


def test_save_unknown_key_raises(tmp_path: Path) -> None:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    with pytest.raises(cfgmod.ConfigError, match="見つかりません"):
        cfg.save({"pid.no_such_gain": 1.0})


def test_save_nested_mapping_key_raises(tmp_path: Path) -> None:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    with pytest.raises(cfgmod.ConfigError, match="入れ子"):
        cfg.save({"pid": 1.0})


def test_split_comment_ignores_hash_inside_quotes() -> None:
    value, comment = cfgmod._split_comment(' "a#b"  # 実コメント')
    assert value.strip() == '"a#b"'
    assert comment.strip() == "# 実コメント"


def test_split_comment_ignores_hash_inside_list() -> None:
    value, comment = cfgmod._split_comment(" [1, 2]  # 説明")
    assert value.strip() == "[1, 2]"
    assert comment.strip() == "# 説明"


def test_format_float_keeps_decimal_point() -> None:
    assert cfgmod._format_value(4.0) == "4.0"
    assert cfgmod._format_value(0.5720877379661851) == "0.572088"
    assert cfgmod._format_value(True) == "true"
    assert cfgmod._format_value(50) == "50"


def test_save_accepts_rounding_to_six_significant_digits(tmp_path: Path) -> None:
    """有効 6 桁に丸めて書くので、桁の多い値でも書き戻し検証で落ちないこと。"""
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save({"feedforward.creep_speed_kmh": 123.456789})
    assert cfgmod.load_config(path).feedforward.creep_speed_kmh == pytest.approx(123.457)


# ── CLI ──────────────────────────────────────────────────────────────


def test_resolve_steps_defaults_to_step0() -> None:
    args = mainmod.build_parser().parse_args([])
    assert [s.number for s in mainmod.resolve_steps(args)] == [0]


def test_resolve_steps_upto() -> None:
    args = mainmod.build_parser().parse_args(["--upto", "3"])
    assert [s.number for s in mainmod.resolve_steps(args)] == [0, 1, 2, 3]


def test_resolve_steps_only_and_list() -> None:
    parser = mainmod.build_parser()
    assert [s.number for s in mainmod.resolve_steps(parser.parse_args(["--only", "4"]))] == [4]
    args = parser.parse_args(["--steps", "0,2,3"])
    assert [s.number for s in mainmod.resolve_steps(args)] == [0, 2, 3]


def test_resolve_steps_rejects_unknown_number() -> None:
    args = mainmod.build_parser().parse_args(["--only", "99"])
    with pytest.raises(SystemExit, match="存在しません"):
        mainmod.resolve_steps(args)


def test_step0_runs_on_default_config(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    # 結果ディレクトリを tmp に逃がし、リポジトリを汚さない
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
        }
    )
    assert mainmod.main(["--only", "0", "--config", str(path)]) == 0
    out = capsys.readouterr().out
    assert "設定の検証: OK" in out
    assert "手順 0 完了" in out
    assert (tmp_path / "results" / "models").is_dir()


def test_step4_stops_as_not_implemented(capsys: pytest.CaptureFixture[str]) -> None:
    assert mainmod.main(["--only", "4"]) == 3
    assert "未実装" in capsys.readouterr().out


def test_dry_run_does_not_touch_config(tmp_path: Path) -> None:
    missing = tmp_path / "cfg.yaml"
    assert mainmod.main(["--upto", "3", "--dry-run", "--config", str(missing)]) == 0
    assert not missing.exists()


def test_invalid_config_returns_exit_code_2(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save({"vehicle.max_decel_g": 2.5})
    assert mainmod.main(["--only", "0", "--config", str(path)]) == 2
    assert "max_decel_g" in capsys.readouterr().out


def test_coverage_mode_names_are_validated() -> None:
    cfg = _load_default()
    assert cfg.modes.coverage_mode_names  # 既定は空でない
    assert not any("coverage_mode_names" in p for p in cfgmod.validate_config(cfg))
    for bad in ([], ["01_WLTP_Low,Mid,Hi,ExHi", "01_WLTP_Low,Mid,Hi,ExHi"], [" "]):
        cfg.modes.coverage_mode_names = bad
        assert any("coverage_mode_names" in p for p in cfgmod.validate_config(cfg))


def test_default_coverage_modes_and_grid_edges_include_us06_range() -> None:
    from tests.research.config import LearningSection, ModesSection

    assert ModesSection().coverage_mode_names == ["01_WLTP_Low,Mid,Hi,ExHi"]
    edges = LearningSection().grid_accel_edges_kmhs
    assert edges[0] == -14.0 and edges[-1] == 14.0  # max_decel_g 0.4G ≒ 14.1 km/h/s


def test_grid_hole_wltp_min_s_key_is_gone(tmp_path: Path) -> None:
    """しきい値は 1 ステップで測れる長さ（窓 − 頭の除外）から自動で決まる。旧キーは未知キー。"""
    path = tmp_path / "cfg.yaml"
    path.write_text("learning:\n  grid_hole_wltp_min_s: 5.0\n", encoding="utf-8")
    with pytest.raises(cfgmod.ConfigError, match="grid_hole_wltp_min_s"):
        cfgmod.load_config(path)
    assert cfgmod.LearningSection().grid_target_min_s == pytest.approx(2.5)


def test_arbiter_band_and_direction_keys_load_and_are_validated() -> None:
    """段3c/3d: 加速度帯・向きのヒステリシスのキーが読め、0 以下は検証で落ちる。"""
    a = _load_default().arbiter
    assert isinstance(a.enable_accel_band, bool)
    assert isinstance(a.enable_direction_hysteresis, bool)
    d = cfgmod.ArbiterSection()
    assert (d.enable_accel_band, d.enable_direction_hysteresis) == (False, False)
    assert (d.accel_band_horizon_s, d.accel_band_kmhs, d.accel_band_dev_escape_kmh) == (
        3.0, 0.25, 0.3
    )
    assert d.accel_band_open_escape_pct == 2.0
    assert isinstance(a.accel_band_open_escape_pct, float)
    assert d.accel_direction_hysteresis_pct == 0.5
    for key in (
        "accel_band_horizon_s",
        "accel_band_kmhs",
        "accel_band_dev_escape_kmh",
        "accel_band_open_escape_pct",
        "accel_direction_hysteresis_pct",
    ):
        cfg = _load_default()
        assert not any(key in p for p in cfgmod.validate_config(cfg))
        setattr(cfg.arbiter, key, 0.0)
        assert any(key in p for p in cfgmod.validate_config(cfg))
