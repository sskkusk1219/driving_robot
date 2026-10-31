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


def test_pedal_overlap_margin_is_validated() -> None:
    cfg = _load_default()
    assert not any("pedal_overlap" in p for p in cfgmod.validate_config(cfg))
    cfg.kpi.pedal_overlap_margin_pct = -0.1
    assert any("pedal_overlap_margin_pct" in p for p in cfgmod.validate_config(cfg))


def test_features_section_is_case_a_and_validated() -> None:
    """手順5-1 案A: dv_0.5 を外し、過去は素の past_speeds。設定の不整合は検証で落ちる。"""
    cfg = _load_default()
    spec = cfg.features.to_feature_spec()
    assert "dv_0.5" not in spec.feature_names()
    assert spec.feature_names()[-2:] == ["past_0.5", "past_1.0"]
    assert cfgmod.validate_config(cfg) == []

    cfg.features.h0_s = 0.0  # 停車保持の判定ホライズンは正値
    assert any("h0_s" in p for p in cfgmod.validate_config(cfg))


def test_search_grid_matches_min_max_step() -> None:
    """手順6: ホライズン自動選択の探索格子（定数）。0.1刻みの浮動小数丸め誤差が出ないこと。"""
    grid = cfgmod.FeaturesSection().search_grid()
    assert grid == pytest.approx(tuple(round(0.1 * i, 1) for i in range(1, 31)))


def test_default_config_has_horizon_search_settings_validated() -> None:
    """同梱の config_testVehicle.yaml の探索パラメータ（手順6）はそのまま検証を通る。"""
    cfg = _load_default()
    assert cfgmod.validate_config(cfg) == []
    assert cfg.features.search_max_s > cfg.features.search_min_s
    assert len(cfg.features.search_grid()) > 0


def test_unknown_candidate_is_rejected() -> None:
    cfg = _load_default()
    # candidate の既定値は config_testVehicle.yaml（ユーザーが書き換えるファイル）依存なので
    # 決め打ちせず、C99・C6 を弾き C5 を通すことだけを主題にする
    assert cfg.feedforward.candidate in cfgmod.CANDIDATE_NAMES
    cfg.feedforward.candidate = "C99"
    problems = cfgmod.validate_config(cfg)
    assert any("candidate" in p for p in problems)
    cfg.feedforward.candidate = "C5"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.candidate = "C6"  # 2026-09-25 段4 で削除（骨格を定速階段にする案）
    assert any("candidate" in p for p in cfgmod.validate_config(cfg))


def test_candidate_c7_is_accepted() -> None:
    cfg = _load_default()
    cfg.feedforward.candidate = "C7"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))


def test_candidate_c8_is_accepted() -> None:
    cfg = _load_default()
    cfg.feedforward.candidate = "C8"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))


def test_forward_map_section_defaults_without_yaml_section(tmp_path: Path) -> None:
    """yaml に forward_map 節が無くても既定値で読め、検証も通る。"""
    lines = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8").splitlines(keepends=True)
    start = lines.index("forward_map:\n")
    end = start + 1
    while lines[end].startswith((" ", "\t")) or lines[end].strip() == "":
        end += 1  # 節の中身（インデント行・空行）を飛ばす
    stripped = "".join(lines[:start] + lines[end:])
    assert "forward_map:" not in stripped
    path = tmp_path / "no_fm.yaml"
    path.write_text(stripped, encoding="utf-8")
    cfg = cfgmod.load_config(path)
    fm = cfg.forward_map
    assert (fm.coast_band_kmhs, fm.need_window_s) == (0.25, 1.0)
    assert fm.accel_lag_s is None and fm.brake_lag_s is None
    assert not any("forward_map" in p for p in cfgmod.validate_config(cfg))


def test_forward_map_yaml_values_are_loaded() -> None:
    fm = _load_default().forward_map
    assert fm.coast_band_kmhs == 0.25 and fm.need_window_s == 0.25  # 段9-a'（2026-10-25）
    assert fm.accel_lag_s is None and fm.brake_lag_s is None


@pytest.mark.parametrize(
    ("field_name", "value"),
    [
        ("coast_band_kmhs", -0.1),
        ("need_window_s", 0.0),
        ("accel_lag_s", -0.05),
        ("brake_lag_s", -0.05),
    ],
)
def test_forward_map_invalid_values_are_rejected(field_name: str, value: float) -> None:
    cfg = _load_default()
    setattr(cfg.forward_map, field_name, value)
    problems = cfgmod.validate_config(cfg)
    assert any(f"forward_map.{field_name}" in p for p in problems)


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

    # reach_horizons_s は pedal_select_mode: window と併用できない（yaml は window）。
    # この検証は reach_horizons_s 自体のものなので point にして確かめる
    cfg.feedforward.pedal_select_mode = "point"
    cfg.feedforward.pedal_select_brake_center_s = None  # window 専用キー（yaml に値があれば外す）
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


def test_pedal_search_brake_step_is_validated_and_search_keys_are_gone() -> None:
    """段2l-6: 2-0 は不感帯探索をやめた。brake_step_mm（旧 search_step_mm）の検証と、
    不感帯探索専用キーが無いこと。"""
    cfg = _load_default()
    ps = cfg.pedal_search
    assert ps.brake_step_mm == 0.1
    assert cfgmod.validate_config(cfg) == []
    for removed in ("onset_delta_kmh", "onset_confirm_kmh", "confirm_count", "deadband_max_pct",
                    "accel_max_pct", "search_step_mm"):
        assert not hasattr(ps, removed), removed

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    # 0.01mm = 1 pulse（PCON-CB の位置指令単位）が機械的な下限
    ps.brake_step_mm = 0.01
    assert cfgmod.validate_config(cfg) == []
    ps.brake_step_mm = 0.009
    assert problems_about("brake_step_mm")
    ps.brake_step_mm = 5.01
    assert problems_about("brake_step_mm")
    ps.brake_step_mm = 0.1
    # 停止確認の上限は vehicle.max_brake_opening_pct を使う（専用の上限キーは無い）
    assert not hasattr(ps, "brake_max_pct")


def test_removed_brake_max_pct_key_gives_clear_error(tmp_path: Path) -> None:
    """2026-10-09: pedal_search.brake_max_pct は廃止。yaml に残っていたら削除を促すエラー。"""
    path = tmp_path / "old.yaml"
    path.write_text("pedal_search:\n  brake_max_pct: 50.0\n", encoding="utf-8")
    with pytest.raises(
        cfgmod.ConfigError, match="brake_max_pct は廃止されました.*max_brake_opening_pct"
    ):
        cfgmod.load_config(path)


def test_pedal_search_creep_keys_validate() -> None:
    """2026-09-20: クリープ安定判定の新3キー（creep_window_s・creep_settle_kmhs・
    creep_settle_min_s）が validate_config を通り、範囲外は弾かれること。"""
    cfg = _load_default()
    ps = cfg.pedal_search
    # 今の yaml の値（2026-10-19 段2s-2 で 5.0 → 7.0 に短縮。creep_settle_min_s も 7.0）
    assert ps.creep_window_s == 7.0
    assert ps.creep_settle_kmhs == pytest.approx(0.020)  # 同上調整で 0.033 → 0.020
    assert ps.creep_settle_min_s == 7.0
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
    assert lr.creep_launch_settle_kmhs == 0.1
    assert lr.creep_launch_settle_s == 2.0
    assert lr.creep_curve_bin_kmh == 1.0
    assert lr.creep_curve_min_bin_samples == 5
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    lr.creep_launch_settle_kmhs = 0.0
    assert problems_about("creep_launch_settle_kmhs")
    lr.creep_launch_settle_kmhs = 0.1

    lr.creep_launch_settle_s = 0.0
    assert problems_about("creep_launch_settle_s")
    lr.creep_launch_settle_s = 2.0

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
    learning.stop_brake_floor_* の範囲検証（feedforward 側の3キーは2026-10-09 に yaml から削除）。
    """
    cfg = _load_default()
    lr = cfg.learning
    assert lr.stop_brake_floor_start_tol_kmh == pytest.approx(1.5)
    assert lr.stop_brake_floor_min_float_s == pytest.approx(5.0)
    assert lr.stop_brake_floor_opening_tol_pct == pytest.approx(0.3)
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

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


def test_min_opening_is_validated_against_max() -> None:
    """段2l-6: 0 ≤ min ＜ 対応する max（アクセル・ブレーキそれぞれ）。"""
    cfg = _load_default()
    v = cfg.vehicle
    assert v.min_accel_opening_pct == 4.2 and v.min_brake_opening_pct == 4.2  # 今の yaml の値
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    for kind in ("accel", "brake"):
        min_key, max_key = f"min_{kind}_opening_pct", f"max_{kind}_opening_pct"
        original = getattr(v, min_key)
        setattr(v, min_key, -0.1)
        assert problems_about(min_key)
        setattr(v, min_key, getattr(v, max_key))  # min == max は不可
        assert problems_about(min_key)
        setattr(v, min_key, 0.0)  # 0 は可
        assert not problems_about(min_key)
        setattr(v, min_key, original)
    assert cfgmod.validate_config(cfg) == []


def test_removed_deadband_and_standby_margin_keys(tmp_path: Path) -> None:
    """段2l-6: feedforward.*_deadband_pct・mode_drive.standby_margin_pct は無い。
    それらを含む古い yaml は「未知のキー」で読めない。"""
    assert not hasattr(cfgmod.FeedforwardSection(), "accel_deadband_pct")
    assert not hasattr(cfgmod.FeedforwardSection(), "brake_deadband_pct")
    assert not hasattr(cfgmod.ModeDriveSection(), "standby_margin_pct")
    path = tmp_path / "old.yaml"
    text = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    path.write_text(
        text.replace("  creep_speed_kmh:", "  accel_deadband_pct: 10.5\n  creep_speed_kmh:", 1),
        encoding="utf-8",
    )
    with pytest.raises(cfgmod.ConfigError, match="未知のキー.*feedforward.accel_deadband_pct"):
        cfgmod.load_config(path)


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
    from tests.research.config import ModesSection

    assert ModesSection().coverage_mode_names == ["01_WLTP_Low,Mid,Hi,ExHi"]
    from tests.research.wltp_grid import GRID_ACCEL_EDGES_KMHS

    edges = GRID_ACCEL_EDGES_KMHS
    assert edges[0] == -14.0 and edges[-1] == 14.0  # max_decel_g 0.4G ≒ 14.1 km/h/s


def test_grid_hole_wltp_min_s_key_is_gone(tmp_path: Path) -> None:
    """しきい値は 1 ステップで測れる長さ（窓 − 頭の除外）から自動で決まる。旧キーは未知キー。"""
    path = tmp_path / "cfg.yaml"
    path.write_text("learning:\n  grid_hole_wltp_min_s: 5.0\n", encoding="utf-8")
    with pytest.raises(cfgmod.ConfigError, match="grid_hole_wltp_min_s"):
        cfgmod.load_config(path)


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


def test_pedal_select_brake_center_is_validated_and_roundtrips(tmp_path: Path) -> None:
    """段2-2: L_b は既定 null（None）。指定時は 0<=L_b<=2、point のときの指定はエラー。"""
    cfg = _load_default()
    # yaml の現在値（ユーザー決定で 0.0 など）に依らず、null から始めて検証する
    cfg.feedforward.pedal_select_brake_center_s = None  # 既定は null（＝L と同じ）
    assert cfgmod.validate_config(cfg) == []

    cfg.feedforward.pedal_select_mode = "window"
    for ok in (0.0, 0.1, 2.0):
        cfg.feedforward.pedal_select_brake_center_s = ok
        assert cfgmod.validate_config(cfg) == []
    for bad in (-0.1, 2.1):
        cfg.feedforward.pedal_select_brake_center_s = bad
        assert any("pedal_select_brake_center_s" in p for p in cfgmod.validate_config(cfg))

    cfg.feedforward.pedal_select_mode = "point"  # point には効かないので指定はエラー
    cfg.feedforward.pedal_select_brake_center_s = 0.0
    assert any("pedal_select_brake_center_s" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.pedal_select_brake_center_s = None  # null なら point でも合格
    assert cfgmod.validate_config(cfg) == []

    # yaml の null/数値の読み込みと、値だけの書き換え（コメント保持）
    path = tmp_path / "c.yaml"
    path.write_text(cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    assert "pedal_select_brake_center_s" in path.read_text(encoding="utf-8")
    cfgmod.update_yaml_values(path, {"feedforward.pedal_select_brake_center_s": 0.1})
    assert cfgmod.load_config(path).feedforward.pedal_select_brake_center_s == 0.1
    cfgmod.update_yaml_values(path, {"feedforward.pedal_select_brake_center_s": None})
    assert cfgmod.load_config(path).feedforward.pedal_select_brake_center_s is None
    # 他の自動保存（L）がこのキーを落とさない
    cfgmod.update_yaml_values(path, {"feedforward.pedal_select_center_s": 0.4})
    assert "pedal_select_brake_center_s" in path.read_text(encoding="utf-8")


def test_ramp_settings_are_validated() -> None:
    """段2k 段A: learning.ramp_g_levels / ramp_mode_min_s / ramp_time_budget_s の検証。"""
    cfg = _load_default()
    lr = cfg.learning
    assert lr.ramp_g_levels == [0.02, 0.045, 0.08, 0.125, 0.175, 0.25, 0.35]
    assert lr.ramp_mode_min_s == pytest.approx(2.0)
    assert lr.ramp_time_budget_s == pytest.approx(1600.0)  # 段2o-8: 2060 → 1600（今の yaml）
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    good = list(lr.ramp_g_levels)
    too_big = cfg.vehicle.max_decel_g + 0.01
    for bad in ([], [0.0, 0.1], [-0.1, 0.1], [0.1, 0.1], [0.2, 0.1], [0.1, too_big]):
        lr.ramp_g_levels = bad
        assert problems_about("ramp_g_levels"), bad
    lr.ramp_g_levels = [0.1, cfg.vehicle.max_decel_g]  # 最大が max_decel_g ちょうどは合格
    assert not problems_about("ramp_g_levels")
    lr.ramp_g_levels = good

    lr.ramp_mode_min_s = 0.0
    assert problems_about("ramp_mode_min_s")
    lr.ramp_mode_min_s = 2.0
    lr.ramp_time_budget_s = 0.0
    assert problems_about("ramp_time_budget_s")
    lr.ramp_time_budget_s = 1200.0
    assert lr.ramp_max_passes == 1  # 今の yaml（2026-10-17 段2o-8 で 2 → 1。config.py の既定は別）
    lr.ramp_max_passes = 0
    assert problems_about("ramp_max_passes")
    lr.ramp_max_passes = 1
    assert not problems_about("ramp_max_passes")
    lr.ramp_max_passes = 2
    assert cfgmod.validate_config(cfg) == []


def test_ramp_top_margin_is_loaded_and_validated() -> None:
    """段2l-4: learning.ramp_top_margin_kmh（選んだモードの最高車速に足す余裕）。"""
    cfg = _load_default()
    lr = cfg.learning
    assert lr.ramp_top_margin_kmh == pytest.approx(5.0)
    lr.ramp_top_margin_kmh = -0.1
    assert any("ramp_top_margin_kmh" in p for p in cfgmod.validate_config(cfg))
    lr.ramp_top_margin_kmh = 0.0  # 0 は合格
    assert cfgmod.validate_config(cfg) == []


def test_ramp_control_settings_are_validated() -> None:
    """段2k 段B: learning.ramp_settle_time_s・gov_soft_frac の検証（段2l-8 で固定ゲイン・
    保持幅のキーを削除）。"""
    cfg = _load_default()
    lr = cfg.learning
    assert lr.ramp_settle_time_s == pytest.approx(2.0)
    assert not hasattr(lr, "ramp_rate_gain_pct_per_g_s")
    assert lr.gov_soft_frac == pytest.approx(0.95)
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    for key, bads, good in (
        ("ramp_settle_time_s", (0.0, -1.0), 2.0),
        ("gov_soft_frac", (0.0, 1.1, -0.5), 0.95),
    ):
        for bad in bads:
            setattr(lr, key, bad)
            assert problems_about(key), (key, bad)
        setattr(lr, key, good)
    lr.gov_soft_frac = 1.0  # 1 ちょうどは合格
    assert not problems_about("gov_soft_frac")
    lr.gov_soft_frac = 0.95
    assert cfgmod.validate_config(cfg) == []


def test_ramp_gref_time_settings_are_loaded_and_validated() -> None:
    """2026-10-09 段2l-9: G ランプの発進・切り替え・停止の時間（0 以上。0 は丸めない）。"""
    cfg = _load_default()
    lr = cfg.learning
    for key in ("ramp_launch_time_s", "ramp_stop_time_s"):
        assert getattr(lr, key) == pytest.approx(1.0)
    assert lr.ramp_turn_time_s == pytest.approx(2.0)  # 2026-10-10 段2l-10: 1.0 → 2.0
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    for key in ("ramp_launch_time_s", "ramp_turn_time_s", "ramp_stop_time_s"):
        setattr(lr, key, -0.1)
        assert problems_about(key), key
        setattr(lr, key, 0.0)  # 0 ちょうどは合格（丸めない）
        assert not problems_about(key), key
        setattr(lr, key, 2.0 if key == "ramp_turn_time_s" else 1.0)
    assert cfgmod.validate_config(cfg) == []


def test_ramp_hold_deadband_and_lag_tolerance_are_loaded_and_validated() -> None:
    """2026-10-10 段2l-10: 保持時間（0 以上）・偏差の不感帯（0 以上）・アクチュエータの遅れの
    許容（正値）。"""
    cfg = _load_default()
    lr = cfg.learning
    assert lr.ramp_hold_time_s == pytest.approx(7.0)
    assert lr.ramp_dev_deadband_kmh == pytest.approx(0.5)
    assert lr.ramp_actuator_lag_tol_pct == pytest.approx(2.0)
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    for key, good in (("ramp_hold_time_s", 7.0), ("ramp_dev_deadband_kmh", 0.5)):
        setattr(lr, key, -0.1)
        assert problems_about(key), key
        setattr(lr, key, 0.0)  # 0 ちょうどは合格（保持なし・不感帯なし）
        assert not problems_about(key), key
        setattr(lr, key, good)
    lr.ramp_actuator_lag_tol_pct = 0.0
    assert problems_about("ramp_actuator_lag_tol_pct")
    lr.ramp_actuator_lag_tol_pct = 2.0
    assert cfgmod.validate_config(cfg) == []


def test_ramp_gprop_without_sensitivity_default_false_and_yaml_true() -> None:
    """2026-10-31 段2b（D1）: 既定 false・本体 yaml は true。"""
    assert cfgmod.LearningSection().ramp_gprop_without_sensitivity is False
    assert _load_default().learning.ramp_gprop_without_sensitivity is True


def test_ramp_one_sided_is_loaded_from_yaml_and_defaults_to_false(tmp_path: Path) -> None:
    """2026-10-12 段2l-12: G ランプの片側調整。yaml は true、キーが無ければ既定の True。"""
    cfg = _load_default()
    assert cfg.learning.ramp_one_sided is True
    assert cfgmod.LearningSection().ramp_one_sided is True
    text = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    assert "  ramp_one_sided: true\n" in text
    off = tmp_path / "off.yaml"
    off.write_text(text.replace("  ramp_one_sided: true\n", "  ramp_one_sided: false\n"),
                   encoding="utf-8")
    assert cfgmod.load_config(off).learning.ramp_one_sided is False


def test_ramp_edge_g_is_loaded_and_validated() -> None:
    """2026-10-09 段2l-9: 発進の始まり・停止の終わりの狙いの G（0 以上。0 は下げ止めなし）。"""
    cfg = _load_default()
    assert cfg.learning.ramp_edge_g == pytest.approx(0.05)
    assert cfgmod.validate_config(cfg) == []
    cfg.learning.ramp_edge_g = -0.01
    assert any("ramp_edge_g" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.ramp_edge_g = 0.0
    assert not any("ramp_edge_g" in p for p in cfgmod.validate_config(cfg))


@pytest.mark.parametrize(
    "key", ["ramp_rate_gain_pct_per_g_s", "ramp_hold_margin_frac", "ramp_hold_margin_min_g"]
)
def test_removed_ramp_keys_give_clear_error(tmp_path: Path, key: str) -> None:
    """2026-10-09 段2l-8: 固定ゲイン・保持幅は廃止。yaml に残っていたら案内つきエラー。"""
    path = tmp_path / "c.yaml"
    path.write_text(f"learning:\n  {key}: 1.0\n", encoding="utf-8")
    with pytest.raises(cfgmod.ConfigError, match=f"{key} は廃止されました.*削除してください"):
        cfgmod.load_config(path)


def test_stop_brake_offset_key_loads_and_gives_absolute_opening(tmp_path: Path) -> None:
    """2026-10-08 段2l-7: yaml は min からの量。絶対開度は min ＋ 量（max で頭打ち）。"""
    path = tmp_path / "c.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save({"vehicle.min_brake_opening_pct": 2.9, "feedforward.stop_brake_offset_pct": 15.73})
    cfg = cfgmod.load_config(path)
    assert cfg.feedforward.stop_brake_offset_pct == pytest.approx(15.73)
    assert cfg.stop_brake_opening_pct == pytest.approx(18.63)
    assert cfgmod.validate_config(cfg) == []
    cfg.vehicle.max_brake_opening_pct = 10.0  # 頭打ち
    assert cfg.stop_brake_opening_pct == pytest.approx(10.0)


def test_old_stop_brake_opening_key_is_rejected_with_hint(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    cfgmod.load_config(path)  # 既定からコピーされる
    text = path.read_text(encoding="utf-8")
    assert "stop_brake_offset_pct:" in text
    path.write_text(
        text.replace("stop_brake_offset_pct:", "stop_brake_opening_pct:", 1), encoding="utf-8"
    )
    with pytest.raises(cfgmod.ConfigError, match="stop_brake_offset_pct"):
        cfgmod.load_config(path)


def test_stop_brake_offset_is_validated() -> None:
    cfg = _load_default()
    cfg.feedforward.stop_brake_offset_pct = -0.1
    assert any("stop_brake_offset_pct" in p for p in cfgmod.validate_config(cfg))
    cfg = _load_default()
    cfg.vehicle.min_brake_opening_pct = 5.0
    cfg.feedforward.stop_brake_offset_pct = 96.0  # min ＋ 量 > 100
    assert any("100 を超え" in p for p in cfgmod.validate_config(cfg))


def test_ramp_min_at_g_s_is_loaded_and_validated() -> None:
    """2026-10-10 段2l-11: 強い G の段で基準が狙いの G にいる時間の下限（0 以上。0 は無効）。"""
    cfg = _load_default()
    assert cfg.learning.ramp_min_at_g_s == pytest.approx(3.25)
    assert cfgmod.validate_config(cfg) == []
    cfg.learning.ramp_min_at_g_s = -0.1
    assert any("ramp_min_at_g_s" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.ramp_min_at_g_s = 0.0  # 0 ちょうどは合格（最小の上端を使わない）
    assert not any("ramp_min_at_g_s" in p for p in cfgmod.validate_config(cfg))


def test_ramp_cruise_and_lag_settings_are_loaded_and_validated() -> None:
    """2026-10-13 段2o-2: 途中の定速保持と遅れの測定のキー。yaml の値を読み、不正値は弾く。"""
    cfg = _load_default()
    lr = cfg.learning
    assert lr.ramp_cruise_band_g == pytest.approx(0.005)
    assert lr.ramp_cruise_hold_s == pytest.approx(5.0)  # 2026-10-19 段2s-2 短縮 7 → 5
    assert lr.ramp_cruise_timeout_s == pytest.approx(15.0)
    assert lr.ramp_cruise_fine_max_kmh == pytest.approx(100.0)
    assert lr.ramp_cruise_coarse_step_kmh == pytest.approx(20.0)
    assert list(lr.ramp_lag_speeds_kmh) == [30.0, 60.0]
    assert lr.ramp_lag_step_g == pytest.approx(0.05)
    assert lr.ramp_lag_step_hold_s == pytest.approx(2.5)
    assert lr.ramp_lag_approach_g == pytest.approx(0.125)
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    for key, good in (
        ("ramp_cruise_band_g", 0.005), ("ramp_cruise_fine_max_kmh", 100.0),
        ("ramp_cruise_coarse_step_kmh", 20.0), ("ramp_lag_step_g", 0.05),
        ("ramp_lag_step_hold_s", 2.5), ("ramp_lag_approach_g", 0.125),
    ):
        setattr(lr, key, 0.0)
        assert problems_about(key), key
        setattr(lr, key, good)
    lr.ramp_cruise_hold_s = -1.0
    assert problems_about("ramp_cruise_hold_s")
    lr.ramp_cruise_hold_s = 5.0
    lr.ramp_lag_speeds_kmh = [30.0, -5.0]
    assert problems_about("ramp_lag_speeds_kmh")
    lr.ramp_lag_speeds_kmh = []  # 空は「測らない」で合格
    assert not problems_about("ramp_lag_speeds_kmh")


def test_ramp_release_settings_are_loaded_and_validated() -> None:
    """2026-10-14 段2o-5: 下りの戻しのキー。yaml の値を読み、不正値は弾く（空は戻し無しで合格）。"""
    cfg = _load_default()
    lr = cfg.learning
    assert list(lr.ramp_release_fracs) == [0.5, 0.0]
    assert lr.ramp_release_move_s == pytest.approx(1.0)
    assert list(lr.ramp_release_hold_s) == [1.5, 2.0]
    assert lr.ramp_release_enabled is True  # 今の yaml の値（2026-10-23 段5-8 で true を採用）
    assert cfgmod.LearningSection().ramp_release_enabled is True  # config.py の既定
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    lr.ramp_release_hold_s = [1.5]  # 長さが合わない
    assert problems_about("ramp_release_fracs")
    lr.ramp_release_hold_s = [1.5, 2.0]
    lr.ramp_release_fracs = [0.0, 0.5]  # 降順でない
    assert problems_about("ramp_release_fracs")
    lr.ramp_release_fracs = [1.0, 0.0]  # 1 以上
    assert problems_about("ramp_release_fracs")
    lr.ramp_release_fracs = []  # 空は戻し無しで合格（保持も空）
    lr.ramp_release_hold_s = []
    assert not problems_about("ramp_release")
    lr.ramp_release_fracs = [0.5, 0.0]
    lr.ramp_release_hold_s = [1.5, 2.0]
    lr.ramp_release_move_s = 0.0
    assert problems_about("ramp_release_move_s")


def test_ramp_release_enabled_is_loaded_and_passed_to_the_loop(tmp_path: Path) -> None:
    """2026-10-16 段2o-6: learning.ramp_release_enabled を yaml から読み、ループの設定へ渡す。"""
    from tests.research.pattern_drive import loop_config_from_config

    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)  # 無ければ既定 yaml から作られる（今の yaml は true）
    assert cfg.learning.ramp_release_enabled is True
    assert loop_config_from_config(cfg).ramp_release_enabled is True
    cfg.save({"learning.ramp_release_enabled": False})
    cfg2 = cfgmod.load_config(path)
    assert cfg2.learning.ramp_release_enabled is False
    assert loop_config_from_config(cfg2).ramp_release_enabled is False


def test_ramp_launch_release_settings_are_validated() -> None:
    """段2o-10 A: G ランプの発進のゆっくり離し。0 以下・空配列は読み込みエラー。"""
    cfg = _load_default()
    lr = cfg.learning
    assert lr.ramp_launch_fast_rate_pctps == 2.0
    assert lr.ramp_launch_release_margin_pct == 2.0
    assert lr.ramp_launch_slow_rates_pctps == [0.25, 0.5]
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    lr.ramp_launch_fast_rate_pctps = 0.0
    assert problems_about("ramp_launch_fast_rate_pctps")
    lr.ramp_launch_fast_rate_pctps = 2.0
    lr.ramp_launch_release_margin_pct = -0.1
    assert problems_about("ramp_launch_release_margin_pct")
    lr.ramp_launch_release_margin_pct = 2.0
    lr.ramp_launch_slow_rates_pctps = []
    assert problems_about("ramp_launch_slow_rates_pctps")
    lr.ramp_launch_slow_rates_pctps = [0.25, 0.0]
    assert problems_about("ramp_launch_slow_rates_pctps")


def test_candidate_c11_is_accepted_and_dob_tq_coef_default_is_40() -> None:
    """config.py の既定は 40（yaml は 20: 段4-1）。yaml は読まず ForwardMapSection() で見る。"""
    cfg = _load_default()
    cfg.feedforward.candidate = "C11"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))
    assert "C11" in cfgmod.CANDIDATE_NAMES
    assert cfgmod.ForwardMapSection().dob_tq_coef == 40.0  # 2026-10-18 段2q（仮の全車共通係数）


def test_dob_tq_coef_is_loaded_from_yaml_and_validated(tmp_path: Path) -> None:
    text = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    assert "forward_map:\n" in text
    path = tmp_path / "dob.yaml"
    assert "\n  dob_tq_coef: 20.0\n" in text  # 今の yaml の値
    assert cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH).forward_map.dob_tq_coef == 20.0
    path.write_text(text.replace("\n  dob_tq_coef: 20.0\n", "\n  dob_tq_coef: 25.0\n", 1),
                    encoding="utf-8")
    cfg = cfgmod.load_config(path)
    assert cfg.forward_map.dob_tq_coef == 25.0  # yaml の値が読まれる
    assert not any("dob_tq_coef" in p for p in cfgmod.validate_config(cfg))
    for bad in (0.0, -1.0):
        cfg.forward_map.dob_tq_coef = bad
        assert any("forward_map.dob_tq_coef" in p for p in cfgmod.validate_config(cfg))


def test_candidate_c12_is_accepted_and_coast_defaults() -> None:
    cfg = _load_default()
    cfg.feedforward.candidate = "C12"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))
    assert "C12" in cfgmod.CANDIDATE_NAMES
    fm = cfgmod.ForwardMapSection()  # 2026-10-18 段2r（仮の全車共通の値）
    assert (fm.coast_on_kmh, fm.coast_off_kmh, fm.coast_hold_coef) == (0.5, 0.2, 4.0)
    assert fm.coast_floor_onset is False  # 既定は今まで通り
    assert fm.coast_one_pedal_per_section is True


def test_coast_floor_onset_is_loaded_and_must_be_bool(tmp_path: Path) -> None:
    path = tmp_path / "floor.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save({"forward_map.coast_floor_onset": True})
    assert cfgmod.load_config(path).forward_map.coast_floor_onset is True
    path.write_text(path.read_text().replace("coast_floor_onset: true", "coast_floor_onset: 1"))
    with pytest.raises(cfgmod.ConfigError):
        cfgmod.load_config(path)


def test_coast_one_pedal_per_section_is_loaded_and_must_be_bool(tmp_path: Path) -> None:
    path = tmp_path / "onepedal.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save({"forward_map.coast_one_pedal_per_section": False})
    assert cfgmod.load_config(path).forward_map.coast_one_pedal_per_section is False
    key = "coast_one_pedal_per_section"
    path.write_text(path.read_text().replace(f"{key}: false", f"{key}: 0"))
    with pytest.raises(cfgmod.ConfigError):
        cfgmod.load_config(path)


def test_coast_settings_are_loaded_from_yaml_and_validated(tmp_path: Path) -> None:
    path = tmp_path / "coast.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save({
        "forward_map.coast_on_kmh": 0.8,
        "forward_map.coast_off_kmh": 0.3,
        "forward_map.coast_hold_coef": 2.0,
    })
    cfg = cfgmod.load_config(path)
    fm = cfg.forward_map
    assert (fm.coast_on_kmh, fm.coast_off_kmh, fm.coast_hold_coef) == (0.8, 0.3, 2.0)
    assert not any("coast_" in p for p in cfgmod.validate_config(cfg))
    for name, bad in (
        ("coast_on_kmh", 0.0), ("coast_hold_coef", 0.0), ("coast_hold_coef", -1.0),
        ("coast_off_kmh", -0.1), ("coast_off_kmh", 0.8),  # off は 0 以上 on 未満
    ):
        cfg = cfgmod.load_config(path)
        setattr(cfg.forward_map, name, bad)
        assert any(f"forward_map.{name}" in p for p in cfgmod.validate_config(cfg)), (name, bad)


def test_fel_pattern_enabled_is_loaded_from_yaml_and_defaults_to_false(tmp_path: Path) -> None:
    """手順2 の 2-3（学習用パターン）を走るか。2026-10-23 ユーザー決定: 今後使わない。
    config.py の既定も yaml も false。"""
    cfg = _load_default()
    assert cfg.learning.fel_pattern_enabled is False
    assert cfgmod.LearningSection().fel_pattern_enabled is False
    text = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    assert "  fel_pattern_enabled: false\n" in text
    on = tmp_path / "on.yaml"
    on.write_text(
        text.replace("  fel_pattern_enabled: false\n", "  fel_pattern_enabled: true\n"),
        encoding="utf-8",
    )
    assert cfgmod.load_config(on).learning.fel_pattern_enabled is True
    # キーが無い yaml は既定の false
    none = tmp_path / "none.yaml"
    none.write_text(text.replace("  fel_pattern_enabled: false\n", ""), encoding="utf-8")
    assert cfgmod.load_config(none).learning.fel_pattern_enabled is False


def test_side_table_settings_default_and_validation(tmp_path: Path) -> None:
    """2026-10-20 段4-3: 発進・停止側の表の開度の元（既定 actual）／戻しの層（既定 false）。"""
    d = cfgmod.LearningSection()
    assert d.side_table_opening_source == "actual" and d.stop_release_layer_enabled is False
    text = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    assert "  side_table_opening_source: actual\n" in text
    assert "  stop_release_layer_enabled: false\n" in text
    cfg = _load_default()
    assert cfg.learning.side_table_opening_source == "actual"
    assert cfg.learning.stop_release_layer_enabled is False
    cfg.learning.side_table_opening_source = "cmd"
    assert not any("side_table_opening_source" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.side_table_opening_source = "bogus"
    assert any("side_table_opening_source" in p for p in cfgmod.validate_config(cfg))


def test_launch_breakaway_move_kmh_default_and_validation() -> None:
    """段7-a: 動き出す開度のしきい値。config.py の既定は None、null か正値だけ通る。"""
    assert cfgmod.LearningSection().launch_breakaway_move_kmh is None
    cfg = _load_default()
    assert cfg.learning.launch_breakaway_move_kmh == pytest.approx(0.05)
    for ok in (None, 0.05, 1.0):
        cfg.learning.launch_breakaway_move_kmh = ok
        assert not any("launch_breakaway_move_kmh" in p for p in cfgmod.validate_config(cfg))
    for bad in (0.0, -0.1):
        cfg.learning.launch_breakaway_move_kmh = bad
        assert any("launch_breakaway_move_kmh" in p for p in cfgmod.validate_config(cfg))


def test_accel_release_input_setting_default_off(tmp_path: Path) -> None:
    """アクセルの戻し量を入力にする設定。config.py の既定は false、今の yaml は true（段5-7）。"""
    assert cfgmod.LearningSection().accel_release_input_enabled is False  # config.py の既定
    text = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    assert "  accel_release_input_enabled: true\n" in text  # 今の yaml
    assert _load_default().learning.accel_release_input_enabled is True


def test_low_speed_side_rule_defaults_to_legacy_and_is_validated() -> None:
    assert cfgmod.ForwardMapSection().low_speed_side_rule == "legacy"  # config.py の既定
    cfg = _load_default()
    assert cfg.forward_map.low_speed_side_rule == "dir"  # 今の yaml（段5-6 でユーザー決定）
    assert not any("low_speed_side_rule" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.low_speed_side_rule = "pedal"
    assert not any("low_speed_side_rule" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.low_speed_side_rule = "bogus"
    assert any("low_speed_side_rule" in p for p in cfgmod.validate_config(cfg))


def test_stage53_settings_default_to_current_behaviour_and_are_validated() -> None:
    d = cfgmod.LearningSection()
    assert d.launch_accel_side_source == "extrapolate" and d.side_table_accel_window_s == 0.5
    assert cfgmod.ForwardMapSection().release_table_min_kmh == 0.0
    cfg = _load_default()  # 今の yaml の値（measured・1.0）
    assert cfg.learning.launch_accel_side_source == "measured"
    assert cfg.learning.side_table_accel_window_s == 1.0
    assert cfg.forward_map.release_table_min_kmh == 0.0
    keys = ("launch_accel_side_source", "side_table_accel_window_s", "release_table_min_kmh")
    assert not any(k in p for p in cfgmod.validate_config(cfg) for k in keys)
    cfg.learning.launch_accel_side_source = "measured"
    cfg.forward_map.release_table_min_kmh = 15.0
    assert not any(k in p for p in cfgmod.validate_config(cfg) for k in keys)
    cfg.learning.launch_accel_side_source = "bogus"
    cfg.learning.side_table_accel_window_s = 0.0
    cfg.forward_map.release_table_min_kmh = -1.0
    probs = cfgmod.validate_config(cfg)
    for k in keys:
        assert any(k in p for p in probs), k


def test_stage54_settings_default_to_current_behaviour_and_are_validated() -> None:
    d = cfgmod.LearningSection()
    assert d.launch_rows_to_stop_max is False and d.side_row_speed_rep == "grid"
    cfg = _load_default()
    assert cfg.learning.launch_rows_to_stop_max is False
    assert cfg.learning.side_row_speed_rep == "grid"
    assert not any("side_row_speed_rep" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.side_row_speed_rep = "data"
    assert not any("side_row_speed_rep" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.side_row_speed_rep = "bogus"
    assert any("side_row_speed_rep" in p for p in cfgmod.validate_config(cfg))


def test_switch_lead_enabled_defaults_to_false() -> None:
    cfg = _load_default()
    assert cfg.forward_map.switch_lead_enabled is False
    assert cfgmod.ForwardMapSection().switch_lead_enabled is False


def test_low_speed_side_rule_accepts_dir_and_rejects_unknown() -> None:
    cfg = _load_default()
    for rule in ("legacy", "pedal", "dir"):
        cfg.forward_map.low_speed_side_rule = rule
        assert not any("low_speed_side_rule" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.low_speed_side_rule = "nope"
    assert any("low_speed_side_rule" in p for p in cfgmod.validate_config(cfg))


def test_accel_push_input_setting_default_off() -> None:
    """2026-10-23 段5-7b: アクセルの踏み増し量を入力にする設定は既定 false。"""
    assert cfgmod.LearningSection().accel_push_input_enabled is False
    text = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    assert "  accel_push_input_enabled: false\n" in text
    assert _load_default().learning.accel_push_input_enabled is False
def test_stop_release_and_stop_exclusion_settings_default_and_validation() -> None:
    """2026-10-23 段5-8: 停止前の戻し B-1・停車まぎわ除外の設定。既定（yaml）と範囲外の検証。"""
    cfg = _load_default()
    lr = cfg.learning
    assert lr.stop_release_enabled is True
    assert lr.stop_release_start_kmh == pytest.approx(7.5)
    assert list(lr.stop_release_fracs) == [0.25, 0.5]
    assert lr.stop_release_rate_pct_s == pytest.approx(2.0)
    assert lr.stop_release_escape_s == pytest.approx(0.5)
    assert lr.stop_release_escape_decel_kmhs == pytest.approx(0.1)
    assert lr.stop_release_max_g == pytest.approx(0.35)
    assert lr.stop_approach_exclusion == "none"
    assert cfgmod.validate_config(cfg) == []
    for ok in ("window", "depth"):
        lr.stop_approach_exclusion = ok
        assert cfgmod.validate_config(cfg) == []
    lr.stop_approach_exclusion = "none"
    # dataclass の既定も同じ
    fresh = cfgmod.LearningSection()
    assert fresh.stop_release_enabled is True and fresh.stop_approach_exclusion == "none"
    assert list(fresh.stop_release_fracs) == [0.25, 0.5]

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    for key, bad, good in (
        ("stop_release_start_kmh", 0.0, 7.5),
        ("stop_release_rate_pct_s", 0.0, 2.0),
        ("stop_release_escape_s", -1.0, 0.5),
        ("stop_release_escape_decel_kmhs", -0.1, 0.1),
        ("stop_release_max_g", 0.0, 0.35),
        ("stop_release_fracs", [], [0.25, 0.5]),
        ("stop_release_fracs", [0.0], [0.25, 0.5]),
        ("stop_release_fracs", [1.5], [0.25, 0.5]),
        ("stop_approach_exclusion", "deep", "none"),
    ):
        setattr(lr, key, bad)
        assert problems_about(key), (key, bad)
        setattr(lr, key, good)
        assert not problems_about(key)


def test_launch_breakaway_settings_defaults_and_yaml() -> None:
    """段5-6 追加 A1・A2: config.py の既定は false、既定の yaml は true。"""
    assert cfgmod.ForwardMapSection().launch_midway_brake_cap is False
    assert cfgmod.ForwardMapSection().launch_table_breakaway_floor is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfg.forward_map.launch_midway_brake_cap is True
    assert cfg.forward_map.launch_table_breakaway_floor is True


def test_launch_midway_fb_no_deeper_default_false() -> None:
    """段6 C: config.py の既定も yaml も false。"""
    assert cfgmod.ForwardMapSection().launch_midway_fb_no_deeper is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfg.forward_map.launch_midway_fb_no_deeper is False


def test_keys_missing_in_yaml_lists_dataclass_keys_not_written(tmp_path: Path) -> None:
    """2026-10-23: yaml に書かれていないキー（既定値が効くもの）を列挙する。今の yaml は
    stop_brake_floor_* の 3 つだけ。キーを 1 つ消すと、そのキーが増える。"""
    text = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    base = cfgmod.keys_missing_in_yaml(cfgmod.DEFAULT_CONFIG_PATH)
    assert base == [
        "learning.stop_brake_floor_start_tol_kmh",
        "learning.stop_brake_floor_min_float_s",
        "learning.stop_brake_floor_opening_tol_pct",
    ]
    cut = tmp_path / "cut.yaml"
    cut.write_text(text.replace("  fel_pattern_enabled: false\n", ""), encoding="utf-8")
    assert set(cfgmod.keys_missing_in_yaml(cut)) == {*base, "learning.fel_pattern_enabled"}
    assert cfgmod.keys_missing_in_yaml(tmp_path / "none.yaml") == []


def test_launch_release_and_gap_compensate_settings() -> None:
    """段7-b・段7-c: 既定 off（2.0・2.0・5.0）。yaml は 7-b・7-c とも true（段7 で採った）。"""
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.launch_release_ramp is False and fm0.switch_gap_compensate is False
    assert (fm0.launch_release_lead_s, fm0.launch_release_pre_pct, fm0.launch_short_stop_s) == (
        2.0, 2.0, 5.0,
    )
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    fm = cfg.forward_map
    assert fm.launch_release_ramp is True and fm.switch_gap_compensate is True
    assert (fm.launch_release_lead_s, fm.launch_release_pre_pct, fm.launch_short_stop_s) == (
        2.0, 2.0, 5.0,
    )
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    for key, bad, good in (
        ("launch_release_lead_s", 0.0, 2.0),
        ("launch_release_lead_s", -1.0, 2.0),
        ("launch_release_pre_pct", -0.1, 0.0),
        ("launch_short_stop_s", -1.0, 0.0),
    ):
        setattr(fm, key, bad)
        assert problems_about(key), (key, bad)
        setattr(fm, key, good)
        assert not problems_about(key)


def test_launch_move_hold_and_fb_no_breakaway_floor_settings() -> None:
    """段8-a・段8-b: 既定は off・yaml は on（段8 で採った）。
    launch_move_hold は launch_release_ramp が必要。"""
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.launch_move_hold is False and fm0.fb_no_breakaway_floor is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfg.forward_map.launch_move_hold is True
    assert cfg.forward_map.fb_no_breakaway_floor is True
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.launch_move_hold = True
    cfg.forward_map.launch_release_ramp = False
    assert any("launch_move_hold" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.launch_release_ramp = True
    assert cfgmod.validate_config(cfg) == []


def test_switch_gap_min_loss_and_accel_release_in_range_only_settings() -> None:
    """段9-b・段9-c: 既定 off（本体 yaml の値は段の採否で変わるので見ない）。"""
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.switch_gap_min_loss is False and fm0.accel_release_in_range_only is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfgmod.validate_config(cfg) == []


def test_dob_fast_defaults_and_validation() -> None:
    fm0 = cfgmod.ForwardMapSection()  # 既定（本体 yaml の on/off は段の採否で変わるので見ない）
    assert fm0.dob_fast_enabled is False
    assert fm0.dob_fast_tq_coef == 10.0 and fm0.dob_fast_need_kmhs == 5.0
    cfg = _load_default()
    fm = cfg.forward_map
    assert cfgmod.validate_config(cfg) == []
    for name, value in (
        ("dob_fast_tq_coef", 0.0),
        ("dob_fast_tq_coef", fm.dob_tq_coef),
        ("dob_fast_need_kmhs", 0.0),
    ):
        bad = _load_default()
        setattr(bad.forward_map, name, value)
        assert any(f"forward_map.{name}" in p for p in cfgmod.validate_config(bad))


def test_switch_gap_rule_and_pedal_track_defaults_and_validation() -> None:
    """段10-1・段10-2: R は既定 off。ペダルの動き方の既定はデータ由来の値。"""
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.switch_gap_rule is False
    assert (fm0.pedal_track_accel_delay_s, fm0.pedal_track_accel_max_rate_pct_s) == (0.02, 100.0)
    assert fm0.pedal_track_accel_max_accel_pct_s2 == 2000.0
    assert (fm0.pedal_track_brake_delay_s, fm0.pedal_track_brake_max_rate_pct_s) == (0.02, 90.0)
    assert fm0.pedal_track_brake_max_accel_pct_s2 == 1400.0
    assert fm0.pedal_track_accel_effect_lag_s == 0.10 == fm0.pedal_track_brake_effect_lag_s
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.pedal_track_brake_max_accel_pct_s2 = 0.0
    assert any("pedal_track_brake_max_accel_pct_s2" in p for p in cfgmod.validate_config(cfg))


def test_switch_gap_min_peak_defaults_and_validation() -> None:
    """段10-3: 案 C は既定 off・窓 2.0 秒。R と同時に true はエラー、窓は正値。"""
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.switch_gap_min_peak is False and fm0.switch_gap_peak_window_s == 2.0
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfg.forward_map.switch_gap_min_peak is False and cfgmod.validate_config(cfg) == []
    cfg.forward_map.switch_gap_rule = False  # 本体 yaml は R on（段10-6）。C 単独は off にして見る
    cfg.forward_map.switch_gap_peak_robust = False  # 頑健化は R なしでは使えない
    cfg.forward_map.switch_gap_valley_robust = False
    cfg.forward_map.switch_gap_min_peak = True
    assert cfgmod.validate_config(cfg) == []  # R が false なら C だけ on は通る
    cfg.forward_map.switch_gap_rule = True
    assert any("switch_gap_min_peak" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.switch_gap_rule = False
    cfg.forward_map.switch_gap_peak_window_s = 0.0
    assert any("switch_gap_peak_window_s" in p for p in cfgmod.validate_config(cfg))


def test_switch_gap_peak_from_corner_default_false_in_class_and_yaml() -> None:
    """段11-4: R の山の下限を基準の頂点から取る設定は既定 false（クラス既定も本体 yaml も）。"""
    assert cfgmod.ForwardMapSection().switch_gap_peak_from_corner is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfg.forward_map.switch_gap_peak_from_corner is False


def test_switch_gap_choose_default_false_and_requires_rule() -> None:
    """段11-5: 選び直し（R のまま／後ろで取る）は既定 false。R なしで true はエラー。"""
    assert cfgmod.ForwardMapSection().switch_gap_choose is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfg.forward_map.switch_gap_choose is False and cfgmod.validate_config(cfg) == []
    cfg.forward_map.switch_gap_choose = True
    cfg.forward_map.switch_gap_rule = True
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.switch_gap_rule = False
    assert any("switch_gap_choose" in p for p in cfgmod.validate_config(cfg))


def test_switch_gap_peak_robust_default_false_and_requires_rule() -> None:
    """段12: R の山の採点の頑健化はクラスの既定は false。R なしで true はエラー。"""
    assert cfgmod.ForwardMapSection().switch_gap_peak_robust is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.forward_map.switch_gap_peak_robust = True
    cfg.forward_map.switch_gap_rule = True
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.switch_gap_rule = False
    assert any("switch_gap_peak_robust" in p for p in cfgmod.validate_config(cfg))


def test_switch_gap_valley_robust_default_false_and_requires_rule() -> None:
    """段12 谷: R の谷の上乗せの頑健化はクラスの既定は false。R なしで true はエラー。"""
    assert cfgmod.ForwardMapSection().switch_gap_valley_robust is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.forward_map.switch_gap_valley_robust = True
    cfg.forward_map.switch_gap_rule = True
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.switch_gap_rule = False
    assert any("switch_gap_valley_robust" in p for p in cfgmod.validate_config(cfg))


def test_step13_launch_keys_default_false_in_section_and_yaml() -> None:
    # 2026-10-27 段13: 発進のずらし・低速の予測の表の側は既定 off。
    # 本体 yaml は 2026-10-28 段5 の実機で on
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.predict_low_speed_sides is False and fm0.launch_shift_choose is False
    cfg = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
    assert cfg.forward_map.predict_low_speed_sides is True
    assert cfg.forward_map.launch_shift_choose is True
    assert cfgmod.validate_config(cfg) == []


def test_step14_keys_default_false_and_validated() -> None:
    # 2026-10-28 段14: クリープ曲線の予測・符号で離す惰行の補助は既定 off（本体 yaml も false）
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.predict_creep_curve is False and fm0.coast_release_on_cross is False
    cfg = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
    # 本体 yaml は 2026-10-28 段6・7 の実機で on
    assert cfg.forward_map.predict_creep_curve is True
    assert cfg.forward_map.coast_release_on_cross is True
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.coast_release_on_cross = True
    cfg.forward_map.predict_creep_curve = True
    cfg.forward_map.predict_low_speed_sides = True
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.predict_low_speed_sides = False
    assert any("predict_creep_curve" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.predict_low_speed_sides = True
    cfg.feedforward.creep_accel_speeds_kmh = []
    cfg.feedforward.creep_accel_kmhs = []
    assert any("predict_creep_curve" in p for p in cfgmod.validate_config(cfg))


def test_ramp_dither_settings_default_off_and_validation() -> None:
    """2026-10-28 段2: 効き始めの少し先での小さな動き。dataclass の既定・本体 yaml とも off、
    値は仕様どおり。範囲外は検証で弾く。"""
    fresh = cfgmod.LearningSection()
    assert fresh.ramp_dither_enabled is False
    cfg = _load_default()
    lr = cfg.learning
    assert lr.ramp_dither_enabled is False
    assert list(lr.ramp_dither_brake_speeds_kmh) == [100.0, 75.0, 50.0, 30.0]
    assert list(lr.ramp_dither_brake_u0_from_onset_pct) == [0.5, 1.5, 2.5]
    assert list(lr.ramp_dither_deltas_pct) == [0.25, 0.5]
    assert lr.ramp_dither_hold_s == pytest.approx(1.5)
    assert lr.ramp_dither_max_g == pytest.approx(0.13)
    assert lr.ramp_dither_accel_band_pct == pytest.approx(10.0)
    assert lr.ramp_dither_accel_max_n == 3
    assert list(fresh.ramp_dither_brake_speeds_kmh) == [100.0, 75.0, 50.0, 30.0]
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    for key, bad, good in (
        ("ramp_dither_brake_speeds_kmh", [30.0, 60.0], [100.0, 75.0, 50.0, 30.0]),
        ("ramp_dither_brake_speeds_kmh", [0.0], [100.0, 75.0, 50.0, 30.0]),
        ("ramp_dither_deltas_pct", [], [0.25, 0.5]),
        ("ramp_dither_deltas_pct", [0.0], [0.25, 0.5]),
        ("ramp_dither_hold_s", 0.0, 1.5),
        ("ramp_dither_brake_u0_from_onset_pct", [], [0.5, 1.5, 2.5]),
        ("ramp_dither_brake_u0_from_onset_pct", [-0.1], [0.5, 1.5, 2.5]),
        ("ramp_dither_max_g", 0.0, 0.13),
        ("ramp_dither_accel_band_pct", 0.0, 10.0),
        ("ramp_dither_accel_max_n", -1, 3),
    ):
        setattr(lr, key, bad)
        assert problems_about(key), (key, bad)
        setattr(lr, key, good)
        assert not problems_about(key)
    lr.ramp_dither_brake_speeds_kmh = []  # 空はブレーキの dither なし（許す）
    assert not problems_about("ramp_dither_brake_speeds_kmh")


def test_step4_local_gain_and_release_layer_keys_default_false() -> None:
    """2026-10-28 段4: 新キーの既定は false（yaml の値は決める時期まで固定しない）。"""
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.low_speed_release_stop_layer is False
    assert fm0.local_gain_brake is False
    assert fm0.local_gain_accel is False
    cfg = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
    assert cfgmod.validate_config(cfg) == []  # キーは yaml にある（値は固定しない）


def test_coast_band_release_at_zero_default_false() -> None:
    # 2026-10-30 ProblemReport_20261030_惰行 段1: 惰行の帯の離しを差 0 にする案 B は既定 off
    # （コードの既定）。2026-10-31 ユーザー決定で本体 yaml は on
    assert cfgmod.ForwardMapSection().coast_band_release_at_zero is False
    cfg = _load_default()
    assert cfg.forward_map.coast_band_release_at_zero is True
    assert cfgmod.validate_config(cfg) == []


def test_coast_fill_max_s_default_zero_and_validated() -> None:
    # 2026-10-30 ProblemReport_20261030_惰行 段2: 短い惰行を埋める案 D は既定 0（無効）
    assert cfgmod.ForwardMapSection().coast_fill_max_s == 0.0
    cfg = _load_default()
    assert cfg.forward_map.coast_fill_max_s == 0.0
    cfg.forward_map.coast_fill_max_s = 2.0
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.coast_fill_max_s = -0.1
    assert any("coast_fill_max_s" in p for p in cfgmod.validate_config(cfg))


def test_stop_approach_keep_stop_table_defaults_false() -> None:
    """2026-10-31 段1（停止間際 A）: 新キーの既定は false（コード既定・本体 yaml とも）。"""
    assert cfgmod.ForwardMapSection().stop_approach_keep_stop_table is False
    cfg = cfgmod.load_config(Path("tests/research/config_testVehicle.yaml"))
    assert cfg.forward_map.stop_approach_keep_stop_table is False


def test_stop_approach_settings_default_yaml_and_validation() -> None:
    """2026-10-31 段2: 停止の手前の測定の設定。既定は off、本体 yaml は on。範囲外の検証。"""
    fresh = cfgmod.LearningSection()
    assert fresh.stop_approach_enabled is False
    assert list(fresh.stop_approach_g_levels) == [0.045, 0.08, 0.125, 0.15]
    assert list(fresh.stop_approach_counts) == [1, 1, 1, 2]
    assert fresh.stop_approach_accel_g == pytest.approx(0.125)
    assert fresh.stop_approach_hold_s == pytest.approx(5.0)
    assert fresh.stop_approach_settle_s == pytest.approx(1.0)
    assert fresh.stop_approach_max_g == pytest.approx(0.15)
    cfg = _load_default()
    lr = cfg.learning
    assert lr.stop_approach_enabled is True
    assert list(lr.stop_approach_g_levels) == [0.045, 0.08, 0.125, 0.15]
    assert list(lr.stop_approach_counts) == [1, 1, 1, 2]
    assert lr.stop_approach_accel_g == pytest.approx(0.125)
    assert lr.stop_approach_hold_s == pytest.approx(5.0)
    assert lr.stop_approach_settle_s == pytest.approx(1.0)
    assert lr.stop_approach_max_g == pytest.approx(0.15)
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    for key, bad, good in (
        # max_g 超え
        ("stop_approach_g_levels", [0.045, 0.08, 0.125, 0.2], [0.045, 0.08, 0.125, 0.15]),
        ("stop_approach_g_levels", [0.0, 0.08, 0.125], [0.045, 0.08, 0.125, 0.15]),
        ("stop_approach_g_levels", [0.045, 0.08], [0.045, 0.08, 0.125, 0.15]),  # counts と長さ違い
        ("stop_approach_g_levels", [], [0.045, 0.08, 0.125, 0.15]),
        ("stop_approach_counts", [1, 0, 1, 2], [1, 1, 1, 2]),
        ("stop_approach_accel_g", 0.2, 0.125),  # max_g 超え
        ("stop_approach_accel_g", 0.0, 0.125),
        ("stop_approach_hold_s", -1.0, 5.0),
        ("stop_approach_settle_s", -0.1, 1.0),
        ("stop_approach_max_g", 0.0, 0.15),
    ):
        setattr(lr, key, bad)
        assert problems_about(key), (key, bad)
        setattr(lr, key, good)
        assert not problems_about(key)
    # 昇順でなくてよい（max_g 以下なら）
    lr.stop_approach_g_levels = [0.15, 0.045, 0.08, 0.125]
    assert cfgmod.validate_config(cfg) == []


def test_stop_table_sub_min_weight_defaults_false() -> None:
    """2026-10-31 段2d（案1）: 停止側のしきい値未満のマスの重み。既定は false、本体 yaml は true
    （段2d の実機の後、ユーザー決定で採用）。"""
    assert cfgmod.LearningSection().stop_table_sub_min_weight is False
    assert _load_default().learning.stop_table_sub_min_weight is True


def test_launch_table_sub_min_weight_defaults_false() -> None:
    """2026-10-31 段4b（案1）: 発進側のしきい値未満のマスの重み。既定も本体 yaml も false
    （段4b の実機 3 モードの後に決める）。"""
    assert cfgmod.LearningSection().launch_table_sub_min_weight is False
    assert _load_default().learning.launch_table_sub_min_weight is False
