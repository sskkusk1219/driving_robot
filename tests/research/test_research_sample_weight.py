"""学習サンプルの WLTP 重み付け（sample_weight.py。ProblemReport_20260925 段2）のユニットテスト。

`compute_weights` 自体のテストと、`ff_candidate.train_inverse_model_effective` へ組み込んだときの
挙動（重みなしと完全に同じ結果になること・重みが実際に fit を変えること・pkl と metrics への
追加キー）を確かめる。
"""

from __future__ import annotations

import pickle
from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np
import pytest

from tests.research.ff_candidate import train_inverse_model_effective
from tests.research.ff_model import STOP_SPEED_KMH
from tests.research.research_types import DriveLog
from tests.research.sample_weight import WltpWeighting, compute_weights, summarize
from tests.research.test_research_ff_candidate import _logs, _mixed_rows, _profile

# 2 セル格子（速度 [0,10)/[10,20]、加速度 [-5,0)/[0,5]）で使う共通の境界
SPEED_EDGES = (0.0, 10.0, 20.0)
ACCEL_EDGES = (-5.0, 0.0, 5.0)


def _weighting(wltp_s: list[list[float]], w_min: float = 0.2, w_max: float = 5.0) -> WltpWeighting:
    return WltpWeighting(
        wltp_s=np.array(wltp_s, dtype=float), speed_edges_kmh=SPEED_EDGES,
        accel_edges_kmhs=ACCEL_EDGES, w_min=w_min, w_max=w_max, enabled=True,
    )


# ─────────────────────────────────────────────────────────────────────
# compute_weights
# ─────────────────────────────────────────────────────────────────────


def test_compute_weights_empty_input_returns_empty_array() -> None:
    w = compute_weights(np.empty(0), np.empty(0), _weighting([[1.0, 1.0], [1.0, 1.0]]))
    assert len(w) == 0


def test_compute_weights_no_rows_in_grid_returns_all_ones() -> None:
    """v0 が格子の外（速度上限超）なら重みは中立の 1.0 のまま（平均 1 の正規化でも変わらない）。"""
    v0 = np.full(5, 100.0)
    a_req = np.zeros(5)
    w = compute_weights(v0, a_req, _weighting([[10.0, 0.0], [0.0, 0.0]]))
    assert np.allclose(w, 1.0)


def test_compute_weights_matches_occupancy_ratio() -> None:
    """重み（clip 前）は「WLTP の占有率 ÷ 学習データの占有率」に一致する。

    WLTP: セル(0,0)=100s・セル(0,1)=0s・セル(1,0)=セル(1,1)=50s（計 200s）。
    学習データ: セル(0,0)に8行・セル(0,1)に2行（計10行、格子内総数10）。
    r(0,0) = (100/200) / (8/10) = 0.625、r(0,1) = 0（WLTP が 0 のセル → clip で w_min）。
    """
    v0 = np.array([5.0] * 8 + [5.0] * 2)
    a_req = np.array([-2.0] * 8 + [2.0] * 2)
    weighting = _weighting([[100.0, 0.0], [50.0, 50.0]])
    w = compute_weights(v0, a_req, weighting)

    raw_ratio = 0.625 / 0.2  # clip 前の r(0,0)/r(0,1)（正規化の定数倍は打ち消される）
    assert np.allclose(w[:8], w[0])  # 同じセル(0,0)の行はすべて同じ重み
    assert np.allclose(w[8:], w[8])  # 同じセル(0,1)の行はすべて同じ重み
    assert float(w[0] / w[8]) == pytest.approx(raw_ratio)
    assert float(np.mean(w)) == pytest.approx(1.0)  # 平均 1 に正規化


def test_compute_weights_wltp_zero_cell_maps_to_w_min() -> None:
    """WLTP が 0 のセルの行は r=0 → clip で w_min になる（占有率がどれだけ高くても上げない）。"""
    v0 = np.array([5.0] * 9 + [15.0])
    a_req = np.array([2.0] * 9 + [-2.0])  # 9 行はセル(0,1)（WLTP=0）、1 行はセル(1,0)
    weighting = _weighting([[0.0, 0.0], [1.0, 0.0]], w_min=0.3, w_max=5.0)
    w = compute_weights(v0, a_req, weighting)
    # セル(1,0) の r = (1/1)/(1/10) = 10 → clip で w_max、セル(0,1) の r は 0 → clip で w_min
    assert float(w[-1] / w[0]) == pytest.approx(5.0 / 0.3)


def test_compute_weights_clips_to_bounds() -> None:
    """占有率比が範囲外なら w_min/w_max に clip する（比の値そのものはどちらも壊す設定で確認）。"""
    v0 = np.array([5.0] + [15.0] * 9)
    a_req = np.array([-2.0] + [2.0] * 9)
    # セル(0,0): WLTP 1000/1000=1.0、データ 1/10=0.1 → r=10 → clip で w_max
    # セル(1,1): WLTP 0/1000=0 → r=0 → clip で w_min
    weighting = _weighting([[1000.0, 0.0], [0.0, 0.0]], w_min=0.2, w_max=5.0)
    w = compute_weights(v0, a_req, weighting)
    assert float(w[0] / w[1]) == pytest.approx(5.0 / 0.2)


def test_compute_weights_out_of_grid_rows_are_neutral_before_normalization() -> None:
    """格子外の行は正規化前 1.0（中立）。格子内の行と異なる値を持つことで確かめる。"""
    v0 = np.array([5.0] * 8 + [5.0] * 2 + [25.0] * 5)  # 末尾 5 行は速度が格子の外
    a_req = np.array([-2.0] * 8 + [2.0] * 2 + [0.0] * 5)
    weighting = _weighting([[100.0, 0.0], [50.0, 50.0]])
    w = compute_weights(v0, a_req, weighting)
    mean_raw = (8 * 0.625 + 2 * 0.2 + 5 * 1.0) / 15  # clip 後・正規化前の平均
    assert float(w[-1]) == pytest.approx(1.0 / mean_raw)
    assert float(np.mean(w)) == pytest.approx(1.0)


def test_compute_weights_v0_below_stop_speed_is_treated_as_out_of_grid() -> None:
    """v0 < STOP_SPEED_KMH は格子の範囲内でも「格子外」（クリープ・停車保持は逆モデルの対象外）。"""
    v0 = np.array([STOP_SPEED_KMH / 2.0])  # 格子(0,0) の範囲内だが停車扱い
    a_req = np.array([-2.0])
    weighting = _weighting([[100.0, 0.0], [50.0, 50.0]])
    w = compute_weights(v0, a_req, weighting)
    assert float(w[0]) == pytest.approx(1.0)


def test_compute_weights_mean_is_always_one() -> None:
    rng = np.random.default_rng(0)
    v0 = rng.uniform(0.0, 20.0, size=200)
    a_req = rng.uniform(-5.0, 5.0, size=200)
    weighting = _weighting([[3.0, 7.0], [40.0, 2.0]])
    w = compute_weights(v0, a_req, weighting)
    assert float(np.mean(w)) == pytest.approx(1.0)


# ─────────────────────────────────────────────────────────────────────
# summarize
# ─────────────────────────────────────────────────────────────────────


def test_summarize_empty() -> None:
    s = summarize(np.empty(0))
    assert s == {
        "n": 0.0, "min": 0.0, "max": 0.0, "mean": 0.0,
        "at_min_ratio": 0.0, "at_max_ratio": 0.0,
    }


def test_summarize_reports_bounds_and_ratios() -> None:
    w = np.array([0.5, 0.5, 1.0, 2.0, 2.0, 2.0])
    s = summarize(w)
    assert s["n"] == 6.0
    assert s["min"] == pytest.approx(0.5)
    assert s["max"] == pytest.approx(2.0)
    assert s["mean"] == pytest.approx(float(np.mean(w)))
    assert s["at_min_ratio"] == pytest.approx(2.0 / 6.0)
    assert s["at_max_ratio"] == pytest.approx(3.0 / 6.0)


# ─────────────────────────────────────────────────────────────────────
# train_inverse_model_effective との結線
# ─────────────────────────────────────────────────────────────────────


def _uniform_weighting(*, enabled: bool) -> WltpWeighting:
    """どの行も同じセルに落ちる（1 セルの格子）ので、必ず全行の重みが 1.0 になる。"""
    return WltpWeighting(
        wltp_s=np.array([[1.0]]), speed_edges_kmh=(0.0, 1e6), accel_edges_kmhs=(-1e6, 1e6),
        w_min=0.2, w_max=5.0, enabled=enabled,
    )


def _logs_multi_session(
    rows_by_session: dict[str, list[tuple[float, float, float]]],
) -> list[DriveLog]:
    """複数セッションの合成ログ（_logs の複数セッション版。段2 の重み付けテスト専用）。"""
    t0 = datetime(2026, 9, 25, tzinfo=UTC)
    out: list[DriveLog] = []
    i = 0
    for session_id, rows in rows_by_session.items():
        for k, (v, a, b) in enumerate(rows):
            out.append(
                DriveLog(
                    id=i, session_id=session_id, timestamp=t0 + timedelta(seconds=0.1 * k),
                    ref_speed_kmh=None, actual_speed_kmh=v, accel_opening=a, brake_opening=b,
                    accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
                )
            )
            i += 1
    return out


def _two_cluster_logs() -> list[DriveLog]:
    """低速クラスタ（アクセル一定22%）・高速クラスタ（アクセル一定38%）・ブレーキ用セッション。

    低速/高速で目的変数（開度ラベル）が違うので、重みでどちらのクラスタを重視するかにより
    Ridge の当てはまりが変わる（`test_weighting_enabled_changes_fit` 用）。
    """
    rows_low: list[tuple[float, float, float]] = []
    v = 2.0
    for _ in range(200):
        v += 0.05
        rows_low.append((v, 22.0, 0.0))
    rows_high: list[tuple[float, float, float]] = []
    v = 40.0
    for _ in range(200):
        v += 0.05
        rows_high.append((v, 38.0, 0.0))
    rows_brake: list[tuple[float, float, float]] = []
    v = 10.0
    for _ in range(200):
        v = max(1.0, v - 0.02)
        rows_brake.append((v, 0.0, 20.0))
    return _logs_multi_session({"s_low": rows_low, "s_high": rows_high, "s_brake": rows_brake})


def test_weighting_none_and_disabled_give_identical_predictions(tmp_path: Path) -> None:
    """weighting=None と、weighting はあるが enabled=False は同じ経路（fit に重みを渡さない）。"""
    logs = _logs(_mixed_rows())
    profile = _profile()
    path_none, metrics_none = train_inverse_model_effective(
        logs, profile, output_dir=str(tmp_path / "none")
    )
    path_disabled, metrics_disabled = train_inverse_model_effective(
        logs, profile, output_dir=str(tmp_path / "disabled"),
        weighting=_uniform_weighting(enabled=False),
    )
    with Path(path_none).open("rb") as f:
        model_none = pickle.load(f)["accel_model"]  # noqa: S301 - テストで作った自前のファイル
    with Path(path_disabled).open("rb") as f:
        model_disabled = pickle.load(f)["accel_model"]  # noqa: S301

    assert metrics_none["accel"]["mae"] == pytest.approx(metrics_disabled["accel"]["mae"])
    assert "mae_wltp" not in metrics_none["accel"]
    assert "mae_wltp" in metrics_disabled["accel"]  # weighting 指定時は enabled に関係なく入る
    assert np.allclose(
        model_none.named_steps["ridge"].coef_, model_disabled.named_steps["ridge"].coef_
    )


def test_uniform_weighting_matches_unweighted(tmp_path: Path) -> None:
    """全行の重みが 1.0 相当（1 セルの格子）なら、重み付き fit は重みなしとほぼ一致する。"""
    logs = _logs(_mixed_rows())
    profile = _profile()
    path_none, metrics_none = train_inverse_model_effective(
        logs, profile, output_dir=str(tmp_path / "none")
    )
    path_uniform, metrics_uniform = train_inverse_model_effective(
        logs, profile, output_dir=str(tmp_path / "uniform"),
        weighting=_uniform_weighting(enabled=True),
    )
    with Path(path_none).open("rb") as f:
        model_none = pickle.load(f)["accel_model"]  # noqa: S301
    with Path(path_uniform).open("rb") as f:
        model_uniform = pickle.load(f)["accel_model"]  # noqa: S301

    assert np.allclose(
        model_none.named_steps["ridge"].coef_, model_uniform.named_steps["ridge"].coef_, atol=1e-8
    )
    assert metrics_uniform["accel"]["mae"] == pytest.approx(metrics_none["accel"]["mae"], abs=1e-8)


def test_weighting_enabled_changes_fit(tmp_path: Path) -> None:
    """偏った重み（片方のクラスタを強く優先）は enabled=True のときだけ fit を変える。"""
    logs = _two_cluster_logs()
    profile = _profile()
    weighting = WltpWeighting(
        wltp_s=np.array([[100.0], [1.0]]),  # 低速クラスタ側を優先
        speed_edges_kmh=(0.0, 20.0, 60.0), accel_edges_kmhs=(-100.0, 100.0),
        w_min=0.2, w_max=5.0, enabled=True,
    )
    _, metrics_unweighted = train_inverse_model_effective(
        logs, profile, output_dir=str(tmp_path / "u")
    )
    _, metrics_weighted = train_inverse_model_effective(
        logs, profile, output_dir=str(tmp_path / "w"), weighting=weighting
    )
    assert metrics_weighted["accel"]["mae"] != pytest.approx(metrics_unweighted["accel"]["mae"])

    # enabled=False なら重みを持っていても fit には反映されない（mae は重みなしと一致）
    weighting_off = WltpWeighting(
        wltp_s=weighting.wltp_s, speed_edges_kmh=weighting.speed_edges_kmh,
        accel_edges_kmhs=weighting.accel_edges_kmhs, w_min=weighting.w_min, w_max=weighting.w_max,
        enabled=False,
    )
    _, metrics_off = train_inverse_model_effective(
        logs, profile, output_dir=str(tmp_path / "off"), weighting=weighting_off
    )
    assert metrics_off["accel"]["mae"] == pytest.approx(metrics_unweighted["accel"]["mae"])
    assert "mae_wltp" in metrics_off["accel"]  # off でも比較用の指標は出る


def test_pkl_sample_weight_key_reflects_weighting(tmp_path: Path) -> None:
    logs = _logs(_mixed_rows())
    profile = _profile()

    path_none, _ = train_inverse_model_effective(logs, profile, output_dir=str(tmp_path / "none"))
    with Path(path_none).open("rb") as f:
        payload_none = pickle.load(f)  # noqa: S301
    assert payload_none["sample_weight"] == {"enabled": False}

    weighting = _uniform_weighting(enabled=True)
    path_w, _ = train_inverse_model_effective(
        logs, profile, output_dir=str(tmp_path / "w"), weighting=weighting
    )
    with Path(path_w).open("rb") as f:
        payload_w = pickle.load(f)  # noqa: S301
    assert payload_w["sample_weight"] == {"enabled": True, "w_min": 0.2, "w_max": 5.0}


def test_metrics_include_weight_summary_when_weighting_given(tmp_path: Path) -> None:
    logs = _two_cluster_logs()
    profile = _profile()
    weighting = WltpWeighting(
        wltp_s=np.array([[100.0], [1.0]]), speed_edges_kmh=(0.0, 20.0, 60.0),
        accel_edges_kmhs=(-100.0, 100.0), w_min=0.2, w_max=5.0, enabled=True,
    )
    _, metrics = train_inverse_model_effective(
        logs, profile, output_dir=str(tmp_path), weighting=weighting
    )
    m = metrics["accel"]
    assert m["weight_mean"] == pytest.approx(1.0)  # ペダルごとに平均 1 へ正規化
    assert 0.0 < m["weight_min"] <= m["weight_mean"] <= m["weight_max"]
    assert 0.0 <= m["weight_at_min_ratio"] <= 1.0
    assert 0.0 <= m["weight_at_max_ratio"] <= 1.0
