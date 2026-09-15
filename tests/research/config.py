"""`config_testVehicle.yaml` の読み込み・検証・（コメント保持）保存。

研究開発用ハーネス（`tests/research/main.py`）が参照する唯一の設定ソース。
ハードウェア設定（シリアルポート・CAN・GPIO）は本番と共通の
`config/settings.toml` を読むため、ここには持たない。

**保存でコメントが消えない**ことを重視している。手順 2/4/6/8 は同定した
パラメータをこのファイルへ書き戻すが、ユーザーが編集するファイルなので
`yaml.safe_dump` による全文再生成は使わず、対象キーの**値の部分だけ**を
行単位で差し替える（`update_yaml_values`）。
"""

from __future__ import annotations

import re
import shutil
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any

import yaml

# 既定の設定ファイル（リポジトリに同梱。--config で別パスを指定すると
# ここからコピーして作られる）
DEFAULT_CONFIG_PATH = Path("tests/research/config_testVehicle.yaml")


class ConfigError(Exception):
    """設定ファイルの読み込み・検証で見つかった問題。"""


# ─────────────────────────────────────────────────────────────────────
# セクション定義（YAML のトップレベルキーと 1:1）
# ─────────────────────────────────────────────────────────────────────


@dataclass
class VehicleSection:
    name: str = "test_vehicle"
    max_speed_kmh: float = 140.0
    max_decel_g: float = 0.4
    max_accel_opening_pct: float = 80.0
    max_brake_opening_pct: float = 80.0
    stop_deviation_threshold_kmh: float = 2.0
    stop_deviation_duration_s: float = 4.0


#: KAIZEN 5.8 節で比べる候補（V1 案スイッチ）。ff_candidate.CANDIDATE_CLASSES のキーと同じ
#: C6 は 2026-09-15 追加（骨格を定速階段の実測テーブルにする案。docs/memo.md 参照）
CANDIDATE_NAMES: tuple[str, ...] = ("C1", "C2", "C3", "C4", "C5", "C6")


@dataclass
class FeedforwardSection:
    model_path: str = "tests/research/results/models/ff_poly2.pkl"
    candidate: str = "C1"  # KAIZEN 表5-5 順7（V1 案スイッチ）。CANDIDATE_NAMES のいずれか
    creep_speed_kmh: float = 5.0
    creep_rate_kmhs: float = 0.5
    stop_brake_opening_pct: float = 20.0
    engine_brake_decel_kmhs: float = 1.6
    coast_decel_speeds_kmh: list[float] = field(default_factory=list)
    coast_decel_kmhs: list[float] = field(default_factory=list)
    pedal_gain_speeds_kmh: list[float] = field(default_factory=list)
    accel_gain_kmhs_per_pct: list[float] = field(default_factory=list)
    brake_gain_kmhs_per_pct: list[float] = field(default_factory=list)
    accel_deadband_pct: float = 0.5
    brake_deadband_pct: float = 10.0

    @property
    def is_model_trained(self) -> bool:
        """手順 2（モデル作成）が済んでいるか。"""
        return Path(self.model_path).exists()

    @property
    def is_curves_identified(self) -> bool:
        """惰行減速カーブとペダルゲイン曲線が同定済みか（2 点以上）。"""
        return (
            len(self.coast_decel_speeds_kmh) >= 2
            and len(self.pedal_gain_speeds_kmh) >= 2
            and len(self.accel_gain_kmhs_per_pct) >= 2
            and len(self.brake_gain_kmhs_per_pct) >= 2
        )


@dataclass
class PidSection:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    output_limit_pct: float = 50.0
    integral_limit_pct: float = 20.0
    derivative_filter_hz: float = 5.0


@dataclass
class ArbiterSection:
    enable_deadband_compensation: bool = False
    enable_rate_limit: bool = False
    enable_hysteresis: bool = False
    switch_hysteresis_pct: float = 0.5
    accel_rate_limit_pct_s: float = 200.0
    brake_rate_limit_pct_s: float = 300.0
    accel_reengage_dwell_s: float = 0.3
    accel_release_rate_pct_s: float = 10.0
    accel_min_step_pct: float = 0.2


@dataclass
class ControlSection:
    loop_interval_ms: int = 50
    log_interval_ms: int = 100
    can_max_speed_age_s: float = 0.2

    @property
    def loop_interval_s(self) -> float:
        return self.loop_interval_ms / 1000.0

    @property
    def log_every_n_cycles(self) -> int:
        return max(1, round(self.log_interval_ms / self.loop_interval_ms))


@dataclass
class ModesSection:
    wltp_mode_name: str = "01_WLTP_Low,Mid,Hi,ExHi"
    tuning_mode_name: str = "__verify_pattern__"
    # モード走行レポートの区間別集計（WLTP の Low/Mid/High/ExHi）。境界は区間数 − 1 個
    segment_names: list[str] = field(default_factory=lambda: ["Low", "Mid", "High", "ExHi"])
    segment_bounds_s: list[float] = field(default_factory=lambda: [589.0, 1022.0, 1477.0])

    def segment_at(self, t_s: float) -> str:
        """モード経過時間 [s] が属する区間名。"""
        for name, bound in zip(self.segment_names, self.segment_bounds_s, strict=False):
            if t_s < bound:
                return name
        return self.segment_names[-1] if self.segment_names else ""


@dataclass
class ModeDriveSection:
    # 実測減速度が vehicle.max_decel_g × 0.98 以上になったらブレーキ開度を頭打ちにし、超えている間
    # 1 周期ごとに governor_reduce_step_pct ずつ下げる安全網（手順 2 のパターン走行と同じ規則）
    decel_governor: bool = True
    governor_reduce_step_pct: float = 2.0
    # 使っていないペダルの待機位置 = 不感帯 − standby_margin_pct（0% から不感帯までの空走をなくす）
    pedal_standby: bool = True
    standby_margin_pct: float = 2.0


@dataclass
class TuningSection:
    max_runs: int = 10
    kp_search_range: list[float] = field(default_factory=lambda: [0.5, 8.0])
    ki_search_range: list[float] = field(default_factory=lambda: [0.0, 2.0])
    kd_search_range: list[float] = field(default_factory=lambda: [0.0, 1.0])
    select_metric: str = "p95_kmh"


@dataclass
class LearningSection:
    # A3・A4 で約 1020s の見積り。900s は記録だけ（本番へ移すときに削る）。2026-09-14 定速階段
    # （段2）を足すとスタブで 1200s に収まらないことを実測したため 1400s に引き上げた
    timeout_s: float = 1400.0
    coast_timeout_s: float = 90.0
    # ペダルの固定開度 = 2-0 で測った不感帯 + offset [%]（本番の絶対開度は原点から測ると遊びの中）
    accel_deadband_probe_offsets_pct: list[float] = field(
        default_factory=lambda: [0.5, 1.0, 2.0, 3.0, 5.0]
    )
    cruise_trim_offsets_pct: list[float] = field(default_factory=lambda: [1.0, 2.0, 3.0])
    brake_hold_offsets_pct: list[float] = field(
        default_factory=lambda: [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
    )
    # 研究側で本番のパターン列に足す段（A2・A5。空リストなら足さない）
    accel_sweep_add_offsets_pct: list[float] = field(default_factory=lambda: [2.0, 5.0, 8.0])
    brake_hold_low_offsets_pct: list[float] = field(default_factory=lambda: [0.5, 2.0, 4.0])
    brake_hold_low_start_kmh: float = 60.0
    # A3 トリム階段: trim_stair_start_kmh の各車速まで上げてから、不感帯 + offset を
    # この順（降順）に trim_stair_step_s ずつ保持する
    trim_stair_start_kmh: list[float] = field(default_factory=lambda: [120.0, 90.0, 50.0])
    trim_stair_offsets_pct: list[float] = field(default_factory=lambda: [8.0, 5.0, 2.0])
    trim_stair_step_s: float = 8.0
    # A4 低速 × 高ブレーキ: 不感帯 + brake_hold_hard_accel_offset_pct で
    # brake_hold_hard_start_kmh まで上げてから、ブレーキ不感帯 + offset を停車まで保持する
    brake_hold_hard_offsets_pct: list[float] = field(
        default_factory=lambda: [7.0, 17.0, 27.0, 37.0]
    )
    brake_hold_hard_start_kmh: float = 20.0
    brake_hold_hard_accel_offset_pct: float = 8.0
    # 2026-09-14 定速階段（段2。空リストなら足さない）: 各車速へ弱い PI で保持し、
    # 50 km/h 以上に定速保持の状態が無かったことへの対策（docs/memo.md 参照）
    cruise_hold_speeds_kmh: list[float] = field(
        default_factory=lambda: [30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0,
                                  130.0]
    )
    cruise_hold_settle_tol_kmh: float = 1.0  # 「保持できた」とみなす速度偏差の許容幅 [km/h]
    cruise_hold_settle_s: float = 3.0        # 許容幅に連続でこの秒数いたら保持タイマー開始
    cruise_hold_hold_s: float = 8.0          # 保持タイマー開始後、この秒数記録したら次の車速へ
    cruise_hold_step_timeout_s: float = 30.0  # 1 車速あたりの打ち切り（保持できなくても次へ）
    # PI ゲイン。実測の感度（手順3 の 110〜140 km/h で約 1.3〜2 km/h/s per %）で
    # 時定数が数秒になる値（kp×感度 ≈ 0.4〜0.6/s）。レートを絞って実開度が指令に追従できる
    # 速さ（手順3 の実測 ≈0.1s）に保つ
    cruise_hold_kp: float = 0.3               # 比例ゲイン [%/(km/h)]
    cruise_hold_ki: float = 0.05              # 積分ゲイン [%/(km/h・s)]
    cruise_hold_max_rate_pct_per_s: float = 1.0  # 1 周期あたりの開度変化量の上限 [%/s]
    cruise_hold_initial_offset_pct: float = 7.0  # 初期開度 = アクセル不感帯 + この値 [%]
    # ペダルゲイン推定に使うサンプル: 開度 ≥ 不感帯 + この値 [%]（本番は 5%）
    accel_gain_min_offset_pct: float = 0.5
    brake_gain_min_offset_pct: float = 0.5


@dataclass
class PedalSearchSection:
    step_mm: float = 0.5
    dwell_s: float = 1.0
    onset_margin_kmh: float = 0.3
    confirm_count: int = 2
    creep_stable_kmh: float = 0.2
    creep_min_speed_kmh: float = 1.0
    creep_timeout_s: float = 90.0
    accel_max_pct: float = 20.0
    brake_max_pct: float = 50.0
    stop_hold_margin_pct: float = 10.0


@dataclass
class DecelStopSection:
    target_decel_g: float = 0.2
    press_margin_g: float = 0.02
    release_above_g: float = 0.3
    step_mm: float = 0.5
    dwell_s: float = 1.0
    slope_window_s: float = 1.0
    approach_margin_pct: float = 1.0
    timeout_s: float = 60.0


@dataclass
class KpiSection:
    max_abs_deviation_kmh: float = 1.0
    p95_deviation_kmh: float = 0.4
    reversal_band_kmh: float = 0.3
    reversal_window_s: float = 5.0
    reversal_limit_per_window: float = 1.0


@dataclass
class ChecksSection:
    """手順 1（初期化）と走行前チェックの実施項目（false でスキップ）。既定は安全側で全 true。"""

    init_servo_comm: bool = True
    init_clear_errors: bool = True
    init_servo_on: bool = True
    init_can: bool = True
    init_ups: bool = True
    init_home_return: bool = True
    pre_communication: bool = True
    pre_servo_state: bool = True
    pre_profile: bool = True
    pre_ups: bool = True
    pre_actuator_position: bool = True
    pre_brake_stop: bool = True
    pre_vehicle_stopped: bool = True


@dataclass
class OutputSection:
    results_dir: str = "tests/research/results"
    csv_interval_s: float = 0.1
    plot: bool = True
    print_interval_s: float = 1.0


@dataclass
class HardwareSection:
    settings_path: str = "config/settings.toml"
    database_url: str = "postgresql://localhost/driving_robot"


@dataclass
class ResearchConfig:
    """`config_testVehicle.yaml` 全体。`source_path` は読み込み元（保存先）。"""

    source_path: Path = DEFAULT_CONFIG_PATH
    vehicle: VehicleSection = field(default_factory=VehicleSection)
    feedforward: FeedforwardSection = field(default_factory=FeedforwardSection)
    pid: PidSection = field(default_factory=PidSection)
    arbiter: ArbiterSection = field(default_factory=ArbiterSection)
    control: ControlSection = field(default_factory=ControlSection)
    modes: ModesSection = field(default_factory=ModesSection)
    mode_drive: ModeDriveSection = field(default_factory=ModeDriveSection)
    tuning: TuningSection = field(default_factory=TuningSection)
    learning: LearningSection = field(default_factory=LearningSection)
    pedal_search: PedalSearchSection = field(default_factory=PedalSearchSection)
    decel_stop: DecelStopSection = field(default_factory=DecelStopSection)
    kpi: KpiSection = field(default_factory=KpiSection)
    checks: ChecksSection = field(default_factory=ChecksSection)
    output: OutputSection = field(default_factory=OutputSection)
    hardware: HardwareSection = field(default_factory=HardwareSection)

    @property
    def results_path(self) -> Path:
        return Path(self.output.results_dir)

    def save(self, updates: dict[str, Any]) -> list[str]:
        """ドット区切りキー → 値の辞書を、コメントを保ったまま書き戻す。

        Returns:
            実際に書き換えた行の説明（`"pid.kp: 0.0 -> 4.76"` 形式）。
        """
        return update_yaml_values(self.source_path, updates)


# YAML のトップレベルキー → セクション dataclass。`from __future__ import annotations`
# により `fields()` の型は文字列になるため、ここで明示的に対応づける。
_SECTION_TYPES: dict[str, type] = {
    "vehicle": VehicleSection,
    "feedforward": FeedforwardSection,
    "pid": PidSection,
    "arbiter": ArbiterSection,
    "control": ControlSection,
    "modes": ModesSection,
    "mode_drive": ModeDriveSection,
    "tuning": TuningSection,
    "learning": LearningSection,
    "pedal_search": PedalSearchSection,
    "decel_stop": DecelStopSection,
    "kpi": KpiSection,
    "checks": ChecksSection,
    "output": OutputSection,
    "hardware": HardwareSection,
}
assert _SECTION_TYPES.keys() == {
    f.name for f in fields(ResearchConfig) if f.name != "source_path"
}, "_SECTION_TYPES と ResearchConfig のフィールドが不一致"


# ─────────────────────────────────────────────────────────────────────
# 読み込み
# ─────────────────────────────────────────────────────────────────────


def ensure_config(path: Path) -> Path:
    """設定ファイルが無ければ同梱の既定ファイルからコピーして作る。"""
    if path.exists():
        return path
    if path.resolve() == DEFAULT_CONFIG_PATH.resolve():
        raise ConfigError(
            f"既定の設定ファイルが見つかりません: {path}\n"
            "リポジトリから復元してください（git checkout -- "
            f"{DEFAULT_CONFIG_PATH}）。"
        )
    if not DEFAULT_CONFIG_PATH.exists():
        raise ConfigError(f"コピー元の既定ファイルがありません: {DEFAULT_CONFIG_PATH}")
    path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(DEFAULT_CONFIG_PATH, path)
    return path


def load_config(path: Path = DEFAULT_CONFIG_PATH) -> ResearchConfig:
    """YAML を読み込んで `ResearchConfig` を返す。未知キー・型不一致は ConfigError。"""
    ensure_config(path)
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:  # pragma: no cover - 構文エラーは手動確認向け
        raise ConfigError(f"YAML の構文エラー: {path}\n{exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError(f"設定ファイルの最上位がマッピングではありません: {path}")

    unknown = sorted(set(raw) - set(_SECTION_TYPES))
    if unknown:
        raise ConfigError(f"未知のセクション: {', '.join(unknown)}（{path}）")

    kwargs: dict[str, Any] = {"source_path": path}
    for name, section_type in _SECTION_TYPES.items():
        kwargs[name] = _build_section(section_type, raw.get(name) or {}, name, path)
    return ResearchConfig(**kwargs)


def _build_section(section_type: type, raw: Any, name: str, path: Path) -> Any:
    if not is_dataclass(section_type):  # pragma: no cover - 定義ミス防止
        raise ConfigError(f"セクション定義が不正です: {name}")
    if not isinstance(raw, dict):
        raise ConfigError(f"セクション {name} がマッピングではありません（{path}）")
    known = {f.name: f for f in fields(section_type)}
    unknown = sorted(set(raw) - set(known))
    if unknown:
        raise ConfigError(f"未知のキー: {', '.join(f'{name}.{k}' for k in unknown)}（{path}）")
    kwargs = {k: _coerce(f"{name}.{k}", v, known[k].type) for k, v in raw.items()}
    return section_type(**kwargs)


def _coerce(dotted: str, value: Any, declared: Any) -> Any:
    """YAML の値を宣言型へ寄せる（int→float、list 要素の float 化）。"""
    if declared is bool or declared == "bool":
        if not isinstance(value, bool):
            raise ConfigError(f"{dotted} は true/false で指定してください（現在: {value!r}）")
        return value
    if declared is int or declared == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{dotted} は整数で指定してください（現在: {value!r}）")
        return value
    if declared is float or declared == "float":
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ConfigError(f"{dotted} は数値で指定してください（現在: {value!r}）")
        return float(value)
    if declared is str or declared == "str":
        if not isinstance(value, str):
            raise ConfigError(f"{dotted} は文字列で指定してください（現在: {value!r}）")
        return value
    if declared == "list[float]":
        if not isinstance(value, list) or any(
            isinstance(v, bool) or not isinstance(v, int | float) for v in value
        ):
            raise ConfigError(f"{dotted} は数値のリストで指定してください（現在: {value!r}）")
        return [float(v) for v in value]
    if declared == "list[str]":
        if not isinstance(value, list) or any(not isinstance(v, str) for v in value):
            raise ConfigError(f"{dotted} は文字列のリストで指定してください（現在: {value!r}）")
        return value
    return value  # pragma: no cover - 未使用の型は素通し


# ─────────────────────────────────────────────────────────────────────
# 検証
# ─────────────────────────────────────────────────────────────────────


def validate_config(cfg: ResearchConfig) -> list[str]:
    """値域と相互整合を検証し、問題のリストを返す（空なら合格）。"""
    problems: list[str] = []

    def need(cond: bool, message: str) -> None:
        if not cond:
            problems.append(message)

    v = cfg.vehicle
    need(bool(v.name.strip()), "vehicle.name が空です")
    need(0.0 < v.max_speed_kmh <= 200.0, f"vehicle.max_speed_kmh が範囲外: {v.max_speed_kmh}")
    need(0.0 < v.max_decel_g <= 1.0, f"vehicle.max_decel_g が範囲外(0<g<=1.0): {v.max_decel_g}")
    for label, opening in (
        ("max_accel_opening_pct", v.max_accel_opening_pct),
        ("max_brake_opening_pct", v.max_brake_opening_pct),
    ):
        need(0.0 < opening <= 100.0, f"vehicle.{label} が範囲外(0<pct<=100): {opening}")
    need(v.stop_deviation_threshold_kmh > 0.0, "vehicle.stop_deviation_threshold_kmh は正値")
    need(v.stop_deviation_duration_s > 0.0, "vehicle.stop_deviation_duration_s は正値")

    ff = cfg.feedforward
    need(bool(ff.model_path.strip()), "feedforward.model_path が空です")
    need(
        ff.candidate in CANDIDATE_NAMES,
        f"feedforward.candidate が不正です: {ff.candidate!r}（{CANDIDATE_NAMES} のいずれか）",
    )
    need(ff.creep_speed_kmh >= 0.0, "feedforward.creep_speed_kmh は 0 以上")
    need(ff.engine_brake_decel_kmhs > 0.0, "feedforward.engine_brake_decel_kmhs は正値")
    for label, opening in (
        ("accel_deadband_pct", ff.accel_deadband_pct),
        ("brake_deadband_pct", ff.brake_deadband_pct),
        ("stop_brake_opening_pct", ff.stop_brake_opening_pct),
    ):
        need(0.0 <= opening <= 100.0, f"feedforward.{label} が範囲外(0<=pct<=100): {opening}")
    problems += _validate_curve(
        "feedforward.coast_decel", ff.coast_decel_speeds_kmh, {"": ff.coast_decel_kmhs}
    )
    problems += _validate_curve(
        "feedforward.pedal_gain",
        ff.pedal_gain_speeds_kmh,
        {"accel_gain_kmhs_per_pct": ff.accel_gain_kmhs_per_pct,
         "brake_gain_kmhs_per_pct": ff.brake_gain_kmhs_per_pct},
    )

    p = cfg.pid
    for label, gain in (("kp", p.kp), ("ki", p.ki), ("kd", p.kd)):
        need(gain >= 0.0, f"pid.{label} は 0 以上（負値は正のフィードバックになる）: {gain}")
    need(0.0 < p.output_limit_pct <= 100.0, f"pid.output_limit_pct が範囲外: {p.output_limit_pct}")
    need(p.integral_limit_pct >= 0.0, "pid.integral_limit_pct は 0 以上")
    need(p.derivative_filter_hz >= 0.0, "pid.derivative_filter_hz は 0 以上（0 で無効）")

    a = cfg.arbiter
    need(a.switch_hysteresis_pct >= 0.0, "arbiter.switch_hysteresis_pct は 0 以上")
    need(a.accel_rate_limit_pct_s > 0.0, "arbiter.accel_rate_limit_pct_s は正値")
    need(a.brake_rate_limit_pct_s > 0.0, "arbiter.brake_rate_limit_pct_s は正値")
    need(a.accel_reengage_dwell_s >= 0.0, "arbiter.accel_reengage_dwell_s は 0 以上")
    need(a.accel_release_rate_pct_s > 0.0, "arbiter.accel_release_rate_pct_s は正値")
    need(a.accel_min_step_pct >= 0.0, "arbiter.accel_min_step_pct は 0 以上")

    c = cfg.control
    need(c.loop_interval_ms > 0, "control.loop_interval_ms は正値")
    need(c.log_interval_ms > 0, "control.log_interval_ms は正値")
    need(
        c.log_interval_ms % c.loop_interval_ms == 0,
        f"control.log_interval_ms({c.log_interval_ms}) は "
        f"loop_interval_ms({c.loop_interval_ms}) の整数倍にしてください",
    )
    need(c.can_max_speed_age_s > 0.0, "control.can_max_speed_age_s は正値")

    need(bool(cfg.modes.wltp_mode_name.strip()), "modes.wltp_mode_name が空です")
    need(bool(cfg.modes.tuning_mode_name.strip()), "modes.tuning_mode_name が空です")
    m = cfg.modes
    need(
        bool(m.segment_names)
        and len(m.segment_bounds_s) == len(m.segment_names) - 1
        and all(a < b for a, b in zip(m.segment_bounds_s, m.segment_bounds_s[1:], strict=False)),
        f"modes.segment_bounds_s（{m.segment_bounds_s}）は昇順で、segment_names"
        f"（{m.segment_names}）より 1 つ少なくしてください",
    )
    need(
        cfg.mode_drive.governor_reduce_step_pct > 0.0,
        "mode_drive.governor_reduce_step_pct は正値",
    )
    need(
        cfg.mode_drive.standby_margin_pct >= 0.0,
        "mode_drive.standby_margin_pct は 0 以上",
    )

    t = cfg.tuning
    need(t.max_runs >= 1, f"tuning.max_runs は 1 以上: {t.max_runs}")
    for label, rng in (
        ("kp_search_range", t.kp_search_range),
        ("ki_search_range", t.ki_search_range),
        ("kd_search_range", t.kd_search_range),
    ):
        if len(rng) != 2 or rng[0] > rng[1] or rng[0] < 0.0:
            problems.append(f"tuning.{label} は [下限, 上限]（0 以上・昇順）: {rng}")
    need(
        t.select_metric in {"p95_kmh", "max_abs_kmh"},
        f"tuning.select_metric は p95_kmh / max_abs_kmh のいずれか: {t.select_metric}",
    )

    need(cfg.learning.timeout_s > 0.0, "learning.timeout_s は正値")
    need(cfg.learning.coast_timeout_s > 0.0, "learning.coast_timeout_s は正値")
    lr = cfg.learning
    for label, offsets in (
        ("accel_deadband_probe_offsets_pct", lr.accel_deadband_probe_offsets_pct),
        ("cruise_trim_offsets_pct", lr.cruise_trim_offsets_pct),
        ("brake_hold_offsets_pct", lr.brake_hold_offsets_pct),
    ):
        need(
            bool(offsets)
            and all(0.0 < p <= 100.0 for p in offsets)
            and all(a < b for a, b in zip(offsets, offsets[1:], strict=False)),
            f"learning.{label} は 0<pct<=100 の昇順リスト: {offsets}",
        )
    for label, offsets in (
        ("accel_sweep_add_offsets_pct", lr.accel_sweep_add_offsets_pct),
        ("brake_hold_low_offsets_pct", lr.brake_hold_low_offsets_pct),
    ):
        need(
            all(0.0 < p <= 100.0 for p in offsets)
            and all(a < b for a, b in zip(offsets, offsets[1:], strict=False)),
            f"learning.{label} は 0<pct<=100 の昇順リスト（空は可）: {offsets}",
        )
    for label, start_kmh in (
        ("brake_hold_low_start_kmh", lr.brake_hold_low_start_kmh),
        ("brake_hold_hard_start_kmh", lr.brake_hold_hard_start_kmh),
    ):
        need(
            0.0 < start_kmh < cfg.vehicle.max_speed_kmh,
            f"learning.{label} は 0 より大きく vehicle.max_speed_kmh 未満: {start_kmh}",
        )
    need(
        all(0.0 < p <= 100.0 for p in lr.brake_hold_hard_offsets_pct)
        and all(a < b for a, b in zip(
            lr.brake_hold_hard_offsets_pct, lr.brake_hold_hard_offsets_pct[1:], strict=False
        )),
        f"learning.brake_hold_hard_offsets_pct は 0<pct<=100 の昇順リスト（空は可）: "
        f"{lr.brake_hold_hard_offsets_pct}",
    )
    need(
        0.0 < lr.brake_hold_hard_accel_offset_pct <= 100.0,
        f"learning.brake_hold_hard_accel_offset_pct は 0<pct<=100: "
        f"{lr.brake_hold_hard_accel_offset_pct}",
    )
    steps = lr.trim_stair_offsets_pct
    need(
        all(0.0 < p <= 100.0 for p in steps)
        and all(a > b for a, b in zip(steps, steps[1:], strict=False)),
        f"learning.trim_stair_offsets_pct は 0<pct<=100 の降順リスト（空は可）: {steps}",
    )
    need(
        all(0.0 < v < cfg.vehicle.max_speed_kmh for v in lr.trim_stair_start_kmh),
        f"learning.trim_stair_start_kmh は 0 より大きく vehicle.max_speed_kmh 未満: "
        f"{lr.trim_stair_start_kmh}",
    )
    need(
        not lr.trim_stair_start_kmh or bool(steps),
        "learning.trim_stair_start_kmh があるときは trim_stair_offsets_pct も要る",
    )
    need(lr.trim_stair_step_s > 0.0, f"learning.trim_stair_step_s は正値: {lr.trim_stair_step_s}")
    need(
        all(0.0 < v < cfg.vehicle.max_speed_kmh for v in lr.cruise_hold_speeds_kmh)
        and all(a < b for a, b in zip(
            lr.cruise_hold_speeds_kmh, lr.cruise_hold_speeds_kmh[1:], strict=False
        )),
        f"learning.cruise_hold_speeds_kmh は 0 より大きく vehicle.max_speed_kmh 未満の"
        f"昇順リスト（空は可）: {lr.cruise_hold_speeds_kmh}",
    )
    need(lr.cruise_hold_settle_tol_kmh > 0.0, "learning.cruise_hold_settle_tol_kmh は正値")
    need(lr.cruise_hold_settle_s > 0.0, "learning.cruise_hold_settle_s は正値")
    need(lr.cruise_hold_hold_s > 0.0, "learning.cruise_hold_hold_s は正値")
    need(lr.cruise_hold_step_timeout_s > 0.0, "learning.cruise_hold_step_timeout_s は正値")
    need(lr.cruise_hold_kp > 0.0, "learning.cruise_hold_kp は正値")
    need(lr.cruise_hold_ki >= 0.0, "learning.cruise_hold_ki は 0 以上")
    need(
        lr.cruise_hold_max_rate_pct_per_s > 0.0, "learning.cruise_hold_max_rate_pct_per_s は正値"
    )
    need(
        lr.cruise_hold_initial_offset_pct >= 0.0,
        "learning.cruise_hold_initial_offset_pct は 0 以上",
    )
    for label, offset in (
        ("accel_gain_min_offset_pct", lr.accel_gain_min_offset_pct),
        ("brake_gain_min_offset_pct", lr.brake_gain_min_offset_pct),
    ):
        need(offset > 0.0, f"learning.{label} は正値: {offset}")

    ps = cfg.pedal_search
    need(0.0 < ps.step_mm <= 5.0, f"pedal_search.step_mm が範囲外(0<mm<=5): {ps.step_mm}")
    need(ps.dwell_s > 0.0, "pedal_search.dwell_s は正値")
    need(ps.onset_margin_kmh > 0.0, "pedal_search.onset_margin_kmh は正値")
    need(ps.confirm_count >= 1, "pedal_search.confirm_count は 1 以上")
    need(ps.creep_stable_kmh > 0.0, "pedal_search.creep_stable_kmh は正値")
    need(ps.creep_min_speed_kmh >= 0.0, "pedal_search.creep_min_speed_kmh は 0 以上")
    need(ps.creep_timeout_s > 0.0, "pedal_search.creep_timeout_s は正値")
    need(
        0.0 < ps.accel_max_pct <= v.max_accel_opening_pct,
        f"pedal_search.accel_max_pct({ps.accel_max_pct}) は 0 超・アクセル開度上限以下",
    )
    need(
        0.0 < ps.brake_max_pct <= v.max_brake_opening_pct,
        f"pedal_search.brake_max_pct({ps.brake_max_pct}) は 0 超・ブレーキ開度上限以下",
    )
    need(ps.stop_hold_margin_pct >= 0.0, "pedal_search.stop_hold_margin_pct は 0 以上")

    ds = cfg.decel_stop
    need(
        0.0 < ds.press_margin_g < ds.target_decel_g,
        f"decel_stop.press_margin_g({ds.press_margin_g}) は 0 超・target_decel_g"
        f"({ds.target_decel_g}) 未満",
    )
    need(
        ds.target_decel_g < ds.release_above_g <= v.max_decel_g,
        f"decel_stop は target_decel_g({ds.target_decel_g}) < release_above_g"
        f"({ds.release_above_g}) <= vehicle.max_decel_g({v.max_decel_g}) にしてください",
    )
    need(0.0 < ds.step_mm <= 5.0, f"decel_stop.step_mm が範囲外(0<mm<=5): {ds.step_mm}")
    need(ds.dwell_s > 0.0, "decel_stop.dwell_s は正値")
    need(ds.slope_window_s > 0.0, "decel_stop.slope_window_s は正値")
    need(ds.approach_margin_pct >= 0.0, "decel_stop.approach_margin_pct は 0 以上")
    need(ds.timeout_s > 0.0, "decel_stop.timeout_s は正値")

    k = cfg.kpi
    need(k.max_abs_deviation_kmh > 0.0, "kpi.max_abs_deviation_kmh は正値")
    need(k.p95_deviation_kmh > 0.0, "kpi.p95_deviation_kmh は正値")
    need(
        k.p95_deviation_kmh <= k.max_abs_deviation_kmh,
        f"kpi.p95_deviation_kmh({k.p95_deviation_kmh}) が "
        f"max_abs_deviation_kmh({k.max_abs_deviation_kmh}) を超えています",
    )
    need(k.reversal_band_kmh > 0.0, "kpi.reversal_band_kmh は正値")
    need(k.reversal_window_s > 0.0, "kpi.reversal_window_s は正値")
    need(k.reversal_limit_per_window > 0.0, "kpi.reversal_limit_per_window は正値")

    o = cfg.output
    need(bool(o.results_dir.strip()), "output.results_dir が空です")
    need(o.csv_interval_s > 0.0, "output.csv_interval_s は正値")
    need(
        o.csv_interval_s >= cfg.control.log_interval_ms / 1000.0,
        f"output.csv_interval_s({o.csv_interval_s}s) が制御ログ周期"
        f"({cfg.control.log_interval_ms}ms)より短く、補間しないと埋められません",
    )
    need(o.print_interval_s > 0.0, "output.print_interval_s は正値")

    need(bool(cfg.hardware.settings_path.strip()), "hardware.settings_path が空です")

    chk = cfg.checks
    need(
        not (chk.pre_ups and not chk.init_ups),
        "checks.pre_ups が true なのに checks.init_ups が false です"
        "（UPS 監視を開始しないため残量が取得できません。両方 true か両方 false にしてください）",
    )
    return problems


def _validate_curve(
    label: str, speeds: list[float], values: dict[str, list[float]]
) -> list[str]:
    """速度グリッドと値列の長さ・単調性を検証する。空（未同定）は合格。"""
    problems: list[str] = []
    if not speeds and not any(values.values()):
        return problems
    if speeds != sorted(speeds):
        problems.append(f"{label}_speeds_kmh が昇順ではありません: {speeds}")
    for name, vals in values.items():
        suffix = f".{name}" if name else "_kmhs"
        if len(vals) != len(speeds):
            problems.append(
                f"{label}{suffix} の点数({len(vals)}) が速度グリッド({len(speeds)}) と不一致"
            )
        if any(v <= 0.0 for v in vals):
            problems.append(f"{label}{suffix} に 0 以下の値があります（正値で持つ規約）")
    return problems


# ─────────────────────────────────────────────────────────────────────
# コメントを保った書き戻し
# ─────────────────────────────────────────────────────────────────────

_KEY_RE = re.compile(r"^(?P<indent>[ ]*)(?P<key>[A-Za-z_][A-Za-z0-9_]*):(?P<rest>.*)$")


def update_yaml_values(path: Path, updates: dict[str, Any]) -> list[str]:
    """`{"pid.kp": 4.76}` のドット区切りキーで、対象行の値だけを書き換える。

    行の並び・インデント・コメントはそのまま残す。対象キーが見つからない、
    またはブロック形式のリスト（`- 1.0`）だった場合は ConfigError。
    書き込み後に読み直して値が一致することを確認する。
    """
    if not updates:
        return []
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    stack: list[tuple[int, str]] = []
    remaining = dict(updates)
    changed: list[str] = []

    for i, line in enumerate(lines):
        m = _KEY_RE.match(line.rstrip("\n"))
        if m is None:
            continue
        indent = len(m.group("indent"))
        while stack and stack[-1][0] >= indent:
            stack.pop()
        dotted = ".".join([k for _, k in stack] + [m.group("key")])
        stack.append((indent, m.group("key")))
        if dotted not in remaining:
            continue
        value_text, comment = _split_comment(m.group("rest"))
        if not value_text.strip():
            raise ConfigError(f"{dotted} は入れ子のマッピングで、値を書き換えられません")
        new_value = _format_value(remaining.pop(dotted))
        newline = "\n" if line.endswith("\n") else ""
        lines[i] = f"{m.group('indent')}{m.group('key')}: {new_value}{comment}{newline}"
        changed.append(f"{dotted}: {value_text.strip()} -> {new_value}")

    if remaining:
        raise ConfigError(
            "設定ファイルに次のキーが見つかりません（ブロック形式のリストは非対応）: "
            + ", ".join(sorted(remaining))
        )

    path.write_text("".join(lines), encoding="utf-8")
    _verify_written(path, updates)
    return changed


def _split_comment(rest: str) -> tuple[str, str]:
    """`" 1.0  # コメント"` を `(" 1.0", "  # コメント")` に分ける。

    引用符・角括弧の内側の `#` はコメント開始とみなさない。
    """
    depth = 0
    quote: str | None = None
    for i, ch in enumerate(rest):
        if quote is not None:
            if ch == quote:
                quote = None
            continue
        if ch in "\"'":
            quote = ch
        elif ch in "[{":
            depth += 1
        elif ch in "]}":
            depth -= 1
        elif ch == "#" and depth == 0 and (i == 0 or rest[i - 1] in " \t"):
            return rest[:i].rstrip(), "  " + rest[i:].rstrip()
    return rest.rstrip(), ""


def _format_value(value: Any) -> str:
    """YAML の 1 行値として書ける形へ整形する（リストは flow 形式）。"""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return _format_float(value)
    if isinstance(value, str):
        return _format_str(value)
    if isinstance(value, list | tuple):
        return "[" + ", ".join(_format_value(v) for v in value) + "]"
    if value is None:
        return "null"
    raise ConfigError(f"YAML の 1 行値へ整形できない型です: {type(value).__name__}")


def _format_float(value: float) -> str:
    """有効桁 6 桁に丸め、必ず小数点を含む表記にする（int と誤読させない）。"""
    text = f"{value:.6g}"
    if "e" in text or "E" in text or "inf" in text or "nan" in text:
        return repr(value)
    if "." not in text:
        text += ".0"
    return text


def _format_str(value: str) -> str:
    """カンマ・コロン・`#` を含む文字列は二重引用符で囲む。"""
    if value == "" or any(ch in value for ch in ",:#[]{}\"'\n") or value.strip() != value:
        escaped = value.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{escaped}"'
    return value


def _verify_written(path: Path, updates: dict[str, Any]) -> None:
    """書き込んだ値が読み直しで一致するか確認する（整形バグの早期検出）。"""
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    for dotted, expected in updates.items():
        node: Any = raw
        for part in dotted.split("."):
            node = node[part]
        if isinstance(expected, float) and isinstance(node, int | float):
            # _format_float は有効 6 桁に丸めるので、相対 1e-5 までは丸め誤差として許す
            if abs(float(node) - expected) > max(1e-9, abs(expected) * 1e-5):
                raise ConfigError(f"書き戻し検証に失敗: {dotted} = {node} (期待 {expected})")
        elif isinstance(expected, list | tuple):
            if len(node) != len(expected):
                raise ConfigError(f"書き戻し検証に失敗: {dotted} の点数不一致")
        elif node != expected:
            raise ConfigError(f"書き戻し検証に失敗: {dotted} = {node!r} (期待 {expected!r})")
