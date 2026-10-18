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
from typing import Any, ClassVar

import yaml

from tests.research.dynamics_estimation import COAST_CURVE_BIN_KMH
from tests.research.ff_model import FeatureSpec

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
    # 2026-10-08 段2l-6（ProblemReport_20261001）: ペダルに触れた位置（手で測る。
    # tests/hardware/test_calibration.py）。待機位置であり、効く開度 u の原点でもある
    # （不感帯の測定はやめた）
    min_accel_opening_pct: float = 6.5
    min_brake_opening_pct: float = 2.9
    max_accel_opening_pct: float = 80.0
    max_brake_opening_pct: float = 80.0
    stop_deviation_threshold_kmh: float = 2.0
    stop_deviation_duration_s: float = 4.0


#: KAIZEN 5.8 節で比べる候補（V1 案スイッチ）。ff_candidate.CANDIDATE_CLASSES のキーと同じ
#: C6（定速階段の実測テーブルを骨格にする案）は 2026-09-25 段4 で削除した
#: C7（順モデルの表を逆に引いて走る前に作った開度の列を FF にする案。段2g）。
#: C8（同じ格子のモデル版。fwd_model_*.pkl）。C9 は後の段で足す。
#: C10（C8 の開度の列 ＋ 走行中に偏差の分だけ上乗せ。pkl は C8 と同じ。2026-10-17 段2p）
#: C11（C10 ＋ 外乱オブザーバ DOB。pkl は C8 と同じ。2026-10-18 段2q）
#: C12（C11 ＋ 惰行の行の補助。pkl は C8 と同じ。2026-10-18 段2r）
CANDIDATE_NAMES: tuple[str, ...] = (
    "C1", "C2", "C3", "C4", "C5", "C7", "C8", "C10", "C11", "C12",
)

# C10 が表を引く車速の行（forward_map.deviation_row）。reference＝案A・actual＝案B
DEVIATION_ROWS: tuple[str, ...] = ("reference", "actual")


@dataclass
class FeedforwardSection:
    model_path: str = "tests/research/results/models/ff_poly2.pkl"
    candidate: str = "C1"  # KAIZEN 表5-5 順7（V1 案スイッチ）。CANDIDATE_NAMES のいずれか
    creep_speed_kmh: float = 5.0
    creep_rate_kmhs: float = 0.5
    # 2026-10-08 段2l-7: 停車保持ブレーキ開度の「ブレーキの min（vehicle.min_brake_opening_pct）
    # からの量」[%]。絶対開度は ResearchConfig.stop_brake_opening_pct（min ＋ この値）。
    # 取り付け直しで min が変わっても手順2 をやり直さずに済ませるため絶対開度から変えた
    stop_brake_offset_pct: float = 15.0
    engine_brake_decel_kmhs: float = 1.6
    coast_decel_speeds_kmh: list[float] = field(default_factory=list)
    coast_decel_kmhs: list[float] = field(default_factory=list)
    # クリープ加速カーブ（0〜creep_speed_kmh の速度依存クリープ加速度。ProblemReport_20260916
    # 課題#2）。ResearchFFParams 側で保持し、FeedforwardParams には持たせない（本番コード不変更）
    creep_accel_speeds_kmh: list[float] = field(default_factory=list)
    creep_accel_kmhs: list[float] = field(default_factory=list)
    # 惰行とみなす要求加速度の帯（半幅）[km/h/s]。段2（惰行レジーム判定）で使う。段1は未参照
    coast_band_kmhs: float = 0.0
    # 段3 到達可能性判定（ProblemReport_20260916 課題#1・#3）に使う先読みホライズン [s]。
    # 空リスト＝段3 無効（従来の regime_horizon 1 点だけの判定）。モデルの
    # lookahead_horizons_s の部分集合であること（mode_drive.load_feedforward が検査する）
    reach_horizons_s: list[float] = field(default_factory=list)
    reach_step_s: float = 0.05  # v_free(t+h) の数値積分の刻み [s]
    # ペダル選択を「区間の傾き G」で行うための窓（ProblemReport_20260929 段1）。G は基準車速の
    # 「今から L 秒先」を中心にした幅 H 秒の区間の最小二乗の傾き。
    # L: 窓の中心（今から何秒先か）[s]。段2 以降は手順2 で自動保存
    pedal_select_center_s: float = 0.5
    # H: 窓の幅 [s]。手で決める値（自動保存の対象外）
    pedal_select_width_s: float = 3.0
    # ペダル選択の方式（段3）: point＝今までどおり 1 点の傾き / window＝窓の傾き G
    pedal_select_mode: str = "point"
    # point の先読み [s]。ペダル選択だけに使う（モデルの h1＝features.h1_s とは別。
    # 1.0 で従来と同じ）
    pedal_select_point_s: float = 1.0
    # L_b: ブレーキの判定だけに使う窓の中心 [s]（ProblemReport_20260930 段2-2）。window 方式のみ。
    # アクセルかどうかは L の窓で先に判定し、アクセルでないときだけ、ブレーキか惰行かを
    # L_b の窓で判定する。None なら L と同じ（従来どおり）。手で決める値（自動保存の対象外）
    pedal_select_brake_center_s: float | None = None
    pedal_gain_speeds_kmh: list[float] = field(default_factory=list)
    accel_gain_kmhs_per_pct: list[float] = field(default_factory=list)
    brake_gain_kmhs_per_pct: list[float] = field(default_factory=list)

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
    # 段3: 本番の調停にある残りの機能。1 つずつ有効化して実機で試す（pedal_arbiter.py）
    enable_min_step: bool = False  # 微小変化の保持（accel_min_step_pct）
    enable_reengage_dwell: bool = False  # ブレーキ後の再踏込ディレイ（accel_reengage_dwell_s）
    enable_release_rate: bool = False  # 惰行でのアクセル解放レート（accel_release_rate_pct_s）
    # 段3c/3d（ProblemReport_20260921 6-15）: 研究専用（src の調停にはない）
    enable_accel_band: bool = False  # 加速度帯の保持（計画加速度が帯の中ならアクセル開度を保つ）
    accel_band_horizon_s: float = 3.0  # 計画加速度＝基準 0〜H 秒先の直線近似の傾きの H [s]
    accel_band_kmhs: float = 0.25  # 前回動かした時の計画加速度からの半幅 [km/h/s]
    accel_band_dev_escape_kmh: float = 0.3  # 前回動かした時からの偏差変化がこれ超で追従 [km/h]
    accel_band_open_escape_pct: float = 2.0  # 保持中の開度と要求開度の差がこれ以上で追従 [%]
    enable_direction_hysteresis: bool = False  # 向きのヒステリシス（逆向きは w% 戻る時だけ追従）
    accel_direction_hysteresis_pct: float = 0.5  # 逆向きに変える時に要る動き w [%]


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
    # 2026-09-26 段4b: 手順2 が狙うモード（DB の driving_modes.name）。
    # 重なる（車速 × 加速度）のマスは 1 回だけ測る（wltp_grid.combined_cell_stats）。
    # wltp_mode_name は手順3 で走るモードと、手順2 の MAE_WLTP 用で、ここには影響しない
    coverage_mode_names: list[str] = field(
        default_factory=lambda: ["01_WLTP_Low,Mid,Hi,ExHi"]
    )
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
    # 使っていないペダルの待機位置 = vehicle.min_*_opening_pct（0% から触れる位置までの空走を
    # なくす）。false なら 0%
    pedal_standby: bool = True
    # 2026-10-17 段2o-9: 走行中の安全網。もう一方のペダルの実開度が min を超えている間は、出そうと
    # しているペダルを min のまま止める（実開度での両踏みを防ぐ。止めた回数・秒数はレポートに出す）
    pedal_interlock_enabled: bool = True


@dataclass
class ExciteSection:
    """加振走行（tests/research/excite.py。ProblemReport_20260921 手順3 の 1.4Hz ばたつき対策）:

    FF・PID を使わず、一定速度の基準開度に小さな正弦波を重ねてペダル→車速の周波数応答
    P(f) を開ループで直接測る。到達フェーズ（定速保持の PI で目標速度に入れる。
    `excite._pi_hold_step`。旧定速階段の PI と同じ構造）→
    加振フェーズ（周波数ごとに正弦波を重ね、遅いトリムだけで平均速度を保つ）の繰り返し。
    """

    speeds_kmh: list[float] = field(default_factory=lambda: [60.0, 100.0])
    frequencies_hz: list[float] = field(
        default_factory=lambda: [0.2, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0]
    )
    amplitude_pct: float = 0.3        # 基準開度に重ねる正弦波の振幅 [%]（ハード上限 1.0）
    hold_s: float = 12.0              # 1 周波数あたりの加振時間 [s]（実際は整数周期に切り上げ）
    analysis_skip_s: float = 2.0      # 各ブロックの先頭で解析から捨てる時間 [s]
    approach_timeout_s: float = 90.0  # 目標速度に入れなかったら中止 [s]
    # 実機の到達フェーズ PI は約 0.34Hz でハンチングし（速度標準偏差 0.34km/h）、狭い帯・短い
    # 整定時間だと settle が成立せず approach_timeout で中止する（ProblemReport_20260921）。
    # 1.0 は旧定速階段（段4 で削除）で実績のあった許容幅と同じ値、6.0s はこの
    # ハンチング（周期約 2.9s）の約 2 周期分をカバーする長さ
    approach_band_kmh: float = 1.0    # この幅に入ったら整定とみなす
    approach_settle_s: float = 6.0    # 上の幅に入り続ける必要がある時間 [s]
    # 到達フェーズの PI（旧定速階段 cruise_hold_* と同じ既定値。同じ車で 9.6〜141km/h の
    # 定速保持の実績がある構造をそのまま使う）
    approach_kp_pct_per_kmh: float = 0.3        # 比例ゲイン [%/(km/h)]
    approach_ki_pct_per_kmh_s: float = 0.05     # 積分ゲイン [%/(km/h・s)]
    approach_max_rate_pct_s: float = 1.0        # 1 周期あたりの開度変化量の上限 [%/s]
    approach_initial_offset_pct: float = 7.0    # 初期開度 = アクセル不感帯 + この値 [%]
    trim_gain_pct_per_kmh_s: float = 0.05      # 加振中の「遅いトリム」の積分ゲイン
    speed_lpf_tau_s: float = 1.0      # トリムに使う車速の1次ローパス時定数 [s]（1.4Hz を通さない）
    abort_band_kmh: float = 8.0       # 目標速度からこれだけ外れたら中止（過速は両フェーズで見る）


@dataclass
class TuningSection:
    max_runs: int = 10
    select_metric: str = "p95_kmh"


@dataclass
class LearningSection:
    # パターン走行全体の打ち切り [s]（2026-09-25 段4: 格子ステップ走行は実車で約 30 分の見込み）
    timeout_s: float = 3600.0
    coast_timeout_s: float = 90.0
    # 2026-10-06 段2k 段A: G ランプ走行（手順2。ramp_planner が段の計画を作る）
    # 狙いの G の段 [G]（昇順）。データ由来（WLTP+US06 の加速度の秒数の帯の中ほど）。
    # 2l の実機後に決め直す
    ramp_g_levels: list[float] = field(
        default_factory=lambda: [0.02, 0.045, 0.08, 0.125, 0.175, 0.25, 0.35]
    )
    ramp_mode_min_s: float = 2.0  # 上端の車速を決める最小秒数 [s]（仮値）
    # 選んだモード（modes.coverage_mode_names）の最高車速にこれを足した車速を、コーストダウンと
    # G ランプの上端にする [km/h]（vehicle.max_speed_kmh で頭打ち）。ユーザー指定 2026-10-07 段2l-4
    ramp_top_margin_kmh: float = 5.0
    # G ランプ全体の時間の枠 [s]（モデル作成時間を測って決め直す）
    ramp_time_budget_s: float = 1500.0
    # 2026-10-17 段2o-8: 1 段あたりの最大周回数（2 周目は枠の余りで足していたもの。1 なら 1 周のみ）
    ramp_max_passes: int = 2
    # 2026-10-13 段2o-2: いちばん弱い段の途中で定速を保持する（上り・下り）。実測の加速度が
    # ±ramp_cruise_band_g 以内だった時間の合計が ramp_cruise_hold_s に達したら次へ。
    # 保持する車速: ramp_cruise_fine_max_kmh 以下は 10 km/h 刻み・それより上は
    # ramp_cruise_coarse_step_kmh 刻み。ramp_cruise_timeout_s は入ってから落ち着くまでの打ち切り
    ramp_cruise_band_g: float = 0.005
    ramp_cruise_hold_s: float = 7.0
    ramp_cruise_timeout_s: float = 15.0
    ramp_cruise_fine_max_kmh: float = 100.0
    ramp_cruise_coarse_step_kmh: float = 20.0
    # 2026-10-14 段2o-5: 下り（減速）の途中の「戻し」。ブレーキを踏んだまま、山の開度 u_peak を
    # ramp_release_fracs の各割合 f へ ramp_release_move_s かけて直線に離し、対応する
    # ramp_release_hold_s 秒だけ保持する（最後の 0 は待機位置）。空リストなら戻しを入れない
    ramp_release_fracs: list[float] = field(default_factory=list)
    ramp_release_move_s: float = 1.0
    ramp_release_hold_s: list[float] = field(default_factory=list)
    # 2026-10-16 段2o-6: false なら戻し（RAMP_RELEASE）に入らない（下りはブレーキで続ける）。
    # 戻す車速・周回数・見込み秒数は planner のままなので、戻しの有無だけを比べられる
    ramp_release_enabled: bool = True
    # 2026-10-13 段2o-2: ペダルの遅れの測定。G ランプの後ろに、ramp_lag_speeds_kmh の各車速を
    # 上端とする段（狙いの G は ramp_lag_approach_g）を足し、定速の保持の後に開度を固定で
    # ステップさせる
    # （アクセル踏み増し → 戻し → 離す → ブレーキ踏み増し → 戻し。1 つの開度につき
    # ramp_lag_step_hold_s 秒）。踏み増しの大きさは ramp_lag_step_g の加速度が出る開度の差。
    # 空のリストなら足さない。
    ramp_lag_speeds_kmh: list[float] = field(default_factory=lambda: [30.0, 60.0])
    ramp_lag_step_g: float = 0.05
    ramp_lag_step_hold_s: float = 2.5
    ramp_lag_approach_g: float = 0.125
    # 2026-10-06 段2k 段B: G ランプ走行の制御（pattern_drive が PatternLoopConfig へ渡す）
    # 偏差（基準車速 − 実車速）を何秒で詰めるか [s]。Kp ＝ 1 ÷（走行中に測った感度 × これ）、
    # Ki ＝ Kp ÷（4 × これ）。アクセル・ブレーキ共通（ユーザー決定 2026-10-09: 最初は共通）。
    # 半分データ由来（変更前の式と同じ強さになる値）。次の手順2 の実機後に決め直す
    ramp_settle_time_s: float = 2.0
    # 2026-10-09 段2l-9: G ランプの狙いの加速度 g_ref の立ち上げ・切り替え・戻しの時間 [s]。
    # 発進: 動き出した周期から 0 → +G、切り替え: 折り返しで +G → -G、停止: -G → 0。
    # 0 なら丸めない（発進は最初から +G、切り替えは即 -G、停止は -G のまま）。
    # 発進・停止は半分データ由来（US06 の発進 5 回は最初の 1 秒が 0.01〜0.12G、停止 5 回は
    # 最後の 1 秒が 0.05〜0.12G）。切り替えは仮の値。次の手順2 実機の後に決め直す
    ramp_launch_time_s: float = 1.0
    # 2026-10-10 段2l-10: 切り替え（+G → 0、0 → -G の各片道）は 1.0 → 2.0 秒。1.0 秒は車の反応
    # （約 1.4 秒）より短かった。ユーザー決定の仮の値
    ramp_turn_time_s: float = 2.0
    ramp_stop_time_s: float = 1.0
    # 発進の始まりと停止の終わりの狙いの G [G]（0 → +G、-G → 0 ではなく edge → +G、-G → edge）。
    # 0 にすると基準車速が 0 km/h に着いたあとブレーキが緩み、停車まで長くかかる
    # （スタブで約 30 秒）。仮の値（ユーザー指定 2026-10-09）。次の手順2 実機の後に決め直す
    ramp_edge_g: float = 0.05
    # 2026-10-18 段2o-10 A: G ランプの発進の離し方。① 停車保持開度 → 止まる点 ＋ margin を fast_rate
    # [%/s] で一定に離し、② そこから slow_rate [%/s] で車が動き出すまで離す
    # （発進ごとに配列を順に使う）。
    # 仮の値。測り方の細かさだけに効き、制御には使わない
    ramp_launch_fast_rate_pctps: float = 2.0
    ramp_launch_release_margin_pct: float = 2.0
    ramp_launch_slow_rates_pctps: list[float] = field(default_factory=lambda: [0.25, 0.5])
    # 2026-10-10 段2l-10: 最高速での保持 [s]（+G → 0 の切り替えの後、0 → -G が始まるまで）。
    # ユーザー決定の仮の値（041505 でアクセルを戻してから減速に転じるまで約 1.4 秒）。長いほど減速に
    # 入る前に車が落ち着くが所要時間が増える。次の手順2 実機の後に決め直す
    ramp_hold_time_s: float = 7.0
    # 2026-10-10 段2l-11: 強い G の段で基準が狙いの G にいる時間の下限 [s]（最小の上端を決める）。
    # ユーザー決定の仮の値。次の手順2 実機の後に決め直す
    ramp_min_at_g_s: float = 3.25
    # 2026-10-10 段2l-11: 発進の寄せの時間 ＝ (G − 発進の G) ÷ この速さ [G/s]、最短 blend_min_s
    ramp_launch_g_rate_gps: float = 0.15
    ramp_launch_blend_min_s: float = 0.5
    # 2026-10-12 段2l-12: G ランプを踏み増し側だけで調整する。False で両方向（根拠は yaml）
    ramp_one_sided: bool = True
    # 偏差（基準 − 実車速）の不感帯 [km/h]。|偏差| がこれ以下なら偏差の項を 0 にし、外側は不感帯を
    # 引いた量（境目で飛ばない）。ユーザー決定の仮の値
    # （041505 の車速の行間の揺れ 0.02〜0.19 km/h）。
    # 効く先: 車速の細かい揺れで開度を動かさない。次の手順2 実機の後に決め直す
    ramp_dev_deadband_kmh: float = 0.5
    # アクチュエータが指令に追いつかない間は基準を待たせ始める |指令 − 実開度| [%]。仮の値（表の
    # 開度のマス幅 2% に合わせた）。送り方を手順3 と同じにしたので通常はほぼ働かない安全網。
    # 次の手順2 実機の後に決め直す
    ramp_actuator_lag_tol_pct: float = 2.0
    # G ガバナーの「踏み増しを止める割合」（上限 G × g_limit_frac に対する割合）。ユーザー決定
    # （2026-10-06）: 0.85 → 0.95（0.95 × 0.4G × 0.98 ≒ 0.37G。0.35G の段が届くように）
    gov_soft_frac: float = 0.95
    # ペダルゲイン推定に使うサンプル: 開度 ≥ 不感帯 + この値 [%]（本番は 5%）
    accel_gain_min_offset_pct: float = 0.5
    brake_gain_min_offset_pct: float = 0.5
    # コーストダウンの終了判定（段7c。pattern_loop の `_coast_creep_settled`）に使う
    creep_launch_settle_kmhs: float = 0.1  # 平衡到達とみなす車速の傾きのしきい値 [km/h/s]
    creep_launch_settle_s: float = 2.0  # 傾きがしきい値を連続で下回り続けたら終了する時間 [s]
    creep_curve_bin_kmh: float = 1.0  # クリープ加速カーブのビン幅 [km/h]
    creep_curve_min_bin_samples: int = 5  # 採用する最小サンプル数/ビン
    # 2026-09-18 段2.5（低速の惰行カーブを実測に合わせる。ProblemReport_20260916）: 惰行減速
    # カーブの低速端（creep_speed_kmh〜coast_curve_low_max_kmh）を細ビンで同定し直す
    # （本番の COAST_CURVE_BIN_KMH=10.0 幅だと 5〜10km/h が 1 ビンに潰れていた）
    coast_curve_low_bin_kmh: float = 1.0  # 低速側のビン幅 [km/h]
    coast_curve_low_max_kmh: float = 15.0  # ここまで細ビン。以上は本番と同じ 10 km/h 幅
    coast_curve_low_min_bin_samples: int = 5  # 低速ビンの最少サンプル数
    # 段4改訂 クリープ域ブレーキの下限（estimate_stop_brake_floor が使う）
    stop_brake_floor_start_tol_kmh: float = 1.5  # 保持プラトー先頭車速の許容窓（±）[km/h]
    stop_brake_floor_min_float_s: float = 5.0  # これ以上続けば「浮いた」とみなす最短時間 [s]
    stop_brake_floor_opening_tol_pct: float = 0.3  # 候補開度への一致とみなす許容誤差 [%]
    # コーストダウンの終了判定（段7c。pattern_loop の `_advance_coast`）に使う
    grid_settle_tol_kmh: float = 1.0  # 「落ち着いた」とみなす車速の許容幅 [km/h]


@dataclass
class PedalSearchSection:
    """手順 2-0（クリープ車速 → 停車保持開度）の設定。不感帯の探索はやめた（段2l-6）。"""

    step_mm: float = 0.5  # 停車保持への刻み送り・走行前チェックの1刻み（search_step_pulse）
    dwell_s: float = 1.0  # 1 刻みごとの車速サンプルの待ち時間 [s]
    onset_margin_kmh: float = 0.3  # ブレーキの「減速中は踏み増さない」判定・停止確認の余白 [km/h]
    # 2026-09-20: クリープ安定判定を「窓平均の変化量」から「窓平均の傾き＋最短時間」へ。
    # 変化量は窓幅を変えると意味が変わるうえ、漸近的に近づく量に対しては収束前に必ず成立して
    # しまう（実測で真の平衡 5.00 に対し 4.88 で確定していた）
    creep_window_s: float = 3.0        # クリープ安定判定に使う平均車速の窓 [s]
    creep_settle_kmhs: float = 0.033   # 窓平均の傾きのしきい値 [km/h/s]（旧 0.1km/h ÷ 3s と等価）
    creep_settle_min_s: float = 10.0   # これだけ経つまで確定しない [s]
    creep_min_speed_kmh: float = 1.0
    creep_timeout_s: float = 90.0
    stop_hold_margin_pct: float = 10.0
    stop_confirm_max_wait_s: float = 10.0  # 停止確認（走行前チェック）で停車を待つ上限 [s]
    # 2026-10-08 段2l-6: 2-0 のブレーキを min から踏む1刻み [mm]。旧 search_step_mm
    # （不感帯探索専用）を改名。停車保持への刻み送り（step_mm）とは分け、停止確認だけ
    # 細かく踏む（踏み過ぎ防止）。
    # 位置指令は 0.01mm 単位（PCON-CB の PCMD）なので 0.01mm = 1 pulse が機械的な下限
    brake_step_mm: float = 0.1


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
    # ペダル操作の滑らかさ（ProblemReport_20260921 手順3）。アクセル指令の往復回数
    pedal_reversal_hyst_pct: float = 0.1  # 山（谷）からこれ以上戻ったら 1 回 [%]
    pedal_reversal_limit_per_s: float = 0.4  # 全体の上限 [回/s]
    pedal_reversal_window_s: float = 60.0  # 局所を見る窓 [s]
    pedal_reversal_window_limit_per_s: float = 1.0  # 窓ごとの最大の上限 [回/s]
    pedal_reversal_min_window_s: float = 15.0  # 対象時間がこれ未満の窓は除く [s]
    # 同時踏みと数える実開度の余裕。待機位置＋これを両ペダルが超えた行 [%]
    pedal_overlap_margin_pct: float = 0.0


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
class FeaturesSection:
    """停車保持・ブレーキ下限が見る最短ホライズン h0_s だけを yaml から読む（C7・C8 が使う）。

    2026-10-09: C1〜C5（多項式）は今後使わないので、その特徴量・ホライズン自動選択の設定キーは
    yaml から外した。C1〜C5 のコード（手順2 では `require_forward_candidate` で止まる）が
    読む値は、外す前の yaml の値を ClassVar の定数として持つ（yaml では変えられない）。
    """

    h0_s: float = 0.5  # 停車保持の判定（最短ホライズン先の基準）。C7・C8 のプラン作りが使う

    # ── 以下は yaml のキーではない定数（外す前の yaml の値） ──────────────
    h1_s: ClassVar[float] = 1.0  # レジーム判定（要求加速度 = dv_h1 / h1_s）
    h2_s: ClassVar[float] = 2.0
    h3_s: ClassVar[float] = 3.0
    use_h0: ClassVar[bool] = False
    use_h1: ClassVar[bool] = True
    use_h2: ClassVar[bool] = True
    use_h3: ClassVar[bool] = True
    p1_s: ClassVar[float] = 0.5
    p2_s: ClassVar[float] = 1.0
    use_p1: ClassVar[bool] = True
    use_p2: ClassVar[bool] = True
    past_as_delta: ClassVar[bool] = False
    use_v0_sq: ClassVar[bool] = True
    use_dv1_x_v0: ClassVar[bool] = True
    horizon_search: ClassVar[bool] = True
    search_min_s: ClassVar[float] = 0.1
    search_max_s: ClassVar[float] = 3.0
    search_step_s: ClassVar[float] = 0.1
    search_max_horizons: ClassVar[int] = 10
    search_min_improvement: ClassVar[float] = 0.01
    search_min_horizon_s: ClassVar[float] = 0.0
    search_cv_splits: ClassVar[int] = 5
    search_gain_check_speeds_kmh: ClassVar[tuple[float, ...]] = (
        20.0, 40.0, 60.0, 80.0, 100.0, 120.0
    )
    search_min_deviation_gain: ClassVar[float] = 0.0

    def search_grid(self) -> tuple[float, ...]:
        """ホライズン自動選択の探索格子（`search_min_s`〜`search_max_s` を `search_step_s` 刻み）。

        `search_min_horizon_s` 未満は除く。丸めは浮動小数の桁誤差を吸収するためだけの措置
        （探索結果の一貫性は「同じ grid を複数回呼んでも同じ float 値が返る」ことで保たれる）。
        """
        if self.search_step_s <= 0.0:
            raise ValueError(f"features.search_step_s は正値にしてください: {self.search_step_s}")
        n = round((self.search_max_s - self.search_min_s) / self.search_step_s)
        grid = [round(self.search_min_s + i * self.search_step_s, 6) for i in range(n + 1)]
        return tuple(g for g in grid if g >= self.search_min_horizon_s)

    def past_horizons_s(self) -> tuple[float, ...]:
        """使う過去ホライズン（`use_p1`/`use_p2` が true のもの。昇順）。

        ホライズン自動選択（手順6）が両ペダル共通で使う過去ホライズンでもある
        （`p1_s`/`p2_s`・`use_p1`/`use_p2` は探索しない。config 冒頭のコメント参照）。
        """
        return tuple(p for p, use in ((self.p1_s, self.use_p1), (self.p2_s, self.use_p2)) if use)

    def to_feature_spec(self) -> FeatureSpec:
        """`ff_model.FeatureSpec` に変換する（先読みの並びは h0〜h3、除外は dv 列だけ）。"""
        lookahead = (self.h0_s, self.h1_s, self.h2_s, self.h3_s)
        excluded = tuple(
            h
            for h, use in zip(
                lookahead, (self.use_h0, self.use_h1, self.use_h2, self.use_h3), strict=True
            )
            if not use
        )
        return FeatureSpec(
            lookahead_horizons_s=lookahead,
            past_horizons_s=self.past_horizons_s(),
            regime_horizon_s=self.h1_s,
            include_v0_sq=self.use_v0_sq,
            include_dv_regime_x_v0=self.use_dv1_x_v0,
            dv_excluded_horizons_s=excluded,
            past_as_delta=self.past_as_delta,
        )


@dataclass
class ForwardMapSection:
    """C7（順モデルの表）の開度の列の作り方（fwd_plan.build_plan へ渡す値）。"""

    # 惰行の帯 [km/h/s]。要る加速度が惰行の加速度から ± この値以内ならペダルを踏まない。
    # feedforward.coast_band_kmhs（C1 用）とは別物
    coast_band_kmhs: float = 0.25
    # 要る加速度を測る窓の幅 [s]
    need_window_s: float = 1.0
    # None なら pkl の meta の値（表を作ったときの遅れ）を使う
    accel_lag_s: float | None = None
    brake_lag_s: float | None = None
    # 2026-10-16 段2o-6: 次に踏むペダルを、踏む直前に min から効き始めの手前へ先に置く
    # （pkl の meta に効き始めがあるペダルだけ。false なら今までと同じ列）
    preposition_enabled: bool = True
    preposition_margin_pct: float = 0.5  # 効き始めからこの量だけ手前に置く [%]
    preposition_lead_s: float = 0.2  # 踏み出しの何秒前から置きにいくか [s]
    # 2026-10-17 段2p（C10 のときだけ効く）: 偏差 Δv を τ 秒で 0 に戻す加速度を足す。
    # τ ＝ deviation_c × 手順2 で測った遅れ（ペダル別）。単位なし。0 なら C10 でも C8 と同じ出力。
    # 初期値 100 は前の車で C5 がばたついた c 13〜24 の 4 倍以上（実機で 100 → 50 と縮める）
    deviation_c: float = 100.0
    # 表を引く車速の行。"reference"＝基準車速（案A）／"actual"＝実車速（案B）
    deviation_row: str = "reference"
    # 2026-10-18 段2q（C11 のときだけ効く）: 外乱オブザーバ（DOB）の時定数
    # Tq ＝ dob_tq_coef × 手順2 の遅れ（ペダル別）。単位なし。
    # 仮の全車共通係数（段2s で決め直す）。40 は遅れ約 0.25 s で Tq ≈ 10 s
    # （加速度の乱れ ±0.4〜0.5 km/h/s をならして、推定の幅 ±0.2 程度）
    dob_tq_coef: float = 40.0
    # 2026-10-18 段2r（C12 のときだけ効く）: 惰行の行の補助（coast assist）
    # on／off ＝ 入る／離す偏差の大きさ [km/h]。仮の全車共通の値（KPI の |偏差|>1 km/h の帯の
    # 半分と 2 割）。入る・離すのばたつきに効く（2r の実機で 1〜2 通り試して決める）
    coast_on_kmh: float = 0.5
    coast_off_kmh: float = 0.2
    # 保持時間 ＝ coast_hold_coef × (手順2 の遅れ ＋ 効き始めまでの戻り時間)（ペダル別）。単位なし。
    # 押した効果が出てから判断するため。2r で決める
    coast_hold_coef: float = 4.0
    # 段2r ②。true なら、補助中のペダルの u を手順2 で測った効き始め以上にする
    # （遊びの中で効かないのを防ぐ）
    coast_floor_onset: bool = False
    # 段2r ③。true なら 1 つの惰行の区間で使うペダルは 1 つだけ。false なら、前の補助のペダルが
    # min に戻り保持時間がたてば反対のペダルにも入る
    coast_one_pedal_per_section: bool = True


@dataclass
class FelSection:
    """フィードバック誤差学習 FEL（2026-10-19 段2s-2）。fel.py が補正表を作り、走行で f + c を引く。

    2026-10-19 段2s-2b: 学習用パターン（fel_pattern.py）の設定もこのセクションに置く。
    すべて全車共通の仮値。何に効くか＝補正表のマスの埋まり方、決める時期＝2s-3 の実機の手順2 の
    後の「埋まり方」の表示を見てから。
    """

    # 補正表を使うか（false または pkl に補正表なしなら今と完全に同じ）
    enabled: bool = False
    # 補正の作り方: "cell"（マスの平均）か "nn"（小さなニューラルネット）
    method: str = "cell"
    # 補正の上限 ±[km/h/s]
    c_max_kmhs: float = 0.5
    # cell で補正を出すマスの最小秒数 [s]
    min_cell_s: float = 2.0
    # マスの車速の境界 [km/h]（昇順）
    speed_edges_kmh: list[float] = field(
        default_factory=lambda: [
            0.3, 2.5, 5.0, 7.5, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0, 60.0, 80.0, 100.0, 140.0,
        ]
    )
    # マスの u の幅 [%]
    accel_u_bin_pct: float = 1.0
    brake_u_bin_pct: float = 0.5
    # ── 学習用パターン（fel_pattern.py。2026-10-19 段2s-2b） ──
    stop_hold_s: float = 5.0  # 頭とトリップの間の停車 [s]
    ease_start_kmh: float = 15.0  # 停車前の形 B で G を緩め始める車速／減速の G を数える下限 [km/h]
    stop_end_window_s: float = 3.0  # 停車前の G（g_end）を数える、各停車の直前の窓 [s]
    low_trip_max_kmh: float = 60.0  # 低速の短い走りに使うトリップの最高車速の上限 [km/h]
    low_trip_count: int = 8  # 低速の短い走りの本数
    cruise_s: float = 5.0  # 各段の定速 [s]
    mid_step_kmh: float = 15.0  # 中速の車速変化の幅 ±[km/h]
    mid_round_trips: int = 3  # 中速の車速変化の往復回数
    mid_speed_min_kmh: float = 20.0  # 中速の上の車速（中央値）を数える下限 [km/h]
    high_speed_quantile: float = 0.9  # 高速の上の車速＝走っている間の基準車速のこの分位
    high_coast_s: float = 6.0  # 高速の惰行 [s]
    half_g_s: float = 3.0  # 形 C で G を半分にしている時間 [s]
    coast_s: float = 4.0  # 形 D の惰行 [s]
    accel_min_g: float = 0.02  # 加速の G の標本に入れる下限 [G]
    pattern_dt_s: float = 0.1  # 学習用パターンの基準車速の刻み [s]


@dataclass
class ResearchConfig:
    """`config_testVehicle.yaml` 全体。`source_path` は読み込み元（保存先）。"""

    source_path: Path = DEFAULT_CONFIG_PATH
    vehicle: VehicleSection = field(default_factory=VehicleSection)
    feedforward: FeedforwardSection = field(default_factory=FeedforwardSection)
    features: FeaturesSection = field(default_factory=FeaturesSection)
    forward_map: ForwardMapSection = field(default_factory=ForwardMapSection)
    fel: FelSection = field(default_factory=FelSection)
    pid: PidSection = field(default_factory=PidSection)
    arbiter: ArbiterSection = field(default_factory=ArbiterSection)
    control: ControlSection = field(default_factory=ControlSection)
    modes: ModesSection = field(default_factory=ModesSection)
    mode_drive: ModeDriveSection = field(default_factory=ModeDriveSection)
    excite: ExciteSection = field(default_factory=ExciteSection)
    tuning: TuningSection = field(default_factory=TuningSection)
    learning: LearningSection = field(default_factory=LearningSection)
    pedal_search: PedalSearchSection = field(default_factory=PedalSearchSection)
    decel_stop: DecelStopSection = field(default_factory=DecelStopSection)
    kpi: KpiSection = field(default_factory=KpiSection)
    checks: ChecksSection = field(default_factory=ChecksSection)
    output: OutputSection = field(default_factory=OutputSection)
    hardware: HardwareSection = field(default_factory=HardwareSection)

    @property
    def stop_brake_opening_pct(self) -> float:
        """停車保持ブレーキの絶対開度 [%]（2026-10-08 段2l-7）。

        yaml は min からの量（feedforward.stop_brake_offset_pct）で持ち、ここで
        `min_brake_opening_pct ＋ 量`（max_brake_opening_pct で頭打ち）に戻す。
        FeedforwardParams.stop_brake_opening_pct 以降の内部は絶対開度のまま。
        """
        return min(
            self.vehicle.min_brake_opening_pct + self.feedforward.stop_brake_offset_pct,
            self.vehicle.max_brake_opening_pct,
        )

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
    "features": FeaturesSection,
    "forward_map": ForwardMapSection,
    "fel": FelSection,
    "pid": PidSection,
    "arbiter": ArbiterSection,
    "control": ControlSection,
    "modes": ModesSection,
    "mode_drive": ModeDriveSection,
    "excite": ExciteSection,
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


#: 廃止したキー → 新しいキー（セクション別）。読み込み時に書き換えを促すエラーにする
_RENAMED_KEYS: dict[str, dict[str, str]] = {
    "feedforward": {"stop_brake_opening_pct": "stop_brake_offset_pct"},
}


#: 廃止したキー（置き換え先なし）→ 案内文（セクション別）。読み込み時に削除を促すエラーにする
_REMOVED_KEYS: dict[str, dict[str, str]] = {
    "pedal_search": {
        "brake_max_pct": "上限は vehicle.max_brake_opening_pct を使います。"
        "この行を削除してください",
    },
    "learning": {
        "ramp_rate_gain_pct_per_g_s": "G ランプは基準車速への追従になり、固定のゲインは"
        "使いません（Kp は走行中に測った感度から決まります）。この行を削除してください",
        "ramp_hold_margin_frac": "G ランプの保持幅は廃止しました（偏差の PI で動かします）。"
        "この行を削除してください",
        "ramp_hold_margin_min_g": "G ランプの保持幅は廃止しました（偏差の PI で動かします）。"
        "この行を削除してください",
    },
}


def _build_section(section_type: type, raw: Any, name: str, path: Path) -> Any:
    if not is_dataclass(section_type):  # pragma: no cover - 定義ミス防止
        raise ConfigError(f"セクション定義が不正です: {name}")
    if not isinstance(raw, dict):
        raise ConfigError(f"セクション {name} がマッピングではありません（{path}）")
    known = {f.name: f for f in fields(section_type)}
    unknown = sorted(set(raw) - set(known))
    # 2026-10-08 段2l-7: 絶対開度で持っていた旧キーは意味が変わるので読み替えず明示エラーにする
    for old, new in _RENAMED_KEYS.get(name, {}).items():
        if old in raw:
            raise ConfigError(
                f"{name}.{old} は廃止されました。{name}.{new}（min からの量 [%]）に書き換えて"
                f"ください（旧値 − 該当ペダルの min。{path}）"
            )
    # 2026-10-09: 置き換え先のない廃止キーは、未知のキーより分かる文面で知らせる
    for old, guide in _REMOVED_KEYS.get(name, {}).items():
        if old in raw:
            raise ConfigError(f"{name}.{old} は廃止されました。{guide}（{path}）")
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
    if declared == "float | None":
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ConfigError(f"{dotted} は数値か null で指定してください（現在: {value!r}）")
        return float(value)
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
    need(
        0.0 <= v.min_accel_opening_pct < v.max_accel_opening_pct,
        f"vehicle.min_accel_opening_pct({v.min_accel_opening_pct}) は 0 以上・"
        f"max_accel_opening_pct({v.max_accel_opening_pct}) 未満",
    )
    need(
        0.0 <= v.min_brake_opening_pct < v.max_brake_opening_pct,
        f"vehicle.min_brake_opening_pct({v.min_brake_opening_pct}) は 0 以上・"
        f"max_brake_opening_pct({v.max_brake_opening_pct}) 未満",
    )
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
    need(
        ff.stop_brake_offset_pct >= 0.0,
        f"feedforward.stop_brake_offset_pct は 0 以上: {ff.stop_brake_offset_pct}",
    )
    need(
        v.min_brake_opening_pct + ff.stop_brake_offset_pct <= 100.0,
        f"vehicle.min_brake_opening_pct({v.min_brake_opening_pct}) ＋ "
        f"feedforward.stop_brake_offset_pct({ff.stop_brake_offset_pct}) が 100 を超えています",
    )
    # 段2.5（ProblemReport_20260916）: coast_decel_kmhs の先頭・creep_accel_kmhs の末尾は
    # クリープ平衡点（creep_speed_kmh。その速度でペダルを離すと加減速しない＝free_accel_at=0
    # の定義値）を意図的に 0.0 で持つ（coast_curve.estimate_coast_decel_curve /
    # creep_curve.estimate_creep_accel_curve が同じ値を置くことで free_accel_at を構造として
    # 連続にする）。この端点だけ「正値で持つ規約」の例外として 0.0 を許す
    problems += _validate_curve(
        "feedforward.coast_decel", ff.coast_decel_speeds_kmh, {"": ff.coast_decel_kmhs},
        allow_zero_at="first",
    )
    problems += _validate_curve(
        "feedforward.creep_accel", ff.creep_accel_speeds_kmh, {"": ff.creep_accel_kmhs},
        allow_zero_at="last",
    )
    need(
        0.0 <= ff.coast_band_kmhs < 5.0,
        f"feedforward.coast_band_kmhs が範囲外(0<=帯<5.0): {ff.coast_band_kmhs}",
    )
    need(
        0.0 < ff.pedal_select_center_s <= 2.0,
        f"feedforward.pedal_select_center_s が範囲外(0<L<=2.0): {ff.pedal_select_center_s}",
    )
    need(
        2.0 <= ff.pedal_select_width_s <= 6.0,
        f"feedforward.pedal_select_width_s が範囲外(2<=H<=6): {ff.pedal_select_width_s}",
    )
    need(
        ff.pedal_select_mode in ("point", "window"),
        f"feedforward.pedal_select_mode は point か window: {ff.pedal_select_mode!r}",
    )
    need(
        0.0 < ff.pedal_select_point_s <= 3.0,
        f"feedforward.pedal_select_point_s が範囲外(0<x<=3.0): {ff.pedal_select_point_s}",
    )
    lb = ff.pedal_select_brake_center_s
    need(
        lb is None or 0.0 <= lb <= 2.0,
        f"feedforward.pedal_select_brake_center_s が範囲外(0<=L_b<=2.0 か null): {lb}",
    )
    need(
        lb is None or ff.pedal_select_mode == "window",
        "feedforward.pedal_select_brake_center_s は pedal_select_mode: window のときだけ"
        "指定できます（point には効きません）",
    )
    need(
        not (ff.pedal_select_mode == "window" and ff.reach_horizons_s),
        "feedforward.pedal_select_mode: window は reach_horizons_s と併用できません",
    )
    # 段3（到達可能性判定）: ホライズンは正値・昇順（狭義単調増加）。空リストは段3 無効として合格
    need(
        all(h > 0.0 for h in ff.reach_horizons_s),
        f"feedforward.reach_horizons_s に 0 以下の値があります: {ff.reach_horizons_s}",
    )
    need(
        ff.reach_horizons_s == sorted(set(ff.reach_horizons_s)),
        f"feedforward.reach_horizons_s は昇順（狭義単調増加）である必要があります: "
        f"{ff.reach_horizons_s}",
    )
    problems += _validate_features(cfg.features)
    fm = cfg.forward_map
    need(
        fm.coast_band_kmhs >= 0.0,
        f"forward_map.coast_band_kmhs は 0 以上: {fm.coast_band_kmhs}",
    )
    need(fm.need_window_s > 0.0, f"forward_map.need_window_s は正値: {fm.need_window_s}")
    for label, lag in (("accel_lag_s", fm.accel_lag_s), ("brake_lag_s", fm.brake_lag_s)):
        need(lag is None or lag >= 0.0, f"forward_map.{label} は 0 以上か null: {lag}")
    need(fm.deviation_c >= 0.0, f"forward_map.deviation_c は 0 以上: {fm.deviation_c}")
    fe = cfg.fel  # 2026-10-19 段2s-2
    need(fe.method in ("cell", "nn"), f"fel.method は cell か nn: {fe.method!r}")
    need(fe.c_max_kmhs > 0.0, f"fel.c_max_kmhs は正値: {fe.c_max_kmhs}")
    need(fe.min_cell_s > 0.0, f"fel.min_cell_s は正値: {fe.min_cell_s}")
    need(fe.accel_u_bin_pct > 0.0, f"fel.accel_u_bin_pct は正値: {fe.accel_u_bin_pct}")
    need(fe.brake_u_bin_pct > 0.0, f"fel.brake_u_bin_pct は正値: {fe.brake_u_bin_pct}")
    edges = fe.speed_edges_kmh
    need(
        len(edges) >= 2
        and edges[0] > 0.0
        and all(b > a for a, b in zip(edges, edges[1:], strict=False)),
        f"fel.speed_edges_kmh は 2 個以上・正値・厳密に昇順: {edges}",
    )
    for label in (
        "stop_hold_s", "ease_start_kmh", "stop_end_window_s", "low_trip_max_kmh", "cruise_s",
        "mid_step_kmh", "mid_speed_min_kmh", "high_coast_s", "half_g_s", "coast_s", "accel_min_g",
        "pattern_dt_s",
    ):
        need(getattr(fe, label) > 0.0, f"fel.{label} は正値: {getattr(fe, label)}")
    need(fe.low_trip_count >= 1, f"fel.low_trip_count は 1 以上: {fe.low_trip_count}")
    need(fe.mid_round_trips >= 1, f"fel.mid_round_trips は 1 以上: {fe.mid_round_trips}")
    need(
        0.0 < fe.high_speed_quantile <= 1.0,
        f"fel.high_speed_quantile は 0 より大きく 1 以下: {fe.high_speed_quantile}",
    )
    need(fm.dob_tq_coef > 0.0, f"forward_map.dob_tq_coef は正値: {fm.dob_tq_coef}")
    need(fm.coast_on_kmh > 0.0, f"forward_map.coast_on_kmh は正値: {fm.coast_on_kmh}")
    need(
        0.0 <= fm.coast_off_kmh < fm.coast_on_kmh,
        f"forward_map.coast_off_kmh は 0 以上 coast_on_kmh 未満: {fm.coast_off_kmh}"
        f"（coast_on_kmh={fm.coast_on_kmh}）",
    )
    need(fm.coast_hold_coef > 0.0, f"forward_map.coast_hold_coef は正値: {fm.coast_hold_coef}")
    need(
        fm.deviation_row in DEVIATION_ROWS,
        f"forward_map.deviation_row が不正です: {fm.deviation_row!r}"
        f"（{DEVIATION_ROWS} のいずれか）",
    )
    for label, val in (
        ("preposition_margin_pct", fm.preposition_margin_pct),
        ("preposition_lead_s", fm.preposition_lead_s),
    ):
        need(val >= 0.0, f"forward_map.{label} は 0 以上: {val}")
    need(
        0.0 < ff.reach_step_s <= 0.5,
        f"feedforward.reach_step_s が範囲外(0<刻み<=0.5): {ff.reach_step_s}",
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

    a = cfg.arbiter
    need(a.switch_hysteresis_pct >= 0.0, "arbiter.switch_hysteresis_pct は 0 以上")
    need(a.accel_rate_limit_pct_s > 0.0, "arbiter.accel_rate_limit_pct_s は正値")
    need(a.brake_rate_limit_pct_s > 0.0, "arbiter.brake_rate_limit_pct_s は正値")
    need(a.accel_reengage_dwell_s >= 0.0, "arbiter.accel_reengage_dwell_s は 0 以上")
    need(a.accel_release_rate_pct_s > 0.0, "arbiter.accel_release_rate_pct_s は正値")
    need(a.accel_min_step_pct >= 0.0, "arbiter.accel_min_step_pct は 0 以上")
    need(a.accel_band_horizon_s > 0.0, "arbiter.accel_band_horizon_s は正値")
    need(a.accel_band_kmhs > 0.0, "arbiter.accel_band_kmhs は正値")
    need(a.accel_band_dev_escape_kmh > 0.0, "arbiter.accel_band_dev_escape_kmh は正値")
    need(a.accel_band_open_escape_pct > 0.0, "arbiter.accel_band_open_escape_pct は正値")
    need(
        a.accel_direction_hysteresis_pct > 0.0,
        "arbiter.accel_direction_hysteresis_pct は正値",
    )

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
    names = cfg.modes.coverage_mode_names
    need(
        bool(names) and all(n.strip() for n in names) and len(set(names)) == len(names),
        f"modes.coverage_mode_names は空でなく、空名・重複なし（現在: {names}）",
    )
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

    ex = cfg.excite
    need(
        bool(ex.speeds_kmh) and all(s > 0.0 for s in ex.speeds_kmh),
        f"excite.speeds_kmh は空でなく全て正: {ex.speeds_kmh}",
    )
    need(
        all(s <= v.max_speed_kmh for s in ex.speeds_kmh),
        f"excite.speeds_kmh は vehicle.max_speed_kmh({v.max_speed_kmh}) 以下にしてください: "
        f"{ex.speeds_kmh}",
    )
    need(
        bool(ex.frequencies_hz) and all(f > 0.0 for f in ex.frequencies_hz),
        f"excite.frequencies_hz は空でなく全て正: {ex.frequencies_hz}",
    )
    # サンプリング（制御周期）に対して十分低い周波数だけを許す: Nyquist（1/(2·loop_interval_s)）
    # の 1/4 以下（DFT で拾えるだけでなく、正弦波の 1 周期が数サイクルに潰れないようにする余裕）
    nyquist_quarter_hz = 1.0 / (8.0 * cfg.control.loop_interval_s)
    need(
        not ex.frequencies_hz or max(ex.frequencies_hz) <= nyquist_quarter_hz,
        f"excite.frequencies_hz の最大({max(ex.frequencies_hz) if ex.frequencies_hz else 0:g}Hz) は"
        f" control.loop_interval_s に対して高すぎます"
        f"（Nyquist の 1/4 = {nyquist_quarter_hz:g}Hz 以下にしてください）",
    )
    need(
        0.0 < ex.amplitude_pct <= 1.0,
        f"excite.amplitude_pct が範囲外(0<pct<=1.0): {ex.amplitude_pct}",
    )
    need(ex.hold_s > 0.0, "excite.hold_s は正値")
    need(
        0.0 <= ex.analysis_skip_s < ex.hold_s,
        f"excite.analysis_skip_s は 0 以上 hold_s({ex.hold_s}) 未満: {ex.analysis_skip_s}",
    )
    need(ex.approach_timeout_s > 0.0, "excite.approach_timeout_s は正値")
    need(ex.approach_band_kmh > 0.0, "excite.approach_band_kmh は正値")
    need(ex.approach_settle_s > 0.0, "excite.approach_settle_s は正値")
    need(ex.approach_kp_pct_per_kmh > 0.0, "excite.approach_kp_pct_per_kmh は正値")
    need(ex.approach_ki_pct_per_kmh_s >= 0.0, "excite.approach_ki_pct_per_kmh_s は 0 以上")
    need(ex.approach_max_rate_pct_s > 0.0, "excite.approach_max_rate_pct_s は正値")
    need(ex.approach_initial_offset_pct >= 0.0, "excite.approach_initial_offset_pct は 0 以上")
    need(ex.trim_gain_pct_per_kmh_s > 0.0, "excite.trim_gain_pct_per_kmh_s は正値")
    need(ex.speed_lpf_tau_s > 0.0, "excite.speed_lpf_tau_s は正値")
    need(ex.abort_band_kmh > 0.0, "excite.abort_band_kmh は正値")

    t = cfg.tuning
    need(t.max_runs >= 1, f"tuning.max_runs は 1 以上: {t.max_runs}")
    need(
        t.select_metric in {"p95_kmh", "max_abs_kmh"},
        f"tuning.select_metric は p95_kmh / max_abs_kmh のいずれか: {t.select_metric}",
    )

    need(cfg.learning.timeout_s > 0.0, "learning.timeout_s は正値")
    need(cfg.learning.coast_timeout_s > 0.0, "learning.coast_timeout_s は正値")
    rg = cfg.learning.ramp_g_levels
    need(
        len(rg) > 0
        and all(g > 0.0 for g in rg)
        and all(a < b for a, b in zip(rg, rg[1:], strict=False))
        and rg[-1] <= cfg.vehicle.max_decel_g,
        f"learning.ramp_g_levels は空でなく、すべて正・厳密に昇順・最大が "
        f"vehicle.max_decel_g({cfg.vehicle.max_decel_g}) 以下: {rg}",
    )
    need(cfg.learning.ramp_mode_min_s > 0.0, "learning.ramp_mode_min_s は正値")
    need(cfg.learning.ramp_top_margin_kmh >= 0.0, "learning.ramp_top_margin_kmh は 0 以上")
    need(cfg.learning.ramp_time_budget_s > 0.0, "learning.ramp_time_budget_s は正値")
    need(cfg.learning.ramp_max_passes >= 1, "learning.ramp_max_passes は 1 以上")
    need(cfg.learning.ramp_settle_time_s > 0.0, "learning.ramp_settle_time_s は正値")
    need(cfg.learning.ramp_launch_time_s >= 0.0, "learning.ramp_launch_time_s は 0 以上")
    need(cfg.learning.ramp_turn_time_s >= 0.0, "learning.ramp_turn_time_s は 0 以上")
    need(cfg.learning.ramp_stop_time_s >= 0.0, "learning.ramp_stop_time_s は 0 以上")
    need(cfg.learning.ramp_edge_g >= 0.0, "learning.ramp_edge_g は 0 以上")
    need(
        cfg.learning.ramp_launch_fast_rate_pctps > 0.0,
        "learning.ramp_launch_fast_rate_pctps は正値",
    )
    need(
        cfg.learning.ramp_launch_release_margin_pct >= 0.0,
        "learning.ramp_launch_release_margin_pct は 0 以上",
    )
    need(
        len(cfg.learning.ramp_launch_slow_rates_pctps) > 0
        and all(v > 0.0 for v in cfg.learning.ramp_launch_slow_rates_pctps),
        "learning.ramp_launch_slow_rates_pctps は空でなく、すべて正値",
    )
    need(cfg.learning.ramp_hold_time_s >= 0.0, "learning.ramp_hold_time_s は 0 以上")
    need(cfg.learning.ramp_cruise_band_g > 0.0, "learning.ramp_cruise_band_g は正値")
    need(cfg.learning.ramp_cruise_hold_s >= 0.0, "learning.ramp_cruise_hold_s は 0 以上")
    need(
        all(v > 0.0 for v in cfg.learning.ramp_lag_speeds_kmh),
        f"learning.ramp_lag_speeds_kmh はすべて正（空なら遅れの測定をしない）: "
        f"{cfg.learning.ramp_lag_speeds_kmh}",
    )
    need(cfg.learning.ramp_lag_step_g > 0.0, "learning.ramp_lag_step_g は正値")
    need(cfg.learning.ramp_lag_step_hold_s > 0.0, "learning.ramp_lag_step_hold_s は正値")
    need(cfg.learning.ramp_lag_approach_g > 0.0, "learning.ramp_lag_approach_g は正値")
    need(cfg.learning.ramp_cruise_timeout_s >= 0.0, "learning.ramp_cruise_timeout_s は 0 以上")
    need(
        cfg.learning.ramp_cruise_fine_max_kmh > 0.0,
        "learning.ramp_cruise_fine_max_kmh は正値",
    )
    need(
        cfg.learning.ramp_cruise_coarse_step_kmh > 0.0,
        "learning.ramp_cruise_coarse_step_kmh は正値",
    )
    rel_f = cfg.learning.ramp_release_fracs
    rel_h = cfg.learning.ramp_release_hold_s
    need(
        len(rel_f) == len(rel_h)
        and all(0.0 <= f < 1.0 for f in rel_f)
        and all(a > b for a, b in zip(rel_f, rel_f[1:], strict=False))
        and all(h >= 0.0 for h in rel_h),
        f"learning.ramp_release_fracs は 0 以上 1 未満の降順、ramp_release_hold_s は同じ長さで"
        f" 0 以上（空なら戻しを入れない）: {rel_f} / {rel_h}",
    )
    need(cfg.learning.ramp_release_move_s > 0.0, "learning.ramp_release_move_s は正値")
    need(cfg.learning.ramp_min_at_g_s >= 0.0, "learning.ramp_min_at_g_s は 0 以上")
    need(
        cfg.learning.ramp_launch_g_rate_gps >= 0.0, "learning.ramp_launch_g_rate_gps は 0 以上"
    )
    need(
        cfg.learning.ramp_launch_blend_min_s >= 0.0, "learning.ramp_launch_blend_min_s は 0 以上"
    )
    need(cfg.learning.ramp_dev_deadband_kmh >= 0.0, "learning.ramp_dev_deadband_kmh は 0 以上")
    need(
        cfg.learning.ramp_actuator_lag_tol_pct > 0.0,
        "learning.ramp_actuator_lag_tol_pct は正値",
    )
    need(
        0.0 < cfg.learning.gov_soft_frac <= 1.0,
        f"learning.gov_soft_frac は 0 より大きく 1 以下: {cfg.learning.gov_soft_frac}",
    )
    lr = cfg.learning
    for label, offset in (
        ("accel_gain_min_offset_pct", lr.accel_gain_min_offset_pct),
        ("brake_gain_min_offset_pct", lr.brake_gain_min_offset_pct),
    ):
        need(offset > 0.0, f"learning.{label} は正値: {offset}")

    need(lr.creep_launch_settle_kmhs > 0.0, "learning.creep_launch_settle_kmhs は正値")
    need(lr.creep_launch_settle_s > 0.0, "learning.creep_launch_settle_s は正値")
    need(lr.creep_curve_bin_kmh > 0.0, "learning.creep_curve_bin_kmh は正値")
    need(
        lr.creep_curve_min_bin_samples >= 1,
        f"learning.creep_curve_min_bin_samples は 1 以上: {lr.creep_curve_min_bin_samples}",
    )
    need(lr.coast_curve_low_bin_kmh > 0.0, "learning.coast_curve_low_bin_kmh は正値")
    need(
        lr.coast_curve_low_max_kmh >= COAST_CURVE_BIN_KMH,
        f"learning.coast_curve_low_max_kmh は COAST_CURVE_BIN_KMH（{COAST_CURVE_BIN_KMH}）以上: "
        f"{lr.coast_curve_low_max_kmh}",
    )
    need(
        lr.coast_curve_low_min_bin_samples >= 1,
        f"learning.coast_curve_low_min_bin_samples は 1 以上: {lr.coast_curve_low_min_bin_samples}",
    )
    need(
        lr.stop_brake_floor_start_tol_kmh > 0.0, "learning.stop_brake_floor_start_tol_kmh は正値"
    )
    need(lr.stop_brake_floor_min_float_s > 0.0, "learning.stop_brake_floor_min_float_s は正値")
    need(
        lr.stop_brake_floor_opening_tol_pct > 0.0,
        "learning.stop_brake_floor_opening_tol_pct は正値",
    )
    need(lr.grid_settle_tol_kmh > 0.0, "learning.grid_settle_tol_kmh は正値")

    ps = cfg.pedal_search
    need(0.0 < ps.step_mm <= 5.0, f"pedal_search.step_mm が範囲外(0<mm<=5): {ps.step_mm}")
    need(ps.dwell_s > 0.0, "pedal_search.dwell_s は正値")
    need(ps.onset_margin_kmh > 0.0, "pedal_search.onset_margin_kmh は正値")
    need(ps.creep_window_s > 0.0, "pedal_search.creep_window_s は正値")
    need(ps.creep_settle_kmhs > 0.0, "pedal_search.creep_settle_kmhs は正値")
    need(ps.creep_settle_min_s >= 0.0, "pedal_search.creep_settle_min_s は 0 以上")
    need(ps.creep_min_speed_kmh >= 0.0, "pedal_search.creep_min_speed_kmh は 0 以上")
    need(ps.creep_timeout_s > 0.0, "pedal_search.creep_timeout_s は正値")
    need(ps.stop_hold_margin_pct >= 0.0, "pedal_search.stop_hold_margin_pct は 0 以上")
    need(ps.stop_confirm_max_wait_s > 0.0, "pedal_search.stop_confirm_max_wait_s は正値")
    # 0.01mm = 1 pulse（PCON-CBの位置指令単位）が機械的な下限
    need(
        0.01 <= ps.brake_step_mm <= 5.0,
        f"pedal_search.brake_step_mm が範囲外(0.01<=mm<=5): {ps.brake_step_mm}",
    )

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
    need(k.pedal_reversal_hyst_pct > 0.0, "kpi.pedal_reversal_hyst_pct は正値")
    need(k.pedal_reversal_limit_per_s > 0.0, "kpi.pedal_reversal_limit_per_s は正値")
    need(k.pedal_reversal_window_s > 0.0, "kpi.pedal_reversal_window_s は正値")
    need(
        k.pedal_reversal_window_limit_per_s >= k.pedal_reversal_limit_per_s,
        f"kpi.pedal_reversal_window_limit_per_s({k.pedal_reversal_window_limit_per_s}) が "
        f"pedal_reversal_limit_per_s({k.pedal_reversal_limit_per_s}) を下回っています",
    )
    need(
        0.0 <= k.pedal_reversal_min_window_s <= k.pedal_reversal_window_s,
        "kpi.pedal_reversal_min_window_s は 0 以上・pedal_reversal_window_s 以下",
    )
    need(k.pedal_overlap_margin_pct >= 0.0, "kpi.pedal_overlap_margin_pct は 0 以上")

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


def _validate_features(ft: FeaturesSection) -> list[str]:
    """features の値域（yaml に残るのは h0_s だけ）。"""
    problems: list[str] = []
    if ft.h0_s <= 0.0:
        problems.append("features.h0_s は 0 より大きい値にしてください")
    return problems


def _validate_curve(
    label: str,
    speeds: list[float],
    values: dict[str, list[float]],
    *,
    allow_zero_at: str | None = None,
) -> list[str]:
    """速度グリッドと値列の長さ・単調性を検証する。空（未同定）は合格。

    値は正値で持つ規約だが、`allow_zero_at`（"first" または "last"）を指定すると、その
    端点だけ厳密に 0.0 のときに限り例外として許す（負値はどの位置でも不正のまま）。
    段2.5（ProblemReport_20260916）: coast_decel_kmhs の先頭・creep_accel_kmhs の末尾は
    クリープ平衡点（free_accel_at=0 の定義値）として意図的に 0.0 を置くための例外
    （呼び出し側のコメント参照）。それ以外の値列（pedal_gain 等）には渡さない。
    """
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
        checked = list(vals)
        if allow_zero_at == "first" and checked and checked[0] == 0.0:
            checked = checked[1:]  # 先頭の平衡点（厳密に 0.0）だけ検査から除く
        elif allow_zero_at == "last" and checked and checked[-1] == 0.0:
            checked = checked[:-1]  # 末尾の平衡点（厳密に 0.0）だけ検査から除く
        if any(v <= 0.0 for v in checked):
            note = "（正値で持つ規約。端点の 0.0 は平衡点として例外）" if allow_zero_at else (
                "（正値で持つ規約）"
            )
            problems.append(f"{label}{suffix} に 0 以下の値があります{note}")
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
