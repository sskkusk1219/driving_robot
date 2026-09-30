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
    max_accel_opening_pct: float = 80.0
    max_brake_opening_pct: float = 80.0
    stop_deviation_threshold_kmh: float = 2.0
    stop_deviation_duration_s: float = 4.0


#: KAIZEN 5.8 節で比べる候補（V1 案スイッチ）。ff_candidate.CANDIDATE_CLASSES のキーと同じ
#: C6（定速階段の実測テーブルを骨格にする案）は 2026-09-25 段4 で削除した
CANDIDATE_NAMES: tuple[str, ...] = ("C1", "C2", "C3", "C4", "C5")


@dataclass
class FeedforwardSection:
    model_path: str = "tests/research/results/models/ff_poly2.pkl"
    # 参照用（走行は pkl を読み、これらは読まない。手順2・relearn が model_path と一緒に自動保存）
    model_accel_horizons_s: list[float] = field(default_factory=list)
    model_brake_horizons_s: list[float] = field(default_factory=list)
    model_accel_features: list[str] = field(default_factory=list)
    model_brake_features: list[str] = field(default_factory=list)
    model_coef_path: str = ""
    candidate: str = "C1"  # KAIZEN 表5-5 順7（V1 案スイッチ）。CANDIDATE_NAMES のいずれか
    creep_speed_kmh: float = 5.0
    creep_rate_kmhs: float = 0.5
    stop_brake_opening_pct: float = 20.0
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
    # 段4改訂 クリープ域ブレーキの下限（ProblemReport_20260916）: クリープ域でブレーキを保持して
    # 実際に停車できた最小の「不感帯からの超過」[%]。手順2 で自動保存。0.0＝未同定
    stop_brake_floor_offset_pct: float = 0.0
    # この速度以下でブレーキの下限を効かせる [km/h]。0.0 で無効（人が決める値・自動保存の対象外）
    brake_trim_max_kmh: float = 0.0
    # 先読み車速（最短ホライズン=0.5s 先の基準）がこの値以下のときだけ下限を掛ける [km/h]
    # （人が決める値・自動保存の対象外）
    brake_trim_ref_kmh: float = 0.3
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
    # 2026-09-26 段4b: 手順2 の格子ステップ走行が狙うモード（DB の driving_modes.name）。
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
    # 使っていないペダルの待機位置 = 不感帯 − standby_margin_pct（0% から不感帯までの空走をなくす）
    pedal_standby: bool = True
    standby_margin_pct: float = 2.0


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
    kp_search_range: list[float] = field(default_factory=lambda: [0.5, 8.0])
    ki_search_range: list[float] = field(default_factory=lambda: [0.0, 2.0])
    kd_search_range: list[float] = field(default_factory=lambda: [0.0, 1.0])
    select_metric: str = "p95_kmh"


@dataclass
class LearningSection:
    # パターン走行全体の打ち切り [s]（2026-09-25 段4: 格子ステップ走行は実車で約 30 分の見込み）
    timeout_s: float = 3600.0
    coast_timeout_s: float = 90.0
    # ペダルゲイン推定に使うサンプル: 開度 ≥ 不感帯 + この値 [%]（本番は 5%）
    accel_gain_min_offset_pct: float = 0.5
    brake_gain_min_offset_pct: float = 0.5
    # 2026-09-17 クリープ発進・低速ブレーキ保持（段1。ProblemReport_20260916 課題#2）:
    # 格子ステップ走行の後、パターン列の末尾に置く
    # （手順3のモード走行と同じ暖機状態でクリープを測るため）
    # 2026-09-27 段7a（ProblemReport_20260925 段7）: 手順2 の計測効率化のため 3→0
    # （単独クリープ発進は網羅表の穴を増やさずに削れることを実測 065502 で確認済み。クリープ
    # カーブは格子ステップ走行の停車ステップ・クリープ域ブレーキ保持からも同定できる）
    creep_launch_count: int = 0  # クリープ発進パターンの本数（両ペダル解放で自走）
    # 2026-09-17 段1b: 終了条件を「target_kmh 到達」から「平衡到達（加速が止まる）」に変更
    # （ProblemReport_20260916 ユーザー決定）。target_kmh は安全上限として残し、既定を
    # 4.5 → 15.0 に上げる（真のクリープ平衡 4.97 km/h 付近では終わらせず、途中で頭打ちに
    # ならないようにするため）。timeout_s も平衡到達に十分な時間を見て 20.0 → 30.0 に上げる
    creep_launch_target_kmh: float = 15.0  # 安全上限。これに達したら平衡を待たず打ち切る
    # 2026-09-17（誤判定修正）: クリープ平衡への収束は漸近的で、pedal_search.creep_timeout_s が
    # 90s を見ていることから 30s では足りない恐れがある（ユーザーが時間が伸びてよいと明言）。
    # 30.0 → 60.0 に延長
    creep_launch_timeout_s: float = 60.0  # 打ち切り [s]
    creep_launch_settle_kmhs: float = 0.1  # 平衡到達とみなす車速の傾きのしきい値 [km/h/s]
    creep_launch_settle_s: float = 2.0  # 傾きがしきい値を連続で下回り続けたら終了する時間 [s]
    # 2026-09-17（誤判定修正）: 停車保持解放直後は車速0・傾き0で誤って平衡到達と判定していた
    # （実機では解放から動き出しまで約0.3〜0.4s、その間の傾きは0.17km/h/s程度でしきい値0.1に
    # 近く確実に誤判定する）。車速がこの値以上になるまで安定カウントを積まない
    # （pedal_search.creep_min_speed_kmh と同じ考え方）
    creep_launch_min_speed_kmh: float = 1.0  # クリープ発進の平衡判定を始める車速のしきい値 [km/h]
    # elapsed がこの値以上になるまで平衡到達で終了しない（creep_settle_min_s と同じ流儀の
    # 最短時間ガード）
    creep_launch_settle_min_s: float = 5.0  # 平衡到達の最短経過時間 [s]
    # クリープ車速から、ブレーキ「不感帯 + frac × (停車保持開度 − 不感帯)」で停車まで保持
    # （低速ブレーキゲイン用。2026-09-25 段4: 車両ごとの 2-0 実測から開度を決める）
    creep_brake_hold_fracs: list[float] = field(
        default_factory=lambda: [0.05, 0.15, 0.25, 0.45, 0.55, 0.7, 0.9, 1.0]
    )
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
    # 2026-09-25 WLTP 網羅マップ（ProblemReport_20260925 段1。wltp_grid.py）: 車速 × 加速度の格子。
    # 加速度は FF の regime ホライズン先との差（学習側と同じ定義）。境界は昇順・両端は WLTP の外側。
    # ±0.5 を 1 列（定速）にまとめる: 定速保持の実測加速度は PI のハンチングで ±1 km/h/s ほど揺れる
    grid_speed_edges_kmh: list[float] = field(
        default_factory=lambda: [float(v) for v in range(0, 150, 10)]
    )
    # 外側の端 ±14 は vehicle.max_decel_g 0.4G ≒ 14.1 km/h/s（G ガバナより強くは測れない）。
    # US06 のように ±7 を超える走りのあるモードを数えるため（段4b）
    grid_accel_edges_kmhs: list[float] = field(
        default_factory=lambda: [-14.0, -7.0, -3.0, -1.5, -0.5, 0.5, 1.5, 3.0, 7.0, 14.0]
    )
    # 狙う・穴と判定するマスの最小秒数は、1 ステップで測れる長さ（grid_step_window_s −
    # grid_step_lag_s。`grid_target_min_s`）から自動で決まる（段4b。別の数値は持たない）
    grid_hole_data_max_s: float = 2.0  # 学習データがこれ未満なら穴 [s]
    # 2026-09-25 格子ステップ走行（ProblemReport_20260925 段3。grid_planner.GridSettings に渡す）:
    # 車速ステーションごとに PI で保持 → 落ち着いたら開度固定のステップで加速度を測る。
    # 感度 g は「開度 1% あたりの加速度 [km/h/s per %]」。初期値は大きめ（踏み増し = 狙い ÷ g が
    # 小さく＝1 回目は弱く外れる側）。PI の kp/ki は g で割って車両に依らないゲインにする
    # （レート上限は実開度が追従できる速さなので g では割らない）
    grid_station_min_kmh: float = 10.0  # これ未満の車速帯はステーションを置かない（発進・停車）
    grid_settle_tol_kmh: float = 1.0  # 「落ち着いた」とみなす車速の許容幅 [km/h]
    grid_settle_s: float = 3.0  # 許容幅の中にこの秒数いたら落ち着いた [s]
    grid_settle_timeout_s: float = 40.0  # 落ち着かないとき、そのステーションの残りをとばす [s]
    grid_step_window_s: float = 3.0  # 開度固定の 1 ステップの長さ [s]
    grid_step_lag_s: float = 0.5  # ステップの頭のこの秒数は傾きの計算から除く [s]
    grid_step_band_max_kmh: float = 10.0  # ステーションからこれ以上離れたらステップを終える [km/h]
    # 強いステップの判定（段4b）: 中心から踏んだとき、帯を出るまでに傾きを測れる時間
    # （帯 ÷ |狙い| − grid_step_lag_s）がこれ未満なら、帯の手前の端まで下がって（上がって）から
    # 踏む（助走）。1.0s = 0.1s 刻みで約 10 点。
    grid_step_min_fit_s: float = 1.0
    grid_overshoot_frac: float = 1.2  # WLTP の最大（最小）加速度のこの倍を超えたら打ち切り
    grid_max_tries: int = 2  # 1 つの狙いに使う最大回数（やり直しは 1 回）
    grid_gain_init_kmhs_per_pct: float = 2.0  # アクセルの感度の初期値
    grid_brake_gain_init_kmhs_per_pct: float = 2.0  # ブレーキの感度の初期値
    grid_gain_min_kmhs_per_pct: float = 0.05  # 更新した感度のクランプ
    grid_gain_max_kmhs_per_pct: float = 20.0
    grid_hold_kp_norm: float = 0.36  # PI 保持: kp = この値 / g_accel（実車 g≈1.2 で kp 0.3 相当）
    grid_hold_ki_norm: float = 0.06  # ki = この値 / g_accel
    grid_hold_max_rate_pct_per_s: float = 1.0  # PI の開度変化の上限 [%/s]（実開度が追従できる速さ）
    grid_launch_end_kmh: float = 20.0  # 発進・停車セルが担当する車速の上端 [km/h]
    # 2026-09-27 段7b（ProblemReport_20260925 段7）: ステップの後・助走の目標を決めたときの
    # 「ステーションの目標へ戻る」区間（`_Phase.GRID_RETURN`）。目標から離れていれば固定開度で
    # 速く戻り、近づいたら PI（CRUISE_HOLD）に切り替える（実測 065502: 戻りが待ち時間の最大要因）。
    # grid_return_accel_kmhs 2.0 は**半分データ由来**: 薄い +1.5〜+3 km/h/s の列（戻りの行が
    # この列を埋める）の中で、約 0.06G と上限 G に遠い。実機の戻り時間・網羅表で決め直す
    grid_return_accel_kmhs: float = 2.0  # GRID_RETURN の狙い加速度（目標より遅いとき）[km/h/s]
    # grid_return_switch_kmh 1.5 は**仮の値**: 2 km/h/s × 遅れ 0.6s（gov_lead_s の実測）≒ 1.2 km/h
    # に余裕を見た。実機で切り替え後の行き過ぎ（±1 km/h に入るまでの時間）を見て決め直す
    grid_return_switch_kmh: float = 1.5  # これ以下まで近づいたら CRUISE_HOLD の PI に切り替える
    # 段6a（門①）: 手順2 の開度の上限に使う G [G]。G 校正・走行中の実測から「この G に届く開度」を
    # 車速帯ごとに予測し、格子ステップ・発進停車の開度をそれ以下にする。**仮の値**:
    # ブレーキは踏むほど急に効く（25 km/h の実測で 12→16% が 0.21・16→24% が 0.34 km/h/s/%、比 1.6）
    # ので 0.2G の点からの割線外挿は開度を大きく見積もる。0.3G と予測した開度は実際 約 0.36G。
    # vehicle.max_decel_g（0.4G）との間の余裕。6d の実測点で比を測り直して決める
    g_cap_g: float = 0.3
    # 段6c: 通し掃引 1 本の最大回数。0 で通し掃引なし。**仮の値**: 1 回で 1 セルあたり約 1 s
    # （10 km/h ÷ 約 9 km/h/s）しか取れず、穴の基準（2 s）に 2〜3 回、遅れ・ばらつきの余裕を
    # 2 倍見て 6。手順2 の実機で「何回で埋まったか・埋まらなかったか」を見て決め直す
    grid_sweep_max_passes: int = 6
    # 2026-09-25 学習サンプルの WLTP 重み付け（ProblemReport_20260925 段2）: 重み =
    # clip(WLTP 占有率 / 学習データ占有率, w_min, w_max) をペダルごとに平均 1 へ正規化して
    # fit に渡す（sample_weight.py）。既定 false は今までと完全に同じ結果になる後方互換
    sample_weight_enabled: bool = False
    sample_weight_min: float = 0.2  # 重みの下限
    sample_weight_max: float = 5.0  # 重みの上限

    @property
    def grid_target_min_s(self) -> float:
        """狙う・穴と判定するマスの最小秒数 = 1 ステップで測れる長さ（窓 − 頭の除外）[s]。"""
        return self.grid_step_window_s - self.grid_step_lag_s


@dataclass
class PedalSearchSection:
    step_mm: float = 0.5  # 停車保持への刻み送り専用（search_step_pulse）。不感帯探索には使わない
    dwell_s: float = 1.0  # 反応判定に使う車速サンプルの待ち時間 [s]（傾きの算出窓もこの長さ）
    onset_margin_kmh: float = 0.3  # ブレーキの「減速中は踏み増さない」判定・停止確認の余白 [km/h]
    confirm_count: int = 2
    # 2026-09-20: クリープ安定判定を「窓平均の変化量」から「窓平均の傾き＋最短時間」へ。
    # 変化量は窓幅を変えると意味が変わるうえ、漸近的に近づく量に対しては収束前に必ず成立して
    # しまう（実測で真の平衡 5.00 に対し 4.88 で確定していた）
    creep_window_s: float = 3.0        # クリープ安定判定に使う平均車速の窓 [s]
    creep_settle_kmhs: float = 0.033   # 窓平均の傾きのしきい値 [km/h/s]（旧 0.1km/h ÷ 3s と等価）
    creep_settle_min_s: float = 10.0   # これだけ経つまで確定しない [s]
    creep_min_speed_kmh: float = 1.0
    creep_timeout_s: float = 90.0
    accel_max_pct: float = 20.0
    brake_max_pct: float = 50.0  # 停止確認まで踏み続ける上限（不感帯検出の上限は deadband_max_pct）
    stop_hold_margin_pct: float = 10.0
    stop_confirm_max_wait_s: float = 10.0  # 停止確認（走行前チェック）で停車を待つ上限 [s]
    # 2026-09-17 段1b（ProblemReport_20260916）: 反応判定を「平均車速が基準を超えたか」から
    # 「車速の傾き」に変える。基準車速との比較をやめるので、クリープのドリフト（実測で踏み込み中
    # でも−0.4 km/h/sのドリフトが起きていた）を反応と誤検出しなくなる
    onset_accel_kmhs: float = 0.2  # この傾き[km/h/s]以上（ブレーキは以下の符号反転）で「反応」
    # 不感帯探索専用の1刻み。停車保持への刻み送り（step_mm）と分離し、探索だけ細かくできるように
    # する（停車保持まで遅くしないため）。位置指令は0.01mm単位（PCON-CBのPCMD）なので
    # 0.01mm = 1 pulse が機械的な下限
    search_step_mm: float = 0.1
    # 不感帯検出専用の探索上限。brake_max_pct（停止確認まで踏み続ける上限）とは兼用しない
    deadband_max_pct: float = 20.0


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
    """C5 の逆モデルが使う特徴量（手順2 の学習時に効く。ProblemReport_20260921 手順5-1）。

    先読み h0〜h3 の dv（= 基準(t+h) − v0）と、過去 p1・p2 を true/false で選ぶ。
    dataclass の既定値は本番の9特徴（全部使う・過去は `v0 − past`）。除外した先読み
    （use_hN=false）もホライズンとしては残るので、停車保持の判定（最短ホライズン先の基準）は
    変わらない。
    """

    h0_s: float = 0.5
    h1_s: float = 1.0  # レジーム判定（要求加速度 = dv_h1 / h1_s）に使う。use_h1=false は不可
    h2_s: float = 2.0
    h3_s: float = 3.0
    use_h0: bool = True
    use_h1: bool = True
    use_h2: bool = True
    use_h3: bool = True
    p1_s: float = 0.5
    p2_s: float = 1.0
    use_p1: bool = True
    use_p2: bool = True
    past_as_delta: bool = True  # true: dv_past = v0 − past（本番）／false: past そのもの
    use_v0_sq: bool = True
    use_dv1_x_v0: bool = True

    # ── ホライズン自動選択（手順2。ProblemReport_20260921 手順6。2026-09-28）────────
    #   true なら手順2 の学習で h0〜h3・use_h0〜use_h3 を使わず、先読みホライズンを
    #   アクセル・ブレーキ別に交差検証で自動選択する（`tests/research/horizon_search.py`）。
    #   選んだホライズンは config には書き戻さず pkl が持つ（config へ書き戻すのは
    #   feedforward.model_path と物理定数だけ、という既存の規約のまま）。
    #   h1_s（レジーム判定）は探索でも固定のまま使う。h0_s は停車保持・ブレーキ下限が見る
    #   ホライズンとして探索と無関係に使われ続ける（pkl の stop_horizon_s に保存）。
    #   p1_s/p2_s・use_p1/use_p2・past_as_delta・use_v0_sq・use_dv1_x_v0 は
    #   両ペダル共通のまま探索しない（今回の範囲外）。
    horizon_search: bool = True
    search_min_s: float = 0.1  # 探索格子の下限 [s]
    search_max_s: float = 3.0  # 探索格子の上限 [s]
    search_step_s: float = 0.1  # 探索格子の刻み [s]
    search_max_horizons: int = 4  # 1 ペダルの先読み本数の上限（h1_s を含む）
    # CV-MAE の相対改善がこれ未満なら打ち切る。仮値（段1 で分割間のばらつきを見て決め直す）
    search_min_improvement: float = 0.01
    search_min_horizon_s: float = 0.0  # 最短ホライズンの下限 [s]（0.0 = 無効。既定は無効）
    search_cv_splits: int = 5  # パターン単位の交差検証の分割数

    # ── 実質Kp（偏差への反応の向き）の合否条件（案a。手順6 段2 の実機破綻の再発防止） ──
    #   段2（2026-09-28）の実機走行で、実車速が基準より遅れるとアクセル FF が下がる
    #   （逆向き）pkl が採用され、遅れが自分で広がって最大逸脱 126km/h に至った。学習データ
    #   （偏差 0 の自分の軌跡）だけでは符号が決まらないため（`ff_model.deviation_gain` の
    #   docstring 参照）、CV-MAE とは別に「正しい向きか」を候補の合否条件にする（ユーザー決定）。
    #   空リストは確認を無効化する（既定の探索格子・重み付けを変えず段A/Bをそれぞれ切り分けたい
    #   ときの逃げ道）。
    search_gain_check_speeds_kmh: list[float] = field(
        default_factory=lambda: [20.0, 40.0, 60.0, 80.0, 100.0, 120.0]
    )
    # 実質Kp（deviation_gain）の下限 [%/(km/h)]。0.0 は「向きが正しいことだけを見る」仮値
    # （上限は置かない。バタつきは段3 の調停で吸収する方針。ユーザー決定 2026-09-28）
    search_min_deviation_gain: float = 0.0

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
class ResearchConfig:
    """`config_testVehicle.yaml` 全体。`source_path` は読み込み元（保存先）。"""

    source_path: Path = DEFAULT_CONFIG_PATH
    vehicle: VehicleSection = field(default_factory=VehicleSection)
    feedforward: FeedforwardSection = field(default_factory=FeedforwardSection)
    features: FeaturesSection = field(default_factory=FeaturesSection)
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
    # 段4改訂 クリープ域ブレーキの下限（ProblemReport_20260916）
    need(
        0.0 <= ff.stop_brake_floor_offset_pct < 20.0,
        f"feedforward.stop_brake_floor_offset_pct が範囲外(0<=pct<20.0): "
        f"{ff.stop_brake_floor_offset_pct}",
    )
    need(
        0.0 <= ff.brake_trim_max_kmh < 20.0,
        f"feedforward.brake_trim_max_kmh が範囲外(0<=km/h<20.0): {ff.brake_trim_max_kmh}",
    )
    need(
        0.0 < ff.brake_trim_ref_kmh <= 2.0,
        f"feedforward.brake_trim_ref_kmh が範囲外(0<km/h<=2.0): {ff.brake_trim_ref_kmh}",
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
    need(
        cfg.mode_drive.standby_margin_pct >= 0.0,
        "mode_drive.standby_margin_pct は 0 以上",
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
    for label, offset in (
        ("accel_gain_min_offset_pct", lr.accel_gain_min_offset_pct),
        ("brake_gain_min_offset_pct", lr.brake_gain_min_offset_pct),
    ):
        need(offset > 0.0, f"learning.{label} は正値: {offset}")

    need(
        lr.creep_launch_count >= 0,
        f"learning.creep_launch_count は 0 以上: {lr.creep_launch_count}",
    )
    need(
        0.0 < lr.creep_launch_target_kmh < cfg.vehicle.max_speed_kmh,
        f"learning.creep_launch_target_kmh は 0 より大きく vehicle.max_speed_kmh 未満: "
        f"{lr.creep_launch_target_kmh}",
    )
    need(lr.creep_launch_timeout_s > 0.0, "learning.creep_launch_timeout_s は正値")
    need(lr.creep_launch_settle_kmhs > 0.0, "learning.creep_launch_settle_kmhs は正値")
    need(lr.creep_launch_settle_s > 0.0, "learning.creep_launch_settle_s は正値")
    need(
        0.0 <= lr.creep_launch_min_speed_kmh < lr.creep_launch_target_kmh,
        f"learning.creep_launch_min_speed_kmh は 0 以上 creep_launch_target_kmh 未満: "
        f"{lr.creep_launch_min_speed_kmh}",
    )
    need(lr.creep_launch_settle_min_s >= 0.0, "learning.creep_launch_settle_min_s は 0 以上")
    need(
        all(0.0 < f <= 1.2 for f in lr.creep_brake_hold_fracs)
        and all(a < b for a, b in zip(
            lr.creep_brake_hold_fracs, lr.creep_brake_hold_fracs[1:], strict=False
        )),
        f"learning.creep_brake_hold_fracs は 0<frac<=1.2 の昇順リスト（空は可）: "
        f"{lr.creep_brake_hold_fracs}",
    )
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
    for name, edges in (
        ("grid_speed_edges_kmh", lr.grid_speed_edges_kmh),
        ("grid_accel_edges_kmhs", lr.grid_accel_edges_kmhs),
    ):
        need(
            len(edges) >= 2 and all(a < b for a, b in zip(edges, edges[1:], strict=False)),
            f"learning.{name} は 2 点以上の昇順（現在: {edges}）",
        )
    need(lr.grid_hole_data_max_s >= 0.0, "learning.grid_hole_data_max_s は 0 以上")
    for name in (
        "grid_station_min_kmh", "grid_settle_tol_kmh", "grid_settle_s", "grid_settle_timeout_s",
        "grid_step_window_s", "grid_step_band_max_kmh", "grid_overshoot_frac",
        "grid_gain_init_kmhs_per_pct", "grid_brake_gain_init_kmhs_per_pct",
        "grid_gain_min_kmhs_per_pct", "grid_hold_kp_norm", "grid_hold_ki_norm",
        "grid_hold_max_rate_pct_per_s", "grid_launch_end_kmh",
        "grid_return_accel_kmhs", "grid_return_switch_kmh",
    ):
        need(getattr(lr, name) > 0.0, f"learning.{name} は正値")
    need(
        lr.grid_return_switch_kmh > lr.grid_settle_tol_kmh,
        f"learning.grid_return_switch_kmh は grid_settle_tol_kmh（{lr.grid_settle_tol_kmh}）"
        f"より大きい: {lr.grid_return_switch_kmh}",
    )
    need(lr.grid_step_lag_s >= 0.0, "learning.grid_step_lag_s は 0 以上")
    need(
        lr.grid_step_lag_s < lr.grid_step_window_s,
        "learning.grid_step_lag_s は grid_step_window_s より短く",
    )
    need(
        0.0 < lr.grid_step_min_fit_s < lr.grid_step_window_s - lr.grid_step_lag_s,
        "learning.grid_step_min_fit_s は 0 より大きく、grid_step_window_s − grid_step_lag_s "
        "より短く",
    )
    need(
        0.0 < lr.g_cap_g < v.max_decel_g,
        f"learning.g_cap_g は 0 より大きく vehicle.max_decel_g({v.max_decel_g}) 未満: {lr.g_cap_g}",
    )
    need(lr.grid_sweep_max_passes >= 0, "learning.grid_sweep_max_passes は 0 以上（0 で掃引なし）")
    need(lr.grid_overshoot_frac >= 1.0, "learning.grid_overshoot_frac は 1 以上")
    need(lr.grid_max_tries >= 1, "learning.grid_max_tries は 1 以上")
    need(
        lr.grid_gain_min_kmhs_per_pct <= lr.grid_gain_init_kmhs_per_pct
        <= lr.grid_gain_max_kmhs_per_pct
        and lr.grid_gain_min_kmhs_per_pct <= lr.grid_brake_gain_init_kmhs_per_pct
        <= lr.grid_gain_max_kmhs_per_pct,
        "learning.grid_gain_init/brake_gain_init は grid_gain_min〜max の範囲内",
    )
    need(
        0.0 < lr.sample_weight_min <= 1.0 <= lr.sample_weight_max,
        f"learning.sample_weight_min/max は 0 < min <= 1 <= max: "
        f"{lr.sample_weight_min}, {lr.sample_weight_max}",
    )

    ps = cfg.pedal_search
    need(0.0 < ps.step_mm <= 5.0, f"pedal_search.step_mm が範囲外(0<mm<=5): {ps.step_mm}")
    need(ps.dwell_s > 0.0, "pedal_search.dwell_s は正値")
    need(ps.onset_margin_kmh > 0.0, "pedal_search.onset_margin_kmh は正値")
    need(ps.confirm_count >= 1, "pedal_search.confirm_count は 1 以上")
    need(ps.creep_window_s > 0.0, "pedal_search.creep_window_s は正値")
    need(ps.creep_settle_kmhs > 0.0, "pedal_search.creep_settle_kmhs は正値")
    need(ps.creep_settle_min_s >= 0.0, "pedal_search.creep_settle_min_s は 0 以上")
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
    need(ps.stop_confirm_max_wait_s > 0.0, "pedal_search.stop_confirm_max_wait_s は正値")
    need(ps.onset_accel_kmhs > 0.0, "pedal_search.onset_accel_kmhs は正値")
    # 0.01mm = 1 pulse（PCON-CBの位置指令単位）が機械的な下限
    need(
        0.01 <= ps.search_step_mm <= 5.0,
        f"pedal_search.search_step_mm が範囲外(0.01<=mm<=5): {ps.search_step_mm}",
    )
    need(
        0.0 < ps.deadband_max_pct <= min(ps.accel_max_pct, ps.brake_max_pct),
        f"pedal_search.deadband_max_pct({ps.deadband_max_pct}) は 0 超・"
        f"accel_max_pct/brake_max_pct の小さい方以下",
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
    """features の値域。FeatureSpec 自体の整合（昇順・除外可否）は ValueError を問題として拾う。"""
    problems: list[str] = []
    if not ft.use_h1:
        problems.append("features.use_h1 は false にできません（レジーム判定に使う）")
    if any(h <= 0.0 for h in (ft.h0_s, ft.h1_s, ft.h2_s, ft.h3_s, ft.p1_s, ft.p2_s)):
        problems.append("features の h*_s / p*_s は 0 より大きい値にしてください")
    if not ft.p1_s < ft.p2_s:
        problems.append(f"features.p1_s < p2_s（昇順）にしてください: {ft.p1_s}, {ft.p2_s}")
    if not problems:
        try:
            ft.to_feature_spec()
        except ValueError as exc:
            problems.append(f"features: {exc}")
    # ホライズン自動選択（手順6）の探索パラメータ。horizon_search が false でも値域は検査する
    # （後で true に戻したときに壊れないようにするため）
    if ft.search_min_s <= 0.0:
        problems.append(f"features.search_min_s は正値にしてください: {ft.search_min_s}")
    if ft.search_max_s <= ft.search_min_s:
        problems.append(
            f"features.search_max_s({ft.search_max_s}) は "
            f"search_min_s({ft.search_min_s}) より大きくしてください"
        )
    if ft.search_step_s <= 0.0:
        problems.append(f"features.search_step_s は正値にしてください: {ft.search_step_s}")
    if ft.search_max_horizons < 1:
        problems.append(
            f"features.search_max_horizons は 1 以上にしてください: {ft.search_max_horizons}"
        )
    if not (0.0 <= ft.search_min_improvement < 1.0):
        problems.append(
            f"features.search_min_improvement は 0 以上 1 未満にしてください: "
            f"{ft.search_min_improvement}"
        )
    if ft.search_min_horizon_s < 0.0:
        problems.append(
            f"features.search_min_horizon_s は 0 以上にしてください: {ft.search_min_horizon_s}"
        )
    if ft.search_cv_splits < 2:
        problems.append(f"features.search_cv_splits は 2 以上にしてください: {ft.search_cv_splits}")
    if any(v <= 0.0 for v in ft.search_gain_check_speeds_kmh):
        problems.append(
            "features.search_gain_check_speeds_kmh は正値のリストにしてください（空なら確認を"
            f"無効化）: {ft.search_gain_check_speeds_kmh}"
        )
    if ft.horizon_search and not problems:
        try:
            grid = ft.search_grid()
        except ValueError as exc:
            problems.append(f"features: {exc}")
        else:
            if ft.h1_s not in grid and ft.h1_s < ft.search_min_horizon_s:
                problems.append(
                    f"features.search_min_horizon_s({ft.search_min_horizon_s}) が "
                    f"h1_s({ft.h1_s}) を超えています（レジーム判定のホライズンは探索の開始点で、"
                    f"下限より必ず小さくなければなりません）"
                )
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
