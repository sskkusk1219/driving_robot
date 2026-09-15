"""アプリケーション設定を config/settings.toml から読み込む。"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, field, fields
from pathlib import Path


@dataclass
class SerialSettings:
    accel_port: str = "/dev/ttyUSB0"
    brake_port: str = "/dev/ttyUSB1"
    baud_rate: int = 38400
    # Modbus 応答待ち [s]。仕様（MODBUS 4-2）の Tout = To + α + (10·Bprt/Kbr) は
    # 38400bps・α=5ms で 12.2〜12.4ms。旧既定 0.3s はその約 24 倍で、再送 3 回を含めた
    # 最悪ブロックが 4×0.3=1.2s となり base_loop.WEDGED_CYCLE_TIMEOUT_S=1.0s を超えていた
    # ＝**単発の再送上限到達が必ずウォッチドッグ非常停止を起こす**構造だった。
    # 実機で観測されているのは「初回は応答が来ず（欠落）、再送は数十ms で成功する」
    # パターンなので、短いタイムアウトで早く再送に入るほうが速い。仕様値の 4 倍の
    # マージンを取って 0.05s（4 試行で 0.2s < 1.0s）とする。
    timeout_s: float = 0.05
    # 再送回数。MODBUS 4-2「リトライは、必ず設定してください」Nrt=3。
    retries: int = 3


# RCP6-ROD の最低速度算出式の分母（MJ3751-2Q 1.2.1 の注意書き）:
#   最低速度〔mm/s〕＝ リード長〔mm〕÷ 800 ÷ 0.001〔秒〕＝ リード長 ÷ 0.8
# 「最低速度以下の速度は設定しないでください。設定した速度では動きません」と明記されている。
_RCP6_MIN_SPEED_LEAD_DIVISOR: float = 0.8


@dataclass
class ActuatorAxisSettings:
    """1 軸ぶんのアクチュエータ機体仕様（docs/manuals/RCP6-ROD(MJ3751-2Q).pdf 由来）。

    最高速度・最低速度・加減速度別可搬質量は**すべてリード長に依存する**ため、型式と
    リード長をここに記録して actuator_driver がそれを参照する。lead_mm=0（未記入）なら
    ドライバはリード長非依存の保守的な既定値にフォールバックする。
    """

    # 実機ラベルの型式（例: RCP6-RA6R-WA-42P-6-100-P3-M-MT）。参照用で制御には使わない。
    model: str = ""
    # ボールねじリード長 [mm]。型式の「モーター種類-リード-ストローク」の中央の数字。
    # 最低速度（= lead_mm / 0.8）の算出に使う。0 なら未記入。
    lead_mm: float = 0.0
    # ストローク [mm]（型式末尾側の数字）。参照用。
    stroke_mm: float = 0.0
    # 運用上の最高速度 [mm/s]。仕様の「速度の制限」表（水平/垂直の小さい方）から取る。
    # PCON-CB パラメーター No.152「高出力化設定」が無効の場合の値を入れておくと、
    # 設定に関わらず安全側になる。
    max_speed_mm_s: float = 100.0
    # 運用上の最大加減速度 [G]。仕様の「加減速度別可搬質量」表で運用速度域の可搬質量が
    # ペダル反力を上回る範囲に収めること（MJ3751-2Q 1.2.2 の注意:「加減速度は、許容値
    # 以上の設定は行わないでください」）。
    max_accel_g: float = 0.3

    @property
    def min_speed_mm_s(self) -> float | None:
        """仕様式によるこの軸の最低速度 [mm/s]。lead_mm 未記入なら None。"""
        if self.lead_mm <= 0.0:
            return None
        return self.lead_mm / _RCP6_MIN_SPEED_LEAD_DIVISOR


@dataclass
class ActuatorSettings:
    accel: ActuatorAxisSettings = field(default_factory=ActuatorAxisSettings)
    brake: ActuatorAxisSettings = field(default_factory=ActuatorAxisSettings)


@dataclass
class CanSettings:
    interface: str = "kvaser"
    channel: int = 0
    bitrate: int = 500000
    dbc_path: str = "config/can/MEIDEN_MEIDACS.dbc"
    # キャッシュ車速の許容鮮度 [s]。シャシダイナモの Speed 送信周期より十分長く、
    # かつ凍結車速での盲目走行が KPI（偏差 1.0km/h 上限）を破らない範囲で設定する
    max_speed_age_s: float = 0.2


@dataclass
class DatabaseSettings:
    dsn: str = "postgresql://localhost/driving_robot"


@dataclass
class GpioSettings:
    ac_detect_pin: int = 27
    emergency_stop_pin: int = 17


@dataclass
class ArchiveSettings:
    usb_ssd_path: str = "/mnt/usb_ssd/archive"
    active_log_days: int = 90
    storage_limit_pct: float = 80.0


@dataclass
class ControlSettings:
    loop_interval_ms: int = 50
    log_interval_ms: int = 100


@dataclass
class SafetySettings:
    overcurrent_limit_ma: float = 3000.0


@dataclass
class ServoSettings:
    """ボタンサーボ（PCA9685 + SG90）設定。押下角度は全ch共通のグローバル値。"""

    # ボタンサーボ機能全体の有効フラグ。Post-MVP のため既定は無効（未接続扱い）。
    # 有効化するには config で servo.enabled=true を設定し、PCA9685 を配線すること。
    enabled: bool = False
    i2c_bus: int = 1
    address: int = 0x40
    pwm_freq_hz: int = 50
    rest_angle: float = 60.0
    press_angle: float = 110.0


@dataclass
class UpsSettings:
    nut_host: str = "localhost"
    nut_port: int = 3493
    ups_name: str = "apcups"
    poll_interval_s: float = 5.0


@dataclass
class ModelSettings:
    """先読み逆モデルの特徴量構成（FeatureSpec 相当）。デフォルトは現行9特徴と完全一致する。

    値の変更は `scripts/evaluate_feature_sets.py` によるオフライン評価の結果を確認してから
    行うこと（このファイルのデフォルト自体は変更しない。design.md 参照）。
    """

    lookahead_horizons_s: tuple[float, ...] = (0.5, 1.0, 2.0, 3.0)
    past_horizons_s: tuple[float, ...] = (0.5, 1.0)
    regime_horizon_s: float = 1.0
    include_v0_sq: bool = True
    include_dv_regime_x_v0: bool = True
    accel_horizons_s: tuple[float, ...] = ()


@dataclass
class LearningSettings:
    """学習サイクルのデフォルトパラメータ（30 分目標の時間予算で調整済み）。"""

    # REFINE_1（PID 粗適合）の走行本数。REFINE_F が最終適合を担う完走型フロー
    # （2026-07-14）では粗適合はゲイン概算のみでよいため 5→3 に削減。
    # 2026-09-08: ProblemReport_20260908 の時間短縮（34.2分→20分以内）で 3→2 に削減。
    # ゲイン概算が目的で TRAINING_2 後の PLAN_LEARN が主戦場のため、本数減の影響は小さい。
    refine_runs_stage1: int = 2
    # VERIFY フェーズ（網羅検証パターンでの走行）の走行本数。
    # 2026-09-08: ProblemReport_20260908 の時間短縮で 1→0（フェーズ廃止）。VERIFY は FF 由来
    # プランのみで走るため p95 が構造的な床（実機 3.1 前後）に達して飽和し、続く PLAN_LEARN の
    # 初回走行と役割が重複していた（実機 2026-09-07 サイクルでは VERIFY p95=2.22 → PLAN_LEARN
    # 1本目 p95=1.74 と PLAN_LEARN の方が既に上回っていた）。モデル確定はそのまま
    # TRAINING_2 が担い、PLAN_LEARN 初回走行が実質の初回検証を兼ねる。
    verify_runs: int = 0
    # 網羅検証パターン（システムモード）の目標長 [s]。PLAN_LEARN（VERIFY 廃止後の唯一の
    # 利用元）で使う。時間予算のため 180→130（2026-09-08）。速度域の被覆（0〜最高速・
    # 加速/巡航/減速）は pid_tuning.py の保持時間の床で維持したまま短縮する。
    verify_pattern_budget_s: float = 130.0
    # PLAN_LEARN フェーズ（KPI 未達でも無条件実行）の最大走行本数。0 でフェーズスキップ。
    # 2026-09-08: ProblemReport_20260908 の時間短縮で 8→4。REFINE_FINAL 廃止（下記）により
    # 時間予算をこちらへ回す。最終結果は best 走行を採用する（learning_cycle.py 参照）ため、
    # 本数減で「悪い最終走行が採用される」リスクは生じない。
    plan_learn_runs_max: int = 4
    # PLAN_LEARN 早期打ち切りの reward 改善幅しきい値。KPI 合格で即打ち切り、または改善幅 <
    # この値（改善なし）が PLAN_LEARN_PATIENCE 回連続で収束打ち切り（2026-07-16: 旧 1 発判定は
    # reward の走行間ばらつき ±5 程度で単調改善中でも誤発動した）。
    plan_learn_reward_epsilon: float = 1.0
    # REFINE_F（PID 仕上げ・収束プラン凍結の座標降下）の走行本数。
    # 2026-09-08: ProblemReport_20260908 の時間短縮で 3→0（フェーズ廃止）。実機 2026-09-07
    # サイクルでは 9.6分かけて p95 を 3.34→2.66 にしただけで、PLAN_LEARN 1本目の 1.74 より
    # 悪い状態で終わっていた（learning_cycle.py の best 採用修正後は尚更、プラン固定後の
    # PID 微調整より PLAN_LEARN の反復本数を増やす方が期待値が高い）。0 でフェーズスキップ。
    refine_final_runs: int = 0
    # 学習運転（開ループパターン走行）完了待ちのタイムアウト [s]。学習パターン総時間は
    # マネージャから取得困難なため定数運用とし、余裕を持たせた値にする。コーストダウン完走化
    # （coast_timeout_s 6→90s）で学習運転が ≈6.5→9分に延びたため 600→900 に拡大。
    learning_timeout_s: float = 900.0


@dataclass
class AppSettings:
    serial: SerialSettings = field(default_factory=SerialSettings)
    actuator: ActuatorSettings = field(default_factory=ActuatorSettings)
    can: CanSettings = field(default_factory=CanSettings)
    database: DatabaseSettings = field(default_factory=DatabaseSettings)
    gpio: GpioSettings = field(default_factory=GpioSettings)
    archive: ArchiveSettings = field(default_factory=ArchiveSettings)
    control: ControlSettings = field(default_factory=ControlSettings)
    safety: SafetySettings = field(default_factory=SafetySettings)
    servo: ServoSettings = field(default_factory=ServoSettings)
    ups: UpsSettings = field(default_factory=UpsSettings)
    model: ModelSettings = field(default_factory=ModelSettings)
    learning: LearningSettings = field(default_factory=LearningSettings)


def load_settings(path: Path = Path("config/settings.toml")) -> AppSettings:
    """settings.toml を読み込んで AppSettings を返す。

    ファイルが存在しない場合は FileNotFoundError を raise する。
    存在するキーのみ上書きし、未定義キーはデフォルト値を使用する。
    """
    if not path.exists():
        raise FileNotFoundError(f"Settings file not found: {path}")

    with path.open("rb") as f:
        raw = tomllib.load(f)

    serial = SerialSettings(**{k: v for k, v in raw.get("serial", {}).items()})
    actuator = _parse_actuator_settings(raw.get("actuator", {}))
    can = CanSettings(**{k: v for k, v in raw.get("can", {}).items()})
    database = DatabaseSettings(**{k: v for k, v in raw.get("database", {}).items()})
    gpio = GpioSettings(**{k: v for k, v in raw.get("gpio", {}).items()})
    archive = ArchiveSettings(**{k: v for k, v in raw.get("archive", {}).items()})
    control = ControlSettings(**{k: v for k, v in raw.get("control", {}).items()})
    safety = SafetySettings(**{k: v for k, v in raw.get("safety", {}).items()})
    servo = ServoSettings(**{k: v for k, v in raw.get("servo", {}).items()})
    ups = UpsSettings(**{k: v for k, v in raw.get("ups", {}).items()})
    model = _parse_model_settings(raw.get("model", {}))
    # 学習セクションは廃止フィールド（refine_runs_stage2 / tuning_on_target_mode）が既存の
    # config に残っていても起動を止めないよう、既知フィールドのみ取り込む。
    _learning_fields = {f.name for f in fields(LearningSettings)}
    learning = LearningSettings(
        **{k: v for k, v in raw.get("learning", {}).items() if k in _learning_fields}
    )

    return AppSettings(
        serial=serial,
        actuator=actuator,
        can=can,
        database=database,
        gpio=gpio,
        archive=archive,
        control=control,
        safety=safety,
        servo=servo,
        ups=ups,
        model=model,
        learning=learning,
    )


def _parse_actuator_settings(raw: dict[str, object]) -> ActuatorSettings:
    """`[actuator.accel]` / `[actuator.brake]` を ActuatorSettings へ変換する。

    セクションごと未記入でも既定値で起動できるようにする（型式が判明していない現場でも
    従来どおり動く）。未知キーは読み捨てる。
    """
    known = {f.name for f in fields(ActuatorAxisSettings)}

    def _axis(section: object) -> ActuatorAxisSettings:
        if not isinstance(section, dict):
            return ActuatorAxisSettings()
        return ActuatorAxisSettings(**{k: v for k, v in section.items() if k in known})

    return ActuatorSettings(accel=_axis(raw.get("accel")), brake=_axis(raw.get("brake")))


def _parse_model_settings(raw: dict[str, object]) -> ModelSettings:
    """`[model]` セクションを ModelSettings へ変換する。

    TOML の配列は list として読み込まれるため、ホライズン系フィールドは tuple へ変換する
    （`ModelSettings`/`FeatureSpec` は不変なタプルを前提とするため）。
    """
    defaults = ModelSettings()
    lookahead = raw.get("lookahead_horizons_s")
    past = raw.get("past_horizons_s")
    accel = raw.get("accel_horizons_s")
    return ModelSettings(
        lookahead_horizons_s=(
            tuple(lookahead) if isinstance(lookahead, list) else defaults.lookahead_horizons_s
        ),
        past_horizons_s=(tuple(past) if isinstance(past, list) else defaults.past_horizons_s),
        regime_horizon_s=float(raw.get("regime_horizon_s", defaults.regime_horizon_s)),  # type: ignore[arg-type]
        include_v0_sq=bool(raw.get("include_v0_sq", defaults.include_v0_sq)),
        include_dv_regime_x_v0=bool(
            raw.get("include_dv_regime_x_v0", defaults.include_dv_regime_x_v0)
        ),
        accel_horizons_s=(tuple(accel) if isinstance(accel, list) else defaults.accel_horizons_s),
    )
