"""アプリケーション設定を config/settings.toml から読み込む。"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class SerialSettings:
    accel_port: str = "/dev/ttyUSB0"
    brake_port: str = "/dev/ttyUSB1"
    baud_rate: int = 38400


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
    """2段階学習フロー（学習サイクル）のデフォルトパラメータ。"""

    # stage1 は kp/ki/kd に加え先読み補償秒数(preview_time_s)も探索する4次元座標降下のため
    # 10→14 に増やしている（1巡=最大8走行、ベースライン+1巡強を確保）。
    refine_runs_stage1: int = 14
    refine_runs_stage2: int = 5
    # 学習運転（開ループパターン走行）完了待ちのタイムアウト [s]。学習パターン総時間は
    # マネージャから取得困難なため定数運用とし、余裕を持たせた値にする。
    learning_timeout_s: float = 600.0


@dataclass
class AppSettings:
    serial: SerialSettings = field(default_factory=SerialSettings)
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
    can = CanSettings(**{k: v for k, v in raw.get("can", {}).items()})
    database = DatabaseSettings(**{k: v for k, v in raw.get("database", {}).items()})
    gpio = GpioSettings(**{k: v for k, v in raw.get("gpio", {}).items()})
    archive = ArchiveSettings(**{k: v for k, v in raw.get("archive", {}).items()})
    control = ControlSettings(**{k: v for k, v in raw.get("control", {}).items()})
    safety = SafetySettings(**{k: v for k, v in raw.get("safety", {}).items()})
    servo = ServoSettings(**{k: v for k, v in raw.get("servo", {}).items()})
    ups = UpsSettings(**{k: v for k, v in raw.get("ups", {}).items()})
    model = _parse_model_settings(raw.get("model", {}))
    learning = LearningSettings(**{k: v for k, v in raw.get("learning", {}).items()})

    return AppSettings(
        serial=serial,
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
        past_horizons_s=(
            tuple(past) if isinstance(past, list) else defaults.past_horizons_s
        ),
        regime_horizon_s=float(raw.get("regime_horizon_s", defaults.regime_horizon_s)),  # type: ignore[arg-type]
        include_v0_sq=bool(raw.get("include_v0_sq", defaults.include_v0_sq)),
        include_dv_regime_x_v0=bool(
            raw.get("include_dv_regime_x_v0", defaults.include_dv_regime_x_v0)
        ),
        accel_horizons_s=(
            tuple(accel) if isinstance(accel, list) else defaults.accel_horizons_s
        ),
    )
