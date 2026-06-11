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
class UpsSettings:
    nut_host: str = "localhost"
    nut_port: int = 3493
    ups_name: str = "apcups"
    poll_interval_s: float = 5.0


@dataclass
class AppSettings:
    serial: SerialSettings = field(default_factory=SerialSettings)
    can: CanSettings = field(default_factory=CanSettings)
    database: DatabaseSettings = field(default_factory=DatabaseSettings)
    gpio: GpioSettings = field(default_factory=GpioSettings)
    archive: ArchiveSettings = field(default_factory=ArchiveSettings)
    control: ControlSettings = field(default_factory=ControlSettings)
    safety: SafetySettings = field(default_factory=SafetySettings)
    ups: UpsSettings = field(default_factory=UpsSettings)


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
    ups = UpsSettings(**{k: v for k, v in raw.get("ups", {}).items()})

    return AppSettings(
        serial=serial,
        can=can,
        database=database,
        gpio=gpio,
        archive=archive,
        control=control,
        safety=safety,
        ups=ups,
    )
