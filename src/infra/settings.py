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
class AppSettings:
    serial: SerialSettings = field(default_factory=SerialSettings)
    can: CanSettings = field(default_factory=CanSettings)
    database: DatabaseSettings = field(default_factory=DatabaseSettings)
    gpio: GpioSettings = field(default_factory=GpioSettings)
    archive: ArchiveSettings = field(default_factory=ArchiveSettings)
    control: ControlSettings = field(default_factory=ControlSettings)


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

    return AppSettings(
        serial=serial,
        can=can,
        database=database,
        gpio=gpio,
        archive=archive,
        control=control,
    )
