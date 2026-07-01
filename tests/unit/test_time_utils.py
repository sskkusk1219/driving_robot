"""src.utils.time.to_jst_naive のテスト。"""

from datetime import UTC, datetime
from zoneinfo import ZoneInfo

from src.utils.time import to_jst_naive


def test_utc_aware_converted_to_jst_naive() -> None:
    """UTC aware datetime が +9h された naive datetime になること。"""
    dt = datetime(2026, 6, 18, 22, 53, 5, 938436, tzinfo=UTC)
    result = to_jst_naive(dt)
    assert result == datetime(2026, 6, 19, 7, 53, 5, 938436)
    assert result.tzinfo is None


def test_naive_input_treated_as_utc() -> None:
    """naive 入力は UTC とみなして JST へ変換されること。"""
    dt = datetime(2026, 1, 1, 0, 0, 0)
    assert to_jst_naive(dt) == datetime(2026, 1, 1, 9, 0, 0)


def test_non_utc_aware_input_converted() -> None:
    """UTC 以外の aware datetime も JST へ正しく変換されること。"""
    dt = datetime(2026, 1, 1, 12, 0, 0, tzinfo=ZoneInfo("America/New_York"))
    # EST(-05:00) 12:00 → UTC 17:00 → JST 翌02:00
    assert to_jst_naive(dt) == datetime(2026, 1, 2, 2, 0, 0)


def test_isoformat_space_separator() -> None:
    """素の時刻フォーマット（スペース区切り・オフセット無し）になること。"""
    dt = datetime(2026, 6, 18, 22, 53, 5, 938436, tzinfo=UTC)
    assert to_jst_naive(dt).isoformat(sep=" ") == "2026-06-19 07:53:05.938436"
