# ruff: noqa: E501
"""モード CSV の取り込み: 最後に車速 0 km/h が 1 秒以上続くこと（ProblemReport_20261123_src全動作チェック2 #12）。"""

from __future__ import annotations

import pytest

from src.web.routers.modes import _parse_csv


def _csv(rows: list[tuple[float, float]]) -> bytes:
    return ("time_s,speed_kmh\n" + "".join(f"{t},{v}\n" for t, v in rows)).encode()


def test_trailing_zero_one_second_or_more_is_accepted() -> None:
    pts = _parse_csv(_csv([(0, 0), (1, 10), (2, 0), (3, 0)]))
    assert pts[-1].time_s == 3
    assert len(_parse_csv(_csv([(0, 10), (1, 0), (2, 0), (3, 0)]))) == 4


@pytest.mark.parametrize(
    "rows, seconds",
    [
        ([(0, 0), (1, 10), (2, 5)], "0"),  # 最後が 0 でない
        ([(0, 0), (1, 10), (2, 0)], "0"),  # 0 の行が 1 本だけ（続いていない）
        ([(0, 0), (1, 10), (2, 0), (2.5, 0)], "0.5"),  # 1 秒に足りない
    ],
)
def test_trailing_zero_too_short_is_rejected(rows: list[tuple[float, float]], seconds: str) -> None:
    with pytest.raises(ValueError, match=f"最後に車速 0 km/h が 1 秒以上続く行が要ります（今は {seconds} 秒）"):
        _parse_csv(_csv(rows))
