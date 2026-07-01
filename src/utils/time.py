"""タイムゾーン変換のユーティリティ。

DB の TIMESTAMPTZ は UTC で保存されるため、CSV 出力など人間が読む箇所では
JST（日本標準時）へ変換する。web 層・infra 層の双方から利用する純粋関数。
"""

from datetime import datetime
from zoneinfo import ZoneInfo

JST = ZoneInfo("Asia/Tokyo")
_UTC = ZoneInfo("UTC")


def to_jst_naive(dt: datetime) -> datetime:
    """aware/naive datetime を JST に変換し、tzinfo を外した naive datetime を返す。

    naive（tzinfo なし）が渡された場合は UTC とみなす（DB の TIMESTAMPTZ は
    asyncpg から UTC aware で返るが、防御的に扱う）。
    """
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=_UTC)
    return dt.astimezone(JST).replace(tzinfo=None)
