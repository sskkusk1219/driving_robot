"""PostgreSQL 接続プール管理のスタブ。実接続は呼び出し元 (RobotController 等) が行う。"""

import asyncpg


class DuplicateNameError(Exception):
    """name の UNIQUE 制約違反のドメイン表現。

    リポジトリ層が asyncpg.UniqueViolationError（または in-memory の同名チェック）を
    この例外に変換し、Web 層が 409 にマッピングする（asyncpg 依存を Web 層に漏らさない）。
    """


async def create_pool(dsn: str) -> asyncpg.Pool:
    """接続プールを作成して返す。"""
    pool = await asyncpg.create_pool(dsn)
    if pool is None:
        raise RuntimeError("asyncpg.create_pool returned None")
    return pool
