"""PostgreSQL 接続プール管理のスタブ。実接続は呼び出し元 (RobotController 等) が行う。"""

import asyncpg


async def create_pool(dsn: str) -> asyncpg.Pool:
    """接続プールを作成して返す。"""
    pool = await asyncpg.create_pool(dsn)
    if pool is None:
        raise RuntimeError("asyncpg.create_pool returned None")
    return pool
