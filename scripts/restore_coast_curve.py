"""学習セッションのログから惰行減速カーブを再同定し、プロファイルへ復元するスクリプト。

WebUI のプロファイル保存が学習済みの coast_decel_speeds_kmh / coast_decel_kmhs を
スキーマ既定値 `()` で消してしまった場合（2026-07-15 sample_004 で発生、
routers/profiles.py のフィールド単位マージ導入で再発防止済み）に、指定学習セッションの
ログから `estimate_dynamics_params` を再実行してカーブのみを復元する。

カーブ以外のフィールド（engine_brake_decel_kmhs 等）は現在値を保持する。
既定は dry-run（表示のみ）。書き込みは --apply を明示したときだけ行う。

使い方:
    .venv/bin/python -m scripts.restore_coast_curve --profile-id <UUID> --session-id <UUID>
                                                    [--apply] [--dsn ...]
"""

from __future__ import annotations

import argparse
import asyncio
import os
from dataclasses import replace

from src.domain.model_training import estimate_dynamics_params
from src.infra.db import create_pool
from src.infra.profile_repository import ProfileRepository
from src.infra.session_repository import SessionRepository


async def _run(dsn: str, profile_id: str, session_id: str, apply: bool) -> int:
    pool = await create_pool(dsn)
    try:
        profile_repo = ProfileRepository(pool)
        session_repo = SessionRepository(pool)
        profile = await profile_repo.get_by_id(profile_id)
        if profile is None:
            print(f"プロファイル {profile_id!r} が見つかりません")
            return 1
        logs = await session_repo.list_logs_for_training(profile_id, [session_id])
        if not logs:
            print(f"セッション {session_id!r} のログがありません")
            return 1

        current = profile.feedforward_params
        estimated = estimate_dynamics_params(logs, current)
        if len(estimated.coast_decel_speeds_kmh) < 2:
            print("カーブを同定できませんでした（有効ビン<2）。プロファイルは変更しません")
            return 1

        print(f"プロファイル: {profile.name} ({profile_id})")
        print(f"ログ: {len(logs)} 件 (session {session_id})")
        print(f"現在のカーブ: speeds={current.coast_decel_speeds_kmh}")
        print(f"              decels={current.coast_decel_kmhs}")
        print(f"復元カーブ:   speeds={estimated.coast_decel_speeds_kmh}")
        print(f"              decels={tuple(round(d, 3) for d in estimated.coast_decel_kmhs)}")
        print("（カーブ以外のフィールドは現在値を保持します）")

        if not apply:
            print("\ndry-run のため書き込みしません。書き込むには --apply を指定してください")
            return 0

        profile.feedforward_params = replace(
            current,
            coast_decel_speeds_kmh=estimated.coast_decel_speeds_kmh,
            coast_decel_kmhs=estimated.coast_decel_kmhs,
        )
        updated = await profile_repo.update(profile)
        if updated is None:
            print("更新に失敗しました（プロファイルが消えています）")
            return 1
        print(f"\n保存しました: カーブ {len(updated.feedforward_params.coast_decel_kmhs)} ビン")
        return 0
    finally:
        await pool.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="惰行減速カーブをプロファイルへ復元する")
    parser.add_argument("--profile-id", required=True, help="対象プロファイル UUID")
    parser.add_argument("--session-id", required=True, help="再同定に使う学習セッション UUID")
    parser.add_argument("--apply", action="store_true", help="実際に書き込む（省略時は dry-run）")
    parser.add_argument(
        "--dsn",
        default=os.environ.get("DATABASE_URL", "postgresql://localhost/driving_robot"),
    )
    args = parser.parse_args()
    return asyncio.run(_run(args.dsn, args.profile_id, args.session_id, args.apply))


if __name__ == "__main__":
    raise SystemExit(main())
