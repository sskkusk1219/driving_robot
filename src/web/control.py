"""操作権（ControlLock）。同時に変更系 API を打てるブラウザウィンドウを 1 つ（操作）に限る。

ウィンドウの印は client_id（タブごと・読み直しで残る）と instance_id（読み込みのたびに新規）。
持ち主が生きているかは WebSocket（/ws/realtime）の接続と ping で見る。単一プロセス・asyncio 前提で、
ロックは使わない（awaitを挟まない同期メソッドだけで状態を変える）。
"""

import time
from collections.abc import Callable
from typing import Any

#: 画面が ping を送る間隔（秒）。仮値。
#: Pi の WebSocket 配信は 0.1 s 周期で、ping 1 通が数百 ms 遅れても切れ扱いにならない程度に
#: 短く、通信を増やしすぎない値として 2 s とした。
PING_INTERVAL_S = 2.0

#: ping がこれだけ途絶えたら「切れた」とみなす（秒）。仮値。
#: ping 5 回分。ブラウザの裏タブでタイマーが間引かれても誤判定しにくい余裕を見ている。
STALE_AFTER_S = 10.0

#: 持ち主が切れてから操作権を空けるまでの待ち（秒）。仮値。
#: Pi でのページ読み直しは 1〜2 s。その数倍の余裕を見て、読み直しなら同じ client_id が
#: 戻ってきて持ち主を付け替えられるようにする。長すぎると他の人が取れない。
GRACE_S = 10.0


class ControlLock:
    """操作権の持ち主と、その WebSocket の生死を管理する。"""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        # 接続中の WebSocket → (client_id, instance_id)
        self._conns: dict[Any, tuple[str, str]] = {}
        self._holder: tuple[str, str] | None = None
        self._holder_ws: Any = None  # 持ち主に結び付いた WS。None なら切れ（待ち）
        self._last_ping: float = 0.0
        self._grace_deadline: float | None = None  # 待ちの期限。WS が生きている間は None

    # ── 内部 ──
    def _expire(self) -> None:
        """ping 途絶を切断として扱い、待ちが過ぎたら操作権を空ける（どの入口からも呼ぶ）。"""
        if self._holder is None:
            return
        now = self._clock()
        if self._holder_ws is not None and now - self._last_ping > STALE_AFTER_S:
            # 途絶は「最後の ping から STALE_AFTER_S 後」に切れたものとして待ちを数える
            self._holder_ws = None
            self._grace_deadline = self._last_ping + STALE_AFTER_S + GRACE_S
        deadline = self._grace_deadline
        if self._holder_ws is None and deadline is not None and now >= deadline:
            self._release()

    def _release(self) -> None:
        self._holder = None
        self._holder_ws = None
        self._grace_deadline = None

    def _bind(self, client_id: str, instance_id: str, ws: Any) -> None:
        self._holder = (client_id, instance_id)
        self._holder_ws = ws
        self._last_ping = self._clock()
        self._grace_deadline = None

    # ── 公開 ──
    @property
    def held(self) -> bool:
        self._expire()
        return self._holder is not None

    def on_connect(self, client_id: str, instance_id: str, ws: Any) -> str:
        """接続を登録し、役割を返す（operator / duplicate / viewer）。"""
        self._expire()
        self._conns[ws] = (client_id, instance_id)
        if self._holder is not None and self._holder[0] == client_id:
            if self._holder[1] == instance_id:
                self._bind(client_id, instance_id, ws)  # 同じ画面の再接続
                return "operator"
            if self._holder_ws is None:
                self._bind(client_id, instance_id, ws)  # 読み直し: 持ち主を付け替える
                return "operator"
            return "duplicate"  # 持ち主が生きているのに同じ client_id の別画面（タブの複製）
        return "viewer"

    def on_ping(self, ws: Any) -> None:
        self._expire()
        if ws is not None and ws is self._holder_ws:
            self._last_ping = self._clock()

    def on_disconnect(self, ws: Any) -> None:
        self._expire()
        self._conns.pop(ws, None)
        if self._holder is not None and ws is self._holder_ws:
            self._holder_ws = None
            self._grace_deadline = self._clock() + GRACE_S

    def has_connection(self, client_id: str, instance_id: str) -> bool:
        return any(ids == (client_id, instance_id) for ids in self._conns.values())

    def claim(self, client_id: str, instance_id: str) -> bool:
        """誰も持っていなければ持ち主になる。WS 接続済みであることが条件。"""
        self._expire()
        if self._holder is not None:
            return False
        for ws, ids in self._conns.items():
            if ids == (client_id, instance_id):
                self._bind(client_id, instance_id, ws)
                return True
        return False

    def is_operator(self, client_id: str | None, instance_id: str | None) -> bool:
        self._expire()
        return self._holder is not None and self._holder == (client_id, instance_id)

    def status(self, client_id: str | None, instance_id: str | None) -> dict[str, bool]:
        self._expire()
        return {
            "held": self._holder is not None,
            "mine": self.is_operator(client_id, instance_id),
            "waiting": self._holder is not None and self._holder_ws is None,
        }

    def tick(self) -> None:
        """定期の期限切れ確認（broadcast_loop から呼ぶ）。"""
        self._expire()
