"""UPS 状態取得 API ルーター。"""

from fastapi import APIRouter, Depends

from src.web.deps import UPSMonitorProtocol, get_ups_monitor
from src.web.schemas import UPSStatusResponse

router = APIRouter(prefix="/api/v1/ups", tags=["ups"])


@router.get("/status", response_model=UPSStatusResponse)
async def get_ups_status(
    ups_monitor: UPSMonitorProtocol = Depends(get_ups_monitor),
) -> UPSStatusResponse:
    """UPS の現在状態（バッテリー残量・AC断フラグ・NUT接続状態）を返す。"""
    status = await ups_monitor.get_status()
    return UPSStatusResponse(
        battery_charge_pct=status.battery_charge_pct,
        on_battery=status.on_battery,
        low_battery=status.low_battery,
        status_flags=status.status_flags,
        is_available=status.is_available,
    )
