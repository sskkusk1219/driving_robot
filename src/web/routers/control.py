"""操作権 API ルーター。"""

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from src.web.control import ControlLock

router = APIRouter(prefix="/api/v1/control", tags=["control"])


class ClaimRequest(BaseModel):
    client_id: str
    instance_id: str


def _lock(request: Request) -> ControlLock:
    lock = getattr(request.app.state, "control", None)
    if lock is None:
        raise HTTPException(status_code=503, detail="操作権の管理が起動していません")
    return lock


@router.get("")
async def get_control(
    request: Request, client_id: str = "", instance_id: str = ""
) -> dict[str, bool]:
    """操作権の状態 {held, mine, waiting}。"""
    return _lock(request).status(client_id, instance_id)


@router.post("/claim")
async def claim_control(request: Request, body: ClaimRequest) -> dict[str, str]:
    """操作権を取る。空いていて WS 接続済みなら 200、使用中・未接続は 409。"""
    lock = _lock(request)
    if not lock.has_connection(body.client_id, body.instance_id):
        raise HTTPException(status_code=409, detail="接続を確認できません。読み直してください")
    if not lock.claim(body.client_id, body.instance_id):
        raise HTTPException(status_code=409, detail="操作権は別のウィンドウが使用中です")
    return {"role": "operator"}
