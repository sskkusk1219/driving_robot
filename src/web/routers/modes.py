from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter(prefix="/api/v1/modes", tags=["modes"])


class ModeStub(BaseModel):
    id: str
    name: str


@router.get("/", response_model=list[ModeStub])
async def list_modes() -> list[ModeStub]:
    return []


@router.get("/{mode_id}", response_model=ModeStub)
async def get_mode(mode_id: str) -> ModeStub:
    raise HTTPException(status_code=404, detail=f"Mode {mode_id!r} not found")


@router.post("/", status_code=501)
async def create_mode() -> dict[str, str]:
    raise HTTPException(status_code=501, detail="Not implemented")
