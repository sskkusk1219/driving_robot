from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter(prefix="/api/v1/profiles", tags=["profiles"])


class ProfileStub(BaseModel):
    id: str
    name: str


@router.get("/", response_model=list[ProfileStub])
async def list_profiles() -> list[ProfileStub]:
    return []


@router.get("/{profile_id}", response_model=ProfileStub)
async def get_profile(profile_id: str) -> ProfileStub:
    raise HTTPException(status_code=404, detail=f"Profile {profile_id!r} not found")


@router.post("/", status_code=501)
async def create_profile() -> dict[str, str]:
    raise HTTPException(status_code=501, detail="Not implemented")


@router.put("/{profile_id}", status_code=501)
async def update_profile(profile_id: str) -> dict[str, str]:
    raise HTTPException(status_code=501, detail="Not implemented")


@router.delete("/{profile_id}", status_code=501)
async def delete_profile(profile_id: str) -> dict[str, str]:
    raise HTTPException(status_code=501, detail="Not implemented")
