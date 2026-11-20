"""学習開始 API は全部学習（full）だけ。部分学習の kind は 422。

PostgreSQL 不要。422 は manager を呼ぶ前の検査で返るので manager は要らない。
"""

import pytest
from httpx import ASGITransport, AsyncClient

from src.web.app import app


@pytest.fixture
async def client():
    app.state.job_manager = object()  # 422 は manager を使う前に返る（依存の解決だけ通す）
    app.state.vehicle_store = object()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        yield c


PATHS = ["/api/v1/drive/learning/start", "/api/v1/drive/learning-cycle/start"]


@pytest.mark.parametrize("kind", ["fel", "hold", "stop", "nope"])
@pytest.mark.parametrize("path", PATHS)
async def test_partial_learning_kind_is_422(client, path, kind):
    r = await client.post(path, json={"kind": kind})
    assert r.status_code == 422
    assert "学習は全部学習だけです" in r.text
