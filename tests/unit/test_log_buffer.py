"""サーバーログのリングバッファ（LogBufferHandler）と GET /api/v1/drive/log のテスト。"""

import logging

import pytest
from httpx import ASGITransport, AsyncClient

from src.infra.log_buffer import LogBufferHandler
from src.web.app import app


@pytest.fixture
def logger_and_buf():
    buf = LogBufferHandler(maxlen=5)
    logger = logging.getLogger("tests.unit.log_buffer")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    logger.addHandler(buf)
    yield logger, buf
    logger.removeHandler(buf)


def test_seq_increments_and_fields(logger_and_buf) -> None:
    logger, buf = logger_and_buf
    logger.info("one")
    logger.warning("two %d", 2)
    lines = buf.get_lines()
    assert [ln["seq"] for ln in lines] == [1, 2]
    assert lines[1]["message"] == "two 2"
    assert lines[1]["level"] == "WARNING"
    assert lines[0]["logger"] == "tests.unit.log_buffer"
    assert lines[0]["t"] >= 0.0
    assert "time" in lines[0]
    assert buf.last_seq == 2


def test_after_filter(logger_and_buf) -> None:
    logger, buf = logger_and_buf
    for i in range(4):
        logger.info("m%d", i)
    assert [ln["message"] for ln in buf.get_lines(after=2)] == ["m2", "m3"]
    assert buf.get_lines(after=4) == []


def test_limit_oldest_first(logger_and_buf) -> None:
    logger, buf = logger_and_buf
    for i in range(5):
        logger.info("m%d", i)
    assert [ln["seq"] for ln in buf.get_lines(after=0, limit=2)] == [1, 2]
    # after 省略時は直近 limit 行
    assert [ln["seq"] for ln in buf.get_lines(limit=2)] == [4, 5]


def test_ring_overflow_drops_oldest(logger_and_buf) -> None:
    logger, buf = logger_and_buf
    for i in range(8):
        logger.info("m%d", i)
    seqs = [ln["seq"] for ln in buf.get_lines()]
    assert seqs == [4, 5, 6, 7, 8]  # maxlen=5。seq は欠けても連番のまま


def test_below_level_not_recorded(logger_and_buf) -> None:
    logger, buf = logger_and_buf
    logger.debug("hidden")
    assert buf.get_lines() == []


@pytest.fixture
async def client():
    buf = LogBufferHandler(maxlen=500)
    app.state.log_buffer = buf
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        yield c, buf


async def test_log_endpoint_returns_lines_and_last_seq(client) -> None:
    c, buf = client
    lg = logging.getLogger("tests.unit.log_endpoint")
    lg.setLevel(logging.INFO)
    lg.propagate = False
    lg.addHandler(buf)
    try:
        for i in range(3):
            lg.info("msg %d", i)
        res = await c.get("/api/v1/drive/log")
        assert res.status_code == 200
        body = res.json()
        assert [ln["message"] for ln in body["lines"]] == ["msg 0", "msg 1", "msg 2"]
        assert body["last_seq"] == 3
        res = await c.get("/api/v1/drive/log", params={"after": 2})
        body = res.json()
        assert [ln["seq"] for ln in body["lines"]] == [3]
        res = await c.get("/api/v1/drive/log", params={"after": 3})
        body = res.json()
        assert body["lines"] == [] and body["last_seq"] == 3
    finally:
        lg.removeHandler(buf)


async def test_log_endpoint_max_200_oldest_first(client) -> None:
    c, buf = client
    lg = logging.getLogger("tests.unit.log_endpoint2")
    lg.setLevel(logging.INFO)
    lg.propagate = False
    lg.addHandler(buf)
    try:
        for i in range(300):
            lg.info("m%d", i)
        body = (await c.get("/api/v1/drive/log", params={"after": 0})).json()
        assert len(body["lines"]) == 200
        assert body["lines"][0]["seq"] == 1
        assert body["last_seq"] == 200
        body = (await c.get("/api/v1/drive/log")).json()  # after 省略は直近 200 行
        assert body["lines"][0]["seq"] == 101 and body["last_seq"] == 300
    finally:
        lg.removeHandler(buf)


async def test_log_endpoint_without_buffer() -> None:
    if hasattr(app.state, "log_buffer"):
        del app.state.log_buffer
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        body = (await c.get("/api/v1/drive/log")).json()
    assert body == {"lines": [], "last_seq": 0}
