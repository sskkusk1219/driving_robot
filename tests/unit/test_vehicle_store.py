# ruff: noqa: E501
"""車両ストア（src/infra/vehicle_store.py）: 一覧・作成・更新・削除・選んでいる車。"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from src.infra import vehicle_store as vs
from src.robot import config_store

COMMON = Path(__file__).resolve().parents[2] / "config" / "robot_common.yaml"

pytestmark = pytest.mark.skipif(not COMMON.exists(), reason="config/robot_common.yaml が無い")


@pytest.fixture()
def store(tmp_path: Path) -> vs.VehicleStore:
    return vs.VehicleStore(tmp_path / "vehicles", COMMON)


def test_create_from_template_has_no_learning_and_unmeasured_min(store: vs.VehicleStore) -> None:
    info = store.create("テスト車 A", vehicle_id="car_a")
    assert info.id == "car_a" and info.name == "テスト車 A"
    assert info.learned is False and info.model_path is None and info.learned_at is None
    assert info.min_accel_measured is False and info.min_brake_measured is False
    assert "enabled" not in info.buttons  # ボタンの有無は無い（モード走行は常に手順 24）
    assert info.candidate  # 共通設定の候補名（読むだけ）
    assert (store.vehicles_dir / "car_a" / "models").is_dir() and (store.vehicles_dir / "car_a" / "data").is_dir()
    # ランナーと同じ読み込み（共通＋車両の合成・core の検証）を通る
    cfg, _ = config_store.load_vehicle_config("car_a", vehicles_dir=store.vehicles_dir, common_path=COMMON)
    assert cfg.vehicle.name == "テスト車 A"


def test_create_makes_id_from_name_and_avoids_collision(store: vs.VehicleStore) -> None:
    a = store.create("My Car")
    b = store.create("My Car")
    c = store.create("日本語だけ")
    assert a.id == "My_Car" and b.id == "My_Car_2"
    assert c.id.startswith("vehicle_")
    assert store.list_ids() == sorted([a.id, b.id, c.id])


def test_create_rejects_bad_input(store: vs.VehicleStore) -> None:
    with pytest.raises(vs.VehicleInvalid):
        store.create("  ")
    with pytest.raises(vs.VehicleInvalid):
        store.create("x", vehicle_id="../evil")
    store.create("x", vehicle_id="dup")
    with pytest.raises(vs.VehicleExists):
        store.create("y", vehicle_id="dup")
    # 失敗した作成のフォルダは残らない
    with pytest.raises(vs.VehicleInvalid):
        store.create("bad", vehicle_id="bad", values={"max_speed_kmh": 9999.0})
    assert not (store.vehicles_dir / "bad").exists()


def test_update_human_values_keeps_comments_and_marks_min_measured(store: vs.VehicleStore) -> None:
    store.create("A", vehicle_id="a")
    path = store.vehicles_dir / "a" / "vehicle.yaml"
    info = store.update(
        "a",
        vehicle={"name": "新名称", "max_speed_kmh": 120.0, "min_accel_opening_pct": 7.5, "test_mass_kg": 1500.0},
    )
    assert info.name == "新名称" and info.vehicle["max_speed_kmh"] == 120.0
    assert info.min_accel_measured is True and info.min_brake_measured is False
    assert info.buttons["shift_press_s"] == 1.2  # ボタンの設定は共通設定の値（読むだけ）
    text = path.read_text(encoding="utf-8")
    assert "# 最高速 [km/h]" in text and "feedforward のキー" not in text  # コメントは保たれる


def test_update_validates_with_core_and_writes_nothing_on_failure(store: vs.VehicleStore) -> None:
    store.create("A", vehicle_id="a")
    path = store.vehicles_dir / "a" / "vehicle.yaml"
    before = path.read_text(encoding="utf-8")
    with pytest.raises(vs.VehicleInvalid) as exc:
        store.update("a", vehicle={"min_accel_opening_pct": 90.0})  # max 80 以上
    assert any("min_accel_opening_pct" in p for p in exc.value.problems)
    assert path.read_text(encoding="utf-8") == before


def test_learned_values_are_read_only(store: vs.VehicleStore) -> None:
    store.create("A", vehicle_id="a")
    for bad in ({"feedforward.model_path": "x.pkl"}, {"model_path": "x.pkl"}, {"road_load_file": "x"}):
        with pytest.raises(vs.VehicleInvalid):
            store.update("a", vehicle=bad)
    with pytest.raises(vs.VehicleInvalid):
        store.update("a", vehicle={"max_speed_kmh": "fast"})  # 型違い
    with pytest.raises(vs.VehicleInvalid):
        store.update("a", vehicle={"max_speed_kmh": True})  # bool は数値ではない


def test_learned_flag_follows_pkl_existence(store: vs.VehicleStore) -> None:
    store.create("A", vehicle_id="a")
    root = store.vehicles_dir / "a"
    text = (root / "vehicle.yaml").read_text(encoding="utf-8").replace("model_path: null", "model_path: m.pkl")
    (root / "vehicle.yaml").write_text(text, encoding="utf-8")
    assert store.read("a").learned is False  # 名前だけで pkl が無い
    (root / "models" / "m.pkl").write_bytes(b"x")
    info = store.read("a")
    assert info.learned is True and info.model_path == "m.pkl" and info.learned_at is not None


def test_legacy_vehicle_without_state_is_measured(store: vs.VehicleStore) -> None:
    store.create("A", vehicle_id="a")
    (store.vehicles_dir / "a" / vs.STATE_FILE).unlink()
    info = store.read("a")
    assert info.min_accel_measured and info.min_brake_measured


def test_mark_min_measured(store: vs.VehicleStore) -> None:
    store.create("A", vehicle_id="a")
    store.mark_min_measured("a", "brake")
    info = store.read("a")
    assert info.min_brake_measured is True and info.min_accel_measured is False
    store.mark_min_measured("a", "both")
    assert store.read("a").min_accel_measured is True


def test_delete_and_active(store: vs.VehicleStore) -> None:
    store.create("A", vehicle_id="a")
    store.create("B", vehicle_id="b")
    assert store.get_active() is None
    store.set_active("a")
    assert store.get_active() == "a"
    assert (store.vehicles_dir / vs.ACTIVE_FILE).read_text().strip() == "a"  # 再起動後も残る
    assert vs.VehicleStore(store.vehicles_dir, COMMON).get_active() == "a"
    with pytest.raises(vs.VehicleNotFound):
        store.set_active("nope")
    store.delete("a")
    assert not (store.vehicles_dir / "a").exists() and store.get_active() is None
    assert store.list_ids() == ["b"]
    with pytest.raises(vs.VehicleNotFound):
        store.delete("a")
    with pytest.raises(vs.VehicleNotFound):
        store.read("../b")


def test_broken_yaml_is_listed_with_error(store: vs.VehicleStore) -> None:
    store.create("A", vehicle_id="a")
    (store.vehicles_dir / "a" / "vehicle.yaml").write_text("vehicle: [unclosed", encoding="utf-8")
    items = store.list_all()
    assert len(items) == 1 and items[0].id == "a" and items[0].error
    shutil.rmtree(store.vehicles_dir / "a")
