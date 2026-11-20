"""src/robot/config_store.py（共通 + 車両の合成・車両 yaml への保存の写し）のテスト。

tests/research は import しない（本番とテストの隔離）。tests の load_config との一致は
scripts/check_config_equivalence.py（等価確認①）で確かめる。
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from src.robot import config_store as cs
from src.robot.core.config import ConfigError, FeedforwardSection
from src.robot.core.pattern_drive import FF_PARAM_KEYS, MEASURED_KEYS

COMMON = """\
# 共通のコメント
pid:
  kp: 1.5          # Kp のコメント
feedforward:
  candidate: C8
  ref_smooth_sigma_s: 0.15
"""

VEHICLE = """\
# 車両のコメント
vehicle:
  name: car_a
  max_speed_kmh: 150.0
  road_load_file: "tests/research/private/road_load.yaml"
feedforward:
  # model_path のコメント
  model_path: m1.pkl   # 行末コメント
  creep_speed_kmh: 5.0  # クリープ
  coast_decel_speeds_kmh: [5.0, 15.0]
buttons:
  enabled: true
  servo_rest_angle_deg: 80.0
"""


@pytest.fixture
def tree(tmp_path: Path) -> dict[str, Path]:
    common = tmp_path / "robot_common.yaml"
    common.write_text(COMMON, encoding="utf-8")
    vdir = tmp_path / "vehicles" / "car_a"
    vdir.mkdir(parents=True)
    (vdir / "vehicle.yaml").write_text(VEHICLE, encoding="utf-8")
    return {"common": common, "vehicles": tmp_path / "vehicles", "vehicle": vdir / "vehicle.yaml"}


def _load(tree: dict[str, Path]):
    return cs.load_vehicle_config(
        "car_a", vehicles_dir=tree["vehicles"], common_path=tree["common"], validate=False
    )


def test_merge_overrides_placement_keys(tree: dict[str, Path]) -> None:
    cfg, merged = _load(tree)
    root = (tree["vehicles"] / "car_a").resolve()
    assert merged == root / ".work" / "merged.yaml"
    assert cfg.source_path == merged
    assert cfg.pid.kp == 1.5  # 共通
    assert cfg.feedforward.candidate == "C8"  # 共通
    assert cfg.vehicle.name == "car_a"  # 車両
    assert cfg.vehicle.road_load_file is None  # 無効にする
    assert cfg.output.plot is False
    assert Path(cfg.output.results_dir) == root / "data"
    assert Path(cfg.feedforward.model_path) == root / "models" / "m1.pkl"
    # 車両 yaml の buttons: は無視する（段11: ボタンの設定は共通設定）
    assert cfg.buttons.servo_rest_angle_deg != 80.0
    assert "buttons" not in yaml.safe_load(merged.read_text(encoding="utf-8"))


def test_buttons_come_from_common_and_vehicle_buttons_are_ignored(
    tree: dict[str, Path], caplog: pytest.LogCaptureFixture
) -> None:
    tree["common"].write_text(
        COMMON + "buttons:\n  servo_rest_angle_deg: 95.0\n  installed: false\n", encoding="utf-8"
    )
    with caplog.at_level("WARNING"):
        cfg, _ = _load(tree)
    assert cfg.buttons.servo_rest_angle_deg == 95.0  # 車両 yaml の 80.0 ではなく共通の値
    assert cfg.src_only["buttons.installed"] is False
    assert any("buttons" in r.getMessage() for r in caplog.records)
    assert not cs.is_vehicle_key("buttons.servo_rest_angle_deg")


def test_missing_model_path_points_to_untrained_pkl(tree: dict[str, Path]) -> None:
    tree["vehicle"].write_text("vehicle:\n  name: car_a\n", encoding="utf-8")
    cfg, _ = _load(tree)
    root = (tree["vehicles"] / "car_a").resolve()
    assert Path(cfg.feedforward.model_path) == root / "models" / cs.UNTRAINED_MODEL_NAME
    assert "tests/research" not in cfg.feedforward.model_path


def test_vehicle_yaml_cannot_override_common_keys(tree: dict[str, Path]) -> None:
    tree["vehicle"].write_text(VEHICLE + "pid:\n  kp: 9.0\n", encoding="utf-8")
    with pytest.raises(cs.VehicleConfigError, match="pid"):
        _load(tree)
    tree["vehicle"].write_text(
        VEHICLE.replace("  creep_speed_kmh: 5.0", "  candidate: C1"), encoding="utf-8"
    )
    with pytest.raises(cs.VehicleConfigError, match="feedforward.candidate"):
        _load(tree)


def test_vehicle_id_must_be_safe(tree: dict[str, Path]) -> None:
    with pytest.raises(cs.VehicleConfigError):
        cs.load_vehicle_config("../x", vehicles_dir=tree["vehicles"], common_path=tree["common"])


def test_sync_writes_saved_vehicle_keys_and_keeps_comments(tree: dict[str, Path]) -> None:
    cfg, _ = _load(tree)
    new_pkl = Path(cfg.feedforward.model_path).with_name("m2.pkl")
    changed = cfg.save(
        {
            "feedforward.model_path": str(new_pkl),
            "feedforward.creep_speed_kmh": 5.25,
            "feedforward.coast_decel_speeds_kmh": [5.0, 15.0, 25.0],
        }
    )
    assert len(changed) == 3
    lines = cs.sync_saved_to_vehicle(
        "car_a", vehicles_dir=tree["vehicles"], common_path=tree["common"]
    )
    assert len(lines) == 3
    text = tree["vehicle"].read_text(encoding="utf-8")
    assert "model_path: m2.pkl  # 行末コメント" in text  # models/ 基準の相対名・コメント保持
    assert "# model_path のコメント" in text and "# 車両のコメント" in text
    data = yaml.safe_load(text)
    assert data["feedforward"]["creep_speed_kmh"] == 5.25
    assert data["feedforward"]["coast_decel_speeds_kmh"] == [5.0, 15.0, 25.0]
    # 共通は変わらない・二度目は何も書かない
    assert tree["common"].read_text(encoding="utf-8") == COMMON
    assert cs.sync_saved_to_vehicle(
        "car_a", vehicles_dir=tree["vehicles"], common_path=tree["common"]
    ) == []


def test_sync_rejects_key_missing_in_vehicle_yaml(tree: dict[str, Path]) -> None:
    cfg, merged = _load(tree)
    # core 自身は合成 yaml に無いキーを保存できない（ConfigError）
    with pytest.raises(ConfigError):
        cfg.save({"feedforward.creep_rate_kmhs": 0.7})
    # 合成 yaml にだけキーがあり、車両 yaml に無い場合は、写さずに例外（キーを勝手に足さない）
    data = yaml.safe_load(merged.read_text(encoding="utf-8"))
    data["feedforward"]["creep_rate_kmhs"] = 0.7
    merged.write_text(cs._dump(data), encoding="utf-8")
    with pytest.raises(cs.VehicleConfigError, match="creep_rate_kmhs"):
        cs.sync_saved_to_vehicle("car_a", vehicles_dir=tree["vehicles"], common_path=tree["common"])
    assert "creep_rate_kmhs" not in tree["vehicle"].read_text(encoding="utf-8")


def test_sync_rejects_changed_common_key(tree: dict[str, Path]) -> None:
    cfg, _ = _load(tree)
    cfg.save({"pid.kp": 2.5})
    with pytest.raises(cs.VehicleConfigError, match="pid.kp"):
        cs.sync_saved_to_vehicle("car_a", vehicles_dir=tree["vehicles"], common_path=tree["common"])
    assert tree["common"].read_text(encoding="utf-8") == COMMON


def test_install_save_sync(tree: dict[str, Path]) -> None:
    cfg, _ = _load(tree)
    cs.install_save_sync(cfg, "car_a", vehicles_dir=tree["vehicles"], common_path=tree["common"])
    cfg.save({"pid.kp": 1.5})  # 手順 0 の同値の書き戻し: 共通のキーでも値が同じなら通る
    cfg.save({"feedforward.creep_speed_kmh": 4.5})
    assert yaml.safe_load(tree["vehicle"].read_text(encoding="utf-8"))["feedforward"][
        "creep_speed_kmh"
    ] == 4.5
    merged_before = cfg.source_path.read_text(encoding="utf-8")
    with pytest.raises(cs.VehicleConfigError, match="pid.kp"):
        cfg.save({"pid.kp": 3.0})  # 共通のキーを変える保存は、何も書く前に断る
    assert cfg.source_path.read_text(encoding="utf-8") == merged_before


def test_vehicle_keys_cover_everything_core_saves() -> None:
    """手順 2・2-0 が cfg.save で書く feedforward のキーが、すべて車両キーに入っている。"""
    saved = {f"{k}" for k in FF_PARAM_KEYS if k not in MEASURED_KEYS}
    # pedal_search.config_updates と build_ff_model が書くキー
    saved |= {"model_path", "creep_speed_kmh", "stop_brake_offset_pct"}
    missing = sorted(saved - set(cs.VEHICLE_FEEDFORWARD_KEYS))
    assert not missing, f"車両キーに無い保存キー: {missing}"
    fields = {f for f in FeedforwardSection.__dataclass_fields__}
    legacy = {
        "model_accel_horizons_s", "model_brake_horizons_s", "model_accel_features",
        "model_brake_features", "model_coef_path",
    }  # fmt: skip
    not_fields = sorted(set(cs.VEHICLE_FEEDFORWARD_KEYS) - fields)
    assert set(not_fields) == legacy  # 今の core のフィールドに無いのは多項式モデルの参考キーだけ


def test_repo_common_yaml_has_no_vehicle_keys() -> None:
    """リポジトリの config/robot_common.yaml（あれば）に車両キーが残っていない。"""
    path = Path(__file__).resolve().parents[2] / "config" / "robot_common.yaml"
    if not path.exists():
        pytest.skip("config/robot_common.yaml がまだありません")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    leaks = [
        f"{sec}.{k}"
        for sec, body in data.items()
        for k in (body if isinstance(body, dict) else {})
        if cs.is_vehicle_key(f"{sec}.{k}")
    ]
    assert not leaks, f"共通設定に車両キーがあります: {leaks}"
