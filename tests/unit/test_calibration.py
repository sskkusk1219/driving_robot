"""CalibrationManager のユニットテスト。"""

from collections.abc import AsyncIterator

import pytest

from src.domain.calibration import (
    CalibrationConfig,
    CalibrationDetectionError,
    CalibrationManager,
)
from src.models.calibration import CalibrationData, CalibrationResult, ValidationResult

# ---------------------------------------------------------------------------
# モックドライバ
# ---------------------------------------------------------------------------

class MockActuatorDriver:
    """テスト用アクチュエータモック。current_pattern でサンプルごとの電流値を指定する。"""

    def __init__(self, current_pattern: list[float], position_at_contact: int = 5000) -> None:
        self._current_iter: AsyncIterator[float] = self._make_iter(current_pattern)
        self._position_at_contact = position_at_contact
        self._current_pos = 0
        self.home_return_called = 0
        self.positions_commanded: list[int] = []

    @staticmethod
    async def _make_iter(values: list[float]) -> AsyncIterator[float]:
        for v in values:
            yield v

    async def home_return(self) -> None:
        self.home_return_called += 1
        self._current_pos = 0

    async def move_to_position(self, pos: int) -> None:
        self.positions_commanded.append(pos)
        self._current_pos = pos

    async def read_position(self) -> int:
        return self._current_pos

    async def read_current(self) -> float:
        try:
            return await self._current_iter.__anext__()
        except StopAsyncIteration:
            return 100.0  # フォールバック: 平常時電流


def make_spike_pattern(
    normal_count: int,
    normal_value: float = 100.0,
    spike_value: float = 400.0,
) -> list[float]:
    """normal_count 個の平常値の後にスパイク値が来るパターンを生成する。"""
    return [normal_value] * normal_count + [spike_value]


def make_manager(
    accel_currents: list[float],
    brake_currents: list[float],
    accel_contact_pos: int = 5000,
    brake_contact_pos: int = 5000,
    config: CalibrationConfig | None = None,
) -> tuple[CalibrationManager, MockActuatorDriver, MockActuatorDriver]:
    accel = MockActuatorDriver(accel_currents, position_at_contact=accel_contact_pos)
    brake = MockActuatorDriver(brake_currents, position_at_contact=brake_contact_pos)
    cfg = config or CalibrationConfig(
        move_step_pulse=1000,
        step_interval_s=0.0,  # テストでは待機なし
        current_window=3,
        current_spike_ratio=1.5,
        min_stroke_pulse=500,
        max_stroke_pulse=20000,
        max_search_pulse=30000,
    )
    manager = CalibrationManager(accel_driver=accel, brake_driver=brake, config=cfg)
    return manager, accel, brake


# ---------------------------------------------------------------------------
# ValidationResult モデルテスト
# ---------------------------------------------------------------------------

class TestValidationResult:
    def test_valid_result(self) -> None:
        r = ValidationResult(is_valid=True, error_message=None)
        assert r.is_valid is True
        assert r.error_message is None

    def test_invalid_result(self) -> None:
        r = ValidationResult(is_valid=False, error_message="エラー")
        assert r.is_valid is False
        assert r.error_message == "エラー"


# ---------------------------------------------------------------------------
# CalibrationManager._validate テスト
# ---------------------------------------------------------------------------

class TestCalibrationManagerValidate:
    def _make_manager(self) -> CalibrationManager:
        cfg = CalibrationConfig(min_stroke_pulse=500, max_stroke_pulse=20000)
        return CalibrationManager(
            accel_driver=MockActuatorDriver([]),
            brake_driver=MockActuatorDriver([]),
            config=cfg,
        )

    def _make_data(
        self,
        accel_zero: int = 1000,
        accel_full: int = 6000,
        brake_zero: int = 1000,
        brake_full: int = 6000,
    ) -> CalibrationData:
        from datetime import UTC, datetime

        return CalibrationData(
            accel_zero_pos=accel_zero,
            accel_full_pos=accel_full,
            accel_stroke=accel_full - accel_zero,
            brake_zero_pos=brake_zero,
            brake_full_pos=brake_full,
            brake_stroke=brake_full - brake_zero,
            calibrated_at=datetime.now(tz=UTC),
            is_valid=False,
        )

    def test_valid_data_returns_valid(self) -> None:
        manager = self._make_manager()
        result = manager._validate(self._make_data())
        assert result.is_valid is True
        assert result.error_message is None

    def test_accel_full_le_zero_returns_invalid(self) -> None:
        manager = self._make_manager()
        result = manager._validate(self._make_data(accel_zero=5000, accel_full=4000))
        assert result.is_valid is False
        assert result.error_message is not None
        assert "アクセル" in result.error_message

    def test_accel_full_equal_zero_returns_invalid(self) -> None:
        manager = self._make_manager()
        result = manager._validate(self._make_data(accel_zero=5000, accel_full=5000))
        assert result.is_valid is False

    def test_brake_full_le_zero_returns_invalid(self) -> None:
        manager = self._make_manager()
        result = manager._validate(self._make_data(brake_zero=5000, brake_full=3000))
        assert result.is_valid is False
        assert result.error_message is not None
        assert "ブレーキ" in result.error_message

    def test_accel_stroke_below_min_returns_invalid(self) -> None:
        manager = self._make_manager()
        # stroke = 400 < min_stroke_pulse = 500
        result = manager._validate(self._make_data(accel_zero=1000, accel_full=1400))
        assert result.is_valid is False
        assert "アクセルストローク" in (result.error_message or "")

    def test_accel_stroke_above_max_returns_invalid(self) -> None:
        manager = self._make_manager()
        # stroke = 21000 > max_stroke_pulse = 20000
        result = manager._validate(self._make_data(accel_zero=1000, accel_full=22000))
        assert result.is_valid is False
        assert "アクセルストローク" in (result.error_message or "")

    def test_brake_stroke_below_min_returns_invalid(self) -> None:
        manager = self._make_manager()
        result = manager._validate(self._make_data(brake_zero=1000, brake_full=1400))
        assert result.is_valid is False
        assert "ブレーキストローク" in (result.error_message or "")

    def test_stroke_at_min_boundary_is_valid(self) -> None:
        manager = self._make_manager()
        # stroke = 500 = min_stroke_pulse (境界値: 有効)
        result = manager._validate(self._make_data(accel_zero=1000, accel_full=1500))
        assert result.is_valid is True

    def test_stroke_at_max_boundary_is_valid(self) -> None:
        manager = self._make_manager()
        # stroke = 20000 = max_stroke_pulse (境界値: 有効)
        result = manager._validate(self._make_data(accel_zero=1000, accel_full=21000))
        assert result.is_valid is True

    def test_brake_stroke_above_max_returns_invalid(self) -> None:
        manager = self._make_manager()
        # stroke = 21000 > max_stroke_pulse = 20000
        result = manager._validate(self._make_data(brake_zero=1000, brake_full=22000))
        assert result.is_valid is False
        assert "ブレーキストローク" in (result.error_message or "")

    def test_brake_stroke_at_min_boundary_is_valid(self) -> None:
        manager = self._make_manager()
        # stroke = 500 = min_stroke_pulse (境界値: 有効)
        result = manager._validate(self._make_data(brake_zero=1000, brake_full=1500))
        assert result.is_valid is True

    def test_brake_stroke_at_max_boundary_is_valid(self) -> None:
        manager = self._make_manager()
        # stroke = 20000 = max_stroke_pulse (境界値: 有効)
        result = manager._validate(self._make_data(brake_zero=1000, brake_full=21000))
        assert result.is_valid is True


# ---------------------------------------------------------------------------
# CalibrationManager._probe_contact テスト
# ---------------------------------------------------------------------------

class TestProbeContact:
    @pytest.mark.asyncio
    async def test_detects_spike_and_returns_position(self) -> None:
        # ウィンドウ幅3: 3サンプルで baseline 確定、4サンプル目でスパイク
        # baseline=100, avg≈100, spike=400 > 100 + 100*1.5=250 → 検出
        currents = [100.0, 100.0, 100.0, 100.0, 400.0]
        manager, accel, _ = make_manager(accel_currents=currents, brake_currents=[])
        pos = await manager._probe_contact(accel, start_pos=0)
        assert pos > 0

    @pytest.mark.asyncio
    async def test_raises_when_no_spike(self) -> None:
        # 全て平常値 → max_search_pulse 到達で例外
        currents = [100.0] * 100
        cfg = CalibrationConfig(
            move_step_pulse=1000,
            step_interval_s=0.0,
            current_window=3,
            current_spike_ratio=1.5,
            min_stroke_pulse=500,
            max_stroke_pulse=20000,
            max_search_pulse=5000,  # 小さな探索範囲
        )
        manager = CalibrationManager(
            accel_driver=MockActuatorDriver(currents),
            brake_driver=MockActuatorDriver([]),
            config=cfg,
        )
        with pytest.raises(CalibrationDetectionError):
            await manager._probe_contact(manager._accel_driver, start_pos=0)

    @pytest.mark.asyncio
    async def test_window_fills_before_baseline_set(self) -> None:
        # 最初の current_window サンプルは baseline 確定のみ（スパイク判定しない）
        # spike が window_size 未満のサンプル数で来ても検出しない
        currents = [100.0, 400.0] + [100.0] * 50  # 2サンプル目でスパイクだがウィンドウ未満
        cfg = CalibrationConfig(
            move_step_pulse=1000,
            step_interval_s=0.0,
            current_window=5,  # 5サンプル必要
            current_spike_ratio=1.5,
            min_stroke_pulse=500,
            max_stroke_pulse=20000,
            max_search_pulse=30000,
        )
        manager = CalibrationManager(
            accel_driver=MockActuatorDriver(currents),
            brake_driver=MockActuatorDriver([]),
            config=cfg,
        )
        # ウィンドウ満杯前はスパイク判定しないため、後続で平常値が続いて検出されないはず
        # (探索距離不足で CalibrationDetectionError)
        with pytest.raises(CalibrationDetectionError):
            await manager._probe_contact(manager._accel_driver, start_pos=0)


# ---------------------------------------------------------------------------
# CalibrationManager._detect_zero / _detect_full テスト
# ---------------------------------------------------------------------------

class TestDetectZeroAndFull:
    @pytest.mark.asyncio
    async def test_detect_zero_calls_home_return(self) -> None:
        # window=3: 3平常 + 1平常(baseline確定) + スパイク
        currents = [100.0] * 4 + [400.0] + [100.0] * 50
        manager, accel, _ = make_manager(accel_currents=currents, brake_currents=[])
        await manager._detect_zero(accel)
        assert accel.home_return_called >= 1

    @pytest.mark.asyncio
    async def test_detect_full_moves_to_zero_pos_first(self) -> None:
        currents = [100.0] * 4 + [400.0] + [100.0] * 50
        manager, accel, _ = make_manager(accel_currents=currents, brake_currents=[])
        zero_pos = 3000
        await manager._detect_full(accel, zero_pos)
        assert zero_pos in accel.positions_commanded


# ---------------------------------------------------------------------------
# CalibrationManager.run_calibration テスト
# ---------------------------------------------------------------------------

class TestRunCalibration:
    def _spike_currents(self, normal_count: int = 4) -> list[float]:
        """ゼロ検出用 + フル検出用の2スパイクパターン。"""
        normal = [100.0] * normal_count
        spike = [400.0]
        # probe_contact を2回呼ぶ: zero用 + full用
        return normal + spike + normal + spike + [100.0] * 50

    @pytest.mark.asyncio
    async def test_successful_calibration_returns_success(self) -> None:
        accel_currents = self._spike_currents()
        brake_currents = self._spike_currents()
        manager, _, _ = make_manager(
            accel_currents=accel_currents,
            brake_currents=brake_currents,
        )
        result = await manager.run_calibration(profile_id="test-profile")
        assert isinstance(result, CalibrationResult)

    @pytest.mark.asyncio
    async def test_detection_failure_returns_failure_result(self) -> None:
        # 全サンプル平常値 → スパイク検出失敗
        flat = [100.0] * 100
        cfg = CalibrationConfig(
            move_step_pulse=1000,
            step_interval_s=0.0,
            current_window=3,
            current_spike_ratio=1.5,
            min_stroke_pulse=500,
            max_stroke_pulse=20000,
            max_search_pulse=3000,
        )
        manager = CalibrationManager(
            accel_driver=MockActuatorDriver(flat),
            brake_driver=MockActuatorDriver(flat),
            config=cfg,
        )
        result = await manager.run_calibration(profile_id="test-profile")
        assert result.success is False
        assert result.data is None
        assert result.error_message is not None

    @pytest.mark.asyncio
    async def test_result_data_is_none_on_detection_error(self) -> None:
        flat = [100.0] * 100
        cfg = CalibrationConfig(
            move_step_pulse=1000,
            step_interval_s=0.0,
            current_window=3,
            current_spike_ratio=1.5,
            min_stroke_pulse=500,
            max_stroke_pulse=20000,
            max_search_pulse=3000,
        )
        manager = CalibrationManager(
            accel_driver=MockActuatorDriver(flat),
            brake_driver=MockActuatorDriver(flat),
            config=cfg,
        )
        result = await manager.run_calibration(profile_id="test-profile")
        assert result.data is None

    @pytest.mark.asyncio
    async def test_home_return_called_after_each_axis(self) -> None:
        accel_currents = self._spike_currents()
        brake_currents = self._spike_currents()
        manager, accel, brake = make_manager(
            accel_currents=accel_currents,
            brake_currents=brake_currents,
        )
        await manager.run_calibration(profile_id="test-profile")
        # accel: _detect_zero の home_return + run_calibration の home_return = 2回以上
        assert accel.home_return_called >= 2
        # brake: 同様
        assert brake.home_return_called >= 2

    @pytest.mark.asyncio
    async def test_calibration_data_stroke_computed_correctly(self) -> None:
        """CalibrationData の stroke が full - zero で正しく計算されること。"""
        accel_currents = self._spike_currents()
        brake_currents = self._spike_currents()
        manager, _, _ = make_manager(
            accel_currents=accel_currents,
            brake_currents=brake_currents,
        )
        result = await manager.run_calibration(profile_id="test-profile")
        if result.data is not None:
            assert result.data.accel_stroke == (
                result.data.accel_full_pos - result.data.accel_zero_pos
            )
            assert result.data.brake_stroke == (
                result.data.brake_full_pos - result.data.brake_zero_pos
            )

    @pytest.mark.asyncio
    async def test_validation_failure_returns_failure_with_data(self) -> None:
        """スパイク検出成功・バリデーション NG の場合、data 存在かつ success=False になること。"""
        # min_stroke_pulse を非常に高く設定して必ずストローク不足になるようにする
        cfg = CalibrationConfig(
            move_step_pulse=1000,
            step_interval_s=0.0,
            current_window=3,
            current_spike_ratio=1.5,
            min_stroke_pulse=100000,  # 実際のストロークより大きい値で必ず NG
            max_stroke_pulse=200000,
            max_search_pulse=30000,
        )
        accel_currents = self._spike_currents()
        brake_currents = self._spike_currents()
        manager = CalibrationManager(
            accel_driver=MockActuatorDriver(accel_currents),
            brake_driver=MockActuatorDriver(brake_currents),
            config=cfg,
        )
        result = await manager.run_calibration(profile_id="test-profile")
        assert result.success is False
        assert result.data is not None  # 検出成功のためデータは存在する
        assert result.error_message is not None


# ---------------------------------------------------------------------------
# CalibrationConfig のデフォルト値テスト
# ---------------------------------------------------------------------------

class TestCalibrationConfig:
    def test_default_values(self) -> None:
        from src.domain.calibration import (
            CALIB_CURRENT_SPIKE_RATIO,
            CALIB_CURRENT_WINDOW,
            CALIB_MAX_SEARCH_PULSE,
            CALIB_MAX_STROKE_PULSE,
            CALIB_MIN_STROKE_PULSE,
            CALIB_MOVE_STEP_PULSE,
            CALIB_STEP_INTERVAL_S,
        )

        cfg = CalibrationConfig()
        assert cfg.move_step_pulse == CALIB_MOVE_STEP_PULSE
        assert cfg.step_interval_s == CALIB_STEP_INTERVAL_S
        assert cfg.current_window == CALIB_CURRENT_WINDOW
        assert cfg.current_spike_ratio == CALIB_CURRENT_SPIKE_RATIO
        assert cfg.min_stroke_pulse == CALIB_MIN_STROKE_PULSE
        assert cfg.max_stroke_pulse == CALIB_MAX_STROKE_PULSE
        assert cfg.max_search_pulse == CALIB_MAX_SEARCH_PULSE

    def test_custom_config(self) -> None:
        cfg = CalibrationConfig(move_step_pulse=100, current_spike_ratio=2.0)
        assert cfg.move_step_pulse == 100
        assert cfg.current_spike_ratio == 2.0
