"""スタブ構成: アクチュエータが指令位置を覚え、キャリブレーションを保存できること。"""

from src.app.stubs import InMemoryProfileRepository, _StubActuator, build_stub_controller
from src.models.system_state import RobotState


async def test_stub_actuator_remembers_position() -> None:
    act = _StubActuator()
    assert await act.read_position() == 0
    await act.move_to_position(1234)
    assert await act.read_position() == 1234
    await act.move_to_position_timed(2000, 1234, 0.5)
    assert await act.read_position() == 2000
    await act.home_return()
    assert await act.read_position() == 0


async def test_stub_calibration_manual_save() -> None:
    ctrl = build_stub_controller(profile_repo=InMemoryProfileRepository())
    await ctrl.start()
    await ctrl.initialize()
    for axis in ("accel", "brake"):
        await ctrl.jog_axis(axis, 500)
        await ctrl.calib_set_zero(axis)
        await ctrl.jog_axis(axis, 3000)
        await ctrl.calib_set_full(axis)
    result = await ctrl.save_manual_calibration()
    assert result.success
    assert ctrl.get_system_state().robot_state == RobotState.READY
