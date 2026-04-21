# 設計: RobotController 実機統合

## RobotController 変更方針

### `__init__` のインターフェース変更

現在の `ActuatorDriverProtocol` / `CANReaderProtocol` に加え、`read_position`・`read_current`・`read_speed` の呼び出しが必要。Protocol を拡張する。

```python
class ActuatorDriverProtocol(Protocol):
    async def home_return(self) -> None: ...
    async def servo_off(self) -> None: ...
    async def servo_on(self) -> None: ...
    async def reset_alarm(self) -> None: ...
    async def is_alarm_active(self) -> bool: ...
    async def read_position(self) -> int: ...
    async def read_current(self) -> float: ...
    async def connect(self) -> None: ...

class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...
    async def connect(self) -> None: ...
```

### `start()` メソッド

```python
async def start(self) -> None:
    try:
        await asyncio.gather(
            self._accel_driver.connect(),
            self._brake_driver.connect(),
            self._can_reader.connect(),
        )
        await self._safety_monitor.start_monitoring()
        self._transition(RobotState.STANDBY)
    except Exception:
        self._transition(RobotState.ERROR)
        raise
```

### `initialize()` メソッド

```python
async def initialize(self) -> None:
    self._transition(RobotState.INITIALIZING)
    await asyncio.gather(
        self._accel_driver.reset_alarm(),
        self._brake_driver.reset_alarm(),
    )
    await asyncio.gather(
        self._accel_driver.servo_on(),
        self._brake_driver.servo_on(),
    )
    if not self._last_normal_shutdown:
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
    self._transition(RobotState.READY)
```

### `get_realtime_data()` メソッド

```python
@dataclass
class RealtimeSnapshot:
    actual_speed_kmh: float
    accel_pos: int
    brake_pos: int
    accel_current_ma: float
    brake_current_ma: float

async def get_realtime_data(self) -> RealtimeSnapshot:
    speed, accel_pos, brake_pos, accel_cur, brake_cur = await asyncio.gather(
        self._can_reader.read_speed(),
        self._accel_driver.read_position(),
        self._brake_driver.read_position(),
        self._accel_driver.read_current(),
        self._brake_driver.read_current(),
    )
    return RealtimeSnapshot(
        actual_speed_kmh=speed,
        accel_pos=accel_pos,
        brake_pos=brake_pos,
        accel_current_ma=accel_cur,
        brake_current_ma=brake_cur,
    )
```

`RealtimeSnapshot` の置き場: `src/models/system_state.py` または `src/app/robot_controller.py` 内に定義（外部依存なし）。

### WebSocket broadcast_loop

```python
async def broadcast_loop(app: Starlette) -> None:
    while True:
        await asyncio.sleep(WS_BROADCAST_INTERVAL_S)
        if not manager.has_connections:
            continue
        controller = app.state.controller
        state = controller.get_system_state()
        try:
            snapshot = await controller.get_realtime_data()
            accel_kmh = snapshot.actual_speed_kmh
            ...
        except Exception:
            accel_kmh = 0.0  # HW エラー時はフォールバック
        data = RealtimeData(...)
        await manager.broadcast(data.model_dump_json())
```

### `build_real_controller()` ファクトリ

```python
# src/app/factory.py
def build_real_controller(settings: AppSettings) -> RobotController:
    accel = ActuatorDriver(port=settings.serial.accel_port, slave_id=1, baud_rate=settings.serial.baud_rate)
    brake = ActuatorDriver(port=settings.serial.brake_port, slave_id=2, baud_rate=settings.serial.baud_rate)
    can = CANReader(interface=settings.can.interface, channel=settings.can.channel)
    gpio = GPIOMonitor(emergency_pin=settings.gpio.emergency_stop_pin, ac_detect_pin=settings.gpio.ac_detect_pin)
    # SafetyMonitor と GPIOMonitor の接続
    safety = SafetyMonitor(gpio_monitor=gpio)
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
    return RobotController(accel, brake, can, safety, pid)
```

### app.py の環境変数フラグ

`DRIVING_ROBOT_USE_REAL_HW=1` 環境変数が設定されている場合に `build_real_controller()` を使用、それ以外はスタブ。

## テスト方針

- `get_realtime_data()` のユニットテスト：モックドライバーで正常系・エラー系
- `start()` のユニットテスト：connect/start_monitoring の呼び出し確認
- `initialize()` のユニットテスト：reset_alarm/servo_on/home_return の呼び出し順序確認
- `build_real_controller()` のユニットテスト：設定値が正しく各ドライバーに渡るか確認
- broadcast_loop の WebSocket テスト：実データが流れるか確認
