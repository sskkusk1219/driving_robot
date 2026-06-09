# 設計: 非常停止の動きの修正

## 方針

「フロントが非常停止画面に遷移しない」根本は **状態配信が HW 読み取りに連結している**
こと。HW 読み取りを境界づけ、状態配信を止めないようにする。あわせて多重 GPIO 発火による
原点復帰の重複起動を防ぐ。フロントは既に `robot_state === 'EMERGENCY'` で
EmergencyResetScreen を出す実装があるため、状態さえ届けば修正不要。

## 変更 1: WebSocket 配信を HW 読み取りから分離 (`src/web/ws.py`)

`broadcast_loop` の `get_realtime_data()` 呼び出しを `asyncio.wait_for(...)` で
タイムアウト (0.5s) を付与する。タイムアウト/例外時は既存の except 節でフォールバック値
(0.0) を使い、**必ず `robot_state` を含むスナップショットを配信する**。

- 通常運転では 5 read が pymodbus 内部で直列化されても 0.5s を超えないため、
  正常時の挙動は変わらない。
- 非常停止で `home_return` がバスを占有しても、最大 0.5s で打ち切って状態を配信する。
- `TimeoutError` は Python 3.11+ で `Exception` のサブクラスのため、既存の
  `except Exception` がそのまま捕捉する。

## 変更 2: `emergency_stop()` の再入防止 (`src/app/robot_controller.py`)

多重 GPIO 発火で `emergency_stop()` が重複起動しても原点復帰を二重実行しないよう、
**既に `EMERGENCY` の場合は DriveLoop 停止のみ行って早期 return** する。

```python
async def emergency_stop(self) -> None:
    if self._drive_loop is not None:
        self._drive_loop.stop()
        self._drive_loop = None
    if self._state == RobotState.EMERGENCY:
        # 多重 GPIO エッジ等による再入。原点復帰・セッション終了の重複起動を防ぐ。
        return
    self._transition(RobotState.EMERGENCY)
    await asyncio.gather(home_return...)  # 以降は従来どおり
    ...
```

- 先行コルーチンは最初の `await` (gather) までに同期的に `_transition(EMERGENCY)` まで
  到達するため、後続コルーチンは確実に `EMERGENCY` を観測して早期 return できる。
- BOOTING 等からの呼び出しは `_state != EMERGENCY` のまま `_transition` に進み、
  従来どおり `InvalidStateTransition` を送出する (既存テスト維持)。

## 変更 3: テスト追加

- `tests/unit/test_robot_controller.py`: `emergency_stop()` 2 回連続呼び出しで
  `home_return` が両軸 1 回ずつ (重複なし)・`trigger_emergency` が 1 回であること。
- `tests/unit/test_ws_broadcast.py` (新規): `get_realtime_data` がハングしても
  `broadcast_loop` が `robot_state` を含むメッセージを配信することを検証。

## 非対象 (スコープ外)

- `ActuatorDriver` への明示ロック追加: pymodbus が内部でトランザクションを直列化するため、
  変更 1+2 で症状は解消する。過剰実装を避け今回は見送る。
- フロントエンドの変更: `robot_state === 'EMERGENCY'` の画面切替は実装済み。
