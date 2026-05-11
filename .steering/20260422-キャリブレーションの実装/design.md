# 設計書

## アーキテクチャ概要

既存のレイヤードアーキテクチャを踏襲し、`CalibrationManagerProtocol` を介してドメイン層と疎結合を保つ。

```
Web レイヤー
  POST /api/v1/drive/calibrate
    RobotController.run_calibration()   <- アプリケーション層
      CalibrationManagerProtocol.run_calibration()
        CalibrationManager             <- ドメイン層
          _detect_zero / _detect_full (accel, brake)
          _validate(data)
```

## コンポーネント設計

### 1. CalibrationManagerProtocol（robot_controller.py に追加）

```python
class CalibrationManagerProtocol(Protocol):
    async def run_calibration(self, profile_id: str) -> CalibrationResult: ...
```

### 2. RobotController（run_calibration 更新）

```python
async def run_calibration(self) -> CalibrationResult:
    self._transition(RobotState.CALIBRATING)
    try:
        if self._calibration_manager is not None:
            return await self._calibration_manager.run_calibration(
                profile_id=self._active_profile_id or ""
            )
        return CalibrationResult(success=False, data=None, error_message="キャリブレーション未設定")
    finally:
        self._transition(RobotState.READY)
```

### 3. Web API スキーマ

```python
class CalibrationDataResponse(BaseModel):
    accel_zero_pos: int
    accel_full_pos: int
    accel_stroke: int
    brake_zero_pos: int
    brake_full_pos: int
    brake_stroke: int
    calibrated_at: datetime
    is_valid: bool

class CalibrationResultResponse(BaseModel):
    success: bool
    error_message: str | None
    data: CalibrationDataResponse | None
```

### 4. factory.py 更新

```python
from src.domain.calibration import CalibrationManager

calib_manager = CalibrationManager(accel_driver=accel_driver, brake_driver=brake_driver)
return RobotController(..., calibration_manager=calib_manager)
```

## ディレクトリ構造（変更ファイル）

```
src/
  app/
    robot_controller.py   <- CalibrationManagerProtocol 追加、run_calibration 更新
    factory.py            <- CalibrationManager 生成・注入
  web/
    schemas.py            <- CalibrationResultResponse 追加
    routers/
      drive.py            <- POST /calibrate エンドポイント追加
tests/
  unit/
    test_robot_controller.py <- CalibrationManager 委譲テスト追加
    test_web_drive.py        <- /calibrate エンドポイントテスト追加
```

## 実装の順序

1. `robot_controller.py`: Protocol 定義 + `run_calibration` 更新 + `__init__` 更新
2. `schemas.py`: `CalibrationResultResponse` 追加
3. `drive.py`: `/calibrate` エンドポイント追加
4. `factory.py`: `CalibrationManager` 生成・注入
5. `test_robot_controller.py`: 委譲テスト追加
6. `test_web_drive.py`: エンドポイントテスト追加
7. テスト実行・品質チェック
