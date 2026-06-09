---
name: production-apply-design
description: 本番環境適用修正の実装設計
metadata:
  type: project
---

# 設計: 本番環境への適用

## 修正方針

各修正は独立しており、依存関係なし。影響範囲が明確で小さい修正のみ実施する。

## 修正1: config/settings.toml 更新

settings.toml.example の内容に合わせて settings.toml を更新する。

追加内容:
- `[can]` に `bitrate = 500000`（コメント付き）
- `[ups]` セクション（nut_host/nut_port/ups_name/poll_interval_s）
- `[safety]` セクション（overcurrent_limit_ma = 3000）
- `[gpio]` コメントを NUT 方式確定後の記述に更新

## 修正2: factory.py の CANReader bitrate 追加

変更箇所: `src/app/factory.py` の `build_real_controller()` 内

```python
# Before
can_reader = CANReader(
    interface=settings.can.interface,
    channel=settings.can.channel,
    dbc_path=settings.can.dbc_path,
)

# After
can_reader = CANReader(
    interface=settings.can.interface,
    channel=settings.can.channel,
    bitrate=settings.can.bitrate,
    dbc_path=settings.can.dbc_path,
)
```

## 修正3: test_calibration.py docstring 修正

変更箇所: モジュールdocstring の「操作方法（ジョグモード）」部分

```python
# Before
操作方法（ジョグモード）:
    +/-  : ±0.5mm 移動
    ./,  : ±0.1mm 移動

# After
操作方法（ジョグモード）:
    e/w  : ±1.0mm 移動
    d/s  : ±0.5mm 移動
```

※ ステップ量は _JOG_STEP_LARGE=100pulse=1.0mm / _JOG_STEP_SMALL=50pulse=0.5mm に基づく
