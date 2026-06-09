---
name: production-apply-requirements
description: 本番環境への適用に向けたテストコードとの差分修正要件
metadata:
  type: project
---

# 要件: 本番環境への適用

## 背景

以下の5つのハードウェアテストを基準として、本番コードに不足している部分を修正する。

- `tests/hardware/test_emergency_stop_home_return.py` - 非常停止スイッチ → 原点復帰
- `tests/hardware/test_overcurrent_home_return.py` - 過電流 → 原点復帰
- `tests/hardware/test_calibration.py` - キャリブレーション（手動ジョグ）
- `scripts/check_can.py` - CAN受信確認
- `tests/hardware/test_ups_monitor.py` - UPS監視

## 発見した差分

### 差分1: config/settings.toml が旧状態

`settings.toml.example` と `settings.toml` を比較すると、以下が欠落:
- `[can]` セクションに `bitrate = 500000` がない
- `[ups]` セクション（NUT設定）が丸ごとない
- `[safety]` セクション（過電流閾値）が丸ごとない
- `[gpio]` のコメントが TBD のまま（NUT方式確定後に更新されていない）

### 差分2: factory.py が CANReader に bitrate を渡していない

`check_can.py`:
```python
reader = CANReader(interface=can.interface, channel=can.channel, bitrate=can.bitrate, dbc_path=...)
```

`factory.py`:
```python
can_reader = CANReader(interface=settings.can.interface, channel=settings.can.channel, dbc_path=...)  # bitrate なし！
```

settings の bitrate が 500000 以外に変更されても反映されない。

### 差分3: test_calibration.py モジュールdocstringのキー表記が誤り

module docstring:
```
操作方法（ジョグモード）:
    +/-  : ±0.5mm 移動
    ./,  : ±0.1mm 移動
```

実装（_jog_axis内）:
```python
elif key == 'e':  # 大ステップ +
elif key == 'w':  # 大ステップ -
elif key == 'd':  # 小ステップ +
elif key == 's':  # 小ステップ -
```

ランタイムで表示される操作ガイドは正しいが、モジュールdocstringが古い記述のまま。

## 修正スコープ

1. `config/settings.toml` を `settings.toml.example` に合わせて更新
2. `src/app/factory.py` で `CANReader` に `bitrate` を渡す
3. `tests/hardware/test_calibration.py` モジュールdocstringの操作キー表記を修正
