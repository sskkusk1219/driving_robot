# 設計書

## アーキテクチャ概要

ハードウェア結合テストスクリプト（手動実行）として `tests/hardware/` に配置する。
本番コード（GPIOMonitor）には一切変更を加えない。

```
tests/hardware/test_emergency_stop.py
  └── RPi.GPIO を直接使用
        ├── GPIO17 setup（PUD_UP、FALLING/RISING エッジ検知）
        ├── 現在状態の読み取り（GPIO.input）
        └── Ctrl+C シグナルで GPIO.cleanup 実行
```

## コンポーネント設計

### 1. test_emergency_stop.py

**責務**:
- GPIO17の初期状態表示
- FALLING / RISING 両エッジの割り込み検知と表示
- 安全なクリーンアップ（KeyboardInterrupt ハンドリング）

**実装の要点**:
- `GPIO.setmode(GPIO.BCM)` で BCM番号指定（GPIO17 = BCM17）
- `GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)` で内部プルアップ有効
- `GPIO.add_event_detect(17, GPIO.BOTH, ...)` で押下・解除の両エッジを検知
- bouncetime=50 でチャタリング除去
- `GPIO.input(17)` で現在状態を読み取り（HIGH=1=開放、LOW=0=押下）
- メインループは `time.sleep(0.1)` でCPU負荷を抑制

## データフロー

### スイッチ押下検知
```
1. スイッチ押下 → GPIO17 が HIGH → LOW（FALLING エッジ）
2. RPi.GPIO 割り込みハンドラが別スレッドで呼ばれる
3. ハンドラが GPIO.input で現在値を確認して押下/解除を判定
4. メッセージを標準出力に表示
```

## テスト戦略

### ハードウェア結合テスト（手動実行）
- スクリプト起動 → 初期状態表示を確認
- スイッチ押下 → 「押下検知」メッセージ確認
- スイッチ解除 → 「解除検知」メッセージ確認
- Ctrl+C → 「クリーンアップ完了」メッセージ確認

## ディレクトリ構造

```
tests/
└── hardware/              # 新規作成
    └── test_emergency_stop.py  # 新規作成
```

## 実装の順序

1. `tests/hardware/` ディレクトリ作成
2. `test_emergency_stop.py` 作成
3. 手動実行して動作確認
