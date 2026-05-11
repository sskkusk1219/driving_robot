# 設計書

## アーキテクチャ概要

既存の `CANReader` インフラクラスを再利用する独立スクリプトとして実装する。
アプリケーション全体（FastAPI・DB・アクチュエータ）を起動せず、
`scripts/check_can.py` 単体で CAN 受信確認ができる。

```
scripts/check_can.py
  └─ src/infra/can_reader.py (CANReader)
       ├─ python-can 4.x (Kvaser backend)
       └─ cantools (DBC decode)
            └─ config/can/MEIDEN_MEIDACS.dbc
```

## コンポーネント設計

### 1. `scripts/check_can.py`

**責務**:
- settings.toml から CAN 設定（interface, channel, dbc_path）を読み込む
- CANReader を使って CAN バスに接続する
- ループで `read_speed()` を呼び出し、車速をターミナルに表示する
- TimeoutError は警告として表示してループを継続する
- Ctrl+C (KeyboardInterrupt) でクリーン終了する

**実装の要点**:
- `asyncio.run(main())` で asyncio エントリーポイントを持つ
- settings.toml が存在しない場合はデフォルト値（kvaser, ch=0, DBC パス）を使用する
- 表示フォーマット: `[HH:MM:SS.mmm] Speed: XXX.XX km/h` (受信成功時)
- 表示フォーマット: `[HH:MM:SS.mmm] TIMEOUT - フレームが受信できませんでした` (タイムアウト時)
- ValueError（不明フレームID等）もスキップして継続する

## データフロー

### 車速受信ループ
```
1. settings.toml 読み込み (CAN section)
2. CANReader.connect() → Kvaser 接続 + DBC ロード
3. ループ開始
   a. CANReader.read_speed() → CAN フレーム受信 → DBC デコード → float 返却
   b. タイムスタンプ付きで "Speed: XX.XX km/h" を print()
   c. TimeoutError → "TIMEOUT" を print() してループ継続
   d. ValueError → "不明フレームをスキップ" を print() してループ継続
4. KeyboardInterrupt → CANReader.close() → "終了" メッセージ表示
```

## エラーハンドリング戦略

| 例外 | 対処 |
|------|------|
| `FileNotFoundError` (DBC) | エラーメッセージ表示して終了 |
| `Exception` (connect失敗) | エラーメッセージ表示して終了 |
| `TimeoutError` (受信タイムアウト) | 警告表示してループ継続 |
| `ValueError` (不明フレームID) | 警告表示してループ継続 |
| `KeyboardInterrupt` | クリーン終了 |

## テスト戦略

### ハードウェアテスト（手動実行）
- `tests/hardware/test_can_receive.py` に pytest 形式のハードウェアテストを追加
- 実際の Kvaser デバイスとシャシダイナモ接続時のみ実行

### ユニットテストは対象外
- スクリプト自体のユニットテストは作成しない（CANReader のユニットテストは既存）

## 依存ライブラリ

追加なし（python-can・cantools・tomllib は既存）

## ディレクトリ構造

```
scripts/
  check_can.py          ← 新規追加
tests/hardware/
  test_can_receive.py   ← 新規追加
```

## 実装の順序

1. `scripts/check_can.py` の実装
2. `tests/hardware/test_can_receive.py` の実装
3. 動作確認（実機または説明書きで確認方法を記載）

## パフォーマンス考慮事項

- `read_speed()` はブロッキング recv(timeout=0.1s) を run_in_executor 経由で呼ぶ
- ループに sleep は不要（recv がタイムアウト待ちを兼ねる）
