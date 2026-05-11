# 設計: Kvaser CANReader 修正・テスト

## 変更ファイル一覧

| ファイル | 変更種別 | 内容 |
|---------|---------|------|
| `pyproject.toml` | 修正 | `cantools>=0.18.0` を dependencies に追加 |
| `src/infra/can_reader.py` | 修正 | `_SPEED_SIGNAL_NAME = "Speed"` に変更 |
| `src/infra/settings.py` | 修正 | `CanSettings.dbc_path: str` フィールド追加 |
| `config/settings.toml.example` | 修正 | `dbc_path` キー追記 |
| `src/app/factory.py` | 修正 | `CANReader(dbc_path=settings.can.dbc_path)` |
| `tests/unit/infra/test_can_reader.py` | 修正 | シグナル名を `"Speed"` に更新 + DBC 統合テスト追加 |
| `config/settings.toml` | 新規作成 | `settings.toml.example` から生成 |

## 技術的アプローチ

### シグナル名修正
- `_SPEED_SIGNAL_NAME = "Speed"` に変更するだけ。DBC の `SG_ Speed` と一致させる。

### DBC パス設定経路
```
settings.toml → CanSettings.dbc_path → factory.py → CANReader(dbc_path=...)
```

### テスト追加: DBC 統合テスト
`cantools` をモック化せず、実際の `config/can/MEIDEN_MEIDACS.dbc` をロードして
フレームのデコードが正しく動作することを確認する。
- 実際の DBC から `Speed` シグナルが取得できること
- 生バイトデータ → `decode_message` → `"Speed"` キー確認
- `CANReader.read_speed()` の統合パス（`can.Bus` のみモック化）

### CanSettings の dbc_path
```python
@dataclass
class CanSettings:
    interface: str = "kvaser"
    channel: int = 0
    dbc_path: str = "config/can/MEIDEN_MEIDACS.dbc"
```

デフォルト値にすることで既存コードの変更を最小限に抑える。
