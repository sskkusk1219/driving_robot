# タスクリスト: SafetyMonitor 実装

## フェーズ1: 実装

- [x] src/domain/safety_monitor.py を実装

## フェーズ2: テスト

- [x] tests/unit/test_safety_monitor.py を実装

## フェーズ3: 静的解析・テスト確認

- [x] ruff check でエラーなし
- [x] mypy でエラーなし
- [x] pytest tests/unit/ で全テストパス（59/59）

## 申し送り事項

**実装完了日**: 2026-04-21

**実装ファイル**:
- `src/domain/safety_monitor.py` — SafetyMonitor（過電流・逸脱判定・コールバック管理・AC電源断処理）
- `tests/unit/test_safety_monitor.py` — 24テスト全パス

**計画と実績の差分**:
- implementation-validator 指摘対応で以下を追加実施:
  - `trigger_emergency()`: コールバック例外伝播をフェイルセーフ対応（try/except + ExceptionGroup）
  - `is_monitoring` プロパティを追加（テストがプライベートフィールドに直接アクセス不要）
  - `check_overcurrent` の `axis` 引数に「将来予約済み」コメントを追記
  - `check_deviation` に境界条件（`>` vs `>=`）の設計意図コメントを追記
  - テスト: コールバック例外時の後続継続・ExceptionGroup テスト追加

**設計上の重要な決定事項**:
- `trigger_emergency()` はパブリックメソッドとして公開（GPIOMonitor infra から呼ぶための API）
  → functional-design.md には未記載だが、GPIOMonitor との連携に必要なため追加
- コールバック例外は ExceptionGroup にまとめて送出（全コールバック実行を保証するフェイルセーフ）

**次フェーズへの引き継ぎ事項**:
- `trigger_emergency()` を functional-design.md の SafetyMonitor インターフェース仕様に追記することを推奨
- 次の実装候補: RobotController状態機械 または LogWriter
