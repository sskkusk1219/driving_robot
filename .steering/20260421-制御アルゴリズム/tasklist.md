# タスクリスト: 制御アルゴリズム実装

## フェーズ1: パッケージ初期化

- [x] src/domain/__init__.py と src/domain/control/__init__.py を作成

## フェーズ2: PIDController 実装

- [x] src/domain/control/pid.py を実装

## フェーズ3: FeedforwardController 実装

- [x] src/domain/control/feedforward.py を実装

## フェーズ4: テスト実装

- [x] tests/unit/test_pid.py を実装
- [x] tests/unit/test_feedforward.py を実装

## フェーズ5: 静的解析・テスト確認

- [x] ruff check でエラーなし
- [x] mypy でエラーなし
- [x] pytest tests/unit/ で全テストパス（39/39）

## 申し送り事項

**実装完了日**: 2026-04-21

**実装ファイル**:
- `src/domain/__init__.py`, `src/domain/control/__init__.py` (新規)
- `src/domain/control/pid.py` — PIDController（離散時間後退差分）
- `src/domain/control/feedforward.py` — FeedforwardController（scipy RegularGridInterpolator）
- `tests/unit/test_pid.py` — 13テスト（P/I/D各項・リセット・積分ワインドアップ）
- `tests/unit/test_feedforward.py` — 10テスト（ロード・補間・クランプ・不正キー）

**計画と実績の差分**:
- implementation-validator 指摘対応で以下を追加実施:
  - `feedforward.py`: 不正 pkl キーを `ValueError` に変換するバリデーション追加
  - `feedforward.py`: `pickle.load` の信頼境界コメント追記
  - `pid.py`: クラス変数アノテーション追加
  - テスト: 不正 pkl キーテスト・積分ワインドアップ境界テスト追加

**次フェーズへの引き継ぎ事項**:
- `PIDController` はアンチワインドアップ未実装。RobotController 側で最終開度をクランプすることで対処している（functional-design.md の設計意図通り）
- `FeedforwardController.predict()` は 0〜100% にクランプ済みの開度を返す。排他制御（両開度同時）は呼び出し元（RobotController）で実施
- 次の実装候補: RobotController 状態機械 または SafetyMonitor
