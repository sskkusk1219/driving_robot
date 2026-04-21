# CalibrationManager 実装要求

## 機能概要
アクセル・ブレーキの独立したゼロフルキャリブレーションを実行するドメインクラスを実装する。

## 要求内容
- `src/domain/calibration.py` に `CalibrationManager` を実装
- アクチュエータをゆっくり正方向に移動し、電流急増で接触点(zero)・フル位置(full)を検出
- 検出結果をバリデーションし、`CalibrationResult` を返す
- `src/models/calibration.py` に `ValidationResult` dataclass を追加
- `tests/unit/test_calibration.py` にユニットテストを実装

## 参照ドキュメント
- `docs/functional-design.md` § CalibrationManager コンポーネント設計
- `docs/functional-design.md` § UC2 キャリブレーション
- `docs/functional-design.md` § CalibrationData モデル定義
- `docs/architecture.md` § ドメインレイヤー

## 制約
- ハードウェア依存部は Protocol で抽象化（テスト可能にする）
- 既存の `src/models/calibration.py` を壊さない
