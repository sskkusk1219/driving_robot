# 開発ガイドライン (Development Guidelines)

## コーディング規約

### 命名規則（Python）

#### 変数・関数

```python
# ✅ 良い例
actual_speed_kmh = can_reader.read_speed()
async def run_calibration(profile_id: str) -> CalibrationResult: ...

# ❌ 悪い例
spd = r.get()
async def cal(pid: str): ...
```

**原則**:
- 変数・関数: `snake_case`、意味のある名詞・動詞句
- 定数: `UPPER_SNAKE_CASE`
- クラス: `PascalCase`
- Boolean: `is_`, `has_`, `can_` で始める
- async関数: 通常と同じ `snake_case`（`async def` で明示されるため）

#### クラス・データモデル

```python
# クラス: PascalCase
class RobotController: ...
class ActuatorDriver: ...

# データクラス: PascalCase
@dataclass
class VehicleProfile: ...
@dataclass
class CalibrationData: ...

# Pydanticモデル（API用）: PascalCase
class DriveStartRequest(BaseModel): ...
class SystemStatusResponse(BaseModel): ...
```

#### 定数

```python
# UPPER_SNAKE_CASE
CONTROL_LOOP_INTERVAL_MS = 10
LOG_INTERVAL_MS = 100
OVERCURRENT_LIMIT_MA = 3000
BATTERY_WARNING_PCT = 20
ARCHIVE_STORAGE_LIMIT_PCT = 80
```

---

### コードフォーマット

- **フォーマッタ**: `ruff format`（Black互換）
- **インデント**: 4スペース
- **行の長さ**: 最大100文字
- **型注釈**: 必須（関数の引数・戻り値すべて）

```python
# ✅ 良い例: 型注釈あり
async def move_to_position(self, pos: int) -> None:
    await self._client.write_register(POS_REGISTER, pos, slave=self._slave_id)

# ❌ 悪い例: 型注釈なし
async def move_to_position(self, pos):
    await self._client.write_register(POS_REGISTER, pos, slave=self._slave_id)
```

---

### コメント規約

コメントは「なぜそうするか」を書く。コードを読めばわかる「何をするか」は書かない。

```python
# ✅ 良い例: 理由を説明
# P-CON-CB はサーボOFF状態での位置指令を無視するため、必ずサーボON後に送信する
await self.servo_on()
await self.move_to_position(target_pos)

# ✅ 良い例: 複雑なアルゴリズムの説明
# 電流移動平均に対して1.5倍の閾値を使用（単純絶対値閾値だと
# アクチュエータごとの個体差で誤検知が起きるため）
threshold = baseline_current * 1.5

# ❌ 悪い例: コードを繰り返すだけ
# 電流値を読み取る
current = await self.read_current()
```

---

### エラーハンドリング

#### カスタム例外クラス

```python
class RobotError(Exception):
    """ロボットシステムの基底例外"""

class ActuatorCommunicationError(RobotError):
    def __init__(self, axis: str, message: str):
        super().__init__(f"[{axis}] Modbus通信エラー: {message}")
        self.axis = axis

class CalibrationError(RobotError):
    def __init__(self, step: str, reason: str):
        super().__init__(f"キャリブレーション失敗 [{step}]: {reason}")
        self.step = step

class SafetyError(RobotError):
    """安全監視による停止"""

class PreCheckError(RobotError):
    def __init__(self, failed_checks: list[str]):
        super().__init__(f"走行前チェック失敗: {', '.join(failed_checks)}")
        self.failed_checks = failed_checks
```

#### エラーハンドリングパターン

```python
# ✅ 良い例: 予期されるエラーと予期しないエラーを区別
async def start_auto_drive(self, mode_id: str) -> DriveSession:
    try:
        await self._run_pre_checks()
        return await self._execute_drive(mode_id)
    except PreCheckError as e:
        # 予期されるエラー: GUIに具体的な失敗項目を返す
        logger.warning("走行前チェック失敗: %s", e.failed_checks)
        raise
    except ActuatorCommunicationError as e:
        # ハードウェアエラー: 緊急停止してから再送出
        await self.emergency_stop()
        raise
    except Exception as e:
        # 予期しないエラー: ログしてから上位に伝播
        logger.exception("予期しないエラー: %s", e)
        raise

# ❌ 悪い例: エラーを握りつぶす
async def start_auto_drive(self, mode_id: str) -> DriveSession | None:
    try:
        return await self._execute_drive(mode_id)
    except Exception:
        return None  # エラー情報が失われる
```

---

### 非同期処理（asyncio）

#### 並列実行

```python
# ✅ 良い例: asyncio.gather で両軸に同時送信（10ms制御ループ）
await asyncio.gather(
    self._accel_driver.move_to_position(accel_pos),
    self._brake_driver.move_to_position(brake_pos),
)

# ❌ 悪い例: 逐次送信（時間がかかりすぎる）
await self._accel_driver.move_to_position(accel_pos)
await self._brake_driver.move_to_position(brake_pos)
```

#### 制御ループの実装

```python
# ✅ 正しい10ms制御ループの実装
async def _control_loop(self) -> None:
    while self._running:
        loop_start = asyncio.get_event_loop().time()

        await self._execute_one_cycle()

        elapsed = asyncio.get_event_loop().time() - loop_start
        sleep_time = max(0.0, CONTROL_LOOP_INTERVAL_MS / 1000 - elapsed)
        await asyncio.sleep(sleep_time)
```

---

### 安全に関わるコードの原則

制御系・安全系のコードには以下のルールを必ず適用します。

1. **フェイルセーフ**: エラー時は必ず安全側（原点復帰）に動く

```python
# エラーが発生しても必ず原点復帰を実行
async def run_drive(self) -> None:
    try:
        await self._drive_loop()
    finally:
        await self.safe_stop()  # 例外の有無にかかわらず実行
```

2. **開度のクランプ**: 計算結果を必ず[0, max_opening]にクリップ

```python
accel_cmd = max(0.0, min(self._profile.max_accel_opening, accel_raw))
brake_cmd = max(0.0, min(self._profile.max_brake_opening, brake_raw))
```

3. **マジックナンバー禁止**: 閾値・タイムアウトはすべて定数または設定値

```python
# ✅ 良い例
OVERCURRENT_LIMIT_MA = 3000
if current_ma > OVERCURRENT_LIMIT_MA:
    raise SafetyError("過電流検知")

# ❌ 悪い例
if current_ma > 3000:  # 何の値か不明
    raise SafetyError("過電流検知")
```

---

## Git運用ルール

### ブランチ戦略

```
main                        # 動作確認済みの安定版
  └─ feature/[機能名]       # 新機能開発
  └─ fix/[修正内容]         # バグ修正
  └─ docs/[ドキュメント名]  # ドキュメント更新のみ
```

**方針**:
- `main` ブランチへの直接プッシュ禁止（PRを経由）
- 1ブランチ = 1機能（ステアリングファイルのタスク単位が目安）
- ブランチ名は kebab-case: `feature/actuator-driver`、`fix/calibration-overcurrent`

### コミットメッセージ規約

**フォーマット**:
```
<type>(<scope>): <subject>

<body>（任意）
```

**Type**:
- `feat`: 新機能
- `fix`: バグ修正
- `docs`: ドキュメントのみの変更
- `refactor`: リファクタリング（機能変更なし）
- `test`: テスト追加・修正
- `chore`: 依存関係更新、設定変更

**Scope**（このプロジェクト固有）:
- `actuator`: アクチュエータドライバ
- `calibration`: キャリブレーション
- `control`: 制御アルゴリズム（FF・PID）
- `safety`: 安全監視
- `can`: CAN車速受信
- `api`: FastAPI・WebSocket
- `log`: ログ・アーカイブ
- `profile`: 車両プロファイル管理
- `ui`: フロントエンド

**例**:
```
feat(calibration): 電流急増検出によるゼロフル自動キャリブレーションを実装

- 移動平均（50ms窓）+ baseline×1.5の閾値で接触点・フル位置を検出
- アクセル・ブレーキを独立してキャリブレーション
- バリデーション（接触点<フル・ストローク妥当性）を実装
```

```
fix(control): 10ms制御ループのジッタ低減

asyncio.sleep の代わりに call_later を使用することで
ループ周期のばらつきを±2ms以内に抑制
```

---

### プルリクエストプロセス

**作成前のチェックリスト**:
- [ ] ユニットテストがパスする（`pytest tests/unit/`）
- [ ] 型チェックがパスする（`mypy src/`）
- [ ] Lintエラーがない（`ruff check src/ tests/`）
- [ ] 関連するステアリングファイルの tasklist.md を完了状態に更新した

**PRテンプレート**:
```markdown
## 概要
[変更内容の簡潔な説明]

## 変更理由
[なぜこの変更が必要か / 対応するステアリングファイル]

## 変更内容
- [変更点1]
- [変更点2]

## テスト
- [ ] ユニットテスト追加・パス確認
- [ ] 統合テスト（該当する場合）
- [ ] ハードウェア結合テスト（該当する場合、手動で確認）

## 安全確認（制御・安全系コードの場合）
- [ ] フェイルセーフ動作を確認
- [ ] 開度クランプを確認
- [ ] 非常停止動作を確認

## 関連
[ステアリングファイルパス / Issue番号]
```

---

## テスト戦略

### テストピラミッド

```
         /\
        /HW\       ハードウェア結合テスト（手動・実機必要）
       /----\
      / 統合  \     統合テスト（ローカルPostgreSQL使用）
     /--------\
    /  ユニット  \   ユニットテスト（モックHW、高速・CI対象）
   /____________\
```

### ユニットテスト

**対象**: `src/domain/`・`src/infra/`の単一クラス  
**方針**: ハードウェアはすべてモック化

```python
# pytest + pytest-asyncio の例
import pytest
from unittest.mock import AsyncMock, MagicMock
from src.domain.control.pid import PIDController

@pytest.mark.asyncio
async def test_pid_proportional_output():
    """比例制御が正しく計算されること"""
    # Given
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.01)
    # When
    output = pid.update(setpoint=60.0, measurement=58.0)
    # Then
    assert output == pytest.approx(2.0, abs=0.01)

@pytest.mark.asyncio
async def test_calibration_detects_zero_on_current_spike():
    """電流急増でゼロ位置を検出できること"""
    # Given
    mock_driver = AsyncMock()
    mock_driver.read_current.side_effect = [100, 105, 110, 200]  # 最後でスパイク
    # When / Then ...
```

**テスト命名**: `test_[対象]_[条件]_[期待結果]`
```python
def test_pid_integral_reset_on_direction_change(): ...
def test_calibration_raises_error_when_stroke_too_short(): ...
def test_safety_monitor_triggers_on_overcurrent(): ...
```

### 統合テスト

**対象**: 複数コンポーネントの連携（モックHW、実DB）

```python
# テスト用DBを使用
@pytest.fixture
async def db_conn():
    conn = await asyncpg.connect(dsn=TEST_DATABASE_URL)
    yield conn
    await conn.execute("TRUNCATE drive_sessions, drive_logs CASCADE")
    await conn.close()

async def test_log_writer_writes_session_and_logs(db_conn):
    """LogWriterがセッションとログをDBに正しく書き込むこと"""
    writer = LogWriter(db_conn)
    session_id = await writer.start_session(profile_id="test-profile", ...)
    await writer.write_log(session_id, sample_log_data)
    await writer.end_session(session_id, status="completed")

    row = await db_conn.fetchrow("SELECT * FROM drive_sessions WHERE id=$1", session_id)
    assert row["status"] == "completed"
```

### ハードウェア結合テスト

`tests/hardware/` 以下に配置。実機環境でのみ手動実行。

```python
# 実行例:
# pytest tests/hardware/test_actuator_modbus.py -v -s
# ※ 実機接続必須、自動CI対象外

@pytest.mark.hardware
async def test_actuator_moves_to_position():
    """アクチュエータが指定位置に移動すること（実機必要）"""
    driver = ActuatorDriver(port="/dev/ttyUSB0", slave_id=1)
    await driver.connect()
    await driver.servo_on()
    await driver.move_to_position(100)
    await asyncio.sleep(1.0)
    pos = await driver.read_position()
    assert abs(pos - 100) < 5  # ±5pulse以内
```

### CIで実行するテスト

```bash
# CI（GitHub Actions等）で自動実行
pytest tests/unit/ tests/integration/ --ignore=tests/hardware/

# ローカル・実機での手動実行のみ
pytest tests/hardware/ -v -s
```

---

## コードレビュー基準

### レビューポイント

**制御・安全系（最重要）**:
- [ ] 非常停止パスにブロッキング処理が入っていないか
- [ ] フェイルセーフ（例外発生時に原点復帰する）が実装されているか
- [ ] 開度が[0, max_opening]にクランプされているか
- [ ] asyncio.gather で両軸が並列送信されているか

**機能性**:
- [ ] PRDの受け入れ条件を満たしているか
- [ ] エッジケース（接続断・タイムアウト・空データ）が考慮されているか
- [ ] エラーハンドリングが適切か（握りつぶしていないか）

**可読性**:
- [ ] 命名が明確か（`s` や `d` のような略語がないか）
- [ ] 型注釈が正しく記載されているか
- [ ] 「なぜ」のコメントが書かれているか（ハードウェア固有の制約は特に重要）

**パフォーマンス**:
- [ ] 10ms制御ループ内でブロッキング処理がないか
- [ ] 100ms以内に完了しないDB処理がないか

### レビューコメントの書き方

```markdown
# ✅ 建設的なフィードバック
[必須] ここでの例外はキャッチされずに制御ループが停止します。
原点復帰処理を finally ブロックに移動してください。

[推奨] この閾値 1.5 はマジックナンバーです。
CURRENT_SPIKE_RATIO のような定数に抽出するのはどうでしょうか？

[質問] ここで asyncio.sleep(0) を呼んでいるのはなぜですか？
意図的なイールドポイントであればコメントがあると助かります。
```

**優先度の明示**:
- `[必須]`: 安全・機能に関わる問題、マージ前に修正必要
- `[推奨]`: 品質向上、対応することを強く推奨
- `[提案]`: 将来の改善案、対応は任意
- `[質問]`: 理解のための確認

---

## 開発環境セットアップ

### 必要なツール

| ツール | バージョン | インストール方法 |
|--------|-----------|-----------------|
| Python | 3.13 | `sudo apt install python3.13` (Raspberry Pi OS) |
| PostgreSQL | 15 | `sudo apt install postgresql-15` |
| Kvaser Linux ドライバ | 最新 | Kvaser公式サイトから |

### セットアップ手順

```bash
# 1. リポジトリのクローン
git clone <repository-url>
cd driving_robot

# 2. 仮想環境の作成・有効化
python3.13 -m venv .venv
source .venv/bin/activate

# 3. 依存関係のインストール
pip install -r requirements.lock

# 4. 設定ファイルのコピー
cp config/settings.toml.example config/settings.toml
# config/settings.toml を環境に合わせて編集

# 5. データベース初期化
python scripts/setup_db.py

# 6. 動作確認（ユニット・統合テスト）
pytest tests/unit/ tests/integration/ -v

# 7. システム起動
bash scripts/start.sh
```

### 推奨開発ツール

- **エディタ**: VS Code + Python拡張 + Pylance（型チェック有効化）
- **型チェック**: `mypy src/`（コミット前に実行）
- **フォーマット**: `ruff format src/ tests/`（自動整形）
- **Lint**: `ruff check src/ tests/`（問題確認）

---

## 実装完了チェックリスト

実装・PRレビュー前に確認:

### コード品質
- [ ] 命名が明確で型注釈が正しい
- [ ] 関数が単一の責務を持っている（300行以下）
- [ ] マジックナンバーがない（定数か設定値を使用）
- [ ] エラーハンドリングが実装されている（握りつぶしなし）

### 制御・安全（制御系コードの場合）
- [ ] フェイルセーフ（例外時の原点復帰）が `finally` で実装されている
- [ ] 開度がクランプされている
- [ ] 10ms制御ループ内にブロッキング処理がない
- [ ] 非常停止ハンドラが優先的に動作する

### テスト
- [ ] ユニットテストが書かれている（ドメインレイヤー）
- [ ] `pytest tests/unit/ tests/integration/` がパスする
- [ ] エッジケース（通信断・タイムアウト）がテストされている

### 品質ツール
- [ ] `ruff check src/ tests/` エラーなし
- [ ] `mypy src/` エラーなし
- [ ] `ruff format src/ tests/` 実行済み
