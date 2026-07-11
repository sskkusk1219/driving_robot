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
CONTROL_LOOP_INTERVAL_MS = 50
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
# ✅ 良い例: asyncio.gather で両軸に同時送信（50ms制御ループ）
await asyncio.gather(
    self._accel_driver.move_to_position(accel_pos),
    self._brake_driver.move_to_position(brake_pos),
)

# ❌ 悪い例: 逐次送信（時間がかかりすぎる）
await self._accel_driver.move_to_position(accel_pos)
await self._brake_driver.move_to_position(brake_pos)
```

#### 制御ループの実装

`asyncio.sleep` はイベントループの処理時間が加算されてジッタが増大するため、
`loop.call_later` を使用してループ周期のばらつきを±5ms以内に抑制する（architecture.md準拠）。

```python
# ✅ 正しい50ms制御ループの実装（call_later使用）
def _start_control_loop(self) -> None:
    loop = asyncio.get_event_loop()
    loop.call_later(CONTROL_LOOP_INTERVAL_MS / 1000, self._schedule_next_cycle)

def _schedule_next_cycle(self) -> None:
    if not self._running:
        return
    asyncio.ensure_future(self._execute_one_cycle())
    loop = asyncio.get_event_loop()
    loop.call_later(CONTROL_LOOP_INTERVAL_MS / 1000, self._schedule_next_cycle)

# ❌ 悪い例: asyncio.sleep はイベントループ処理時間が加算されてジッタが増大
async def _control_loop(self) -> None:
    while self._running:
        loop_start = asyncio.get_event_loop().time()
        await self._execute_one_cycle()
        elapsed = asyncio.get_event_loop().time() - loop_start
        sleep_time = max(0.0, CONTROL_LOOP_INTERVAL_MS / 1000 - elapsed)
        await asyncio.sleep(sleep_time)  # ジッタが±5msを超える可能性あり
```

---

### ハードウェア固有の実装パターン

#### lgpio GPIO コールバック（Raspberry Pi OS Bookworm以降）

`lgpio` は `RPi.GPIO` の後継ライブラリ。コールバックのシグネチャが異なるため注意。

```python
import lgpio

# ✅ lgpio のコールバックシグネチャ: (chip, gpio, level, timestamp)
def _on_emergency_stop(chip: int, gpio: int, level: int, timestamp: int) -> None:
    ...

handle = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(handle, pin)
lgpio.callback(handle, pin, lgpio.RISING_EDGE, _on_emergency_stop)

# ❌ RPi.GPIO の旧シグネチャ（lgpio では使えない）
def _on_emergency_stop(channel):  # 'channel' キーワード引数は lgpio には存在しない
    ...
```

#### CAN フレームデコード（cantools）

受信フレームのデータ長が DBC 定義より短い場合（例: 4バイト受信 / DBC 定義8バイト）は
`DecodeError` が発生する。`allow_truncated=True` を使うことで先頭部分の有効シグナルをデコードできる。

```python
# ✅ allow_truncated=True で短いフレームも正常デコード
try:
    decoded = db.decode_message(msg.arbitration_id, msg.data, allow_truncated=True)
except KeyError:
    raise ValueError(f"不明な CAN フレーム ID: 0x{msg.arbitration_id:X}") from None
except Exception as e:
    raise ValueError(f"デコードエラー: ID=0x{msg.arbitration_id:X} ({e})") from None

# ❌ デフォルト（allow_truncated=False）では長さ不一致で DecodeError
decoded = db.decode_message(msg.arbitration_id, msg.data)  # 4バイト受信・8バイト定義で例外
```

#### PCA9685 / I2C ボタンサーボ（Post-MVP）

`ButtonServoDriver` は `ActuatorDriver` と同様に、ハードウェアライブラリ（`adafruit-*` / `smbus2`）を
**メソッド内で遅延 import** し、非ハードウェア環境でもモジュールを import 可能に保つ。インターフェースは
`Protocol` で定義し、非HW環境ではスタブ（`_StubButtonServoDriver`）へ差し替える（既存の `_StubActuator` と同パターン）。

```python
# ✅ SG90 は 50Hz PWM。角度は「待機／押下」の2ポジションのみ（設定値）
class ButtonServoDriver:
    async def press(self, channel: int, duration_s: float) -> None:
        pca = self._require_pca()           # 遅延 import した PCA9685 インスタンス
        pca.channels[channel].angle = self._press_angle
        try:
            await asyncio.sleep(duration_s)  # 押下時間だけ保持
        finally:
            pca.channels[channel].angle = self._rest_angle  # 必ず待機位置へ戻す

# ❌ 任意角度・任意保持は不可（角度は設定の2値、時間のみ可変）
```

> PCA9685 の I2C バスはアクセル・ブレーキの RS-485（Modbus RTU）とは独立しており、50ms制御ループの
> バスとは競合しない。ただしボタン押下は状態機械でゲートし、RUNNING／該当モード中のみ許可する。

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

4. **安全包絡は生成側で機構的に保証**: 自動生成する基準軌跡・パターンは、プロファイルの
   安全包絡（最高車速・上限G）を「生成段階」で満たすように作る（実行時の監視だけに頼らない）。

```python
# ✅ 良い例: 規定パターンの減速区間長を上限Gから算出し、レート ≤ max_decel_g を保証
rate = max(DECEL_MARGIN * profile.max_decel_g * G_TO_KMHS, MIN_RATE_KMHS)
dt = (v_from - v_to) / rate            # この区間の所要時間（減速レートが上限以内になる）
speed = min(speed, profile.max_speed)  # 全点を最高車速以内にクランプ
```

5. **ボタンサーボは2値角度に制限・非常停止時は待機位置へ**: ボタンサーボ（SG90）の角度は
   「待機／押下」の2ポジション（設定値）に限定し、任意角度は取らない。非常停止・エラー遷移時は
   `release_all()` で全チャンネルを待機（非押下）位置へ戻す（ペダルアクチュエータの原点復帰に相当）。

```python
# 例外の有無にかかわらず全ボタンサーボを待機位置へ
async def safe_stop(self) -> None:
    ...
    await self._button_servo.release_all()  # 押下途中でも待機位置へ復帰
```

---

### 純粋ロジックとハード実行の分離（テスト容易性）

ハードを動かす制御・適合ロジックは、**純粋な計算ロジック**と**ハード実行**を分離し、計算側を
注入式（走行結果をコールバックで受け取る等）にしてハード無しで単体テストできるようにする。

```python
# ✅ 良い例: 座標降下チューナーは走行（ハード）を持たず、候補→コスト報告の純粋ロジック
tuner = CoordinateDescentTuner(initial_gains, max_runs=15)
while (cand := tuner.next_candidate()) is not None:
    cost = run_and_evaluate(cand)   # ← ハード実行は呼び出し側（コントローラ）
    tuner.report(cand, cost)
# テストでは run_and_evaluate を既知の凸コスト関数に差し替えて収束を検証できる
```

> FOPDT同定・SIMC算出・規定パターン生成・コスト関数も同様に `src/domain/pid_tuning.py` の
> 純粋関数として実装し、ハード結合のオーケストレーションは `RobotController` 側に置く。

---

### フロントエンド規約

React 18 + `@babel/standalone`（CDN・ビルド工程なし）でブラウザ内トランスパイルする構成。グラフは外部チャートライブラリではなく **SVG を直接描画**する。

1. **共有コンポーネントを優先**: 学習運転・自動運転のモニターは共有の `DriveMonitorScreen`（`src/web/static/js/screens/auto-drive.js`）を props 違いで再利用する。画面ごとに重複実装しない。

2. **開始系ボタンは必ず arm→確認ポップアップ→start の順序にする**: `DriveMonitorScreen` の
   `driveArmPath`（arm→ポップアップ→start）を使い、`confirmOnly: true`（ポップアップ表示 →
   「はい」で arm+start を同期実行）は新規実装で使わない。確認ポップアップは「チェック済みで
   開始してよい状態か」を確認するものであり、チェック未実施のまま表示すると停車保持ブレーキ・
   走行前チェック・車速0確認が「開始しますか？」の後回しになってしまう。
   **Why**: 学習サイクル開始ボタンが2026-07-03の刷新で `confirmOnly` に切り替わり、この順序が
   一時的に逆転する回帰が発生した（2026-07-06 修正）。

3. **時間→座標は1箇所に集約**: リアルタイムグラフの時間軸マッピング（スライディングウィンドウ式と `toXFull(frac)`）を一元化し、表示挙動の変更はこのマッピング式を起点に行う。

4. **ライブ表示マーカーは現在値に紐づける**: 「現在位置」など常時表示すべきマーカーは、軌跡（描画済みデータ列）の最終点ではなく `realtimeData` の現在値に紐づける。これにより停止/初期状態（データ0件）でも表示される。

```jsx
// ✅ 良い例: 走行前(値0)でも中央に表示される
<circle cx={toXFull(0.5)} cy={toY(rd.actual_speed_kmh, maxSpeed, PH1)} r="4.5" />

// ❌ 悪い例: 走行データが無いと消える
const p = speedAct_pts[speedAct_pts.length - 1];
<circle cx={p.x} cy={p.y} r="4.5" />
```

4. **画面外データはクランプではなく除外**: ウィンドウ外の点は座標をクランプ（端に固定）すると横線アーティファクトになるため、`frac∈[0,1]` でフィルタ除外する。

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
- `servo`: ボタンサーボ（PCA9685 / SG90）・タイムスケジュール
- `api`: FastAPI・WebSocket
- `log`: ログ・アーカイブ
- `profile`: 車両プロファイル管理
- `ui`: フロントエンド

**例**:
```
feat(calibration): 手動ジョグ方式によるゼロフルキャリブレーションを実装

- オペレーターがジョグ操作（+/-キー）でゼロ/フル位置を目視確認して記録
- アクセル・ブレーキを独立してキャリブレーション
- バリデーション（ゼロ<フル・ストローク妥当性）を実装
```

```
fix(control): 50ms制御ループのジッタ低減

asyncio.sleep の代わりに call_later を使用することで
ループ周期のばらつきを±5ms以内に抑制
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

## 運転モデル・特徴量セット変更のガイドライン

逆モデル（FF用 poly-Ridge）の特徴量構成（`FeatureSpec`）や推定器を変更する場合は、以下の手順を必ず踏む。

### 変更手順

1. **オフライン評価を先に実施**: `scripts/evaluate_feature_sets.py` で既存ログを使い候補セットを比較する
   （train=学習運転セッション、holdout=閉ループ走行セッション。DB・モデル・設定は書き換えない）
2. **判定は holdout-B（基準速度から特徴構築）の加重MAE を主指標にする**
   - 学習はノイジーな実速度・推論は滑らかな基準速度を使うため、holdout-B が配備条件に最も近い
   - in-sample／CV の改善だけで採用しない（GBM の教訓: in-sample が大幅改善しても
     off-manifold 域で閉ループ追従が破綻した。20260630-learning-wide-speed-range 参照）
3. **採用前に実機の閉ループ検証（ベンチ）を行う**: オフライン指標は閉ループ性能を保証しない
4. 本番反映は `config/settings.toml` の `[model]` セクション経由のみ（コードのデフォルト値は変更しない）

### 過去の評価から得た経験則（2026-07-03 実ログ評価）

- **既存特徴の線形結合になる特徴（Δv同士の差・傾き等）は追加しない**: 2次多項式+Ridge では
  理論上冗長で、in-sample だけ改善し holdout で悪化する（実効自由度の増加=過学習方向）
- 特徴量の追加・削減より**訓練データ量（セッション数・速度域カバレッジ）の効果が支配的**。
  精度改善はまず学習サイクル（サイクル全ログ再学習）等のデータ側から検討する
- 短期ホライズン（0.1〜0.3s）は実速度ノイズを増幅する（std比〜1.3倍）。訓練データが
  十分な場合のみ有効なため、採用判断はデータ量とセットで行う

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
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.05)
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

# カバレッジ計測（ドメインレイヤー目標: 80%以上）
pytest tests/unit/ tests/integration/ --cov=src/domain --cov-report=term-missing

# ローカル・実機での手動実行のみ
pytest tests/hardware/ -v -s
```

**カバレッジ目標**（architecture.md準拠）:
- `src/domain/`: **80%以上**（制御・安全ロジックの中核）
- `src/infra/`: ハードウェア依存のためモック化してユニットテスト
- `src/web/`・`src/app/`: 統合テストで動作確認

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
- [ ] 50ms制御ループ内でブロッキング処理がないか
- [ ] ログ書き込み（asyncpg INSERT）が5ms以内に完了するか（architecture.md準拠、100ms周期での書き込み1回あたり）

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

# 4. 設定ファイルのコピーと編集
cp config/settings.toml.example config/settings.toml
# 以下の必須項目を環境に合わせて編集:
#   [serial] accel_port  : アクセル用シリアルポート（例: /dev/ttyUSB0）← ls /dev/ttyUSB* で確認
#   [serial] brake_port  : ブレーキ用シリアルポート（例: /dev/ttyUSB1）
#   [gpio] emergency_stop_pin : 非常停止 GPIO番号（デフォルト: 17）
#   [gpio] ac_detect_pin      : AC断検知 GPIO番号（デフォルト: 27）
#   [archive] usb_ssd_path   : 外付けSSDのマウントポイント（例: /mnt/usb_ssd）

# 5. PostgreSQL ロール・DB作成（初回のみ）
sudo -u postgres createuser --createdb $USER
sudo -u postgres createdb driving_robot

# 6. データベース初期化
python scripts/setup_db.py

# 7. 動作確認（ユニット・統合テスト）
pytest tests/unit/ tests/integration/ -v

# 8. システム起動
bash scripts/start.sh
```

> **lgpio (GPIO ライブラリ)**: `lgpio` は apt パッケージで管理するため pip install できない。
> `sudo apt install python3-lgpio` でインストール後、venv から参照するために以下の `.pth` ファイルを作成する:
> ```
> echo "/usr/lib/python3/dist-packages" > .venv/lib/python3.13/site-packages/system-dist-packages.pth
> ```
> 確認: `.venv/bin/python -c "import lgpio; print(lgpio.__file__)"` が出力されれば OK。
>
> **Kvaser Linux ドライバ**: Kvaser公式サイト（https://www.kvaser.com/downloads-kvaser/）から
> `linuxcan.tar.gz` をダウンロードし、`sudo make KV_NO_PCI=1 && sudo make install KV_NO_PCI=1` でビルド・インストールする。
> インストール後に `sudo modprobe usbcanII leaf mhydra` でモジュールをロードし、
> `listChannels` で Kvaser デバイスが表示されること・`/dev/usbcanII0` が作成されることを確認する。
> `ip link show` には `can0` は出ない（Kvaser は SocketCAN ではないため正常）。
>
> **python-can aarch64 パッチ**: Raspberry Pi 5 (aarch64) では python-can に2つのバグがあり、インストール後に手動パッチが必要。
> 詳細と修正方法は `docs/architecture.md` の「python-can aarch64 既知バグ」セクションを参照。
> `pip install --upgrade python-can` を実行した場合は再パッチが必要なため注意。

> **udev rules によるシリアルポート固定**: 複数の USB-RS485 デバイスを接続する場合、
> 再接続でポート番号が変わることがある。`udev rules` でシリアル番号に基づき
> `/dev/ttyUSB_accel`・`/dev/ttyUSB_brake` に固定することを推奨する。

> **PCA9685 / I2C（ボタンサーボ, Post-MVP）**: `raspi-config`（または `/boot/firmware/config.txt` の
> `dtparam=i2c_arm=on`）で I2C を有効化し、`sudo apt install i2c-tools` の `i2cdetect -y 1` で
> PCA9685 が `0x40` に見えることを確認する。Python ライブラリは
> `pip install adafruit-circuitpython-pca9685`（または `smbus2`）で導入する。
> SG90 ×16 の電源は PCA9685 の V+ に外部5Vを供給し、本体ロジック電源とは分離する（GNDは共通）。

### 推奨開発ツール

- **エディタ**: VS Code + Python拡張 + Pylance（型チェック有効化）
- **型チェック**: `mypy src/`（コミット前に実行）
  - `strict = false`、`ignore_missing_imports = true`（RPi.GPIO・python-can は型スタブなし）
  - `pyproject.toml` の `[tool.mypy]` に設定を記載
- **フォーマット**: `ruff format src/ tests/`（自動整形）
- **Lint**: `ruff check src/ tests/`（問題確認）

### 依存関係の追加・更新手順

```bash
# 1. pyproject.toml の [dependencies] に追記
# 2. 仮想環境にインストール
pip install -e .
# 3. lockファイルを更新
pip freeze > requirements.lock
# 4. 両ファイルをコミット
git add pyproject.toml requirements.lock
git commit -m "build: <ライブラリ名> を追加"
```

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
- [ ] 50ms制御ループ内にブロッキング処理がない
- [ ] 非常停止ハンドラが優先的に動作する

### テスト
- [ ] ユニットテストが書かれている（ドメインレイヤー）
- [ ] `pytest tests/unit/ tests/integration/` がパスする
- [ ] エッジケース（通信断・タイムアウト）がテストされている

### 品質ツール
- [ ] `ruff check src/ tests/` エラーなし
- [ ] `mypy src/` エラーなし
- [ ] `ruff format src/ tests/` 実行済み
