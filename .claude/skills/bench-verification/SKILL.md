---
name: bench-verification
description: driving_robot アプリの起動・動作確認の手順。スタブHW／ベンチ（GPIOのみ実機）／実機の3モードの使い分け、環境変数、安全上の禁止事項を定義する。アプリを起動して動作確認したい、WebUI/APIの変更を検証したい、uvicorn を立ち上げたい、実装後の verify をしたい、というときは必ずこのスキルを使うこと。環境変数を誤ると実機アクチュエータが動くため、スキルなしで起動コマンドを組み立てない。
---

# ベンチ/実機検証（bench-verification）

FastAPI アプリ（WebUI + 制御スタック）を安全に起動して動作確認するスキル。
対象は実車のペダルを物理アクチュエータで操作するロボットなので、**モード選択を誤ると実機が動く**。

## 3つのハードウェアモード

| モード | 環境変数 | HW | 用途 |
|---|---|---|---|
| **スタブ（既定）** | なし | 全てスタブ | UI/API/ロジックの動作確認。Claude が自律的に使ってよい |
| ベンチ | `DRIVING_ROBOT_USE_REAL_HW=1` + `DRIVING_ROBOT_BENCH_GPIO_ONLY=1` | 非常停止スイッチ(GPIO)のみ実機、アクチュエータ/CANはスタブ | 非常停止スイッチの実機検証（ユーザー立ち会い） |
| 実機 | `DRIVING_ROBOT_USE_REAL_HW=1` | 全て実機（Modbus RTU アクチュエータ・CAN・GPIO） | WebUI での本番走行。**ユーザーが自分で起動する。Claude は設定しない** |

**`DRIVING_ROBOT_USE_REAL_HW=1` を Claude が自律的に設定することは禁止**（WebUI の実機・ベンチ起動はユーザーが行う）。
理由: 実機モードではアクセル/ブレーキアクチュエータが物理的に動作し、シリアルポートを占有する。
研究用の実機走行（`tests.research.main --hw real`）は別扱いで、下の「実機走行（tests/research）」に従えば Claude が無人で実行してよい。

## DB の有無

- `DATABASE_URL` 未設定 → **in-memory リポジトリ**（プロファイル・モード・セッションは揮発）。
  純粋な UI 動作確認はこれで十分なことが多い。
- `DATABASE_URL=postgresql://localhost/driving_robot` → 本番 DB に接続。
  起動時に `running` のまま残った孤児セッションを `error` に是正する副作用があるので、
  **本番 DB を汚したくないテストでは DATABASE_URL を付けずに起動する**のが安全。

## 起動手順

1. **ポート確認（必須）**: `ss -tln | grep -E ':8080'`
   - 8080 が使用中なら、それは**ユーザーが手動起動したインスタンスの可能性があり、実機接続かもしれない**。
     勝手に走行系 API を叩かない・kill しない。自分の検証には別ポート（8081）で新規に立てる。
2. **スタブモードで起動**（バックグラウンド）:
   ```bash
   .venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8081
   ```
3. **疎通確認**: `curl -s http://localhost:8081/api/v1/drive/status`
   （`/` は 307 → `/static/index.html`）
4. **検証後は自分が立てた uvicorn を必ず停止する**（ポートとプロセスを残さない）。

## WebUI の確認方法

- **対話的な確認・スクショ**: Playwright MCP（`mcp__playwright__browser_navigate` → `browser_snapshot` →
  `browser_take_screenshot`）。SSH/ヘッドレス環境なのでブラウザ表示はない。スクショを Read で確認する。
- **スクリプト化した再現テスト**: `webapp-testing` スキル（Python Playwright、`headless=True` 必須）。
- WebUI コンポーネントの罠（共有 Box の onClick、Btn の boolean prop 等）は既知の注意点がある。
  挙動が不可解なときは `src/web/static/js/` の該当コンポーネント実装を確認する。

## テストとの使い分け

- ロジックの検証はまず `pytest tests/unit/`・`pytest tests/integration/`（統合テストはローカル PostgreSQL 使用）。
- このスキルの出番は「実際にアプリを立ち上げて UI/API を通しで確認する」とき。
  テストが通っただけで「動作確認済み」と報告しない。
- WebUI の実機モードの検証は計画だけ立ててユーザーに依頼する。研究用の実機走行は下の節に従って Claude が行う。
  結果の解析は `drive-log-analysis` スキルで行う。

## 実機走行（tests/research）— Claude が無人で実行してよい（ユーザー決定 2026-10-18）

ユーザーの立ち会い・合図は不要。実装計画の最後の実機確認は Claude が走らせて解析し、結果を報告する。

### Claude が実行してよいもの（人の操作・目視が要らないもの）

| コマンド | 内容 |
|---|---|
| `.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"` | 手順2（学習運転 → FF モデル → yaml 書き戻し） |
| `.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"` | 手順3（WLTP モード走行）。`--limit-s N` 付きも可 |
| `.venv/bin/python -m tests.research.main --steps 1,2,3 --hw real; echo "exit=$?"` | 手順2 → 手順3 |
| `.venv/bin/python -m tests.research.main --steps 1,11 --hw real; echo "exit=$?"` | 加振（周波数応答の測定） |
| `.venv/bin/python -m tests.research.main --retrain <CSV> --hw real` | 走らずにモデルを作り直して yaml へ書き戻す |
| `sudo .venv/bin/python scripts/check_can.py` など読むだけの確認 | CAN 受信・UPS（チェック1・2）。終わらないものは timeout を付ける |

### ユーザーに頼むもの（人の操作・目視が要る）

- `tests/hardware/test_calibration.py`（ペダル位置の合わせ込み。ジョグ＋目視で Enter）と、その 4 値の yaml への書き写し
- `tests/hardware/test_emergency_stop_home_return.py`（非常停止ボタンを押す）
- `tests/hardware/test_overcurrent_home_return.py`（手で押さえて過電流を出す・目視）
- `tests/hardware/test_servo_pca9685.py`（サーボの動きを目視）、UPS チェック3（AC ケーブルを抜く）
- WebUI の実機・ベンチ起動（`DRIVING_ROBOT_USE_REAL_HW=1`）

### 走らせる前の確認（毎回）

1. `ss -tln | grep -E ':8080'` — ユーザーの WebUI が動いていたら走らせない（シリアルを取り合う）。ユーザーに伝えて止める
2. `fuser /dev/ttyUSB0 /dev/ttyUSB1` — 他のプロセスがアクチュエータのシリアルを使っていたら走らせない
3. `pgrep -af "tests.research.main"` — 前の走行が残っていたら走らせない
4. 変更箇所のテスト・ruff・スタブ（`--hw stub`）が通っていること

### 走らせ方

- CLAUDE.md の指示どおり、実機走行はサブエージェント（haiku）で行い、結果を opus が受け取る。
- 手順2・手順3 は 10 分を超えるので Bash の `run_in_background` で起動し、出力をファイルに残す
  （例: `... --hw real > <scratchpad>/run_<日時>.log 2>&1; echo "exit=$?" >> <同じファイル>`）。
- 終了コード（0 成功 / 2 設定 / 4 初期化 / 5 走行エラー / 6 モデル作成 / 7 走行前チェック / 8 手順2 の学習結果 NG）を必ず報告する。
- yaml の `candidate`・`model_path`・`vehicle` を Claude が手で書き換えない（プログラム自身の書き戻しは可）。
  比べたい制御は候補名で切り替える。

### 途中で止めるとき

- **`kill -INT <pid>`（Ctrl+C と同じ）だけを使う**。`main` の `finally` が原点復帰・切断・ログ保存をする作り
  （コードで確認しただけで、実機で試したことはない）。`kill -9`・`SIGTERM` は `finally` が走らずペダルを踏んだまま残るおそれがあるので使わない。
- 止める基準: 想定時間の 1.5 倍を超えても終わらない、ログが 60 秒以上進まない。
  中断したら、ユーザーへの報告に「中断した・理由・その時の出力の最後」を書く。

### 走った後

- `tests/research/results/drive_log_real_<日時>.csv` を解析する（DB を見るなら `drive-log-analysis` スキル）。
- 結果は ProblemReport の該当する段の「結果」に、CSV 名を出典にして書く（`problem-report` スキル）。
- 「直った」は生データで確かめてから書く。
