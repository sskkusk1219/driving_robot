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
| **スタブ（既定）** | なし | 全てスタブ | UI/API/ロジックの動作確認。**Claude が自律的に使ってよいのはこれだけ** |
| ベンチ | `DRIVING_ROBOT_USE_REAL_HW=1` + `DRIVING_ROBOT_BENCH_GPIO_ONLY=1` | 非常停止スイッチ(GPIO)のみ実機、アクチュエータ/CANはスタブ | 非常停止スイッチの実機検証（ユーザー立ち会い） |
| 実機 | `DRIVING_ROBOT_USE_REAL_HW=1` | 全て実機（Modbus RTU アクチュエータ・CAN・GPIO） | ダイナモ上の本番走行。**ユーザーが自分で起動する。Claude は設定しない** |

**`DRIVING_ROBOT_USE_REAL_HW=1` を Claude が自律的に設定することは禁止**。
理由: 実機モードではアクセル/ブレーキアクチュエータが物理的に動作し、シリアルポートを占有する。

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
- ハードウェア結合（実機モード）の検証は計画だけ立ててユーザーに依頼する。
  結果の解析は `drive-log-analysis` スキルで行う。
