---
name: drive-log-analysis
description: driving_robot の走行ログ・学習サイクルを PostgreSQL から解析するための手順とスキーマ知識。走行セッションの解析、KPI 判定（p95/max/切替回数）、学習サイクルの調査、「昨日の走行どうだった？」「○時の学習サイクルを見て」のような依頼、drive_logs/drive_sessions/learning_cycles への SQL、ペダルワークや速度追従の評価を求められたら必ずこのスキルを使うこと。DB の時刻や run_type の扱いに罠があるため、スキルなしで直接 SQL を書かない。
---

# 走行ログ分析（drive-log-analysis）

シャシダイナモ自動運転ロボットの走行ログを PostgreSQL から解析するスキル。

## 接続

- DB: `postgresql://localhost/driving_robot`（環境変数 `DATABASE_URL`、未設定時も各スクリプトはこの既定値を使う）
- 対話的な確認は `psql -d driving_robot`、スクリプトは asyncpg
- **読み取り専用を厳守**。本番走行データへの UPDATE/DELETE は行わない（必要なら必ずユーザーに確認）

## ⚠️ 必ず踏む罠（先に読む）

1. **時刻は UTC**。ユーザーが言う時刻は JST（UTC+9）。「18:55 の学習サイクル」→ DB では `09:55`。
   出力時は JST に変換して報告する: `started_at AT TIME ZONE 'Asia/Tokyo'`
2. **auto セッションも cycle_id を持つ**。学習サイクルのモデル学習データを集計するときは
   `run_type IN ('learning','tuning')` で絞らないと auto 走行が混入する。
3. **learning セッションの `ref_speed_kmh` は NULL**（開ループで基準速度がない）。
   numpy 配列化の前に None 処理が必要。
4. ペダル「踏んでいる」判定の閾値は **0.5%**（`kpi_monitor._PEDAL_ON_THRESHOLD_PCT` と揃える）。

## スキーマ早見表

| テーブル | 主要カラム | 備考 |
|---|---|---|
| `drive_sessions` | id, profile_id, mode_id, run_type, started_at, ended_at, status, cycle_id | run_type: `auto`/`manual`/`learning`/`tuning`、status: `running`/`completed`/`error`/`emergency` |
| `drive_logs` | session_id, timestamp, ref_speed_kmh, actual_speed_kmh, accel_opening, brake_opening, accel_pos, brake_pos, accel_current, brake_current, plan_effort_pct, trim_effort_pct, applied_effort_pct, phase | 1セッション分は `WHERE session_id = '<session_id>' ORDER BY timestamp` |
| `learning_cycles` | id, profile_id, status, started_at, ended_at, detail(jsonb) | status: `running`/`completed`/`error`/`aborted`。サイクル配下のセッションは `drive_sessions.cycle_id` で引く |
| `vehicle_profiles` | id, feedforward_params(jsonb) ほか | 車両定数（クリープ・エンジンブレーキ等） |
| `pedal_plans` / `driving_modes` / `time_schedules` / `calibration_data` | — | プラン・モード・スケジュール・キャリブレーション |

## 定番スクリプト（車輪の再発明をしない）

まず既存スクリプトで足りるか確認してから、独自 SQL/Python を書く。

- **セッション KPI 解析**（第一選択）:
  ```bash
  .venv/bin/python -m scripts.analyze_session <session_id>
  .venv/bin/python -m scripts.analyze_session --latest auto   # 直近の auto セッション
  ```
  プライマリー KPI ゲート: **p95≤0.4 km/h / max≤1.0 km/h（例外なし）/ 符号反転≤1回/5s窓**。
  （p95 は 2026-07-16 に 0.2→0.4 へ変更済み。実装値は `kpi_monitor.KPI_P95_LIMIT_KMH`）
  系統ラグ（相互相関）、ペダルON立ち上がり回数/min（110-135km/h帯の目安 ≤10）も出る。
- **減速側の R² 原因調査**: `scripts/analyze_decel_fit.py --profile-id <UUID>`（読み取り専用）
- **ペダルプランの机上検証**: `scripts/preview_pedal_plan.py`（フェーズ内訳・切替回数、`--verify` で包絡確認）
- **特徴量セットのオフライン比較**: `scripts.evaluate_feature_sets`（train=learning / holdout=tuning、A-Bギャップに注意）

## 定番クエリ

直近のセッション一覧（JST 表示）:
```sql
SELECT id, run_type, status, cycle_id,
       started_at AT TIME ZONE 'Asia/Tokyo' AS started_jst,
       ended_at   AT TIME ZONE 'Asia/Tokyo' AS ended_jst
FROM drive_sessions ORDER BY started_at DESC LIMIT 10;
```

ユーザー指定時刻（JST）から学習サイクルを特定:
```sql
SELECT id, status, started_at AT TIME ZONE 'Asia/Tokyo' AS started_jst, detail
FROM learning_cycles
WHERE started_at BETWEEN '2026-07-14 09:50+00' AND '2026-07-14 10:10+00'  -- JST 18:50-19:10
ORDER BY started_at;
```

サイクル配下のセッション（学習データ集計は run_type で絞る）:
```sql
SELECT id, run_type, status FROM drive_sessions
WHERE cycle_id = '<cycle_id>' AND run_type IN ('learning','tuning')
ORDER BY started_at;
```

## 分析の作法

- **「解決した」を要約統計だけで言い切らない**。自分で選んだ閾値のサマリではなく、
  生波形・分布（該当区間の drive_logs そのもの）を確認してから結論を出す。
  ユーザーは WLTP ±0.5 km/h で自ら運転できるレベルの精度感覚を持っている。
- 報告には session_id / cycle_id と JST 時刻を明記する（ユーザーが照合できるように）。
- グラフが必要なら matplotlib で PNG に保存し、SendUserFile で渡す。
