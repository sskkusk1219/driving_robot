# 要求: 非常停止の動きの修正

## 背景・報告された不具合

実機 (`DRIVING_ROBOT_USE_REAL_HW=1`) で非常停止スイッチ (GPIO17) を押した際:

1. **HW は原点復帰したが、フロントエンドが非常停止画面 (EmergencyResetScreen) に遷移しない。**
2. スイッチを戻した後も操作不能。フロントは初期化画面のまま `POST /initialize`
   (409 Conflict) や `POST /select-profile` (500, 現在: EMERGENCY) を投げ続ける。

## 根本原因の分析

- バックエンドの状態は正しく `EMERGENCY` に遷移している (409/500 がその証拠)。
- フロントエンドは WebSocket `/ws/realtime` の `robot_state` を見て画面を切り替えるが、
  `robot_state` が `EMERGENCY` として届いていないため画面が変わらない。
- `broadcast_loop` (`src/web/ws.py`) は `get_realtime_data()` (Modbus 位置・電流読み取り)
  が返ってから初めて `broadcast()` する構造。HW 読み取りがストールすると `robot_state`
  も配信されなくなる。
- ストール要因: GPIO 割り込みが多重発火 (`非常停止スイッチ検知` が複数回) し、
  `emergency_stop()` が重複起動 → 共有 Modbus クライアント上で最大30秒の `home_return()`
  ポーリングが複数同時実行され、`get_realtime_data()` のバス確保が長時間待たされる。
- スイッチを戻すのは FALLING エッジで監視対象外。バックエンドが `EMERGENCY` に留まるのは
  仕様どおり (UI のリセット操作で復帰させる設計)。画面さえ出れば復帰可能。

## 受け入れ条件

1. 非常停止検知後、HW 読み取りが滞っても `robot_state` (= `EMERGENCY`) が
   WebSocket で確実に配信され、フロントが EmergencyResetScreen を表示する。
2. GPIO 割り込みが多重発火しても `emergency_stop()` の原点復帰が重複起動しない (冪等)。
3. 既存の非常停止関連テストを壊さない (BOOTING からの `emergency_stop()` は引き続き例外)。
4. UI のリセットボタンで `EMERGENCY → READY` に復帰できる (既存フローを維持)。
