# 要求内容

## 概要

`src/` 本番コード全体のコードレビュー(/code-review high)で検出・検証済み(CONFIRMED)の問題点を修正する。安全性に直結する正確性バグ10件と、補足所見(正確性5件・クリーンアップ/効率5件)が対象。

## 背景

2026-06-10 実施のマルチエージェントレビューで、非常停止系統・制御ループ・状態機械に重大な欠陥が検出された。特に「AC電源喪失時の非常停止が配線されておらず完全に無動作」「CANバス無音時に制御ループが凍結しペダルが踏まれたまま放置」など、実機運用で危険な挙動につながるものを含む。

## 実装対象の修正

### A. 非常停止系統の一元化(C1, C4, C5, C10)

- **C1**: AC断→`SafetyMonitor.trigger_emergency()` のコールバックリストが空で無動作。`controller.emergency_stop` を SafetyMonitor に登録し、GPIO/UPS とも SafetyMonitor 経由の単一ディスパッチに統一する
- **C4**: `_execute_one_cycle` が await 後に `_running` を再確認せず、EMERGENCY 宣言後にアクチュエータ指令を送出する → アクチュエータ書き込み直前に再チェック
- **C5**: `start_auto_drive`/`start_learning_drive` が `_open_session` の await 後に状態を再確認せず、EMERGENCY 中に DriveLoop を起動する → 状態再チェックを追加
- **C10**: `gpio_monitor._fire_callbacks` が Future を破棄し非常停止コルーチンの例外を黙殺。さらに ERROR→EMERGENCY が遷移表で許可されていない → done_callback でログ記録 + 遷移表修正

### B. 制御ループ堅牢化(C3, C7, C8)

- **C7**: CANReader の無制限キュー+最古フレーム取り出しによる速度値の陳腐化・メモリ増加 → 常時ドレインする最新値キャッシュ方式に変更
- **C3**: CAN無音時の永久ブロック+サイクル重複実行 → read_speed に鮮度チェック(古ければ例外→非常停止)、ループに重複実行ガード
- **C8**: `asyncio.ensure_future` の戻り値未保持(GC でタスク消失リスク) → タスク参照保持+例外時は非常停止

### C. 状態機械・シャットダウン(C2, C9, C13)

- **C2**: `shutdown()` がペダル解放せずプロセス終了 → ベストエフォートで home_return + servo_off を実施
- **C9**: 初期化途中失敗で INITIALIZING にスタック → 失敗時 ERROR へ遷移し clear_error 経由で再試行可能にする
- **C13**: `/drive/start` で mode が None でも RUNNING 遷移 → 404 を先に返す

### D. API/Web層(C6, C12, C14, C11)

- **C6**: jog の step が無制限でクランプなし → スキーマバリデーション + クランプ
- **C12**: 同期の sklearn 学習がイベントループをブロック → `asyncio.to_thread` + 走行中は 409
- **C14**: モード/プロファイル複製時の UNIQUE 制約違反が 500 → 409 を返す
- **C11**: 一時停止ボタンが存在しないルートを叩く(常に404) → バックエンド未実装機能のためボタンを削除

### E. プロファイル適用(C15)

- プロファイルの `pid_gains`/`stop_config` が制御スタックに適用されない → `select_profile` で PIDController と SafetyMonitor に反映し、閾値の単一ソース化

### F. 効率・クリーンアップ(C16, C17 + 重複排除)

- **C16**: `_ref_speed_at` の O(n) 線形走査 → bisect 化
- **C17**: WS配信が走行中も100ms毎に4回のModbusトランザクションを発行 → DriveLoop のサイクル計測値キャッシュを利用(あわせて ref_speed_kmh も配信)
- セッション/キャリブレーションのレスポンス変換重複(5箇所/2箇所)をヘルパー化
- `start_learning_drive` の `start_auto_drive` 全文コピーを共通ヘルパーに統合(manual のセッション永続化脱落も修正)
- 50ms 周期の3箇所独立定義を settings 由来の単一ソースに統一

## 受け入れ条件

- [ ] AC断時に `controller.emergency_stop` が呼ばれる配線がユニットテストで検証できる
- [ ] EMERGENCY 宣言後の実行中サイクルがアクチュエータ書き込みを行わない
- [ ] CAN 速度値が一定時間更新されない場合に非常停止が発火する
- [ ] shutdown() がペダル解放(home_return)とサーボOFFを試行する
- [ ] 初期化失敗後に clear_error → 再初期化が可能
- [ ] jog の step がバリデーションされる
- [ ] プロファイルの PID ゲイン・逸脱閾値が制御に反映される
- [ ] 既存ユニットテスト・統合テストが全て成功する

## スコープ外

- 一時停止/再開機能のバックエンド実装(新機能のため別作業)
- UPS低バッテリー(LB)での走行中停止(別途要件定義が必要)
- フロントエンドの AC_LOSS 画面/showToast 重複/未使用コンポーネント等(今回レビュー報告対象外)
- hardware テスト(実機接続が必要なため手動検証に委ねる)

## 参照ドキュメント

- `docs/functional-design.md` - 機能設計書
- `docs/architecture.md` - アーキテクチャ設計書
- レビュー結果: 本ステアリング作成元の /code-review 出力(requirements 冒頭参照)
