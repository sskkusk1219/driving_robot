# 要件定義: UPS監視・制御機能

## 背景・目的

APC Smart-UPS 750 (SUA750JB) を用意した。RaspiとUPSはシリアルポート（DB-9）→ USB変換アダプタで接続済み。

これまでの設計では AC断検知を GPIO27 の接点出力で行う予定だったが（TBD状態）、
SUA750JB には標準の NC/NO リレー接点出力がないため、シリアル通信経由で UPS を監視する。

監視には Linux 標準の NUT (Network UPS Tools) を使用する。

## 実装する機能

### 1. バッテリー残量監視（走行前チェック）
- NUT から `battery.charge` を取得して走行前チェックに使用
- 残量 20% 未満で走行不可（既存の `UPSPreCheckProtocol` を実装）

### 2. AC断検知（安全停止シーケンス）
- NUT の `ups.status` に `OB`（On Battery）が含まれたら AC断と判定
- 5秒ポーリングで検知（GPIO27 接点方式から変更）
- 既存の SafetyMonitor.handle_ac_power_loss() を呼ぶ

### 3. UPS状態のリアルタイム配信
- WebSocket の `RealtimeData` に UPS 情報を追加
  - `ups_battery_pct: float | None` バッテリー残量 [%]
  - `ups_on_battery: bool` AC断フラグ
- GUIでのAC断状態表示に使用

### 4. UPS状態 REST API
- `GET /api/v1/ups/status` エンドポイントを追加
- バッテリー残量・AC状態・NUT接続状態を返す

### 5. 走行前チェックの factory 組み込み
- 現在 factory.py に `PreCheckRunner` が未接続（TBD）
- NutUPSMonitor を使った PreCheckRunner を factory に追加

## スコープ外

- NUT のインストール・設定（setup_nut.sh スクリプトを提供するが、実行はユーザーが行う）
- GPIO27 AC断接点入力の完全廃止（GPIOMonitor クラスは残す、factory からのみ外す）
- UPS のシャットダウン制御（upsmon による自動シャットダウンは NUT に任せる）

## 接続情報

- UPS モデル: APC Smart-UPS 750 (SUA750JB)
- 接続: シリアルポート (DB-9) → USB 変換アダプタ → Raspberry Pi USB ポート
- NUT ドライバ: `apcsmart`（APC Smart プロトコル経由）
- 予想デバイスパス: `/dev/ttyUSB2`（ttyUSB0: アクセル, ttyUSB1: ブレーキ に続く番号）
- NUT サーバー: localhost:3493
- NUT UPS 名: `apcups`（設定可能）
