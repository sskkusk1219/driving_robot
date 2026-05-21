# 設計: GUI実装

## アーキテクチャ方針

### フロントエンド構成

```
src/web/static/
├── index.html              # React SPA エントリポイント
└── js/
    ├── sketch.js           # UI プリミティブ (Frame, Box, Btn, etc.)
    ├── calib-components.js # JogKey, PosRuler
    ├── screens/
    │   ├── init.js         # 初期化・システム状態
    │   ├── profiles.js     # 車両プロファイル
    │   ├── calibration.js  # キャリブレーション
    │   ├── modes.js        # 走行モード
    │   ├── auto-drive.js   # 自動走行モニター
    │   ├── manual.js       # 手動運転
    │   ├── learning.js     # 学習運転
    │   ├── logs.js         # ログ管理
    │   └── schedule-sequence.js  # タイムスケジュール/シーケンス(P1)
    └── app.js              # メインアプリ・ルーティング・API・WebSocket

## 状態管理

グローバル状態（useContext で各画面に配布）:
  nav: 現在の画面
  robotState: RobotState
  activeProfileId/Name: 選択中プロファイル
  activeModeId/Name: 選択中走行モード
  upsLoss: AC UPS 電源断状態
  realtimeData: WebSocket からのリアルタイムデータ
  realtimeBuf: グラフ用リングバッファ

## 各画面

1. 初期化: BOOTING/ERROR/STANDBY/INITIALIZING 状態表示
2. 自動走行: 3軸グラフ + 一時停止/走行終了ボタン
3. キャリブレーション: ジョグ操作 + ZERO/FULL 確定 + 保存
4. 手動運転: ジョグ操作 + 運転終了ボタン
5. 車両プロファイル: 一覧/作成/編集
6. 走行モード: 一覧/新規作成(CSV)
7. 学習運転: 自動走行と同じグラフUI
8. ログ: セッション一覧/詳細

## 実装方針

- ワイヤーフレームの JSX をほぼそのまま流用
- mock データ -> API / WebSocket データに置き換え
- エラーハンドリング: API 失敗時は toast 通知
