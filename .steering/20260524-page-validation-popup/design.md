# 実装設計

## アーキテクチャ方針

### ValidationPopup コンポーネント
- sketch.js に追加（既存の ConfirmStopPopup と同パターン）
- position: absolute, inset: 0, zIndex: 997 でスクリーン上にオーバーレイ
- Props: message, actionLabel, onAction

### バリデーションのタイミング
- 各スクリーンの useEffect([], []) (マウント時) に実装
- プロファイル未選択: activeProfileId === null をチェック（AppContext から取得）
- キャリブレーションなし: GET /api/v1/profiles/{id} を呼び出し calibration.is_valid をチェック
- モード未選択: activeModeId === null をチェック（AppContext から取得）

### 変更ファイル
1. sketch.js - ValidationPopup 追加
2. calibration.js - プロファイルチェック追加
3. learning.js - 本格コンポーネントに拡張
4. auto-drive.js - AutoDriveScreen バリデーション追加、3軸目データアクセス修正

### 3軸目修正
Before: refProfile.reduce with p.duration_s, seg.speed
After: refProfile.map with p.time_s / p.speed_kmh (time-series)
