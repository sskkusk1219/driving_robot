# タスクリスト

## 実装タスク

- [x] T1: sketch.js に ValidationPopup コンポーネントを追加
- [x] T2: calibration.js にプロファイル未選択チェックを追加
- [x] T3: learning.js を本格コンポーネントに拡張
- [x] T4: auto-drive.js の AutoDriveScreen にバリデーションを追加
- [x] T5: auto-drive.js の DriveMonitorScreen 3軸目データアクセスを修正

## 申し送り

### 実装完了: 2026-05-24

### 変更ファイル
- `src/web/static/js/sketch.js` - ValidationPopup コンポーネント追加・エクスポート
- `src/web/static/js/screens/calibration.js` - プロファイル未選択チェック追加
- `src/web/static/js/screens/learning.js` - 1行ラッパーを本格コンポーネントに拡張
- `src/web/static/js/screens/auto-drive.js` - AutoDriveScreen バリデーション追加、3軸目データアクセス修正

### 技術的判断
- 3軸目の不具合原因: reference_speed が `{time_s, speed_kmh}` の時系列データなのに `{duration_s, speed}` セグメント形式でアクセスしていた
- 修正: `totalDurS = refProfile[last].time_s` + `refProfile.map(p => {x: p.time_s/totalDurS, y: p.speed_kmh})`
- バリデーション優先順位: プロファイル未選択 → キャリブレーションなし → モード未選択（自動運転のみ）
- キャリブレーションチェックは `GET /api/v1/profiles/{id}` で `calibration.is_valid` を確認

---

## 実装後の振り返り

### 実装完了日
2026-05-24

### 計画と実績の差分
- 計画通り5タスクすべて完了。スコープ変更なし。
- learning.js の拡張は「1行 → 本格コンポーネント」と記述したが、実際には40行程度のシンプルな構成で済んだ。

### 学んだこと
- **フロントエンドのデータフォーマット不一致はサイレントに失敗する**: `duration_s` / `speed` へのアクセスは `undefined` を返し、グラフが表示されないだけでエラーにならない。バックエンドのスキーマと JSの実装を常に突き合わせる必要がある。
- **ValidationPopup の再利用パターン**: `POPUP_CONFIG` オブジェクトで message / actionLabel / nav をまとめると、複数バリデーション条件の管理がすっきりする。

### 次回への改善提案
- AppContext に `activeProfileCalibIsValid` を持たせると、各画面での API 呼び出し（`GET /api/v1/profiles/{id}`）を省略できる。プロファイル選択時に一度取得してキャッシュする設計も検討余地あり。
- バリデーション条件が増えるなら、`useValidation(checks)` のようなカスタムフックにまとめると各スクリーンをシンプルに保てる。
