# タスクリスト: フロントエンド状態のページリロード後保持

## フェーズ1: app.js の修正

- [x] localStorage ヘルパー関数 (lsGet/lsSet) をファイル先頭に追加
- [x] nav/activeProfileId/activeProfileName/activeModeId/activeModeName の useState 初期値を localStorage から読み込む形式に変更
- [x] 各状態の変化を localStorage に同期する useEffect を追加
- [x] マウント時のAPI整合性チェックを修正（プロファイルID不一致時に名前を再取得、未選択時にキャッシュクリア）

## 追加タスク（バリデーション指摘対応）

- [x] API失敗時に activeProfileId/activeProfileName をセットでクリアするよう修正（app.js）
- [x] profiles.js: handleSaveName で選択中プロファイルの名前変更時に setActiveProfileName を呼ぶ
- [x] modes.js: handleSaveName で選択中モードの名前変更時に setActiveModeName を呼ぶ
- [x] modes.js: onSave コールバックで選択中モードの名前変更時に setActiveModeName を呼ぶ

## 実装後の振り返り

**実装完了日**: 2026-05-31

**計画と実績の差分**:
- 設計通りに app.js のみ修正する予定だったが、バリデーション指摘で profiles.js と modes.js も修正が必要だった
- localStorage 化によって既存の「名前変更後に右上表示が更新されないバグ」が顕在化した

**学んだこと**:
- localStorage で名前を永続化すると、名前変更フロー全体でコンテキスト更新が必要になる
- API整合性チェックで「名前取得失敗」のエラーケースも必ず考慮が必要

**次回への改善提案**:
- プロファイル名・モード名の更新は、AppContext の setActiveProfileName/setActiveModeName を通じて一元管理する仕組みがあると漏れを防げる
