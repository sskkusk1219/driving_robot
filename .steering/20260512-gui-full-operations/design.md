# 設計: GUI全操作対応

## ページ構成（変更後）

```
Header: タイトル | 状態バッジ | 選択中プロファイル名
Main:
  1. 車両プロファイルカード
     - プロファイル一覧（カード形式、選択/編集/削除/キャリブレーションボタン付き）
     - 「＋ 新規作成」ボタン
  2. 走行制御カード（既存 + キャリブレーションボタン追加）
  3. リアルタイムデータ（速度・開度・グラフ）

Modal: プロファイル作成・編集フォーム
```

## 対象ファイル
- `src/web/static/index.html` - 構造追加
- `src/web/static/js/app.js` - プロファイル管理ロジック追加
- `src/web/static/css/style.css` - モーダル・フォーム・プロファイルカードのスタイル追加

## 使用API
- GET    /api/v1/profiles/             プロファイル一覧
- POST   /api/v1/profiles/             プロファイル作成
- PUT    /api/v1/profiles/{id}         プロファイル更新
- DELETE /api/v1/profiles/{id}         プロファイル削除
- POST   /api/v1/drive/select-profile  プロファイル選択
- POST   /api/v1/drive/calibrate       キャリブレーション実行
