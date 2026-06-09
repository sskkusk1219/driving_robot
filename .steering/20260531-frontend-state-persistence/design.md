# 設計: フロントエンド状態のページリロード後保持

## 変更対象ファイル

- `src/web/static/js/app.js` のみ（バックエンド変更なし）

## 実装アプローチ

### 1. localStorage ヘルパー関数

App() 関数の外部（ファイル先頭）に定義:

```javascript
function lsGet(key, fallback = null) {
  try { return localStorage.getItem(key) ?? fallback; } catch { return fallback; }
}
function lsSet(key, value) {
  try {
    if (value == null) localStorage.removeItem(key);
    else localStorage.setItem(key, value);
  } catch {}
}
```

### 2. useState の初期値を localStorage から読み込む

```javascript
const [nav, setNav] = useState(() => lsGet('drv_nav', 'init'));
const [activeProfileId, setActiveProfileId] = useState(() => lsGet('drv_profile_id', null));
const [activeProfileName, setActiveProfileName] = useState(() => lsGet('drv_profile_name', null));
const [activeModeId, setActiveModeId] = useState(() => lsGet('drv_mode_id', null));
const [activeModeName, setActiveModeName] = useState(() => lsGet('drv_mode_name', null));
```

### 3. useEffect で状態変化を localStorage に同期

各状態の変化を useEffect でキャッチして localStorage を更新する。

### 4. マウント時のAPI整合性チェック修正

既存の API ポーリングを拡張:
- サーバーの active_profile_id とキャッシュが異なる場合、プロファイル名を再取得
- サーバーに active_profile_id がない場合、キャッシュもクリア

## 注意事項

- try/catch で localStorage のエラーを無視
- useState の関数形式でマウント時のみ実行
- null は removeItem で削除（localStorage は文字列のみ）
