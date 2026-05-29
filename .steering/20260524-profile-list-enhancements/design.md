# 設計書

## アーキテクチャ概要

変更対象は `src/web/static/js/screens/profiles.js` のみ。
バックエンド変更なし。既存の PUT /api/v1/profiles/{id} と POST /api/v1/profiles/ を使う。

## コンポーネント設計

### 1. インライン名前編集（ProfilesScreen 内）

**状態追加**:
- `editingNameId: string | null` — 現在インライン編集中のプロファイル ID

**実装の要点**:
- 名前セルの表示を条件分岐:
  - `editingNameId === p.id` のとき: `<input>` + 確定/キャンセルボタン
  - それ以外: `<b>名前</b>` + 鉛筆アイコン
- `editingNameValue` state で入力中の値を保持
- 保存処理: `apiFetch('PUT', /api/v1/profiles/${id}, { name: trimmed })` — 部分更新
  - バックエンドの ProfileUpdateRequest は全フィールド任意なので name だけ送れる
- 保存後: `editingNameId = null`, `loadProfiles()`

**UI フロー**:
```
[通常表示] Tanaka ✎
         ↓ ✎クリック
[編集中]  [Tanaka____] ✓ ✗
         ↓ Enter / ✓ / blur
[保存完了] Tanaka2 ✎  → トースト「名前を更新しました」
```

**注意点**:
- blur は confirm ダイアログ等との相性が悪いためボタンでの確定を主とし、blur は補助（別の行の ✎ クリック時に現在の編集を保存）
- 空白バリデーションは保存直前にチェック

### 2. プロファイルコピー（ProfilesScreen 内）

**実装の要点**:
- 操作列に `コピー` ボタンを追加
- クリックハンドラ `handleCopy(p)`:
  1. payload を構築: `{ name: p.name + ' のコピー', ...その他パラメータ }`
  2. `apiFetch('POST', '/api/v1/profiles/', payload)` で新規作成
  3. 成功時: `showToast('コピーを作成しました', 'success')` + `loadProfiles()`

**payload の構築**:
```js
{
  name: `${p.name} のコピー`,
  max_accel_opening: p.max_accel_opening,
  max_brake_opening: p.max_brake_opening,
  max_speed: p.max_speed,
  max_decel_g: p.max_decel_g,
  pid_gains: { kp: p.pid_gains?.kp ?? 1.0, ki: p.pid_gains?.ki ?? 0.0, kd: p.pid_gains?.kd ?? 0.0 },
  stop_config: {
    deviation_threshold_kmh: p.stop_config?.deviation_threshold_kmh ?? 2.0,
    deviation_duration_s: p.stop_config?.deviation_duration_s ?? 4.0,
  },
  model_path: null,  // キャリブレーション同様、モデルもコピーしない
}
```

## データフロー

### インライン名前保存
```
1. ✎ クリック → setEditingNameId(p.id), setEditingNameValue(p.name)
2. 入力 → setEditingNameValue(value)
3. Enter / ✓ クリック → PUT /api/v1/profiles/{id} { name }
4. 成功 → setEditingNameId(null), loadProfiles(), showToast
5. 失敗 → apiFetch が null を返す → 何もしない（トーストはapiFetch内で表示）
```

### プロファイルコピー
```
1. コピー クリック → POST /api/v1/profiles/ { ...コピー payload }
2. 成功 → loadProfiles(), showToast('コピーを作成しました')
3. 失敗 → apiFetch が null を返す → 何もしない
```

## グリッドレイアウトの変更

現在の操作列幅 `1fr` では「選択」「編集」「コピー」3ボタンが収まらないため、
操作列を `1.4fr` に拡大する。

## 実装の順序

1. `ProfilesScreen` に `editingNameId`, `editingNameValue` state を追加
2. `handleSaveName(p)` 関数を実装
3. 一覧テーブルの名前セルのレンダリングを条件分岐に変更
4. `handleCopy(p)` 関数を実装
5. 操作列に「コピー」ボタンを追加
6. グリッドの操作列幅を調整
