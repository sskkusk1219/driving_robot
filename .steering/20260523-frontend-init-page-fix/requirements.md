# 要求定義: フロントエンド初期化ページ修正

## 対象ファイル
`src/web/static/js/screens/init.js`

## 修正内容

### 1. キャンセルボタン削除
- 現在: `<Btn onClick={() => setNav('profiles')} disabled={isInitializing}>キャンセル</Btn>` が存在
- 変更後: 削除

### 2. STEPSの置き換え

#### 現在のSTEPS
| label | sub |
|---|---|
| 通信確認 (ttyUSB0) | pymodbus connect OK |
| 通信確認 (ttyUSB1) | pymodbus connect OK |
| 通信確認 (CAN/Kvaser) | /dev/usbcanII0 ready |
| アラームリセット (両軸) | FC05 0x0407 → FF00 |
| サーボON (両軸) | FC05 0x0403 |
| 原点復帰 | 前回正常終了 → スキップ予定 |

#### 変更後のSTEPS
| label | sub |
|---|---|
| 通信確認 (ブレーキ) | (なし) |
| 通信確認 (アクセル) | (なし) |
| 通信確認 (CAN) | (なし) |

### 3. 削除するsub文言
- pymodbus connect OK
- /dev/usbcanII0 ready
- FC05 0x0407 → FF00
- FC05 0x0403
- 前回正常終了 → スキップ予定
