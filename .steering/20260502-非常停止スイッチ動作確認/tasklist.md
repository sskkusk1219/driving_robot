# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: ハードウェアテストスクリプト作成

- [x] `tests/hardware/` ディレクトリを作成する
- [x] `tests/hardware/test_emergency_stop.py` を作成する
  - [x] GPIO17 初期状態表示
  - [x] BOTH エッジ（押下・解除）割り込み検知
  - [x] Ctrl+C による安全なクリーンアップ

## フェーズ2: 動作確認（手動実行）

- [x] スクリプトを実行して初期状態が正しく表示されることを確認
  - [x] `$ python3 tests/hardware/test_emergency_stop.py` （lgpio はシステムPython使用）
  - [x] GPIO17 の初期状態が LOW（NC接点が閉じている=通常状態）と表示される ← NC接点に合わせてロジックを修正
- [x] スイッチを押して検知されることを確認
  - [x] 「非常停止スイッチ 押下検知 (HIGH)」が表示される
- [x] スイッチを離して解除が検知されることを確認
  - [x] 「非常停止スイッチ 解除検知 (LOW)」が表示される
- [x] Ctrl+C で正常終了することを確認
  - [x] 「GPIOクリーンアップ完了」が表示される

## フェーズ3: 振り返り

- [x] 実装後の振り返りを記録する

---

## 実装後の振り返り

### 実装完了日
2026-05-02

### 計画と実績の差分

**計画と異なった点**:
- 当初設計（architecture.md）は NO接点・LOW=停止を想定していたが、実際のスイッチが NC固定型だった
- Raspberry Pi 5 では RPi.GPIO が非対応（RP1チップ）のため、lgpio（システムパッケージ）に変更
- lgpio は pip でインストール不可（apt パッケージ）のため、実行は `.venv` ではなく `/usr/bin/python3` を使用
- `gpio_claim_input` ではエッジ検知コールバックが動作せず、`gpio_claim_alert` が必要だった
- 起動直後のノイズ対策として `gpio_set_debounce_micros` + `time.sleep(0.1)` を追加

**架 docs/architecture.md の修正**:
- `LOW=停止` → `HIGH=停止（RISING検知）` に変更（NC接点の実態に合わせる）

### 学んだこと

**技術的な学び**:
- Raspberry Pi 5 は GPIO チップが RP1 に変わり RPi.GPIO は動作しない。lgpio（システムPython）を使う
- NC接点スイッチは HIGH=停止（RISING エッジ）となり、ケーブル断線でも停止するフェイルセーフ設計
- lgpio でエッジ検知するには `gpio_claim_alert`（`gpio_claim_input` では不可）
- チャタリング除去は `gpio_set_debounce_micros(h, pin, 50_000)` で設定する
