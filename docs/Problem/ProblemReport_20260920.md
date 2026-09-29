# 逆モデルを速度で分ける（C7）— 2026-09-20

`docs/Problem/ProblemReport_20260919.md` で、アクセル側の `|偏差| > 1.0 km/h` を減らすために
**手順2 に低開度の走行パターンを足してモデルに反映**したところ、**低速は改善したが中高速が
悪化**した（合計 22.1 → 53.8s）。原因は**データではなく器**で、アクセル逆モデルが
**2 次多項式 Ridge 1 本**しかないため、低速に当たりに行くと高速の係数がずれる。

このドキュメントは**その器を変える 1 案（逆モデルを速度で分ける）を試す**ためのもの。
**ダメなら手順4（Kp 適合）へ移行する。**

- 基準にする走行: `tests/research/results/report20260919_RunFF_5.md`
  （`drive_log_real_20260919_071816.csv`、03_WLTP_Low 589s 完走、`|偏差|>1.0` 合計 **22.1s**）
- 基準の FF モデル: `tests/research/results/models/test_vehicle_20260918_193408.pkl`（現在の `model_path`）
- 学習に使うログ: **2026-09-20 07:30 に走った手順2**（本稿執筆時点で走行中）。
  低開度階段を含み、不感帯 **8.74%** で走っている。以下の数字は 1 本前の手順2
  （`drive_log_real_20260920_042349.csv`、不感帯 8.21%）で数えたもので、
  **新しいログが出たら数え直す**
- 制御構成: FF のみ（Kp=Ki=Kd=0）→ effort の符号でアクセル/ブレーキに振り分け

---

## 1. 用語

- **逆モデル**: 「いまの車速と、この先の基準車速の並び」から「出すべきペダル開度」を直接予測する
  回帰モデル。`tests/research/ff_candidate.py` の `train_inverse_model_effective` が学習し、
  pkl に保存する
- **特徴量**: `v0`（実車速）/ `dv_0.5` `dv_1.0` `dv_2.0` `dv_3.0`（先読み 0.5〜3.0 秒先の基準との差）/
  `v0_sq` / `dv1_x_v0` / `dv_past_0.5` `dv_past_1.0`（過去 0.5・1.0 秒の実車速との差）の **9 本**。
  これを 2 次多項式に展開して **54 項**、Ridge（α=1.0）で回帰する
- **候補**: FF の作り方の版。C1 が素の逆モデル、C5 が「過去＋V0 実測」、C6 が「骨格＋残差」。
  いま走っているのは **C5**
- **学習行**: 逆モデルの学習に使う行。アクセルモデルは `実アクセル開度 >= 不感帯` の行だけ

## 2. なぜこれをやるのか

`ProblemReport_20260919.md` 12.1 の実測（手順3 を各 2 本）:

| 基準車速帯 | 時間 | 旧 pkl(9/18) | 新 pkl(9/20・階段あり) |
|---|---|---|---|
| 0〜20 km/h | 320s | 24.2 / 19.2s | **14.6 / 12.7s** |
| 20〜40 km/h | 199s | 3.3 / 0.8s | **20.5 / 21.5s** |
| 40〜60 km/h | 69s | 1.6 / 2.1s | **17.7 / 19.6s** |
| **合計** | 589s | **29.2 / 22.1s** | **52.8 / 53.8s** |

**低速は狙いどおり良くなっている**（平均偏差 −0.17 → +0.02、A1 の最悪イベントは −1.78 → −1.43）。
**悪くなったのは 20 km/h 以上だけ。**

つまり **低開度階段のデータは正しい**。1 本の多項式がそれを飲み込むときに、高速側の係数を
犠牲にしている。**速度で分ければ、低速の改善だけを取って高速を元のまま保てる**はず、
というのがこの案。

**何が困るのか**: いまの pkl は 9/18 のもの（低速の改善なし）に差し戻してある。
このままだと残りの `|偏差|>1.0` の **83〜87% が 0〜20 km/h** に居座り続ける
（RunFF_4: 24.2s / 29.2s、RunFF_5: 19.2s / 22.1s）。

## 3. 分かっている数字

### 3.1 学習行の速度分布（アクセルモデル。`実開度 >= 不感帯 8.21%` の行）

| 速度帯 | 9/19 手順2 | **9/20 手順2（階段あり）** |
|---|---|---|
| 0〜10 km/h | 302 (4.6%) | **939 (11.5%)** |
| 10〜16 km/h | 121 (1.8%) | **561 (6.9%)** |
| 16〜20 km/h | 319 (4.9%) | 520 (6.4%) |
| 20〜24 km/h | 153 (2.3%) | 314 (3.9%) |
| 24〜30 km/h | 204 (3.1%) | 402 (4.9%) |
| 30〜40 km/h | 513 (7.8%) | 495 (6.1%) |
| 40〜60 km/h | 1146 (17.4%) | 1009 (12.4%) |
| 60〜80 km/h | 1214 (18.5%) | 1382 (17.0%) |
| 80〜100 km/h | 1144 (17.4%) | 1100 (13.5%) |
| 100〜140 km/h | 1459 (22.2%) | 1417 (17.4%) |
| **合計** | **6,575** | **8,139** |

**0〜20 km/h は 742 行（11.3%）→ 2,020 行（24.8%）と 2.7 倍**になった。低開度階段の成果。

### 3.2 分割したときの各モデルの学習行（境界 20 km/h・のりしろ ±4 km/h）

実際の特徴量行列（先読み・過去の分だけ端が落ちる）で数えた値:

| 不感帯 | アクセル学習行 | 低速 `v0<=24` | 高速 `v0>=16` |
|---|---|---|---|
| 8.21%（このログを走ったときの値） | 8,168 | **2,425** | 6,567 |
| 8.74%（**現在の yaml**。3.4 参照） | 7,915 | **2,182** | 6,557 |

のりしろの約 820 行は**両方に入れる**（境界付近をどちらのモデルも見ている状態にする）。
どちらも `MIN_REGIME_SAMPLES = 8`（`src/domain/model_training.py:64`）を大きく上回る。
2 次多項式の項数は 54 なので、低速モデルでも **1 項あたり 40 サンプル**ある。

### 3.3 境界をどこに置くか

`ProblemReport_20260919.md` 12.4 の実測（階段から測ったペダルゲインと yaml の模型ゲインの比）:

| 車速 | 8 | 10 | 12 | 14 | **16** | 20 | 24 | 30 |
|---|---|---|---|---|---|---|---|---|
| 上り/模型 | 1.28 | 1.19 | 1.11 | 1.04 | **1.00** | 0.99 | 0.98 | 0.99 |

**16 km/h でぴったり 1.00 になり、それ以上はずっと 1.0 前後。** 車の性格が変わるのがこの
あたりで、手順3 の悪化も 20 km/h から始まっている。**境界 20 km/h・のりしろ ±4 km/h
（＝ 16〜24 km/h で混ぜる）**を初期値にする。

### 3.4 不感帯は手順2-0 のたびに測り直される

2026-09-20 07:30 の手順2-0 で yaml が書き換わった:

| キー | 前（9/20 04:23 の手順2） | 後（9/20 07:30 の手順2） |
|---|---|---|
| `accel_deadband_pct` | 8.21 | **8.74**（+0.53） |
| `brake_deadband_pct` | 11.89 | 12.00 |
| `stop_brake_opening_pct` | 28.32 | 28.21 |

**この走行で学習するので、ログと不感帯は一致する。** 低開度階段の刻みは
`不感帯 + offset`（`pattern_drive.build_patterns`）で作られるので、1 段目は
8.71% → **9.24%** に自動で置き直される。**3.1 / 3.2 の行数は 1 本前のログの値なので、
新しいログが出たら数え直すこと**（8.21% で走ったログを 8.74% で学習すると、
学習行の条件 `実開度 >= 不感帯` に 1 段目の 207 行中 205 行が引っかかって落ちる。
**今回はその組み合わせにならない**）。

**ただし記録として**: 2 回の 2-0 で不感帯が **0.53% 動いた**。
`ProblemReport_20260919.md` 12.2 で A1 の正体は「開度 **0.09%** の不足」だったので、
**追いかけている誤差の 6 倍**の幅で測定値が動いていることになる。
独立な 2 つの推定（段3-3 の `x0` = 8.21 − 0.09、段1 の 8.21 + 0.2）はどちらも
真の立ち上がりを 8.1〜8.4% と言っており、8.74% はそれより 0.3〜0.6% 高い。
サンプルは 2 点だけなので断定はしないが、**C7 で結果が出たあとも FF の精度が
頭打ちなら、ここを疑うこと**（`ProblemReport_20260919.md` 14.4 の分離用パターンと同じ入口）。

## 4. 案: C7 — 速度で分けたアクセル逆モデル

### 4.1 やること

**アクセルモデルだけを 2 本にする。ブレーキ側は一切触らない**（1 変数比較を保つため）。

```
低速モデル  … v0 <= split + overlap の行で学習
高速モデル  … v0 >= split - overlap の行で学習

推論:
  w = clip((v0_raw - (split - overlap)) / (2 * overlap), 0.0, 1.0)
  開度 = (1 - w) * 低速モデル.predict(特徴量) + w * 高速モデル.predict(特徴量)
```

`v0_raw` は**学習域クリップ前の実車速**を使う（C6 の骨格が `v0_raw` を使うのと同じ理由。
`_accel_opening` の第 1 引数として既に渡ってきている）。

**のりしろで線形に混ぜるのが要点。** 単純に切り替えると境界で指令開度が跳び、
`ProblemReport_20260919.md` 11.1 で見たように実開度が指令に追いつかない区間ができる。
`w` は 16 km/h で 0、24 km/h で 1 になり、その間を直線で移る。

### 4.2 なぜこの形か（他をやらない理由）

- **骨格＋残差（C6 の流儀）にしない**: 骨格は `pedal_gain_at` のペダルゲイン曲線を読むが、
  その曲線は割線推定なので低速で不正確（12.4 で `上り/模型` が 1.28）。骨格を直すところから
  始めることになり、変数が増える
- **特徴量に速度帯のフラグを足すだけにしない**: 2 次多項式なので交互作用は入るが、
  低速と高速で**係数そのものを別にしたい**（いまの問題はまさに係数の取り合い）。
  フラグ 1 本では取り合いは解けない
- **3 分割以上にしない**: まず 2 分割で効くかを見る。効いたうえで足りなければ増やす

## 5. 実装（すべて `tests/` 配下。`src/` は読み取り・import のみ）

### 5.1 `tests/research/ff_candidate.py` — 学習側

`train_inverse_model_effective`（`:129`）に引数を 2 つ足す:

```python
def train_inverse_model_effective(
    logs, profile, output_dir="data/models",
    feature_spec=DEFAULT_FEATURE_SPEC,
    cruise_curve=None,
    accel_split_kmh: float | None = None,      # None なら今までどおり 1 本
    accel_split_overlap_kmh: float = 4.0,
) -> tuple[str, dict[str, dict[str, float]]]:
```

- `accel_split_kmh is None` のときは**いまと完全に同じ動作**（既定。C1〜C6 を壊さない）
- 指定時は `x_accel[:, 0]`（= `v0`）で行を 2 つに分け、それぞれで `_make_estimator()` を
  `fit` する。**`accel_model`（全域 1 本）も従来どおり学習して pkl に残す**
  （`src` の `FeedforwardController.load_model` が読むキーなので、C1〜C6 の pkl 互換を保つ）
- pkl に研究用キーを足す（`load_model` は読み飛ばす）:

```python
payload["accel_model_low"] = accel_model_low
payload["accel_model_high"] = accel_model_high
payload["accel_split_kmh"] = accel_split_kmh
payload["accel_split_overlap_kmh"] = accel_split_overlap_kmh
```

- `metrics` に `accel_low` / `accel_high` を足す（`_metrics` をそのまま使う）。
  さらに **全域 1 本のモデルとの比較を速度帯ごとに出す**こと（下の 7 章の判定に使う）

`cruise_curve`（C6）と `accel_split_kmh` の同時指定は**エラーにする**（骨格＋分割は今回やらない）。

### 5.2 `tests/research/ff_candidate.py` — 推論側

`CandidateC6`（`:556`）の隣に足す:

```python
class CandidateC7(CandidateFeedforward):
    """C7: アクセル逆モデルを速度で 2 本に分け、のりしろで線形に混ぜる。

    ブレーキ側・停車保持・クリープ・学習域クリップは C1 と同じ。
    """

    candidate = "C7"

    def load_model(self, model_path: str) -> None:
        # 本体の load_model に加えて accel_model_low / _high / split / overlap を読む。
        # 無ければ ValueError（C7 用でない pkl を拒否する。CandidateC6 と同じ流儀）

    def _accel_opening(self, v0_raw, desired_accel, features) -> float:
        lo = float(self._accel_low.predict(features)[0])
        hi = float(self._accel_high.predict(features)[0])
        w = (v0_raw - (self._split - self._overlap)) / (2.0 * self._overlap)
        w = min(1.0, max(0.0, w))
        return max(0.0, (1.0 - w) * lo + w * hi)
```

`CANDIDATE_CLASSES`（`:595`）に `"C7": CandidateC7` を足す。

### 5.3 `tests/research/config.py` / `config_testVehicle.yaml`

`feedforward` に 2 キー足す（既定は無効＝いまと同じ動作）:

```yaml
  accel_split_kmh: 0.0            # 0 で無効。正値ならアクセル逆モデルをこの車速で 2 本に分ける
  accel_split_overlap_kmh: 4.0    # のりしろ [km/h]（この幅で 2 本を線形に混ぜる）
```

`validate_config`: `accel_split_kmh >= 0`、`0 < accel_split_overlap_kmh`、
`accel_split_kmh > 0` のとき `accel_split_kmh - accel_split_overlap_kmh > 0`。

`pattern_drive.build_ff_model`（`:644` 付近の `train_inverse_model_effective` 呼び出し）から
設定値を渡す。**`candidate: C7` のときだけ分割する**（`candidate` と `model_path` は
必ず組で書き換える、という既存の約束を守る）。

### 5.4 テスト（変更箇所だけ）

`tests/research/test_research_ff_candidate.py`:
- `test_train_without_split_is_unchanged` … `accel_split_kmh=None` で pkl のキーと
  `accel_model` の予測が現行と一致すること（**回帰の要**）
- `test_train_with_split_saves_both_models` … `accel_model_low` / `_high` / `split` /
  `overlap` が pkl に入り、`accel_model`（全域）も残ること
- `test_split_training_rows_include_overlap` … のりしろの行が両方に入ること
- `test_c7_blends_linearly_across_overlap` … `v0` が `split-overlap` で低速モデルの値、
  `split+overlap` で高速モデルの値、`split` でちょうど中点になること
- `test_c7_rejects_pkl_without_split_keys` … C7 用でない pkl で `ValueError`
- `test_train_rejects_cruise_curve_with_split` … 同時指定でエラー

`tests/research/test_research_config.py`:
- 新 2 キーが `validate_config` を通り、範囲外が弾かれること

## 6. 段取り（実機走行はユーザーが実施）

**手順2 を走り直す必要はない。** 走行中の手順2 のログに低開度階段のデータが入るので、
C7 を実装したあと `relearn.py`（既存ログからモデルを作り直すオフライン入口。実機不要）で
pkl を作り直せる。実機走行は**手順3 を 2 本だけ**。

```bash
cd /home/raspi5_16gb/projects/driving_robot

# (0) 走行中の手順2 が終わったら、そのログのファイル名を確認する
ls -t tests/research/results/drive_log_real_*.csv | head -1

# (1) 実装後、まず差分だけ見る（設定は書き換わらない）
.venv/bin/python -m tests.research.relearn \
  tests/research/results/<新しい手順2 のログ>.csv --dry-run

# (2) yaml を C7 に切り替えてから pkl を作る（candidate と model_path は必ず組で）
#     feedforward.candidate: C7 / accel_split_kmh: 20.0 にしてから
.venv/bin/python -m tests.research.relearn \
  tests/research/results/<新しい手順2 のログ>.csv

# (3) 実機で手順3 を 2 本（周囲の安全を確認してから）
.venv/bin/python -m tests.research.main --steps 3 --hw real
.venv/bin/python -m tests.research.main --steps 3 --hw real
```

**比較の前に**: 走行中の手順2 は不感帯も pkl も更新する。C7 を入れる前に
**分割なしで手順3 を 1〜2 本走らせて新しい基準を取り直す**と、C7 の効果だけを
切り分けられる（分割なしの pkl は同じ `relearn.py` で `accel_split_kmh: 0.0` のまま作れる）。

**その日の 1 本目は判定に使わない**（`ProblemReport_20260919.md` 9 章）。

## 7. 判定の基準

**採否は実機比較で決める。**

**まず基準を取り直すこと。** 2026-09-20 07:30 の手順2 は不感帯も pkl も更新するので、
下の 2 つの基準（9/18 pkl・9/20 04:23 pkl で測ったもの）はそのままでは比較相手にならない。
**新しい手順2 の pkl を分割なし（`accel_split_kmh: 0.0`）で作って手順3 を 1〜2 本**走らせ、
それを基準にする。下の表は**目安**として残す:

| | `|偏差|>1.0` 合計 | 0〜20 km/h | 20〜40 km/h | 40〜60 km/h |
|---|---|---|---|---|
| **基準A: 9/18 pkl（いまの設定）** | **22.1 / 29.2s** | 19.2 / 24.2s | 0.8 / 3.3s | 2.1 / 1.6s |
| 基準B: 9/20 pkl（分割なし・階段あり） | 52.8 / 53.8s | 12.7 / 14.6s | 21.5 / 20.5s | 19.6 / 17.7s |

**C7 が成功と言えるのは、次の 2 つを同時に満たしたとき:**

1. **0〜20 km/h が基準B 並み**（12〜15s）まで下がる ＝ 低開度階段の効果が残っている
2. **20〜60 km/h が基準A 並み**（合計 3〜5s）に留まる ＝ 高速を壊していない

合計で言えば **15〜20s**（基準A の 22.1s より良い）になるはず。

**どちらか一方しか満たせない場合:**

- 低速だけ良くて高速が悪い → **のりしろ・境界の取り方の問題**。境界を上げる（24 km/h）か
  のりしろを広げる（±6 km/h）
- 高速は守れたが低速が良くならない → **低速モデルの表現力かデータの問題**。
  `ProblemReport_20260919.md` 14.3 の「階段の形の見直し」を先にやる

**どちらも満たせない場合は打ち切り、手順4（Kp 適合）へ移行する**
（`ProblemReport_20260919.md` 14.1 に設計判断が書いてある）。

なお 1 本ごとに `|偏差|>1.0` の合計は 22〜47s ばらつくので、**合計 1 個では判定しない**。
帯別と、イベント単位（レポート 4.4 の表）で並べて比べること。

## 8. 気をつけること

- **境界の不連続**: のりしろで混ぜても、`w` が動く間は 2 つのモデルの予測差がそのまま
  開度の動きになる。手順3 のログで **16〜24 km/h を通過するときの指令開度**を見て、
  跳びや振動が出ていないか確認する
- **低速モデルの外挿**: 低速モデルは `v0 <= 24 km/h` の行しか見ていない。発進直後など
  「実車速は低いが先読み（`dv_2.0` / `dv_3.0`）が大きい」場面は学習域の端になる。
  `speed_clip_max` による入力クリップは `v0` にしか効かないので、ここは実走で確認するしかない
- **ブレーキ側は触らない。** アクセルだけ変えることで、手順3 の差分がアクセル由来だと言える
- **`accel_model`（全域 1 本）を pkl に残す**のを忘れないこと。消すと C1〜C6 の pkl として
  読めなくなり、候補を戻せなくなる

## 9. 遵守する制約

- **`src/` は一切変更しない**。読み取り・import のみ。実行環境はすべて `tests/` に置く
- 1 度にすべて実装しない。1 段ずつ、ユーザーが実機で確認してから次へ
- **踏み増しの速度制限（レートリミット）は設けない**
- 実装は sonnet サブエージェントで行う。議論・計画は opus 以上
- ゲートは `ruff check` のみ。**`ruff format` は走らせない**（無関係な 32 ファイルが書き換わる）
- テストは変更箇所だけ実施する
- **実機走行はユーザーが実施する。Claude は `pytest` と `ruff check` とスタブだけ**
- **採否は実機比較で決める。オフラインの模擬・試算で結論づけない**
- 同じ設定でも 1 本ごとに `|偏差| > 1.0` の合計がばらつく。イベント単位で比べる。
  **その日の 1 本目は判定に使わない**
- 説明の順番: 用語の定義 → 実測した数字 → なぜそうなるのか → 何が困るのか
- 各 Phase の実装が完了したら、ユーザーも動作確認を実施する。**実行コードを提示すること**

## 10. 参照

- **前提となる結論**: `docs/Problem/ProblemReport_20260919.md`
  - 11 章: 低開度階段（段3-1）と `k(v)` の推定器（段3-3）
  - 12.1: 新 pkl の実機結果（低速改善・中高速悪化）と不採用の理由
  - 12.2: A1 の正体（水準の誤差）
  - 12.4: 模型ゲインの現状（境界 16 km/h の根拠）
  - 14.1: **うまくいかなかったときの移行先（手順4 の設計判断）**
  - 14.3: 階段の形の見直し案
- 手順の定義（手順0〜10）: `docs/Problem/ProblemReport_20260910.md`
- 段1〜段4 の経緯: `docs/Problem/ProblemReport_20260916.md`

### 触るファイル

| ファイル | 何をするか |
|---|---|
| `tests/research/ff_candidate.py` | `train_inverse_model_effective` に分割学習、`CandidateC7` を追加 |
| `tests/research/config.py` | `feedforward.accel_split_kmh` / `accel_split_overlap_kmh` |
| `tests/research/config_testVehicle.yaml` | 同上（既定は無効） |
| `tests/research/pattern_drive.py` | `build_ff_model` から設定値を渡す |
| `tests/research/test_research_ff_candidate.py` | 上記のテスト |
| `tests/research/test_research_config.py` | 新キーの検証 |

### 使うログ

| 用途 | ファイル |
|---|---|
| 学習（低開度階段を含む手順2） | `tests/research/results/drive_log_real_20260920_042349.csv` |
| 基準A の手順3 | `drive_log_real_20260919_065336.csv`（RunFF_4）/ `_071816.csv`（RunFF_5） |
| 基準B の手順3 | `drive_log_real_20260920_051538.csv`（1 本目）/ `_052601.csv`（RunFF_2） |
