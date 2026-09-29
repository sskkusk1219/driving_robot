"""手順 3 の FF 候補 C5 に、実は比例フィードバックが埋め込まれている量（実質 Kp）を測る。

C5 は特徴量を v0 = 実車速・先読み = 基準車速の絶対値 で作るため
    dv_h = 基準(t+h) − 実車速(t)
になり、偏差（基準 − 実車速）がそのまま特徴量に入る。つまり FF の中に比例フィードバックが
1 個ぶら下がっていて、その大きさ（∂アクセル開度/∂実車速）は「学習した ML モデルの形」だけで
決まり、本番の PID Kp とは別にもう 1 段かかっている。この実質 Kp を pkl から直接測るのがこの
モジュールで、2026-09-21 の走行でアクセルが 1.4Hz でばたついた原因切り分け用に作った
（docs/Problem/ProblemReport_20260921.md 手順2）。

使うのは学習済み pkl の `accel_model`/`brake_model`（sklearn Pipeline）と
`tests.research.ff_model.build_feature_row` / `FeatureSpec` だけ。**`src/` と制御コード
（ff_candidate.py / mode_drive.py）は一切変更しない**（import と読み取りのみ）。車両・
アクチュエータには触らない。

    .venv/bin/python -m tests.research.model_gain <pkl> [<pkl> ...] \\
        [--side accel|brake] [--speeds 10,30,60,96,120,140] [--freq 1.4]

2 つの測り方:
    level_gain     定常状態で実車速の「高さ」だけをずらしたときの傾き（中心差分・解析的）。
    sinusoid_gain  実車速が正弦波で揺れたときの開度振幅の比（実際のばたつきに近い測り方。
                   dv 経路（先読みとの差）と dv_past 経路（過去方向Δv）を分けて見られる）。

手順6（2026-09-28 ホライズン自動選択。ProblemReport_20260921）: `--side` でアクセル・ブレーキの
どちらを測るか選べる（既定 accel。旧来どおり）。ペダル別ホライズンの pkl は
`accel_feature_spec`/`brake_feature_spec` を、無い旧 pkl は両ペダル共通の `feature_spec` を使う。

手順6 段2 の実機破綻（2026-09-28）: `level_gain` は絶対値を返すため、実車速のずれに
「逆向きに」反応する pkl（遅れているのにアクセルを戻す等）を見た目上の実質Kpだけでは
見分けられず、これが実機で最大逸脱 126km/h に至る原因を見落とした一因だった。CLI が表示する
表は符号つきの `deviation_gain`（`ff_model.deviation_gain`）に変更した。正 = 正しい向き
（ずれると開度で押し戻す）、負 = 逆向き（ずれが自分で広がる）。`level_gain`/`gain_table`
自体は絶対値のまま残している（`excite.py` の加振ゲイン表示・既存テストが使うため）。
"""

from __future__ import annotations

import argparse
import pickle
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any, Literal

import numpy as np

from tests.research.ff_model import (
    DEFAULT_FEATURE_SPEC,
    FeatureSpec,
    build_feature_row,
    deviation_gain,
    pkl_is_pedal_separated,
)

# 既定の速度グリッド: 10〜140km/h を 5km/h 刻み
DEFAULT_SPEEDS_KMH: tuple[float, ...] = tuple(float(v) for v in range(10, 141, 5))

Side = Literal["accel", "brake"]


def _md_table(header: Sequence[str], rows: Sequence[Sequence[str]]) -> str:
    """既存の `_table` 系スクリプトと同じ見た目の Markdown 表を組み立てる。"""
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(lines)


def _load_pkl(path: Path, side: Side = "accel") -> tuple[Any, FeatureSpec, dict[str, Any]]:
    """手順2の pkl を読み、(side のモデル, FeatureSpec, メタ情報) を返す。

    手順6: `{side}_feature_spec`（ペダル別ホライズン）があればそれを、無ければ両ペダル共通の
    `feature_spec`（旧 pkl。または `feature_spec` も無い最旧形式は現行9特徴の
    `DEFAULT_FEATURE_SPEC`）を使う（`tests/research/ff_candidate.py` の `CandidateC6.load_model`
    と同じ読み方）。

    Raises:
        ValueError: pkl が読めない、または `{side}_model` キーが無い場合
    """
    try:
        with Path(path).open("rb") as f:
            data = pickle.load(f)  # noqa: S301 - 手順2で作った信頼済みファイルのみを扱う
    except Exception as exc:  # noqa: BLE001 - 壊れた pkl の原因を問わずメッセージ化して終了コード2にする
        raise ValueError(f"{path}: pkl を読み込めません ({exc})") from exc
    model_key = f"{side}_model"
    if not isinstance(data, dict) or model_key not in data:
        raise ValueError(f"{path}: {model_key} がありません（手順2の pkl 形式ではない可能性）")
    spec_dict = data.get(f"{side}_feature_spec", data.get("feature_spec"))
    spec = FeatureSpec(**spec_dict) if spec_dict else DEFAULT_FEATURE_SPEC
    meta = {
        "speed_clip_max": data.get("speed_clip_max"),
        "deadbands_pct": data.get("deadbands_pct"),
        "pedal_separated": pkl_is_pedal_separated(data),
    }
    return data[model_key], spec, meta


def level_gain(model: Any, spec: FeatureSpec, v0: float, *, eps: float = 0.25) -> float:
    """定常状態で実車速の「高さ」だけをずらしたときの ∂アクセル開度/∂実車速 [%/(km/h)]（正値）。

    基準車速は v0 に等しい定常点とし、過去も同じ値（dv_past = 0）に置く。つまり
    features = build_feature_row(v0±eps, future=[v0]*len(先読み), past=[v0±eps]*len(過去)) の
    中心差分。符号は「実車速が上がると開度が下がる」ので、戻り値は絶対値（正）にする。
    """
    n_future = len(spec.lookahead_horizons_s)
    n_past = len(spec.past_horizons_s)
    future = [v0] * n_future
    row_plus = build_feature_row(v0 + eps, future, [v0 + eps] * n_past, spec)
    row_minus = build_feature_row(v0 - eps, future, [v0 - eps] * n_past, spec)
    preds = np.asarray(model.predict(np.vstack([row_plus, row_minus])), dtype=float).reshape(-1)
    return abs(float((preds[0] - preds[1]) / (2.0 * eps)))


def sinusoid_gain(
    model: Any, spec: FeatureSpec, v_mean: float, freq_hz: float,
    *, amp_kmh: float = 0.15, n: int = 200,
) -> tuple[float, float, float]:
    """実車速が v_mean を中心に振幅 amp_kmh・周波数 freq_hz で揺れたときの開度振幅の比
    [%/(km/h)] を (合計, dv 経路のみ, dv_past 経路のみ) で返す。

    1 周期を n 点に分けて、各時刻 t で
        v(t) = v_mean + amp*sin(2*pi*freq*t)
        future = [v_mean]*len(先読み)            # 基準は一定
        past   = [v(t - h) for h in 過去ホライズン]
    として開度を計算し、(最大 - 最小) / (2*amp) を返す。
    - 「dv 経路のみ」= past を v(t) と同じ値にして dv_past を 0 に固定した場合
    - 「dv_past 経路のみ」= v0 と future を v_mean に固定し、dv_past だけ動かした場合
    """
    n_future = len(spec.lookahead_horizons_s)
    past_horizons = spec.past_horizons_s
    period_s = 1.0 / freq_hz
    t = np.linspace(0.0, period_s, n, endpoint=False)

    def v(tt: float) -> float:
        """時刻 tt [s] の車速 [km/h]（スカラー。特徴量は 1 点ずつ作るため配列にしない）。"""
        return float(v_mean + amp_kmh * np.sin(2.0 * np.pi * freq_hz * tt))

    vt = [v(float(tt)) for tt in t]
    future_const = [v_mean] * n_future

    def _amplitude(rows: list[np.ndarray]) -> float:
        preds = np.asarray(model.predict(np.vstack(rows)), dtype=float).reshape(-1)
        return float((preds.max() - preds.min()) / (2.0 * amp_kmh))

    # 合計: v0 = v(t)、future は一定、past は v(t-h)（実際の C5 推論と同じ経路）
    rows_total = [
        build_feature_row(vt[i], future_const, [v(float(t[i]) - h) for h in past_horizons], spec)
        for i in range(n)
    ]
    gain_total = _amplitude(rows_total)

    # dv 経路のみ: past を v0 と同じ値にして dv_past を 0 に固定（future との差 dv だけが動く）
    rows_dv = [
        build_feature_row(vt[i], future_const, [vt[i]] * len(past_horizons), spec)
        for i in range(n)
    ]
    gain_dv = _amplitude(rows_dv)

    # dv_past 経路のみ: v0・future を v_mean に固定し、past だけ v(t-h) で動かす
    rows_dv_past = [
        build_feature_row(v_mean, future_const, [v(float(t[i]) - h) for h in past_horizons], spec)
        for i in range(n)
    ]
    gain_dv_past = _amplitude(rows_dv_past)

    return gain_total, gain_dv, gain_dv_past


def gain_table(paths: Sequence[Path], speeds_kmh: Sequence[float], side: Side = "accel") -> str:
    """pkl ごと・速度ごとの level_gain を Markdown 表（既存 `_table` 系と同じ見た目）で返す。"""
    header = ["pkl", *[f"{v:g}km/h" for v in speeds_kmh]]
    rows = []
    for path in paths:
        model, spec, _meta = _load_pkl(path, side)
        gains = [level_gain(model, spec, v) for v in speeds_kmh]
        rows.append([Path(path).name, *[f"{g:.3f}" for g in gains]])
    return _md_table(header, rows)


def deviation_gain_table(
    paths: Sequence[Path], speeds_kmh: Sequence[float], side: Side = "accel"
) -> str:
    """pkl ごと・速度ごとの `deviation_gain`（符号つき実質Kp）を Markdown 表で返す。

    手順6 段2（2026-09-28。ProblemReport_20260921）: 正 = 実車速のずれに正しい向きで反応
    （遅れたら踏み増す/基準より速すぎたらブレーキを踏み増す）。負 = 逆向き（ずれが自分で
    広がる。段2 の実機破綻の原因）。`gain_table`（絶対値の `level_gain`）と違い符号を残す。
    """
    header = ["pkl", *[f"{v:g}km/h" for v in speeds_kmh]]
    rows = []
    for path in paths:
        model, spec, _meta = _load_pkl(path, side)
        gains = [deviation_gain(model, spec, v, pedal=side) for v in speeds_kmh]
        rows.append([Path(path).name, *[f"{g:.3f}" for g in gains]])
    return _md_table(header, rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m tests.research.model_gain",
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("pkls", nargs="+", type=Path, help="手順2の学習済みモデル pkl（複数可）")
    parser.add_argument(
        "--side", choices=("accel", "brake"), default="accel",
        help="どちらのペダルの戻し時間を測るか（既定 accel。手順6: ペダル別ホライズンの pkl は"
        " accel/brake で結果が変わる）",
    )
    parser.add_argument(
        "--speeds", default=None,
        help="level_gain を計算する速度のカンマ区切りリスト [km/h]（既定: 10〜140 を5km/h刻み）",
    )
    parser.add_argument(
        "--freq", type=float, default=None,
        help="指定すると sinusoid_gain の表（速度×合計/dv/dv_past）も追加で出す [Hz]（例: 1.4）",
    )
    args = parser.parse_args(argv)
    side: Side = args.side

    if args.speeds:
        try:
            speeds = [float(s) for s in args.speeds.split(",") if s.strip()]
        except ValueError:
            parser.error("--speeds はカンマ区切りの数値で指定してください（例: 30,60,96,120）")
    else:
        speeds = list(DEFAULT_SPEEDS_KMH)

    try:
        loaded = [(path, *_load_pkl(path, side)) for path in args.pkls]
    except ValueError as exc:
        print(f"エラー: {exc}", file=sys.stderr)
        return 2

    print(f"## pkl 一覧（どの学習結果か取り違えないための確認用。--side {side}）")
    print()
    for path, _model, _spec, meta in loaded:
        note = "（アクセル/ブレーキ別ホライズン）" if meta["pedal_separated"] else ""
        print(
            f"- {path.name}: speed_clip_max={meta['speed_clip_max']}, "
            f"deadbands_pct={meta['deadbands_pct']}{note}"
        )
    print()
    print(
        "## level_gain（実質 Kp、符号つき: deviation_gain）[%/(km/h)]  "
        "※正=正しい向き（ずれを押し戻す）、負=逆向き（ずれが広がる。手順6 段2 の破綻の原因）"
    )
    print()
    print(deviation_gain_table(args.pkls, speeds, side))

    if args.freq is not None:
        for path, model, spec, _meta in loaded:
            print()
            print(f"## sinusoid_gain: {path.name}（{side}・{args.freq:g}Hz）[%/(km/h)]")
            print()
            header = ["v_mean km/h", "合計", "dv経路のみ", "dv_past経路のみ"]
            rows = []
            for v in speeds:
                total, dv, dv_past = sinusoid_gain(model, spec, v, args.freq)
                rows.append([f"{v:g}", f"{total:.3f}", f"{dv:.3f}", f"{dv_past:.3f}"])
            print(_md_table(header, rows))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
