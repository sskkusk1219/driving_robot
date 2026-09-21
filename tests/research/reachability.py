"""段3: 到達可能性判定（多点先読み。ProblemReport_20260916 課題#1・#3）。

段2 までの `predict_effort` は、レジーム（惰行／アクセル／ブレーキ）の判定を先読み
ホライズン 1.0s の 1 点だけで行っていた。これには 2 つの問題がある。

    (a) 惰行の加速度 `free_accel_at(v0)` を 1 秒間一定とみなしている。段2.5 で
        `free_accel_at` は速度に強く依存する形（4.793 km/h で 0.00、5.293 で
        −1.74、10.293 で −3.15）になったため、この近似の誤差がそのまま効く。
    (b) 1.0s しか見ていないため、0.5s 先では足りているが 2s 先では足りない
        （またはその逆）場面で 1 周期ごとに判断が反転する。

この 2 つを直すのが本モジュール。`free_accel_at` を刻み `step_s` の前進オイラー法で
数値積分し、惰行のまま進んだ先の速度 `v_free(t+h)` を作る（`free_speeds_at`）。1 秒
一定の近似をやめることで (a) を直し、複数ホライズンで判定することで (b) を直す。

なぜ数値積分が必要か: `free_accel_at` は速度に応じて符号も大きさも変わる（クリープ域は
正、惰行域は負、クリープ平衡速度ではちょうど 0）。区分的に一定と近似できる区間が狭いため、
解析的な式ではなく刻みの細かい数値積分でしか軌跡を追えない。クリープ平衡速度（クリープ
だけで到達して静止する速度）では `free_accel_at` が 0 になるので、積分はその速度へ収束
して止まる。これは実車がクリープ平衡速度（4.99 km/h 付近）に張り付く挙動と構造的に一致
する（段2.5 の実測。ProblemReport_20260916 段2.5 の Context 参照）。
"""

from __future__ import annotations

from collections.abc import Sequence

from src.models.profile import FeedforwardParams
from tests.research.ff_params import ResearchFFParams, free_accel_at

__all__ = ["decide_regime", "free_speeds_at", "reach_needs"]


def free_speeds_at(
    params: FeedforwardParams,
    research: ResearchFFParams,
    v0: float,
    horizons_s: Sequence[float],
    *,
    step_s: float,
) -> tuple[float, ...]:
    """v0 から両ペダルを離したまま `horizons_s` 先まで進めた速度 `v_free(t+h)`。

    前進オイラー法 `v ← v + free_accel_at(params, research, v) * dt` を刻み `step_s` で
    繰り返す。`horizons_s`（昇順が前提）と同じ順・同じ長さのタプルを返す。1 回の掃引で
    全ホライズンを記録するため、ホライズンをまたぐたびに `v`・経過時間を積み上げていく
    （前のホライズンからやり直さない）。刻みの倍数でないホライズンでも、最後の 1 ステップ
    だけそのホライズンまでの端数 dt に縮めて進めるため正確な値になる。速度は 0 未満に
    クランプする（惰行で停止した後、逆走はしない）。

    Args:
        step_s: 積分刻み [s]。0 以下は誤り。

    Raises:
        ValueError: step_s が 0 以下のとき。
    """
    if step_s <= 0.0:
        raise ValueError(f"step_s は正値である必要があります: {step_s}")
    if not horizons_s:
        return ()

    results: list[float] = []
    t = 0.0
    v = v0
    for h in horizons_s:
        while True:
            remaining = h - t
            if remaining <= 1e-12:
                break
            dt = min(step_s, remaining)
            v = max(0.0, v + free_accel_at(params, research, v) * dt)
            t += dt
        results.append(v)
    return tuple(results)


def reach_needs(
    future_speeds: Sequence[float],
    free_speeds: Sequence[float],
    horizons_s: Sequence[float],
) -> tuple[float, ...]:
    """各ホライズンで「惰行だけでは足りない平均加速度」`need(h)` [km/h/s]。

    `need(h) = (future_speeds[i] − free_speeds[i]) / horizons_s[i]`。正なら基準に届く
    にはアクセル側が要る（惰行では加速が足りない）、負ならブレーキ側が要る。

    Raises:
        ValueError: 3 つの長さが一致しないとき。
    """
    n = len(horizons_s)
    if len(future_speeds) != n or len(free_speeds) != n:
        raise ValueError(
            "future_speeds / free_speeds / horizons_s の長さが一致しません: "
            f"{len(future_speeds)} / {len(free_speeds)} / {n}"
        )
    return tuple(
        (f - fr) / h for f, fr, h in zip(future_speeds, free_speeds, horizons_s, strict=True)
    )


def decide_regime(needs: Sequence[float], band_kmhs: float) -> int | None:
    """帯（半幅 `band_kmhs`）の外に出た最も近いホライズンの添字。全部帯の中なら None（惰行）。

    判定は `abs(needs[i]) >= band_kmhs`。既存の帯判定（`abs(desired_accel - coast) <
    coast_band_kmhs` が「帯の中」）の厳密な補集合になるようにしている（境界を二重に
    数えたり、逆に漏らしたりしない）。

    `needs` は昇順ホライズンに対応しているので、先頭（最短ホライズン）から走査して最初に
    帯の外に出た添字を返す。符号が違うホライズンが複数あるとき（基準の山・谷の直前で、
    近いホライズンは片方のペダルが要り、遠いホライズンは逆側が要る場面）は、最短のものを
    採る。理由: 遠い側の不足・過剰は次の周期以降でまだ取り返せるが、近い側はそのホライズン
    に到達するまでの猶予がほとんど無く、今すぐ手当てしないと取り返せないため。
    """
    for i, n in enumerate(needs):
        if abs(n) >= band_kmhs:
            return i
    return None
