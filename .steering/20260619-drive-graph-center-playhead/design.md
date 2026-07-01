# 設計: 走行グラフの中央固定プレイヘッド

## 対象ファイル

`src/web/static/js/screens/auto-drive.js` 内の共有コンポーネント `DriveMonitorScreen`。
学習運転(`learning.js`)・自動運転(`auto-drive.js` 末尾)が props 違いで再利用しており、
この1ファイルの変更で両ページに反映される。

グラフは **React + SVG** 描画（p5.js やキャンバスではない）。現在位置のX座標は
スライディングウィンドウのマッピング式だけで決まるため、その式の変更が中心。

## 1. 中央固定プレイヘッドのウィンドウ式

現在時刻 `nowS`（RUNNING 中のみ requestAnimationFrame で進む既存ロジック）を
常にグラフ中央 `frac=0.5` に置く。クランプはしない（now を常に中央に保つため）。

```js
const WINDOW_S = 30;
const HALF_S = WINDOW_S / 2;
const windowStartS = nowS - HALF_S;   // 走行前 nowS=0 → [-15, +15]
const windowEndS   = nowS + HALF_S;
```

- 基準車速サンプリングは既存式 `t = windowStartS + frac*WINDOW_S` のまま。
  `getRefSpeedAtTime` は `t<0` で null→0 を返すため、右半分に未来の基準が自然に描画される。

## 2. 実測・開度の点はクランプ→フィルタ

中央固定だと左半分が15秒しかなく、`recent`(最大300サンプル≒30秒) の古い点が
左端 frac=0 に山積みされて横線アーティファクトになる。クランプをやめ、
画面外(frac<0)の点を除外する（最新点は elapsedS≤nowS なので frac≤0.5、上限クランプ不要）。

```js
const toFrac = elapsedS => (elapsedS - windowStartS) / WINDOW_S;
const onScreen = (d, key, max, h) => {
  const frac = toFrac((d.ts - driveStart) / 1000);
  return frac >= 0 && frac <= 1 ? { x: toXFull(frac), y: toY(d[key], max, h) } : null;
};
// speedAct_pts / accelPts / brakePts は recent.map(onScreen(...)).filter(Boolean)
```

## 3. 現在位置はポインタ(●)、中央固定

赤色点線の縦線は廃止。中央 `toXFull(0.5)` 固定で、現在のセンサ値 `realtimeData`(`rd`) を
y にとる ● を描く。走行前(0)でも表示され、走行中は中央で上下にのみ動く。

```js
function marker(val, valMax, height, color) {
  return <circle cx={toXFull(0.5)} cy={toY(val, valMax, height)} r="4.5" fill={color} stroke={PAPER} strokeWidth="1.5" />;
}
```

- 1軸: `marker(rd.actual_speed_kmh, maxSpeed, PH1, '#c8922a')`
- 2軸: `marker(rd.accel_opening, 100, PH2, '#78c8f0')`, `marker(rd.brake_opening, 100, PH2, '#f07070')`
- 軌跡(polyline)はそのまま線で描画し、その先端にポインタが乗る。

**ポイント**: ポインタは軌跡の最終点ではなく `realtimeData` の現在値を使う。
これにより走行前でも常に表示され、「中央固定・上下のみ移動」が成立する。

## 4. 時間軸ラベルの負値ガード

`windowStartS` が負になり得るため、ティック生成の起点を 0 でクランプ。

```js
const first = Math.max(0, Math.ceil(windowStartS / 5) * 5);
```

## 配色・定数

既存のテーマ定数（`sketch.js` 定義、`window` 経由でグローバル）を流用:
`PAPER`(背景 #141210, ポインタ縁取り), 金 `#c8922a`(実測), 青 `#78c8f0`(アクセル), 赤 `#f07070`(ブレーキ)。
