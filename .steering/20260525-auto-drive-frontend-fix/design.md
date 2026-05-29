# 設計: 自動運転ページ フロントエンド修正

## 変更ファイル
- `src/web/static/js/screens/auto-drive.js` のみ

## 実装アプローチ

### A. タイマーと走行状態管理の刷新

**現行の問題:**
```javascript
// コンポーネントマウント時に即時開始 → 走行前から動く
useEffect(() => {
  timerRef.current = setInterval(() => setElapsed(s => s + 1), 1000);
  return () => clearInterval(timerRef.current);
}, []);
```

**修正後:**
- `driveStartTimeRef = useRef(null)`: 走行開始タイムスタンプ（Date.now()）
- `isDriving` state: 走行中かどうか
- `robotState` の変化を監視する useEffect でタイマー制御

```javascript
useEffect(() => {
  if (robotState !== 'RUNNING') {
    driveStartTimeRef.current = null;
    setIsDriving(false);
    return; // cleanup で interval が消える
  }
  if (!driveStartTimeRef.current) {
    driveStartTimeRef.current = Date.now();
    setElapsed(0);
    setIsDriving(true);
  }
  const id = setInterval(() => {
    if (driveStartTimeRef.current) {
      setElapsed(Math.floor((Date.now() - driveStartTimeRef.current) / 1000));
    }
  }, 1000);
  return () => clearInterval(id);
}, [robotState]);
```

### B. グラフデータを走行開始後のみに絞る

```javascript
const WINDOW = 300; // 30秒 at 100ms
const driveStart = driveStartTimeRef.current;
const recent = driveStart
  ? buf.filter(d => d.ts >= driveStart).slice(-WINDOW)
  : []; // 走行前は空 → グラフに線が描画されない
```

### C. 基準車速をプロファイルから計算

サーバーが `ref_speed_kmh=None` の場合、モードの `reference_speed` プロファイルを参照して計算:

```javascript
function getRefSpeedAtTime(elapsedS) {
  if (refProfile.length === 0 || elapsedS < 0) return null;
  let speed = refProfile[0].speed_kmh;
  for (const p of refProfile) {
    if (p.time_s <= elapsedS) speed = p.speed_kmh;
    else break;
  }
  return speed; // ステップ関数補間
}

const speedRef_pts = recent.map((d, i) => {
  const elapsedS = driveStart ? (d.ts - driveStart) / 1000 : 0;
  const refSpeed = d.ref_speed_kmh ?? getRefSpeedAtTime(elapsedS);
  return {
    x: toX(i, recent.length),
    y: PH1 - (Math.min(refSpeed ?? 0, maxSpeed) / maxSpeed) * (PH1 - 12),
  };
});
```

### D. マーカーと進捗表示の制御

```javascript
const progressFrac = isDriving && totalDurS > 0 ? Math.min(1, elapsed / totalDurS) : 0;

// マーカーは走行中のみ
{isDriving && totalDurS > 0 && (
  <>
    <line x1={markerX} ... />
    <text ... />
  </>
)}

// 進捗テキストは走行中のみ
{isDriving && totalDurS > 0 ? `${fmt(elapsed)} / ...` : '—'}
```

### E. ラベル更新

- 1軸目: `"直近 30 秒ウィンドウ"` (固定文字列)
- 2軸目: `"直近 30 秒ウィンドウ"` (固定文字列)

### F. 経過時間表示

走行中以外は "—" を表示:
```javascript
<Row cells={[['経過時間', '1.4fr'], [isDriving ? fmt(elapsed) : '—', '1fr', 'mono']]} />
```
