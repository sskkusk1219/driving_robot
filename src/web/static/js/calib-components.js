// ── JogKey ────────────────────────────────────────────────
// Square jog button with label and keyboard shortcut hint
function JogKey({ label, hint, onClick, disabled, size = 64 }) {
  const { INK, INK_SOFT, PAPER, PAPER_2, HATCH } = window;
  const style = {
    // 幅は size 以上で文字に合わせて広がる（「−0.01」が 48px に収まらない）。列の中では一番広いボタンにそろう
    minWidth: size, height: size, padding: '0 4px', boxSizing: 'border-box',
    display: 'flex', flexDirection: 'column',
    alignItems: 'center', justifyContent: 'center',
    border: `2px solid ${INK}`,
    borderRadius: 4,
    background: disabled ? HATCH : PAPER,
    cursor: disabled ? 'default' : 'pointer',
    userSelect: 'none',
    gap: 2,
    opacity: disabled ? 0.45 : 1,
    boxShadow: disabled ? 'none' : `3px 3px 0 ${INK}`,
    transition: 'box-shadow 0.1s, transform 0.1s',
  };
  const handleDown = e => {
    if (disabled) return;
    e.currentTarget.style.boxShadow = '1px 1px 0 ' + INK;
    e.currentTarget.style.transform = 'translate(2px,2px)';
  };
  const handleUp = e => {
    e.currentTarget.style.boxShadow = `3px 3px 0 ${INK}`;
    e.currentTarget.style.transform = '';
    if (!disabled && onClick) onClick();
  };
  return React.createElement('div', {
    style,
    onMouseDown: handleDown,
    onMouseUp: handleUp,
    onMouseLeave: e => { e.currentTarget.style.boxShadow = `3px 3px 0 ${INK}`; e.currentTarget.style.transform = ''; },
    onTouchStart: handleDown,
    onTouchEnd: handleUp,
  },
    React.createElement('span', { style: { fontSize: 20, fontWeight: 700, lineHeight: 1 } }, label),
    hint && React.createElement('span', { style: { fontSize: 10, color: INK_SOFT } }, hint),
  );
}

// ── PosRuler ──────────────────────────────────────────────
// SVG ruler showing ZERO (green) and FULL (red) markers plus current position triangle
function PosRuler({ zeroPos, fullPos, currentPos, maxPulse = 9500, label = '' }) {
  const { INK } = window;
  const W = 480, H = 56;
  const PL = 24, PR = 24;
  const trackW = W - PL - PR;

  const toX = pulse => PL + (pulse / maxPulse) * trackW;
  const zeroX  = toX(zeroPos  ?? 0);
  const fullX  = toX(fullPos  ?? maxPulse);
  const curX   = toX(Math.max(0, Math.min(maxPulse, currentPos ?? 0)));

  // triangle pointing down
  const tri = (x, color) => `M${x},${H - 14} L${x - 7},${H - 26} L${x + 7},${H - 26} Z`;

  const tickVals = [0, 2000, 4000, 6000, 8000, 9500];

  return React.createElement('svg', { viewBox: `0 0 ${W} ${H}`, width: '100%', height: H, style: { display: 'block', overflow: 'visible' } },
    // track line
    React.createElement('line', { x1: PL, y1: H - 14, x2: PL + trackW, y2: H - 14, stroke: INK, strokeWidth: 2 }),
    // tick marks at 0, 2000, 4000, 6000, 8000, 9500
    ...tickVals.map(v => {
      const x = PL + (v / maxPulse) * trackW;
      return React.createElement('g', { key: v },
        React.createElement('line', { x1: x, y1: H - 10, x2: x, y2: H - 18, stroke: INK, strokeWidth: 1.5 }),
        React.createElement('text', { x, y: H - 3, textAnchor: 'middle', fontSize: 9, fill: '#9a9080' }, v),
      );
    }),
    // ZERO marker
    zeroPos != null && React.createElement('g', null,
      React.createElement('path', { d: tri(zeroX, '#3aac3a'), fill: '#3aac3a' }),
      React.createElement('text', { x: zeroX, y: H - 30, textAnchor: 'middle', fontSize: 9, fill: '#3aac3a', fontWeight: 700 }, 'ZERO'),
    ),
    // FULL marker
    fullPos != null && React.createElement('g', null,
      React.createElement('path', { d: tri(fullX, '#e05050'), fill: '#e05050' }),
      React.createElement('text', { x: fullX, y: H - 30, textAnchor: 'middle', fontSize: 9, fill: '#e05050', fontWeight: 700 }, 'FULL'),
    ),
    // Current position (hollow triangle)
    currentPos != null && React.createElement('g', null,
      React.createElement('path', { d: tri(curX, INK), fill: 'none', stroke: INK, strokeWidth: 2 }),
      React.createElement('text', { x: curX, y: 14, textAnchor: 'middle', fontSize: 10, fill: INK }, currentPos),
    ),
    // label
    label && React.createElement('text', { x: PL, y: 12, fontSize: 11, fill: '#9a9080' }, label),
  );
}

// ── OpeningBar ────────────────────────────────────────────
// Horizontal bar showing current opening percentage
function OpeningBar({ value = 0, color = '#ddd4c4', label = '' }) {
  const { INK, PAPER, HATCH } = window;
  const pct = Math.max(0, Math.min(100, value));
  return React.createElement('div', { style: { width: '100%' } },
    React.createElement('div', { style: { display: 'flex', justifyContent: 'space-between', fontSize: 11, marginBottom: 2 } },
      React.createElement('span', null, label),
      React.createElement('span', { style: { fontWeight: 700 } }, `${pct.toFixed(1)}%`),
    ),
    React.createElement('div', {
      style: {
        width: '100%', height: 16,
        border: `1.5px solid ${INK}`,
        borderRadius: 2,
        background: PAPER,
        overflow: 'hidden',
      }
    },
      React.createElement('div', {
        style: {
          width: `${pct}%`, height: '100%',
          background: color,
          transition: 'width 0.1s',
        }
      }),
    ),
  );
}

// ── DragSlider ────────────────────────────────────────────
// Horizontal drag slider (left=decrease, right=increase).
// Optional zeroPos / fullPos show calibration markers.
function DragSlider({ currentPos, maxPulse = 9500, axisId, onJog, zeroPos, fullPos, disabled, displayValue, unit = 'pulse', minPos, maxPos }) {
  const { useRef } = React;
  const { INK, INK_SOFT, PAPER, HATCH } = window;

  const ref = useRef(null);
  const dragging = useRef(false);
  const lastX = useRef(0);
  const pending = useRef(0);
  const timer = useRef(null);

  function flush() {
    if (pending.current !== 0) { onJog(axisId, pending.current); pending.current = 0; }
    timer.current = null;
  }

  function onPointerDown(e) {
    if (disabled) return;
    e.currentTarget.setPointerCapture(e.pointerId);
    dragging.current = true;
    lastX.current = e.clientX;
  }

  function onPointerMove(e) {
    if (disabled || !dragging.current || !ref.current) return;
    const w = ref.current.getBoundingClientRect().width;
    if (w === 0) return;
    const dx = e.clientX - lastX.current;
    const step = Math.round((dx / w) * maxPulse);
    if (step !== 0) {
      pending.current += step;
      lastX.current = e.clientX;
      if (!timer.current) timer.current = setTimeout(flush, 80);
    }
  }

  function onPointerUp() { dragging.current = false; }

  const pct = v => Math.max(0, Math.min(100, (v / maxPulse) * 100));
  const fillPct = pct(currentPos);
  const ticks = [0, 0.25, 0.5, 0.75, 1];

  return (
    <div ref={ref} style={{
      flex: 1, minHeight: 0, position: 'relative',
      border: `1.4px solid ${INK}`, background: PAPER,
      cursor: disabled ? 'not-allowed' : 'ew-resize', userSelect: 'none', overflow: 'hidden',
      opacity: disabled ? 0.4 : 1,
    }}
    onPointerDown={onPointerDown} onPointerMove={onPointerMove}
    onPointerUp={onPointerUp} onPointerCancel={onPointerUp}
    >
      {/* Tick lines */}
      {ticks.map(f => (
        <div key={f} style={{ position: 'absolute', top: 0, bottom: 0, left: `${f * 100}%`, width: 1, background: HATCH, opacity: 0.6, pointerEvents: 'none' }} />
      ))}

      {/* Fill */}
      <div style={{ position: 'absolute', top: 0, bottom: 0, left: 0, width: `${fillPct}%`, background: INK, opacity: 0.10, pointerEvents: 'none' }} />

      {/* Handle line */}
      <div style={{ position: 'absolute', top: 0, bottom: 0, left: `${fillPct}%`, width: 3, background: INK, pointerEvents: 'none' }} />

      {/* ZERO marker */}
      {zeroPos != null && (
        <div style={{ position: 'absolute', top: 0, bottom: 0, left: `${pct(zeroPos)}%`, width: 2, background: '#3aac3a', pointerEvents: 'none' }}>
          <div style={{ position: 'absolute', top: 4, left: 4, fontSize: 9, color: '#3aac3a', fontWeight: 700, whiteSpace: 'nowrap' }}>ZERO</div>
        </div>
      )}

      {/* FULL marker */}
      {fullPos != null && (
        <div style={{ position: 'absolute', top: 0, bottom: 0, left: `${pct(fullPos)}%`, width: 2, background: '#e05050', pointerEvents: 'none' }}>
          <div style={{ position: 'absolute', top: 4, left: 4, fontSize: 9, color: '#e05050', fontWeight: 700, whiteSpace: 'nowrap' }}>FULL</div>
        </div>
      )}

      {/* min（ペダルに触れた位置）の印。キャリブレーション画面だけが渡す */}
      {minPos != null && (
        <div style={{ position: 'absolute', top: 0, bottom: 0, left: `${pct(minPos)}%`, width: 2, background: '#c8922a', pointerEvents: 'none' }}>
          <div style={{ position: 'absolute', bottom: 4, left: 4, fontSize: 9, color: '#c8922a', fontWeight: 700, whiteSpace: 'nowrap' }}>保存済み min</div>
        </div>
      )}

      {/* max（開度上限）の印 */}
      {maxPos != null && (
        <div style={{ position: 'absolute', top: 0, bottom: 0, left: `${pct(maxPos)}%`, width: 2, background: '#e0607a', pointerEvents: 'none' }}>
          <div style={{ position: 'absolute', bottom: 4, right: 4, fontSize: 9, color: '#e0607a', fontWeight: 700, whiteSpace: 'nowrap' }}>保存済み max</div>
        </div>
      )}

      {/* Current value */}
      <div style={{ position: 'absolute', inset: 0, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 2, pointerEvents: 'none' }}>
        <div style={{ fontSize: 26, fontFamily: 'inherit', fontWeight: 700, lineHeight: 1 }}>{displayValue ?? currentPos ?? '—'}</div>
        <div style={{ fontSize: 10, color: INK_SOFT }}>{unit}</div>
      </div>

      {/* Scale labels */}
      <div style={{ position: 'absolute', top: 4, left: 6, fontSize: 12, color: INK_SOFT, pointerEvents: 'none' }}>0</div>
      <div style={{ position: 'absolute', top: 4, right: 6, fontSize: 12, color: INK_SOFT, pointerEvents: 'none' }}>{maxPulse}</div>
    </div>
  );
}

// ── useJogKeys ────────────────────────────────────────────
// キャリブレーション・手動運転共通のキー操作。
//   キーボードの上の段ほど小さい量（ボタンの並びと同じ。左手の左列が −・右列が +）:
//   3 / 2 = +1 / −1 pulse（0.01%）、e / w = ±10（0.1%）、d / s = ±100（1%）、c / x = ±1000（10%）、Tab = 軸切替、Enter = onEnter(axis)（任意）
// 入力欄（input/textarea/select）と修飾キー付きは無視。応答待ちの間のキーリピートは捨てる。
// enabled が false の間は何もしない。onJog(axisId, step) / onEnter(axisId) は Promise を返せる。
function useJogKeys({ enabled, selectedAxis, setSelectedAxis, onJog, onEnter }) {
  const { useEffect, useRef } = React;
  const busyRef = useRef(false);
  const latest = useRef({});
  latest.current = { enabled, selectedAxis, onJog, onEnter };

  useEffect(() => {
    async function run(fn) {
      if (busyRef.current) return;
      busyRef.current = true;
      try { await fn(); } finally { busyRef.current = false; }
    }
    function onKeyDown(e) {
      const st = latest.current;
      if (!st.enabled) return;
      const tag = (e.target && e.target.tagName) || '';
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
      if (e.ctrlKey || e.metaKey || e.altKey) return;
      const axis = st.selectedAxis;
      const steps = { 3: 1, 2: -1, e: 10, w: -10, d: 100, s: -100, c: 1000, x: -1000 };
      const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
      if (k in steps) {
        e.preventDefault();
        run(() => st.onJog(axis, steps[k]));
      } else if (k === 'Enter' && st.onEnter) {
        e.preventDefault();
        run(() => st.onEnter(axis));
      } else if (k === 'Tab') {
        e.preventDefault();
        setSelectedAxis(a => (a === 'brake' ? 'accel' : 'brake'));
      }
    }
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, []);
}

// ── AxisPanel ─────────────────────────────────────────────
// 軸パネルの外枠。クリックで軸を選び、選択中は橙の枠と「（キー操作中）」を表示する。
// 共有 Box は onClick を受け取らないので外側の div の onMouseDown で拾う。
function AxisPanel({ label, axisId, selected, enabled = true, onSelect, children }) {
  const { Box } = window;
  const on = selected && enabled;
  return (
    <div onMouseDown={() => onSelect && onSelect(axisId)} style={{ display: 'flex', flex: 1, minHeight: 0, minWidth: 0 }}>
      <Box label={on ? `${label}（キー操作中）` : label} style={{
        padding: '10px 12px', display: 'flex', flexDirection: 'column', gap: 8, flex: 1, minHeight: 0,
        ...(on ? { border: '2px solid #c8922a' } : {}),
      }}>
        {children}
      </Box>
    </div>
  );
}

// ── JogRow ────────────────────────────────────────────────
// −1/−10/−100/−1000 ボタン（左列）+ ドラッグスライダー + +1/+10/+100/+1000 ボタン（右列）。小さい量が上。
// ヒント文字は 2/w/s/x（−）と 3/e/d/c（+）＝キーボードの上の段から。4 段になるので JogKey は 48px。
// 表示はキャリブレーション・手動運転とも開度 %（キリの良い値。実際は 9500 pulse = 100% なので 10 は 10.53%）。
// displayValue / unit / minPos はスライダーの中央表示と min の印（キャリブレーション画面用）。
const JOG_STEPS = [1, 10, 100, 1000];
const JOG_HINTS_NEG = ['2', 'w', 's', 'x'];
const JOG_HINTS_POS = ['3', 'e', 'd', 'c'];
function JogRow({ axisId, currentPos, onJog, zeroPos, fullPos, disabled, labels = ['0.01', '0.1', '1', '10'], displayValue, unit, minPos, maxPos }) {
  const col = { display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: 6 };
  return (
    <div style={{ flex: 1, minHeight: 0, display: 'flex', gap: 8 }}>
      <div style={col}>
        {JOG_STEPS.map((st, i) => (
          <JogKey key={st} size={48} label={`−${labels[i]}`} hint={JOG_HINTS_NEG[i]} onClick={() => onJog(axisId, -st)} disabled={disabled} />
        ))}
      </div>
      <DragSlider currentPos={currentPos} axisId={axisId} onJog={onJog} zeroPos={zeroPos} fullPos={fullPos} disabled={disabled}
                  displayValue={displayValue} unit={unit} minPos={minPos} maxPos={maxPos} />
      <div style={col}>
        {JOG_STEPS.map((st, i) => (
          <JogKey key={st} size={48} label={`+${labels[i]}`} hint={JOG_HINTS_POS[i]} onClick={() => onJog(axisId, st)} disabled={disabled} />
        ))}
      </div>
    </div>
  );
}

Object.assign(window, { JogKey, PosRuler, OpeningBar, DragSlider, useJogKeys, AxisPanel, JogRow });
