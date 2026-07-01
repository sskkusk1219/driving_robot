// ── JogKey ────────────────────────────────────────────────
// Square jog button with label and keyboard shortcut hint
function JogKey({ label, hint, onClick, disabled }) {
  const { INK, INK_SOFT, PAPER, PAPER_2, HATCH } = window;
  const style = {
    width: 64, height: 64,
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
function DragSlider({ currentPos, maxPulse = 9500, axisId, onJog, zeroPos, fullPos, disabled }) {
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

      {/* Current value */}
      <div style={{ position: 'absolute', inset: 0, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 2, pointerEvents: 'none' }}>
        <div style={{ fontSize: 26, fontFamily: 'inherit', fontWeight: 700, lineHeight: 1 }}>{currentPos}</div>
        <div style={{ fontSize: 10, color: INK_SOFT }}>pulse</div>
        <div style={{ fontSize: 10, color: INK_SOFT, marginTop: 4 }}>← ドラッグ →</div>
      </div>

      {/* Scale labels */}
      <div style={{ position: 'absolute', top: 3, left: 5, fontSize: 9, color: INK_SOFT, pointerEvents: 'none' }}>0</div>
      <div style={{ position: 'absolute', top: 3, right: 5, fontSize: 9, color: INK_SOFT, pointerEvents: 'none' }}>{maxPulse}</div>
    </div>
  );
}

Object.assign(window, { JogKey, PosRuler, OpeningBar, DragSlider });
