// ── JogKey ────────────────────────────────────────────────
// Square jog button with label and keyboard shortcut hint
function JogKey({ label, hint, onClick, disabled }) {
  const { INK, PAPER, PAPER_2, HATCH } = window;
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
    hint && React.createElement('span', { style: { fontSize: 10, color: '#888' } }, hint),
  );
}

// ── PosRuler ──────────────────────────────────────────────
// SVG ruler showing ZERO (green) and FULL (red) markers plus current position triangle
function PosRuler({ zeroPos, fullPos, currentPos, maxPulse = 5000, label = '' }) {
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

  return React.createElement('svg', { viewBox: `0 0 ${W} ${H}`, width: '100%', height: H, style: { display: 'block', overflow: 'visible' } },
    // track line
    React.createElement('line', { x1: PL, y1: H - 14, x2: PL + trackW, y2: H - 14, stroke: INK, strokeWidth: 2 }),
    // tick marks every 1000
    ...[0,1,2,3,4,5].map(i => {
      const x = PL + (i / 5) * trackW;
      return React.createElement('g', { key: i },
        React.createElement('line', { x1: x, y1: H - 10, x2: x, y2: H - 18, stroke: INK, strokeWidth: 1.5 }),
        React.createElement('text', { x, y: H - 3, textAnchor: 'middle', fontSize: 9, fill: '#666' }, i * 1000),
      );
    }),
    // ZERO marker
    zeroPos != null && React.createElement('g', null,
      React.createElement('path', { d: tri(zeroX, '#2a7a2a'), fill: '#2a7a2a' }),
      React.createElement('text', { x: zeroX, y: H - 30, textAnchor: 'middle', fontSize: 9, fill: '#2a7a2a', fontWeight: 700 }, 'ZERO'),
    ),
    // FULL marker
    fullPos != null && React.createElement('g', null,
      React.createElement('path', { d: tri(fullX, '#a23232'), fill: '#a23232' }),
      React.createElement('text', { x: fullX, y: H - 30, textAnchor: 'middle', fontSize: 9, fill: '#a23232', fontWeight: 700 }, 'FULL'),
    ),
    // Current position (hollow triangle)
    currentPos != null && React.createElement('g', null,
      React.createElement('path', { d: tri(curX, INK), fill: 'none', stroke: INK, strokeWidth: 2 }),
      React.createElement('text', { x: curX, y: 14, textAnchor: 'middle', fontSize: 10, fill: INK }, currentPos),
    ),
    // label
    label && React.createElement('text', { x: PL, y: 12, fontSize: 11, fill: '#555' }, label),
  );
}

// ── OpeningBar ────────────────────────────────────────────
// Horizontal bar showing current opening percentage
function OpeningBar({ value = 0, color = '#1f1f1f', label = '' }) {
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

Object.assign(window, { JogKey, PosRuler, OpeningBar });
