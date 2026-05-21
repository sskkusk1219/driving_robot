// sketch.js — UI プリミティブ
// ワイヤーフレームの sketch.jsx をベースに、SPA 用に調整

const INK       = '#1f1f1f';
const INK_SOFT  = '#5a5a5a';
const INK_MUTE  = '#9a9a9a';
const PAPER     = '#fafaf7';
const PAPER_2   = '#f1efe9';
const HATCH     = '#d8d6d0';

const STATE_TINT = {
  STANDBY:      { stroke: '#7a7a7a', fill: '#e9e7e1', text: '#2a2a2a' },
  BOOTING:      { stroke: '#7a5c1c', fill: '#faf3dc', text: '#5a3e0a' },
  ERROR:        { stroke: '#a23232', fill: '#f1d8d2', text: '#5e1414' },
  INITIALIZING: { stroke: '#2f5780', fill: '#dde6f0', text: '#1a3859' },
  READY:        { stroke: '#3f6b3f', fill: '#dfeadc', text: '#22421f' },
  RUNNING:      { stroke: '#2f5780', fill: '#dde6f0', text: '#1a3859' },
  EMERGENCY:    { stroke: '#a23232', fill: '#f1d8d2', text: '#5e1414' },
  AC_LOSS:      { stroke: '#a23232', fill: '#f1d8d2', text: '#5e1414' },
};

const NAV = [
  ['init',       '初期化'],
  ['profiles',   '車両プロファイル'],
  ['calibration','キャリブレーション'],
  ['modes',      '走行モード'],
  ['learning',   '学習運転'],
  ['auto',       '自動運転'],
  ['manual',     '手動運転'],
  ['schedule',   'タイムスケジュール'],
  ['sequence',   'シーケンス'],
  ['logs',       'ログ'],
];

// ── Toast notification ───────────────────────────────────────
function showToast(msg, type = 'info', duration = 3000) {
  const container = document.getElementById('toast-container');
  if (!container) return;
  const el = document.createElement('div');
  el.className = `toast${type === 'error' ? ' error' : type === 'success' ? ' success' : ''}`;
  el.textContent = msg;
  container.appendChild(el);
  setTimeout(() => {
    el.style.opacity = '0';
    el.style.transition = 'opacity 0.3s';
    setTimeout(() => el.remove(), 300);
  }, duration);
}
window.showToast = showToast;

// ── EmergencyOverlay ─────────────────────────────────────────
function EmergencyOverlay({ onGoInit }) {
  return (
    <div style={{
      position: 'absolute', inset: 0, zIndex: 999,
      background: 'rgba(90,10,10,0.55)',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    }}>
      <div style={{
        background: '#fff8f6', border: '3px solid #a23232',
        borderRadius: 8, padding: '36px 52px',
        textAlign: 'center', boxShadow: '4px 4px 0 rgba(0,0,0,0.25)',
        transform: 'rotate(-0.5deg)', maxWidth: 460,
        fontFamily: "'Patrick Hand', cursive",
      }}>
        <div style={{ fontSize: 52, lineHeight: 1, marginBottom: 12, color: '#a23232' }}>⚠</div>
        <div style={{ fontSize: 28, fontWeight: 800, color: '#5e1414', letterSpacing: 1, marginBottom: 8 }}>
          非常停止スイッチが押されています
        </div>
        <div style={{ fontSize: 16, color: '#8a3030', lineHeight: 1.6, marginBottom: 24 }}>
          スイッチを解除してから「初期化」画面で<br />
          アラームリセット → サーボON を実施してください。
        </div>
        <div onClick={onGoInit} style={{
          display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
          border: '2px solid #a23232', borderRadius: 6,
          padding: '9px 28px', background: '#f1d8d2', color: '#5e1414',
          fontWeight: 700, fontSize: 16, fontFamily: 'inherit',
          boxShadow: '2px 2px 0 rgba(0,0,0,0.15)', cursor: 'pointer',
        }}>初期化画面へ</div>
      </div>
    </div>
  );
}

// ── UpsIndicator ─────────────────────────────────────────────
function UpsIndicator({ acLoss = false }) {
  return (
    <div style={{
      display: 'flex', alignItems: 'center', gap: 5,
      border: `1.2px solid ${acLoss ? '#a23232' : INK_MUTE}`,
      borderRadius: 12, padding: '2px 10px',
      background: acLoss ? '#f1d8d2' : 'transparent', fontSize: 13,
    }}>
      <span style={{ fontSize: 14, color: acLoss ? '#a23232' : '#3f6b3f' }}>⚡</span>
      <span style={{ color: acLoss ? '#5e1414' : INK_SOFT, fontWeight: acLoss ? 700 : 400 }}>
        UPS {acLoss ? 'AC断！' : 'AC OK'}
      </span>
    </div>
  );
}

// ── StateBadge ───────────────────────────────────────────────
function StateBadge({ state }) {
  const t = STATE_TINT[state] || STATE_TINT.READY;
  return (
    <div style={{
      border: `1.5px solid ${t.stroke}`, background: t.fill, color: t.text,
      padding: '3px 12px', borderRadius: 14, fontSize: 16,
      fontWeight: 700, letterSpacing: 1, transform: 'rotate(-0.4deg)',
    }}>● {state}</div>
  );
}

// ── TopBar ───────────────────────────────────────────────────
function TopBar({ state, screen, upsLoss, profileName, modeName }) {
  const tint = STATE_TINT[state] || STATE_TINT.READY;
  const showUpsLoss = upsLoss || state === 'AC_LOSS';
  return (
    <div style={{
      gridColumn: '1 / -1',
      borderBottom: `1.5px solid ${INK}`,
      display: 'flex', alignItems: 'center',
      padding: '0 22px', gap: 14,
      background: PAPER,
    }}>
      <div style={{ fontSize: 22, fontWeight: 700, letterSpacing: 0.5 }}>Driving_Robot</div>
      <div style={{ width: 1, height: 22, background: INK_MUTE }} />
      <div style={{ fontSize: 17, color: INK_SOFT }}>{screen}</div>
      <div style={{ flex: 1 }} />
      <UpsIndicator acLoss={showUpsLoss} />
      <StateBadge state={state} />
      {profileName && (
        <div style={{ fontSize: 15, color: INK_SOFT }}>profile: <b>{profileName}</b></div>
      )}
      {modeName && (
        <div style={{ fontSize: 15, color: INK_SOFT }}>mode: <b>{modeName}</b></div>
      )}
    </div>
  );
}

// ── Sidebar ──────────────────────────────────────────────────
function Sidebar({ active, onNav }) {
  return (
    <div style={{
      borderRight: `1.5px solid ${INK}`,
      padding: '18px 0',
      background: PAPER_2,
      display: 'flex', flexDirection: 'column', gap: 2,
      fontSize: 17, overflowY: 'auto',
    }}>
      {NAV.map(([k, label]) => {
        const on = k === active;
        return (
          <div key={k} onClick={() => onNav(k)} style={{
            padding: '7px 18px',
            borderLeft: on ? `4px solid ${INK}` : '4px solid transparent',
            background: on ? PAPER : 'transparent',
            fontWeight: on ? 700 : 400,
            color: on ? INK : INK_SOFT,
            cursor: 'pointer',
            userSelect: 'none',
            whiteSpace: 'nowrap',
          }}>{label}</div>
        );
      })}
      <div style={{ flex: 1 }} />
    </div>
  );
}

// ── Frame ────────────────────────────────────────────────────
function Frame({ children, state = 'READY', screen = '', activeNav = '',
                 upsLoss = false, profileName, modeName, onNav, onGoInit }) {
  return (
    <div style={{
      width: '100%', height: '100%',
      background: PAPER,
      fontFamily: "'Patrick Hand', 'Kalam', 'Comic Sans MS', cursive",
      color: INK,
      display: 'grid',
      gridTemplateColumns: '200px 1fr',
      gridTemplateRows: '52px 1fr',
      position: 'relative',
      overflow: 'hidden',
    }}>
      <TopBar state={state} screen={screen} upsLoss={upsLoss}
              profileName={profileName} modeName={modeName} />
      <Sidebar active={activeNav} onNav={onNav || (() => {})} />
      <div style={{ padding: '20px 28px 24px', overflow: 'hidden', display: 'flex', flexDirection: 'column' }}>
        {children}
      </div>
      {state === 'EMERGENCY' && <EmergencyOverlay onGoInit={onGoInit || (() => {})} />}
    </div>
  );
}

// ── Box ──────────────────────────────────────────────────────
function Box({ children, style = {}, label, dashed, thick, tint }) {
  return (
    <div style={{
      border: `${thick ? 2 : 1.4}px ${dashed ? 'dashed' : 'solid'} ${INK}`,
      background: tint || 'transparent',
      position: 'relative',
      borderRadius: 4,
      ...style,
    }}>
      {label && (
        <div style={{
          position: 'absolute', top: -10, left: 14,
          background: PAPER, fontSize: 13, color: INK_SOFT, padding: '0px 6px',
        }}>{label}</div>
      )}
      {children}
    </div>
  );
}

// ── Btn ──────────────────────────────────────────────────────
function Btn({ children, primary, danger, big, style = {}, ghost, onClick, disabled }) {
  const base = {
    display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
    border: `${big ? 2 : 1.5}px solid ${danger ? '#a23232' : INK}`,
    borderRadius: 6,
    padding: big ? '10px 22px' : '6px 14px',
    background: disabled ? PAPER_2 : ghost ? 'transparent' : primary ? INK : danger ? '#f1d8d2' : PAPER,
    color: disabled ? INK_MUTE : ghost ? INK : primary ? PAPER : danger ? '#5e1414' : INK,
    fontFamily: 'inherit',
    fontWeight: primary || danger ? 700 : 500,
    fontSize: big ? 19 : 15,
    boxShadow: primary && !disabled ? '2px 2px 0 rgba(0,0,0,0.15)' : 'none',
    cursor: disabled ? 'not-allowed' : 'pointer',
    opacity: disabled ? 0.6 : 1,
    whiteSpace: 'nowrap',
    flexShrink: 0,
  };
  return (
    <div onClick={disabled ? undefined : onClick} style={{ ...base, ...style }}>
      {children}
    </div>
  );
}

// ── Pill ─────────────────────────────────────────────────────
function Pill({ children, accent, style = {} }) {
  const t = accent ? STATE_TINT[accent] : null;
  return (
    <span style={{
      display: 'inline-block',
      border: `1.2px solid ${t ? t.stroke : INK}`,
      background: t ? t.fill : 'transparent',
      color: t ? t.text : INK,
      padding: '2px 10px', borderRadius: 12, fontSize: 14,
      ...style,
    }}>{children}</span>
  );
}

// ── Note ─────────────────────────────────────────────────────
function Note({ children, style = {} }) {
  return (
    <div style={{
      fontSize: 13, color: '#7a5c1c',
      background: '#faf3dc', border: `1px dashed #c8a94a`,
      padding: '6px 10px', borderRadius: 4,
      ...style,
    }}>※ {children}</div>
  );
}

// ── H2 ───────────────────────────────────────────────────────
function H2({ children, sub }) {
  return (
    <div style={{ marginBottom: 14 }}>
      <div style={{ fontSize: 22, fontWeight: 700 }}>{children}</div>
      {sub && <div style={{ fontSize: 14, color: INK_SOFT, marginTop: -2 }}>{sub}</div>}
    </div>
  );
}

// ── Input ────────────────────────────────────────────────────
function Input({ label, value, defaultValue, width = 200, mono, type = 'text',
                 min, max, step, onChange, placeholder, required, style = {} }) {
  const inputStyle = {
    width: '100%', padding: '5px 10px',
    border: `1.3px solid ${INK}`, borderRadius: 3,
    fontFamily: mono ? 'monospace' : "'Patrick Hand', cursive",
    fontSize: 15, background: PAPER,
    outline: 'none',
  };
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 3, width, ...style }}>
      {label && <div style={{ fontSize: 13, color: INK_SOFT }}>{label}</div>}
      <input
        type={type}
        defaultValue={defaultValue}
        value={value}
        min={min} max={max} step={step}
        onChange={onChange ? e => onChange(e.target.value) : undefined}
        placeholder={placeholder}
        required={required}
        style={inputStyle}
      />
    </div>
  );
}

// ── Row ──────────────────────────────────────────────────────
function Row({ cells, header, style = {} }) {
  return (
    <div style={{
      display: 'grid',
      gridTemplateColumns: cells.map(c => c[1] || '1fr').join(' '),
      borderBottom: `1px ${header ? 'solid' : 'dashed'} ${header ? INK : INK_MUTE}`,
      padding: '6px 8px',
      fontSize: 14,
      fontWeight: header ? 700 : 400,
      ...style,
    }}>
      {cells.map((c, i) => (
        <div key={i} style={{ fontFamily: c[2] === 'mono' ? 'monospace' : 'inherit' }}>
          {c[0]}
        </div>
      ))}
    </div>
  );
}

// ── Squig ────────────────────────────────────────────────────
function Squig({ width = '100%', style = {} }) {
  return (
    <svg width={width} height="10" viewBox="0 0 100 10" preserveAspectRatio="none"
         style={{ display: 'block', ...style }}>
      <path d="M 0 5 Q 5 0 10 5 T 20 5 T 30 5 T 40 5 T 50 5 T 60 5 T 70 5 T 80 5 T 90 5 T 100 5"
            fill="none" stroke={INK_SOFT} strokeWidth="1.2" />
    </svg>
  );
}

// ── Hatch ────────────────────────────────────────────────────
function Hatch({ width, height, style = {}, label }) {
  return (
    <div style={{ position: 'relative', width, height, ...style }}>
      <svg width="100%" height="100%" style={{ position: 'absolute', inset: 0 }}>
        <defs>
          <pattern id="h45" width="8" height="8" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
            <line x1="0" y1="0" x2="0" y2="8" stroke={HATCH} strokeWidth="1" />
          </pattern>
        </defs>
        <rect width="100%" height="100%" fill="url(#h45)" />
        <rect width="100%" height="100%" fill="none" stroke={INK} strokeWidth="1.4" strokeDasharray="3 2" />
      </svg>
      {label && (
        <div style={{
          position: 'absolute', inset: 0, display: 'flex',
          alignItems: 'center', justifyContent: 'center',
          fontSize: 13, color: INK_SOFT, fontFamily: 'monospace',
          textAlign: 'center', padding: 8,
        }}>{label}</div>
      )}
    </div>
  );
}

// ── SpeedGraph ───────────────────────────────────────────────
function SpeedGraph({ width = 600, height = 220, refPts, actPts, style = {} }) {
  const W = 600, H = 220;
  const padL = 38, padR = 12, padT = 14, padB = 24;
  const pw = W - padL - padR, ph = H - padT - padB;

  const ref = refPts || (() => {
    const pts = [];
    const N = 120;
    for (let i = 0; i <= N; i++) {
      const t = i / N;
      const v = 60 + 50 * Math.sin(t * Math.PI * 3.2) * (0.5 + 0.5 * Math.sin(t * Math.PI * 1.3)) +
                15 * Math.sin(t * Math.PI * 7);
      pts.push(Math.max(0, Math.min(120, v + 30)));
    }
    return pts;
  })();

  const act = actPts || ref.map((v, i) => {
    const lag = i > 0 ? ref[i - 1] : v;
    return v * 0.7 + lag * 0.3 + Math.sin(i * 0.7) * 1.6;
  });

  const N = ref.length - 1;
  const xAt = (i) => padL + (i / N) * pw;
  const yAt = (v) => padT + (1 - v / 130) * ph;

  const refPath = ref.map((v, i) => `${i === 0 ? 'M' : 'L'}${xAt(i).toFixed(1)},${yAt(v).toFixed(1)}`).join(' ');
  const actPath = act.map((v, i) => `${i === 0 ? 'M' : 'L'}${xAt(i).toFixed(1)},${yAt(v).toFixed(1)}`).join(' ');

  return (
    <svg viewBox={`0 0 ${W} ${H}`} width={width} height={height}
         style={{ ...style, display: 'block' }}>
      <rect x={padL} y={padT} width={pw} height={ph} fill="none" stroke={INK} strokeWidth="1.2" />
      {[0, 40, 80, 120].map(v => (
        <g key={v}>
          <line x1={padL - 4} y1={yAt(v)} x2={padL} y2={yAt(v)} stroke={INK} strokeWidth="1" />
          <text x={padL - 6} y={yAt(v) + 4} fontSize="11" textAnchor="end" fontFamily="monospace" fill={INK_SOFT}>{v}</text>
          <line x1={padL} y1={yAt(v)} x2={padL + pw} y2={yAt(v)} stroke={HATCH} strokeWidth="0.7" strokeDasharray="2 3" />
        </g>
      ))}
      <text x={padL + pw / 2} y={H - 6} fontSize="11" textAnchor="middle" fontFamily="monospace" fill={INK_SOFT}>time [s] →</text>
      <text x={6} y={padT - 2} fontSize="11" fontFamily="monospace" fill={INK_SOFT}>km/h</text>
      <path d={refPath} fill="none" stroke={INK} strokeWidth="1.5" strokeDasharray="4 3" />
      <path d={actPath} fill="none" stroke="#2f5780" strokeWidth="2.2" />
      <g transform={`translate(${padL + 10}, ${padT + 8})`}>
        <line x1="0" y1="6" x2="22" y2="6" stroke={INK} strokeWidth="1.5" strokeDasharray="4 3" />
        <text x="28" y="10" fontSize="12" fontFamily="inherit" fill={INK}>基準</text>
        <line x1="68" y1="6" x2="90" y2="6" stroke="#2f5780" strokeWidth="2.2" />
        <text x="96" y="10" fontSize="12" fontFamily="inherit" fill={INK}>実測</text>
      </g>
    </svg>
  );
}

// ── PedalGauge ───────────────────────────────────────────────
function PedalGauge({ label, value = 0, max = 100, width = 60, height = 180, color = INK }) {
  const pct = Math.max(0, Math.min(1, value / max));
  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 6 }}>
      <div style={{ fontSize: 14, color: INK_SOFT }}>{label}</div>
      <div style={{ width, height, border: `1.4px solid ${INK}`, position: 'relative', background: PAPER }}>
        <div style={{
          position: 'absolute', bottom: 0, left: 0, right: 0,
          height: `${pct * 100}%`, background: color, opacity: 0.85,
        }} />
        {[25, 50, 75].map(t => (
          <div key={t} style={{
            position: 'absolute', left: -4, right: -4,
            bottom: `${t}%`, height: 0,
            borderTop: `1px dashed ${INK_MUTE}`,
          }} />
        ))}
      </div>
      <div style={{ fontSize: 18, fontWeight: 700, fontFamily: 'monospace' }}>
        {value.toFixed(1)}<span style={{ fontSize: 12, color: INK_SOFT }}> %</span>
      </div>
    </div>
  );
}

// ── BigSpeed ─────────────────────────────────────────────────
function BigSpeed({ value = 0, refSpeed = null, size = 220 }) {
  return (
    <div style={{ width: size, textAlign: 'center', padding: '8px 0' }}>
      <div style={{ fontSize: 14, color: INK_SOFT, letterSpacing: 1 }}>実車速</div>
      <div style={{ fontSize: size * 0.42, fontWeight: 700, lineHeight: 1, fontFamily: 'inherit' }}>
        {(typeof value === 'number' ? value : 0).toFixed(1)}
      </div>
      <div style={{ fontSize: 16, color: INK_SOFT, marginTop: -4 }}>km/h</div>
      {refSpeed !== null && (
        <div style={{ fontSize: 14, color: INK_SOFT, marginTop: 6 }}>
          基準 {refSpeed.toFixed(1)} / 偏差 {(value - refSpeed).toFixed(2)}
        </div>
      )}
    </div>
  );
}

// ── TimeProgress ─────────────────────────────────────────────
function TimeProgress({ elapsed = 380, total = 1800, style = {} }) {
  const pct = Math.max(0, Math.min(1, elapsed / total));
  const fmt = (s) => {
    const m = Math.floor(s / 60), ss = Math.floor(s % 60);
    return `${m}:${String(ss).padStart(2, '0')}`;
  };
  return (
    <div style={style}>
      <div style={{
        display: 'flex', justifyContent: 'space-between',
        fontSize: 13, color: INK_SOFT, marginBottom: 4, fontFamily: 'monospace',
      }}>
        <span>経過 {fmt(elapsed)}</span>
        <span>{Math.round(pct * 100)}%</span>
        <span>残り {fmt(total - elapsed)}</span>
      </div>
      <div style={{ height: 14, border: `1.4px solid ${INK}`, position: 'relative', background: PAPER }}>
        <div style={{
          position: 'absolute', left: 0, top: 0, bottom: 0,
          width: `${pct * 100}%`, background: '#2f5780', opacity: 0.7,
        }} />
      </div>
    </div>
  );
}

// ── EmergencyBtn ─────────────────────────────────────────────
function EmergencyBtn({ size = 130, style = {} }) {
  return (
    <div style={{
      width: size, height: size, borderRadius: '50%',
      border: `3px solid #a23232`,
      background: 'radial-gradient(circle at 35% 30%, #f1d8d2 0%, #d95555 80%)',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
      textAlign: 'center', color: '#fff',
      fontWeight: 800, fontSize: size * 0.13,
      letterSpacing: 1,
      boxShadow: '3px 3px 0 rgba(0,0,0,0.2), inset 0 -8px 12px rgba(0,0,0,0.25)',
      transform: 'rotate(-1deg)',
      ...style,
    }}>非常停止<br />STOP</div>
  );
}

// ── ConfirmStopPopup ─────────────────────────────────────────
function ConfirmStopPopup({ message = '停止しますか？', onYes, onNo }) {
  return (
    <div style={{
      position: 'absolute', inset: 0, zIndex: 998,
      background: 'rgba(30,20,20,0.45)',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    }}>
      <div style={{
        background: PAPER, border: `2px solid ${INK}`,
        borderRadius: 8, padding: '32px 44px',
        textAlign: 'center', boxShadow: '4px 4px 0 rgba(0,0,0,0.2)',
        transform: 'rotate(-0.3deg)', minWidth: 320,
        fontFamily: "'Patrick Hand', cursive",
      }}>
        <div style={{ fontSize: 24, fontWeight: 700, marginBottom: 24, color: INK }}>{message}</div>
        <div style={{ display: 'flex', gap: 16, justifyContent: 'center' }}>
          <div onClick={onYes} style={{
            display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
            border: '2px solid #a23232', borderRadius: 6, padding: '10px 32px',
            background: '#f1d8d2', color: '#5e1414',
            fontWeight: 700, fontSize: 18, fontFamily: 'inherit', cursor: 'pointer',
            boxShadow: '2px 2px 0 rgba(0,0,0,0.12)',
          }}>はい</div>
          <div onClick={onNo} style={{
            display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
            border: `2px solid ${INK}`, borderRadius: 6, padding: '10px 32px',
            background: PAPER, color: INK,
            fontWeight: 700, fontSize: 18, fontFamily: 'inherit', cursor: 'pointer',
            boxShadow: '2px 2px 0 rgba(0,0,0,0.12)',
          }}>いいえ</div>
        </div>
      </div>
    </div>
  );
}

// ── Export ───────────────────────────────────────────────────
Object.assign(window, {
  INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH, STATE_TINT, NAV,
  Frame, Box, Btn, Pill, Note, H2, Input, Row, Squig, Hatch,
  SpeedGraph, PedalGauge, BigSpeed, TimeProgress, EmergencyBtn,
  EmergencyOverlay, ConfirmStopPopup, StateBadge, UpsIndicator, TopBar, Sidebar,
});
