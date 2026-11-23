// ── Manual drive screen ───────────────────────────────────

function AxisJog({ label, axisId, currentPos, openingPct, maxPct, onJog, enabled = true, selected = false, onSelect }) {
  const { INK, INK_SOFT, PAPER, HATCH, Btn } = window;
  const { AxisPanel, JogRow } = window;
  const dis = !enabled;

  return (
    <AxisPanel label={label} axisId={axisId} selected={selected} enabled={enabled} onSelect={onSelect}>

      {/* Jog buttons + drag slider */}
      <JogRow axisId={axisId} currentPos={currentPos} onJog={onJog} disabled={dis}
              displayValue={currentPos == null ? '—' : openingPct.toFixed(2)} unit="% (開度)" />

      {/* Opening bar */}
      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 10, color: INK_SOFT, marginBottom: 3, fontFamily: 'inherit' }}>
          <span>0 %</span><span>50</span><span>100 % (max {maxPct ?? '—'})</span>
        </div>
        <div style={{ height: 10, border: `1px solid ${HATCH}`, position: 'relative', background: PAPER, borderRadius: 2 }}>
          <div style={{ position: 'absolute', left: 0, top: 0, bottom: 0, width: `${openingPct}%`, background: '#c8922a', opacity: 0.9 }} />
        </div>
      </div>

      {/* Info */}
      <div style={{ display: 'flex', gap: 12, fontSize: 12, color: INK_SOFT, alignItems: 'center' }}>
        <span>開度 <b style={{ fontFamily: 'inherit', color: INK }}>{openingPct.toFixed(1)} %</b></span>
      </div>
    </AxisPanel>
  );
}

// ── スイッチ（スタート・P・N・D）: 押している間だけ押す ─────────────────
// 押しているスイッチは最大 1 つ（先に押した方が優先。先を離しても、後から押していた側は押されない）。
// 押している間は SWITCH_KEEPALIVE_MS ごとに「まだ押している」（down）を送り直す。ランナー側は
// 1.5 s 届かなければ離す。キーは u / i / o / p。
const SWITCHES = [
  { id: 'start', label: 'スタート', key: 'u' },
  { id: 'P', label: 'P', key: 'i' },
  { id: 'N', label: 'N', key: 'o' },
  { id: 'D', label: 'D', key: 'p' },
];
const SWITCH_KEEPALIVE_MS = 500;

function useManualSwitches({ enabled, keysEnabled }) {
  const { useState, useEffect, useRef } = React;
  const [pressed, setPressed] = useState(null);
  const pressedRef = useRef(null);
  const timerRef = useRef(null);
  const chainRef = useRef(Promise.resolve());   // 送る順番を保つ（down → up が入れ替わらない）
  const inflightRef = useRef(false);
  const latest = useRef({});
  latest.current = { enabled, keysEnabled };

  function stopTimer() {
    if (timerRef.current) { clearInterval(timerRef.current); timerRef.current = null; }
  }
  function clearPressed() {
    stopTimer();
    pressedRef.current = null;
    setPressed(null);
  }
  function send(button, action) {
    // 送る順番は守る。応答が NG（別のスイッチを押している等）なら押している状態を消す（トーストは apiFetch が出す）
    const job = chainRef.current.then(async () => {
      inflightRef.current = true;
      try {
        if (action === 'up') {
          // 離すのは静かに送る（運転終了で手動運転が無くなった後の 409 でトーストを出さない）
          await fetch('/api/v1/drive/manual/button', {
            method: 'POST', keepalive: true,
            headers: window.drvControlHeaders({ 'Content-Type': 'application/json' }),
            body: JSON.stringify({ button, action }),
          }).catch(() => {});
          return;
        }
        const r = await window.apiFetch('POST', '/api/v1/drive/manual/button', { button, action });
        if (!r && pressedRef.current === button) clearPressed();
      } finally {
        inflightRef.current = false;
      }
    });
    chainRef.current = job.catch(() => {});
    return job;
  }

  function down(button) {
    const st = latest.current;
    if (window.__drvViewer || !st.enabled) return;
    if (pressedRef.current) return;               // 画面側でも先押し優先
    pressedRef.current = button;
    setPressed(button);
    send(button, 'down');
    timerRef.current = setInterval(() => {
      if (pressedRef.current !== button || inflightRef.current) return;
      send(button, 'down');
    }, SWITCH_KEEPALIVE_MS);
  }
  function up(button) {
    if (!button || pressedRef.current !== button) return;    // 押していないスイッチを離しても何もしない
    clearPressed();
    return send(button, 'up');
  }
  function releaseAll() {
    const b = pressedRef.current;
    if (b) return up(b);
    return Promise.resolve();
  }

  // 手動運転でなくなったら離す
  useEffect(() => { if (!enabled) releaseAll(); }, [enabled]);

  // キー（u / i / o / p）。ペダルのキーと同じ除外条件。押したまま画面を離れる・隠す・フォーカスを失うと離す
  useEffect(() => {
    const keyToButton = k => (SWITCHES.find(sw => sw.key === k) || {}).id;
    function onKeyDown(e) {
      if (e.repeat) return;                       // キーリピートは無視（押し直しが要る）
      if (window.__drvViewer) return;
      if (!latest.current.keysEnabled) return;
      const tag = (e.target && e.target.tagName) || '';
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
      if (e.ctrlKey || e.metaKey || e.altKey) return;
      const b = keyToButton(e.key.length === 1 ? e.key.toLowerCase() : e.key);
      if (!b) return;
      e.preventDefault();
      down(b);
    }
    function onKeyUp(e) {
      const b = keyToButton(e.key.length === 1 ? e.key.toLowerCase() : e.key);
      if (b) up(b);
    }
    function onHidden() { releaseAll(); }
    function onVisibility() { if (document.hidden) releaseAll(); }
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    window.addEventListener('blur', onHidden);
    document.addEventListener('visibilitychange', onVisibility);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      window.removeEventListener('blur', onHidden);
      document.removeEventListener('visibilitychange', onVisibility);
      releaseAll();                               // 画面を離れる（unmount）
    };
  }, []);

  return { pressed, down, up };
}

// スイッチの枠。Btn はポインタのイベントを渡さないので、素の div を Btn と同じ見た目で作る。
function SwitchPanel({ enabled, pressed, down, up }) {
  const { Box, INK, INK_MUTE, PAPER_2, HATCH } = window;
  return (
    <Box label="スイッチ" style={{ padding: '10px 12px' }}>
      <div style={{ display: 'flex', gap: 6, height: '100%' }}>
        {SWITCHES.map(sw => {
          const on = pressed === sw.id;
          const dim = pressed != null && !on;       // 押している間、残りは薄く
          return (
            <div key={sw.id}
                 data-switch={sw.id}
                 onPointerDown={e => {
                   if (!enabled) return;
                   e.preventDefault();
                   try { e.currentTarget.setPointerCapture(e.pointerId); } catch (err) { /* 無視 */ }
                   down(sw.id);
                 }}
                 onPointerUp={() => up(sw.id)}
                 onPointerCancel={() => up(sw.id)}
                 onLostPointerCapture={() => up(sw.id)}
                 style={{
                   display: 'flex', alignItems: 'center', justifyContent: 'center',
                   border: `1.5px solid ${on ? '#c8922a' : HATCH}`, borderRadius: 5, padding: '6px 10px', minWidth: 48, boxSizing: 'border-box',
                   background: !enabled ? PAPER_2 : on ? '#3a2a0a' : PAPER_2,
                   color: !enabled ? INK_MUTE : on ? '#f0c060' : INK,
                   fontFamily: 'inherit', fontWeight: on ? 700 : 500, fontSize: 14, whiteSpace: 'nowrap',
                   cursor: enabled ? 'pointer' : 'not-allowed',
                   opacity: !enabled ? 0.6 : dim ? 0.4 : 1,
                   userSelect: 'none', touchAction: 'none',
                 }}>
              {sw.label}
            </div>
          );
        })}
      </div>
    </Box>
  );
}

function ManualScreen() {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, activeProfileId, realtimeData, robotState, setNav, setNavLock } = useContext(window.AppContext);
  const { Box, Btn, Note, Row } = window;
  const { ConfirmStopPopup, ValidationPopup, useJogKeys } = window;

  const [brk, setBrk] = useState({ currentPos: 0 });
  const [acc, setAcc] = useState({ currentPos: 0 });
  const [selectedAxis, setSelectedAxis] = useState('accel'); // キー操作の対象（Tab・パネルのクリックで切替）
  const [confirmStop, setConfirmStop] = useState(false);
  const [confirmStart, setConfirmStart] = useState(false);
  const [popup, setPopup] = useState(null);
  const [maxBrake, setMaxBrake] = useState(null);  // アクティブプロファイルの最大開度 [%]
  const [maxAccel, setMaxAccel] = useState(null);
  const startedHereRef = useRef(false);  // この画面で開始した（位置は 0 と分かっている）

  const enabled = robotState === 'MANUAL';
  // 開始できるのは初期化済み（READY）のときだけ
  const canStart = robotState === 'READY';

  // 選んでいる車の最大開度を取得。車が選ばれていなければポップアップ
  useEffect(() => {
    if (!activeProfileId) {
      setPopup('no_profile');
      return;
    }
    apiFetch('GET', `/api/v1/profiles/${activeProfileId}`).then(p => {
      if (!p) return;
      if (p.max_brake_opening_pct != null) setMaxBrake(p.max_brake_opening_pct);
      if (p.max_accel_opening_pct != null) setMaxAccel(p.max_accel_opening_pct);
    });
  }, []);

  // 手動運転中は他ページへの離脱をロック
  useEffect(() => {
    setNavLock(enabled);
    return () => setNavLock(false);
  }, [enabled]);

  // 位置の表示: MANUAL 中にこの画面で開始していない（再読み込みなど）ときは位置が分からないので
  // 「—」にし、次のジョグの応答で埋める。step 0 のジョグで読み直す手は使わない
  // （位置がキャリブレーション範囲の外＝原点のままだと、範囲の端へペダルが動いてしまう）。
  // MANUAL でなくなったら（終了で両軸は原点へ戻る）表示を 0 に戻す。
  useEffect(() => {
    if (robotState === 'MANUAL') {
      if (!startedHereRef.current) {
        setBrk({ currentPos: null });
        setAcc({ currentPos: null });
      }
    } else {
      setBrk({ currentPos: 0 });
      setAcc({ currentPos: 0 });
    }
  }, [robotState]);

  // キー操作（キャリブレーションと同じ割り当て）: 3/2 = ±1 pulse（0.01%）、e/w = ±10、d/s = ±100、c/x = ±1000（10%）、Tab = 軸切替
  useJogKeys({
    enabled: enabled && !confirmStart && !confirmStop && !popup,
    selectedAxis, setSelectedAxis,
    onJog: handleJog,
  });

  // スイッチ（スタート・P・N・D）。キーはペダルのキーと同じ条件（ポップアップ表示中は反応しない）
  const sw = useManualSwitches({ enabled, keysEnabled: enabled && !confirmStart && !confirmStop && !popup });

  function setAxisState(axisId, updater) {
    if (axisId === 'brake') setBrk(updater);
    else setAcc(updater);
  }

  async function handleJog(axisId, step) {
    const r = await apiFetch('POST', '/api/v1/drive/manual/jog', { axis: axisId, step });
    if (!r) return;
    setAxisState(axisId, s => ({ ...s, currentPos: r.position ?? s.currentPos }));
  }

  async function handleHome() {
    // API は軸を選べない（両軸が原点へ戻る）
    const r = await apiFetch('POST', '/api/v1/drive/manual/home', { axis: 'brake' });
    if (!r) return;
    setBrk(s => ({ ...s, currentPos: 0 }));
    setAcc(s => ({ ...s, currentPos: 0 }));
    window.showToast('両軸を原点へ戻しました', 'success');
  }

  async function handleStart() {
    startedHereRef.current = true;
    const r = await apiFetch('POST', '/api/v1/drive/manual/start');
    if (!r) { startedHereRef.current = false; return; }
    window.showToast('手動運転を開始しました', 'success');
  }

  async function handleStop() {
    const r = await apiFetch('POST', '/api/v1/drive/manual/stop');
    if (r) {
      window.showToast('手動運転を終了しました', 'success');
      setConfirmStop(false);
      startedHereRef.current = false;
    }
  }

  // 開度 [%]: 位置 [pulse] から換算する（開度の定義は 原点 0%・9500 pulse = 100%）。位置が分からないときは 0。
  const PULSE_FULL = 9500;
  const toOpening = pos => (pos == null ? 0 : Math.max(0, Math.min(100, (pos / PULSE_FULL) * 100)));
  // 手動運転ジョブの実測位置（interactive_status）があればそれを優先（再読み込み後も実際と一致）。無ければ操作の応答から覚えた値
  const job = realtimeData?.job ?? null;
  const isManualJob = job?.kind === 'interactive' && job?.purpose === 'manual';
  const st = isManualJob ? (job.interactive_status ?? null) : null;
  const brkPos = st?.brake_pulse ?? brk.currentPos;
  const accPos = st?.accel_pulse ?? acc.currentPos;
  const brkOpeningPct = toOpening(brkPos);
  const accOpeningPct = toOpening(accPos);

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 10, flex: 1, minHeight: 0 }}>

      {popup === 'no_profile' && (
        <ValidationPopup
          message="車を選択してください"
          actionLabel="車両プロファイルへ"
          onAction={() => setNav('profiles')}
        />
      )}

      {!enabled && !canStart && (
        <div style={{ display: 'flex', gap: 10, alignItems: 'center' }}>
          <Note style={{ flex: 1 }}>先に初期化を実行してください。初期化が終わる（READY になる）と「開始」を押せます（現在: {robotState}）。</Note>
          <Btn onClick={() => setNav('init')}>初期化へ</Btn>
        </div>
      )}

      {/* Top: axis panels (fill remaining height) */}
      <div style={{ display: 'flex', gap: 10, flex: 1, minHeight: 0 }}>
        <AxisJog
          label="ブレーキ" axisId="brake"
          currentPos={brkPos}
          openingPct={brkOpeningPct} maxPct={maxBrake}
          onJog={handleJog}
          enabled={enabled} selected={selectedAxis === 'brake'} onSelect={setSelectedAxis}
        />
        <AxisJog
          label="アクセル" axisId="accel"
          currentPos={accPos}
          openingPct={accOpeningPct} maxPct={maxAccel}
          onJog={handleJog}
          enabled={enabled} selected={selectedAxis === 'accel'} onSelect={setSelectedAxis}
        />
      </div>

      {/* Bottom: shortcuts | status + button */}
      <div style={{ display: 'flex', gap: 10 }}>
        <Box label="キーボードショートカット" style={{ padding: '10px 12px', flex: '0 1 auto' }}>
          <div style={{ fontSize: 12, lineHeight: 1.7, fontFamily: 'inherit' }}>
            <div>3 / 2  : ±0.01 %</div>
            <div>e / w  : ±0.1 %</div>
            <div>d / s  : ±1 %</div>
            <div>c / x  : ±10 %</div>
            <div>Tab    : 軸切替 (アクセル ↔ ブレーキ)</div>
            <div>パネルをクリック : その軸を操作対象にする</div>
            <div>u / i / o / p : スタート / P / N / D（押している間）</div>
          </div>
        </Box>

        <SwitchPanel enabled={enabled} pressed={sw.pressed} down={sw.down} up={sw.up} />

        <div style={{ display: 'flex', gap: 10, flex: 1 }}>
          <Box style={{ padding: '10px 14px', fontSize: 13, flex: 1 }}>
            <Row cells={[['実車速',       '1.4fr'], [`${(realtimeData.actual_speed_kmh ?? 0).toFixed(1)} km/h`, '1fr', 'mono']]} />
            <Row cells={[['ブレーキ開度', '1.4fr'], [brkPos == null ? "—" : `${brkOpeningPct.toFixed(2)} %`, '1fr', 'mono']]} />
            <Row cells={[['アクセル開度', '1.4fr'], [accPos == null ? "—" : `${accOpeningPct.toFixed(2)} %`, '1fr', 'mono']]} />
          </Box>
          <div style={{ display: 'flex', flexDirection: 'column', flex: 1, gap: 6 }}>
            <Btn onClick={handleHome} disabled={!enabled}>両軸を原点へ</Btn>
            {robotState === 'MANUAL' ? (
              <Btn danger big style={{ flex: 1 }} onClick={() => setConfirmStop(true)}>■ 運転終了</Btn>
            ) : (
              <Btn big disabled={!canStart} style={{ flex: 1, borderColor: '#3c8c3c', background: '#0e220e', color: '#68d468' }}
                   onClick={() => { if (canStart) setConfirmStart(true); }}>開始</Btn>
            )}
          </div>
        </div>
      </div>

      {confirmStart && (
        <ConfirmStopPopup
          message="開始しますか？"
          onYes={() => { handleStart(); setConfirmStart(false); }}
          onNo={() => setConfirmStart(false)}
        />
      )}

      {confirmStop && (
        <ConfirmStopPopup
          message="運転を終了しますか？"
          onYes={async () => { await sw.up(sw.pressed); handleStop(); }}
          onNo={() => setConfirmStop(false)}
        />
      )}
    </div>
  );
}

window.ManualScreen = ManualScreen;
