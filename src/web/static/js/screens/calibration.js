// ── キャリブレーション: ペダルの min（触れた位置）と max（開度上限）を測る ──────────
// 開度は「原点 0%・9500 pulse = 100%」で固定。
//   開始 ＝ 対話ジョブを始める（初期化 → コマンド待ち。最初の API 呼び出しで始まる）
//   ジョグ ＝ POST /drive/calib/jog（pulse 指定。1 pulse ≒ 0.01%・10 pulse ≒ 0.1%・100 pulse ≒ 1%・1000 pulse ≒ 10%。ボタンの表示はキリの良い値で、実際は 9500 pulse = 100% なので少し大きい（10 は 10.5%）。キャリブレーション中は max を超えて 99% まで動かせる）
//   ここを min として保存 ＝ POST /drive/calib/save-min ／ ここを max として保存 ＝ POST /drive/calib/save-max
//   終了 ＝ POST /drive/stop（原点へ戻って待機に戻る）

const PULSE_FULL_UI = 9500;

function AxisMinPanel({ label, axisId, openingPct, pos, minPct, measured, maxPct, onJog, onSaveMin, onSaveMax, enabled, selected, onSelect }) {
  const { INK, INK_SOFT, INK_MUTE, PAPER, HATCH, Btn, Pill, AxisPanel, JogRow } = window;
  const dis = !enabled;
  const minPos = minPct != null ? Math.round(minPct / 100 * PULSE_FULL_UI) : null;
  const maxPos = maxPct != null ? Math.round(maxPct / 100 * PULSE_FULL_UI) : null;
  return (
    <AxisPanel label={label} axisId={axisId} selected={selected} enabled={enabled} onSelect={onSelect}>
      <JogRow axisId={axisId} currentPos={pos} onJog={onJog} disabled={dis}
              labels={['0.01', '0.1', '1', '10']} displayValue={openingPct == null ? '—' : openingPct.toFixed(2)} unit="% (開度)" minPos={minPos} maxPos={maxPos} />

      {/* 開度バー（0〜100%）。保存済み min（緑）・max（赤）の位置に印 */}
      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 10, color: INK_SOFT, marginBottom: 3 }}>
          <span>0 %</span><span>100 %</span>
        </div>
        <div style={{ height: 10, border: `1px solid ${HATCH}`, position: 'relative', background: PAPER, borderRadius: 2 }}>
          <div style={{ position: 'absolute', left: 0, top: 0, bottom: 0, width: `${Math.max(0, Math.min(100, openingPct ?? 0))}%`, background: '#c8922a', opacity: 0.9 }} />
          {minPct != null && <div style={{ position: 'absolute', top: -2, bottom: -2, left: `${Math.max(0, Math.min(100, minPct))}%`, width: 2, background: '#68d468' }} />}
          {maxPct != null && <div style={{ position: 'absolute', top: -2, bottom: -2, left: `${Math.max(0, Math.min(100, maxPct))}%`, width: 2, background: '#e0607a' }} />}
        </div>
      </div>

      <div style={{ display: 'flex', gap: 10, alignItems: 'center', flexWrap: 'wrap', fontSize: 12, color: INK_SOFT }}>
        <span>保存済み min <b style={{ color: INK }}>{minPct == null ? '—' : `${minPct} %`}</b></span>
        <span>max <b style={{ color: INK }}>{maxPct == null ? '—' : `${maxPct} %`}</b></span>
        {measured ? <Pill accent="READY" style={{ fontSize: 12 }}>測定済み</Pill> : <Pill accent="PAUSED" style={{ fontSize: 12 }}>未測定（仮値）</Pill>}
      </div>
      <div style={{ display: 'flex', gap: 8 }}>
        <Btn big disabled={dis} style={{ flex: 1, justifyContent: 'center', borderColor: '#2a6c2a', color: '#54bc54' }}
             onClick={() => onSaveMin(axisId)}>
          ここを min として保存
        </Btn>
        <Btn big disabled={dis} style={{ flex: 1, justifyContent: 'center', borderColor: '#943050', color: '#e0607a' }}
             onClick={() => onSaveMax(axisId)}>
          ここを max として保存
        </Btn>
      </div>
    </AxisPanel>
  );
}

function CalibrationScreen() {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, activeProfileId, setNav, setNavLock, robotState, realtimeData } = useContext(window.AppContext);
  const { Box, Btn, Note, ValidationPopup, ConfirmStopPopup, INK, INK_SOFT, INK_MUTE } = window;

  const [profile, setProfile] = useState(null);
  const [pos, setPos] = useState({ accel: 0, brake: 0 });          // ジョグの応答の実位置 [pulse]
  const [popup, setPopup] = useState(null);
  const [starting, setStarting] = useState(false);
  const [confirmStart, setConfirmStart] = useState(false);
  const [confirmEnd, setConfirmEnd] = useState(false);
  const [selectedAxis, setSelectedAxis] = useState('brake');        // キー操作の対象（Tab・パネルのクリックで切替）

  const job = realtimeData?.job ?? null;
  const isCalibJob = job?.kind === 'interactive' && job?.purpose === 'calib';
  const started = robotState === 'CALIBRATING' && isCalibJob;
  const st = isCalibJob ? (job.interactive_status ?? null) : null;   // accel_pct・brake_pct・accel_pulse・min_*_pct …
  const otherJob = !!job && !isCalibJob;
  const canStart = robotState === 'READY' && !!activeProfileId && !starting;

  async function loadProfile() {
    if (!activeProfileId) return;
    const p = await apiFetch('GET', `/api/v1/profiles/${activeProfileId}`);
    if (p) setProfile(p);
  }
  useEffect(() => {
    if (!activeProfileId) { setPopup('no_profile'); return; }
    loadProfile();
  }, []);

  // 開始〜終了のあいだ他ページへの離脱をロック（開始処理中も含む）
  useEffect(() => {
    setNavLock(started || starting || isCalibJob);
    return () => setNavLock(false);
  }, [started, starting, isCalibJob]);
  useEffect(() => { if (started) setStarting(false); }, [started]);
  useEffect(() => { if (!isCalibJob && !starting) setPos({ accel: 0, brake: 0 }); }, [isCalibJob]);

  async function handleStart() {
    setConfirmStart(false);
    setStarting(true);
    // 対話ジョブは最初の calib API 呼び出しで始まる。「原点へ」は始めるときの無害な呼び出し（初期化の直後は原点にいる）
    const r = await apiFetch('POST', '/api/v1/drive/calib/home', { axis: 'brake' });
    if (!r) { setStarting(false); return; }
    setPos({ accel: 0, brake: r.position ?? 0 });
    window.showToast('キャリブレーションを開始しました', 'success');
    loadProfile();
  }

  async function handleEnd() {
    setConfirmEnd(false);
    const r = await apiFetch('POST', '/api/v1/drive/stop');
    if (r) { window.showToast('終了処理中です（原点へ戻って待機に戻ります）', 'success'); loadProfile(); }
  }

  async function handleJog(axisId, step) {
    const r = await apiFetch('POST', '/api/v1/drive/calib/jog', { axis: axisId, step });
    if (!r) return;
    setPos(p => ({ ...p, [axisId]: r.position ?? p[axisId] }));
  }

  async function handleHome() {
    // API は軸を選べない（両軸が原点へ戻る）
    const r = await apiFetch('POST', '/api/v1/drive/calib/home', { axis: 'brake' });
    if (!r) return;
    setPos({ accel: 0, brake: 0 });
    window.showToast('両軸を原点へ戻しました', 'success');
  }

  async function handleSaveMin(axisId) {
    const r = await apiFetch('POST', '/api/v1/drive/calib/save-min', { axis: axisId });
    if (!r) return;
    const name = axisId === 'brake' ? 'ブレーキ' : 'アクセル';
    window.showToast(`${name}の min を保存しました${r.message ? `\n${r.message}` : ''}`, 'success');
    loadProfile();
  }

  async function handleSaveMax(axisId) {
    const r = await apiFetch('POST', '/api/v1/drive/calib/save-max', { axis: axisId });
    if (!r) return;
    const name = axisId === 'brake' ? 'ブレーキ' : 'アクセル';
    window.showToast(`${name}の max を保存しました${r.message ? `\n${r.message}` : ''}`, 'success');
    loadProfile();
  }

  const { useJogKeys } = window;
  useJogKeys({
    enabled: started && !confirmStart && !confirmEnd && !popup,
    selectedAxis, setSelectedAxis,
    onJog: handleJog,
  });

  const pctOf = (stKey, posKey) => {
    if (st && st[stKey] != null) return st[stKey];
    return pos[posKey] != null ? pos[posKey] / PULSE_FULL_UI * 100 : null;
  };
  const accelPct = started ? pctOf('accel_pct', 'accel') : null;
  const brakePct = started ? pctOf('brake_pct', 'brake') : null;
  const accelPulse = st?.accel_pulse ?? pos.accel;
  const brakePulse = st?.brake_pulse ?? pos.brake;
  const minAccel = profile?.min_accel_opening_pct ?? st?.min_accel_pct ?? null;
  const minBrake = profile?.min_brake_opening_pct ?? st?.min_brake_pct ?? null;
  const maxAccel = profile?.max_accel_opening_pct ?? st?.max_accel_pct ?? null;
  const maxBrake = profile?.max_brake_opening_pct ?? st?.max_brake_pct ?? null;

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 8, flex: 1, minHeight: 0 }}>

      {popup === 'no_profile' && (
        <ValidationPopup message="車を選択してください" actionLabel="車両プロファイルへ" onAction={() => setNav('profiles')} />
      )}

      {!started && !starting && otherJob && (
        <Note>別のジョブ（{window.JOB_KIND_LABEL?.[job.kind] ?? job.kind}）が実行中です。終わってから開始してください。</Note>
      )}
      {!started && !starting && !otherJob && robotState !== 'READY' && (
        <Note>今は開始できません（現在: {robotState}）。待機（READY）に戻ってから開始してください。</Note>
      )}
      {!started && starting && <Note>開始しています（初期化・原点復帰に数十秒かかることがあります）…</Note>}

      <div style={{ display: 'flex', gap: 10, flex: 1, minHeight: 0 }}>
        <AxisMinPanel
          label="ブレーキ" axisId="brake" openingPct={brakePct} pos={brakePulse}
          minPct={minBrake} measured={!!profile?.min_brake_measured} maxPct={maxBrake}
          onJog={handleJog} onSaveMin={handleSaveMin} onSaveMax={handleSaveMax}
          enabled={started} selected={selectedAxis === 'brake'} onSelect={setSelectedAxis}
        />
        <AxisMinPanel
          label="アクセル" axisId="accel" openingPct={accelPct} pos={accelPulse}
          minPct={minAccel} measured={!!profile?.min_accel_measured} maxPct={maxAccel}
          onJog={handleJog} onSaveMin={handleSaveMin} onSaveMax={handleSaveMax}
          enabled={started} selected={selectedAxis === 'accel'} onSelect={setSelectedAxis}
        />
      </div>

      <div style={{ display: 'flex', gap: 10, alignItems: 'stretch' }}>
        <div style={{ flex: 1, display: 'flex', alignItems: 'center', gap: 12, fontSize: 13, color: INK_SOFT }}>
          <span>車: <b style={{ color: INK }}>{profile?.name ?? '—'}</b></span>
          <span>ブレーキ min {profile?.min_brake_measured ? '測定済み' : '未測定'}</span>
          <span>アクセル min {profile?.min_accel_measured ? '測定済み' : '未測定'}</span>
          <Btn onClick={handleHome} disabled={!started}>両軸を原点へ</Btn>
          {profile?.min_accel_measured && profile?.min_brake_measured && (
            <Btn onClick={() => setNav('learning')}>学習運転へ</Btn>
          )}
        </div>
        <div style={{ flex: '0 0 30%', display: 'flex' }}>
          {started ? (
            <Btn danger big style={{ flex: 1 }} onClick={() => setConfirmEnd(true)}>終了</Btn>
          ) : (
            <Btn big disabled={!canStart} style={{ flex: 1, borderColor: '#3c8c3c', background: '#0e220e', color: '#68d468' }}
                 onClick={() => { if (canStart) setConfirmStart(true); }}>開始</Btn>
          )}
        </div>
      </div>

      {confirmStart && (
        <ConfirmStopPopup message="開始しますか？（ペダルが動きます）" onYes={handleStart} onNo={() => setConfirmStart(false)} />
      )}
      {confirmEnd && (
        <ConfirmStopPopup message="キャリブレーションを終了しますか？" onYes={handleEnd} onNo={() => setConfirmEnd(false)} />
      )}
    </div>
  );
}

window.CalibrationScreen = CalibrationScreen;
