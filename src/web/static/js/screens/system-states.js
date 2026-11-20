// ── System state screens ──────────────────────────────────
// Matches wireframe: missing-system-states.jsx

const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH, Box, Btn, Pill, H2, Note } = window;
const { useContext } = React;

// ── Step icon helper ─────────────────────────────────────
function StepIcon({ st, i }) {
  const iconColor = st === 'done' ? '#54bc54' : st === 'now' ? '#5aacdc' : INK_MUTE;
  const icon = st === 'done' ? '✓' : st === 'now' ? '⟳' : '—';
  const bg   = st === 'done' ? '#0e1e0e' : st === 'now' ? '#0e1a24' : 'transparent';
  return (
    <div style={{
      width: 26, height: 26, borderRadius: '50%',
      border: `1.4px solid ${iconColor}`,
      background: bg,
      display: 'flex', alignItems: 'center', justifyContent: 'center',
      fontSize: 13, fontWeight: 700, color: iconColor, flexShrink: 0,
    }}>{icon}</div>
  );
}

// ── BOOTING ──────────────────────────────────────────────
// 起動中は実際の機器の確認結果を持っていないので、固定の見本は出さず、起動中であることだけを伝える
function BootingScreen() {
  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 28 }}>
      <div style={{ textAlign: 'center' }}>
        <div style={{ fontSize: 40, fontWeight: 800, letterSpacing: 1 }}><span style={{ fontSize: 22 }}>ﾈﾝ0系</span> あま</div>
        <div style={{ fontSize: 18, color: INK_SOFT, marginTop: 4 }}>起動しています…</div>
      </div>
    </div>
  );
}

// ── ERROR ────────────────────────────────────────────────
// 実際に分かる情報（最後のジョブの結果・失敗した項目・理由）だけを出す。固定の機器名や確認事項は出さない
function ErrorScreen() {
  const { apiFetch, setNav, realtimeData } = useContext(window.AppContext);
  const liveJob = realtimeData?.job ?? null;
  const lastJob = window.useLastJob(liveJob);
  const job = liveJob ?? lastJob;
  const info = job && !liveJob ? window.describeJobResult?.(job) : null;
  const failedSteps = (job?.init_steps ?? []).filter(st => st.status === 'error');

  async function handleRetry() {
    const r = await apiFetch('POST', '/api/v1/drive/initialize');
    if (r) window.showToast('再接続を試みています', 'info');
  }

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', justifyContent: 'center', alignItems: 'center' }}>
      <Box style={{ width: 700, padding: '22px 28px', border: '2px solid #a23232' }}>
        <div style={{ display: 'flex', gap: 16, alignItems: 'flex-start', marginBottom: 18 }}>
          <div style={{ fontSize: 36, color: '#a23232', lineHeight: 1 }}>⚠</div>
          <div>
            <div style={{ fontSize: 26, fontWeight: 800, color: '#5e1414' }}>エラー状態です</div>
            <div style={{ fontSize: 15, color: '#c87070', marginTop: 4 }}>
              原因を確認してから再試行してください。詳しい経過はログ画面で見られます。
            </div>
          </div>
        </div>

        {(info || failedSteps.length > 0) && (
          <div style={{ display: 'flex', flexDirection: 'column', gap: 8, marginBottom: 18 }}>
            {info && (
              <div style={{ padding: '10px 12px', border: '1.2px solid #943030', background: '#200e0e', borderRadius: 4 }}>
                <div style={{ fontSize: 15, fontWeight: 700, color: '#e05050' }}>最後のジョブ: {info.title}</div>
                {info.what && <div style={{ fontSize: 12, color: '#c87070', marginTop: 2 }}>{info.what}</div>}
                {info.reason && <div style={{ fontSize: 12, color: '#c87070', marginTop: 2 }}>理由: {info.reason}</div>}
                {info.next && <div style={{ fontSize: 12, color: '#c87070', marginTop: 2 }}>次にすること: {info.next}</div>}
              </div>
            )}
            {failedSteps.map(st => (
              <div key={st.key} style={{ display: 'flex', gap: 10, alignItems: 'flex-start', padding: '10px 12px', border: '1.2px solid #943030', background: '#200e0e', borderRadius: 4 }}>
                <div style={{ fontSize: 16, color: '#e05050', fontWeight: 700, flexShrink: 0, marginTop: 1 }}>✗</div>
                <div style={{ flex: 1 }}>
                  <div style={{ fontSize: 15, fontWeight: 700, color: '#e05050' }}>{st.label}</div>
                  {st.detail && <div style={{ fontSize: 12, color: '#c87070', marginTop: 2 }}>{st.detail}</div>}
                </div>
              </div>
            ))}
          </div>
        )}

        <div style={{ display: 'flex', gap: 10, justifyContent: 'flex-end' }}>
          <Btn onClick={() => setNav('logs')}>ログを確認</Btn>
          <Btn primary big onClick={handleRetry}>⟳ 再試行</Btn>
        </div>
      </Box>
    </div>
  );
}

// ── AC電源断 ──────────────────────────────────────────────
// 経過はこの画面が出てからのクライアント側の実時間。段階は実行中のジョブの emergency_stage があれば出す
function AcPowerLossScreen() {
  const { realtimeData } = useContext(window.AppContext);
  const [elapsed, setElapsed] = React.useState(0);
  React.useEffect(() => {
    const t0 = Date.now();
    const id = setInterval(() => setElapsed((Date.now() - t0) / 1000), 100);
    return () => clearInterval(id);
  }, []);
  const stage = realtimeData?.job?.emergency_stage ?? null;
  const stageText = stage ? (window.EMERGENCY_STAGE_LABEL?.[stage] ?? stage) : null;
  const steps = [
    'AC断検知',
    '制御ループ停止 / 走行セッション終了',
    '原点復帰 (両軸)',
    '走行ログ PostgreSQL フラッシュ',
    'PostgreSQL 正常終了',
    'Raspberry Pi シャットダウン',
  ];

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 20 }}>
      <div style={{
        width: 680, background: '#2a1212',
        border: '2.5px solid #943030', borderRadius: 8,
        padding: '16px 24px', display: 'flex', alignItems: 'center', gap: 16,
      }}>
        <div style={{ fontSize: 40, color: '#e05050' }}>⚡</div>
        <div>
          <div style={{ fontSize: 26, fontWeight: 800, color: '#e05050' }}>AC電源断を検知しました</div>
          <div style={{ fontSize: 15, color: '#c87070' }}>
            UPS バッテリー給電中 — 安全停止シーケンスを実行しています。操作しないでください。
          </div>
        </div>
        <div style={{ marginLeft: 'auto', textAlign: 'right' }}>
          <div style={{ fontSize: 12, color: INK_SOFT }}>経過</div>
          <div style={{ fontSize: 28, fontFamily: 'inherit', fontWeight: 700, color: '#e05050' }}>{elapsed.toFixed(1)}s</div>
        </div>
      </div>

      <Box style={{ width: 680, padding: '18px 22px' }} label="安全停止シーケンス">
        {stageText && (
          <div style={{ fontSize: 15, fontWeight: 700, color: '#e05050', marginBottom: 10 }}>いまの段階: {stageText}</div>
        )}
        <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
          {steps.map((label, i) => (
            <div key={label} style={{ display: 'flex', alignItems: 'center', gap: 12, padding: '8px 10px' }}>
              <div style={{
                width: 28, height: 28, borderRadius: '50%',
                border: `1.6px solid ${INK_MUTE}`,
                display: 'flex', alignItems: 'center', justifyContent: 'center',
                fontSize: 13, fontWeight: 700, color: INK_MUTE, flexShrink: 0,
              }}>{i + 1}</div>
              <div style={{ flex: 1, fontSize: 15 }}>{label}</div>
            </div>
          ))}
        </div>
        <div style={{ marginTop: 14 }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 12, color: INK_SOFT, marginBottom: 4, fontFamily: 'inherit' }}>
            <span>0s (AC断)</span>
            <span style={{ color: '#a23232' }}>▼ 現在 {elapsed.toFixed(1)}s</span>
            <span>30s (タイムリミット)</span>
          </div>
          <div style={{ height: 10, border: `1.4px solid ${INK}`, position: 'relative', background: PAPER }}>
            <div style={{ position: 'absolute', left: 0, top: 0, bottom: 0, width: `${Math.min(100, (elapsed / 30) * 100)}%`, background: '#a23232', opacity: 0.7 }} />
          </div>
        </div>
      </Box>

      <Note style={{ width: 680 }}>
        AC UPS のバッテリーで Raspberry Pi (5V) と P-CON-CB (24V) を同時給電中。シャットダウン完了後に自動で電源が切れます。
      </Note>
    </div>
  );
}

// ── EMERGENCY リセット — InitB ────────────────────────────
// 非常停止（スイッチ・画面のボタン・AC 断）→ JobManager.emergency() → ランナーが停止の流れ（減速して停車 → N → スタート 10 秒（IGN-OFF）→ 原点復帰）→ 終了処理（サーボOFF・切断）。
// 終わるまで解除できない（409）。解除してもスイッチが押されたままなら 423。
function EmergencyResetScreen() {
  const { apiFetch, setNav, realtimeData } = useContext(window.AppContext);
  const liveJob = realtimeData?.job ?? null;
  const lastJob = window.useLastJob(liveJob);
  const shuttingDown = !!liveJob;                   // ランナーがまだ動いている（終了処理中）
  const ended = !liveJob;
  // 非常停止で止めたジョブだけを出す。前に普通に終わったジョブ（emergency でない）は、止めたものとして扱わない（段 R）
  const job = liveJob ?? (lastJob?.emergency ? lastJob : null);
  const jobLabel = job ? (window.JOB_KIND_LABEL?.[job.kind] ?? job.kind) : null;
  // きっかけ（サーバーの emergency_cause。分からなければ従来の文のまま）
  const CAUSE_TEXT = {
    screen: '画面の非常停止ボタン',
    switch: '非常停止スイッチ',
    ac_loss: 'AC 電源断',
    auto: '走行中の自動の非常停止',
  };
  const cause = realtimeData?.emergency_cause ?? null;
  const causeDetail = cause === 'auto' ? (realtimeData?.emergency_cause_detail ?? null) : null;
  const causeLine = CAUSE_TEXT[cause]
    ? `非常停止しました（きっかけ: ${CAUSE_TEXT[cause]}${causeDetail ? `: ${causeDetail}` : ''}）`
    : '非常停止しました（非常停止スイッチ・画面の非常停止ボタン・AC 電源断のどれか）';

  const STOP_LOG = [
    { label: '非常停止を受け付けました', note: jobLabel ? `止めたジョブ: ${jobLabel}` : '実行中のジョブはありませんでした', state: 'done' },
    { label: 'ランナーへ停止を要求（SIGINT）', note: job ? '終了処理に入ります' : '—', state: job ? 'done' : 'skip' },
    { label: '停止の流れ（減速して停車 → N → スタート 10 秒（IGN-OFF）→ 原点復帰）', note: shuttingDown ? (liveJob.emergency_stage ? `${window.EMERGENCY_STAGE_LABEL?.[liveJob.emergency_stage] ?? liveJob.emergency_stage}` : `${window.PHASE_LABEL?.[liveJob.phase] ?? liveJob.phase}`) : (job ? (job.brake_kept ? '止まらず、ブレーキを踏んだまま終了' : `完了（終了コード ${job.exit_code ?? '—'}）`) : '—'), state: shuttingDown ? 'now' : (job ? 'done' : 'skip') },
    { label: 'セッションの記録（status=emergency）', note: ended && job?.session_id ? 'DB へ保存済み' : (shuttingDown ? '終了処理のあと' : '記録なし'), state: ended && job?.session_id ? 'done' : (shuttingDown ? 'wait' : 'skip') },
  ];
  const mark = st => st === 'done' ? ['✓', '#54bc54'] : st === 'now' ? ['⟳', '#5aacdc'] : st === 'skip' ? ['—', INK_MUTE] : ['…', INK_MUTE];

  async function handleReset() {
    // 終了処理中は 409、スイッチが押されたままなら 423（どちらも「次にすること」つきで apiFetch が表示する）
    const r = await apiFetch('POST', '/api/v1/drive/reset-emergency');
    if (r) {
      window.showToast('非常停止を解除しました', 'success');
      setNav('init');
    }
  }

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 24 }}>
      <div style={{
        border: '3px solid #943030', background: '#2a1212',
        padding: '24px 40px', borderRadius: 8,
        textAlign: 'center', transform: 'rotate(-0.6deg)',
        boxShadow: '4px 4px 0 rgba(0,0,0,0.5)',
      }}>
        <div style={{ fontSize: 38, fontWeight: 800, color: '#e05050', letterSpacing: 1 }}>
          EMERGENCY
        </div>
        <div style={{ fontSize: 16, color: '#c87070', marginTop: 4 }}>
          {causeLine}
        </div>
      </div>

      <Box style={{ width: 600, padding: 18 }}>
        <H2 sub={shuttingDown ? '停止の流れと終了処理の途中です。終わると解除できます' : '停止シーケンスが完了しました'}>停止シーケンスの記録</H2>
        {STOP_LOG.map(({ label, note, state }) => {
          const [m, c] = mark(state);
          return (
            <div key={label} style={{
              display: 'flex', alignItems: 'center', gap: 10,
              padding: '6px 0', fontSize: 14,
            }}>
              <span style={{ color: c, fontWeight: 700, width: 16, textAlign: 'center' }}>{m}</span>
              <span style={{ flex: 1 }}>{label}</span>
              <span style={{ fontSize: 11, fontFamily: 'inherit', color: INK_SOFT }}>{note}</span>
            </div>
          );
        })}
      </Box>

      <div style={{ display: 'flex', gap: 14 }}>
        <Btn big onClick={() => setNav('logs')}>ログを確認</Btn>
        <Btn primary big disabled={shuttingDown} onClick={shuttingDown ? undefined : handleReset}>
          非常停止を解除する
        </Btn>
      </div>
      <Note style={{ width: 600 }}>
        解除すると待機に戻ります。物理の非常停止スイッチが押されたままのときは解除できません（スイッチを戻してから押してください）。
        {job?.brake_kept
          ? '警告: 停車を確認できなかったため、ブレーキを最大開度で踏んだまま終えています（サーボ ON のまま・原点復帰と IGN-OFF はしていません）。車を止めた状態を保ち、車とペダルの状態を確認してください。'
          : !job
          ? '実行中のジョブはありませんでした。'
          : '走行していたジョブは、停車して N → スタート 10 秒（IGN-OFF）→ 原点復帰まで進めています。'}
      </Note>
    </div>
  );
}

Object.assign(window, {
  BootingScreen,
  ErrorScreen,
  AcPowerLossScreen,
  EmergencyResetScreen,
});
