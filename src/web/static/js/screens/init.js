// ── Init screen ───────────────────────────────────────────
// 初期化はジョブ（init_only: 手順 1）として走り、終わると待機（READY）に戻る。
// 各項目の進み具合は WebSocket の job.init_steps（実行中）か、終わった直後は GET /drive/status の last_job.init_steps。
// 手順 1（初期化）を行うのはこの画面だけ（D-15c）。成功すると学習・自動運転を始められる
// （サーバーの起動後・非常停止の後・エラー終了の後は、もう一度必要）。
// UPS・非常停止(GPIO)・SG90(ボタン) が設定で off のときは「なし（設定で off）」と出す（GET /drive/status の devices）。

const { INK, INK_SOFT, INK_MUTE, PAPER_2, Box, Btn, H2 } = window;

// まだ一度も走っていないときの表示（core の初期化の項目名）。全て未実行。
const FALLBACK_INIT_STEPS = [
  { key: 'conn',   label: '接続 (アクセル/ブレーキ/CAN)',        status: 'pending' },
  { key: 'comm',   label: 'サーボ通信確認 (現在位置読み出し)',   status: 'pending' },
  { key: 'alarm',  label: 'エラー消去 (アラームリセット)',        status: 'pending' },
  { key: 'servo',  label: 'サーボON (両軸)',                      status: 'pending' },
  { key: 'can',    label: 'CAN 通信チェック (車速読み出し)',      status: 'pending' },
  { key: 'ups',    label: 'UPS 通信チェック (残量)',              status: 'pending' },
  { key: 'home',   label: 'アクチュエータ初期位置 (原点復帰)',    status: 'pending' },
];

function InitScreen() {
  const { useState, useContext } = React;
  const { robotState, realtimeData, apiFetch, activeProfileId, setNav } = useContext(window.AppContext);

  const [loading, setLoading] = useState(false);
  const [dismissedJobId, setDismissedJobId] = useState(null);
  const [devices, setDevices] = useState(null);

  const liveJobId = realtimeData?.job?.job_id ?? null;
  React.useEffect(() => {
    apiFetch('GET', '/api/v1/drive/status').then(d => { if (d?.devices) setDevices(d.devices); });
  }, [activeProfileId, liveJobId]);
  const initialized = realtimeData?.initialized;

  const liveJob = realtimeData?.job ?? null;
  const lastJob = window.useLastJob(liveJob);
  const isInitJob = liveJob?.kind === 'init_only';
  const otherJob = !!liveJob && !isInitJob;
  const isInitializing = loading || isInitJob;

  // 表示する項目: 実行中のジョブ → 終わった直後のジョブ（どの種類でも初期化の結果は持つ）→ 未実行の一覧
  // 未初期化（非常停止・エラーの後など）のときは、前のジョブの ✓ を今の状態として出さない（段 T）。
  // ただし直前の初期化が失敗して終わったときは、どの項目が失敗したか見られるよう結果を残す
  const lastInitFailed = lastJob?.kind === 'init_only' && lastJob.exit_code != null && lastJob.exit_code !== 0;
  const lastStepsUsable = !!lastJob?.init_steps?.length && (initialized !== false || lastInitFailed);
  const fromJob = (liveJob?.init_steps?.length ? liveJob.init_steps : null)
    ?? (lastStepsUsable ? lastJob.init_steps : null);
  const hidingLast = !liveJob && initialized === false && !!lastJob?.init_steps?.length && !lastStepsUsable;
  // 接続と Modbus 操作権は 1 項目にまとめて見せる（段 V。表示だけ。ランナーの項目は変えない）
  // ランナーの key はラベルそのもの（例「Modbus 操作権 (ブレーキ→アクセル)」）なので、key の頭の語で見分ける
  const isConn = x => x.key === 'conn' || String(x.key ?? x.label).startsWith('接続');
  const isModbus = x => x.key === 'modbus' || String(x.key ?? x.label).startsWith('Modbus');
  function mergeConnSteps(list) {
    const conn = list.find(isConn);
    const mod = list.find(isModbus);
    if (!mod) return list;
    if (!conn) return list.filter(x => !isModbus(x));
    const sts = [conn.status, mod.status];
    const status = sts.includes('error') ? 'error' : sts.includes('running') ? 'running'
      : sts.every(x => x === 'done') ? 'done' : 'pending';
    const detail = [conn.detail, mod.detail].filter(Boolean).join(' / ');
    return list.filter(x => !isModbus(x)).map(x => isConn(x)
      ? { ...x, label: '接続 (アクセル/ブレーキ/CAN)', status, detail: detail || undefined } : x);
  }
  const baseSteps = mergeConnSteps(fromJob ?? FALLBACK_INIT_STEPS);
  const OFF_NOTE = 'なし（設定で off）';
  // 設定で off の機器は「なし」と出す。UPS は初期化の項目（UPS 通信チェック）を書き換え、
  // 非常停止(GPIO)・SG90(ボタン) は初期化の項目に無いので末尾に足す
  const steps = [
    ...baseSteps.map(st => (devices?.ups === false && /UPS/.test(st.label))
      ? { ...st, status: 'skipped', detail: OFF_NOTE } : st),
    ...(devices ? [
      { key: 'dev_gpio', label: '非常停止スイッチ',
        status: devices.gpio ? 'done' : 'skipped', detail: devices.gpio ? '見張り中' : OFF_NOTE },
      { key: 'dev_buttons', label: 'ボタンサーボ',
        status: devices.buttons ? 'done' : 'skipped',
        detail: devices.buttons ? '取り付け済み' : OFF_NOTE },
    ] : []),
  ];
  const showingLast = !liveJob && lastStepsUsable;

  // status → 表示インジケータ (丸印・色・補足ラベル)
  function indicator(status) {
    switch (status) {
      case 'done':    return { mark: '✓', border: '#2a6c2a', bg: '#0e1e0e',    note: null };
      case 'running': return { mark: '⟳', border: INK,       bg: PAPER_2,      note: '実行中...' };
      case 'skipped': return { mark: '⤼', border: INK_MUTE,  bg: '#1e1e18',    note: 'スキップ' };
      case 'error':   return { mark: '✗', border: '#943030', bg: '#200e0e',    note: 'エラー' };
      default:        return { mark: '—', border: INK_MUTE,  bg: 'transparent', note: null };
    }
  }

  async function handleInitialize() {
    setLoading(true);
    const r = await apiFetch('POST', '/api/v1/drive/initialize');
    setLoading(false);
    if (r) { setDismissedJobId(null); window.showToast('初期化を開始しました（終わると待機に戻ります）', 'success'); }
  }

  // 成功（「初期化: 完了しました」）の帯は出さない（上の「現在: 初期化済み」と各項目の ✓ で分かる。ユーザー決定 2026-11-20）。
  // 失敗・非常停止など、理由と次の操作を伝える帯だけ出す
  const showResult = !liveJob && lastJob?.kind === 'init_only' && lastJob.job_id !== dismissedJobId
    && window.describeJobResult(lastJob)?.level !== 'ok';
  const statusLine = isInitJob
    ? (liveJob.stop_requested ? '停止を要求しました。終了処理中…' : (window.PHASE_LABEL?.[liveJob.phase] ?? liveJob.phase))
    : null;

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 12, height: '100%', minHeight: 0 }}>
      <div style={{ fontSize: 12.5, color: INK_SOFT, lineHeight: 1.5 }}>
        <b style={{ color: initialized ? '#68d468' : '#f0c84a' }}>現在: {initialized === undefined ? '—' : initialized ? '初期化済み' : '未初期化'}</b>
        {hidingLast && <span style={{ color: INK_MUTE }}>　前回の初期化の結果は表示していません（未初期化）</span>}
        {showingLast && <span>　（表示は直前のジョブ「{window.JOB_KIND_LABEL?.[lastJob.kind] ?? lastJob.kind}」の結果です）</span>}
      </div>

      {showResult && (
        <window.JobResultBanner job={lastJob} onDismiss={() => setDismissedJobId(lastJob.job_id)} onLogs={() => setNav('logs')} />
      )}

      <Box style={{ padding: 18, flex: 1, minHeight: 0, overflowY: 'auto' }}>
        {steps.map(({ key, label, status, detail }) => {
          const ind = indicator(status);
          const active = status === 'running';
          return (
            <div key={key} style={{
              display: 'flex', alignItems: 'center', gap: 16,
              padding: '14px 8px',
              borderBottom: `1px dashed ${INK_MUTE}`,
            }}>
              <div style={{
                width: 36, height: 36, borderRadius: '50%',
                border: `1.8px solid ${ind.border}`,
                background: ind.bg,
                display: 'flex', alignItems: 'center', justifyContent: 'center',
                fontWeight: 700, fontSize: 18, flexShrink: 0,
              }}>
                {ind.mark}
              </div>
              <div style={{ flex: 1 }}>
                <div style={{ fontSize: 19, fontWeight: active ? 700 : 400 }}>{label}</div>
                {detail ? <div style={{ fontSize: 12, color: INK_SOFT, marginTop: 2 }}>{detail}</div> : null}
              </div>
              {ind.note && <div style={{ fontSize: 13, color: INK_SOFT }}>{ind.note}</div>}
            </div>
          );
        })}
      </Box>

      <div style={{ display: 'flex', justifyContent: 'flex-end', alignItems: 'center', gap: 12 }}>
        {statusLine && <div style={{ fontSize: 13, color: INK_SOFT }}>{statusLine}</div>}
        {otherJob && <div style={{ fontSize: 13, color: '#f0c84a' }}>別のジョブ（{window.JOB_KIND_LABEL?.[liveJob.kind] ?? liveJob.kind}）が実行中です</div>}
        {isInitJob && (
          <Btn danger onClick={() => apiFetch('POST', '/api/v1/drive/stop')}>■ 中止</Btn>
        )}
        <Btn primary big onClick={isInitializing || otherJob ? undefined : handleInitialize} disabled={isInitializing || otherJob}>
          {isInitializing ? '初期化中…' : '初期化を実行'}
        </Btn>
      </div>
    </div>
  );
}

window.InitScreen = InitScreen;
