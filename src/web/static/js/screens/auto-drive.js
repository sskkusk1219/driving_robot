// ── Auto-drive / learning monitor screen（段11 UI 見直し）─────────────────
// 4段レイアウト（縦の比率 車速 1 : ペダル 1 : 進捗 0.5 : 情報 0.5。画面の高さに合わせて伸縮し、ページはスクロールしない）
//   車速グラフ / ペダル開度グラフ（ボタン押下の矩形つき） / 進捗（モードの全体進捗。学習は段階表示） /
//   情報1行（左: 実車速・開度、中: 状態の 1 行、右: 「一時停止」（モード走行のみ）と「走行終了」）
// 非常停止ボタンはこの画面に出さない。showRefLine=false（学習運転・スケジュール）は基準の線・カーブを出さない。

// ボタン（スタート・P・N・D）の色。ペダルの線（アクセル #78c8f0・ブレーキ #f07070）と区別する
const BUTTON_COLORS = { start: '#b58cf0', P: '#e0b030', N: '#58d0b0', D: '#f090c0' };
const BUTTON_LABELS = { start: 'スタート', P: 'P', N: 'N', D: 'D' };
// ペダルの縦軸（0〜100%）の上で、ボタンは「押していない = 0・押している = 50」
const BUTTON_PRESSED_LEVEL = 50;
// ログ画面の走行グラフも同じ色・高さを使う（logs.js は auto-drive.js の後に読み込まれる）
window.BUTTON_COLORS = BUTTON_COLORS;
window.BUTTON_LABELS = BUTTON_LABELS;
window.BUTTON_PRESSED_LEVEL = BUTTON_PRESSED_LEVEL;

// 計画の事前作成つきの開始確認ポップアップ（ConfirmStopPopup と同じ見た目。計画ができてから出すので、はい／いいえだけ）
function PrepareConfirmPopup({ message, onYes, onNo }) {
  const { INK, PAPER } = window;
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
        fontFamily: "'Meiryo UI', 'Meiryo', 'Yu Gothic UI', sans-serif",
      }}>
        <div style={{ fontSize: 24, fontWeight: 700, marginBottom: 24, color: INK }}>{message}</div>
        <div style={{ display: 'flex', gap: 16, justifyContent: 'center' }}>
          <div onClick={onYes} style={{
            display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
            border: '2px solid #943030', borderRadius: 6, padding: '10px 32px',
            background: '#2a1212', color: '#e05050',
            fontWeight: 700, fontSize: 18, fontFamily: 'inherit',
            cursor: 'pointer',
            boxShadow: '2px 2px 0 rgba(0,0,0,0.4)',
          }}>はい</div>
          <div onClick={onNo} style={{
            display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
            border: `2px solid ${INK}`, borderRadius: 6, padding: '10px 32px',
            background: PAPER, color: INK,
            fontWeight: 700, fontSize: 18, fontFamily: 'inherit', cursor: 'pointer',
            boxShadow: '2px 2px 0 rgba(0,0,0,0.4)',
          }}>いいえ</div>
        </div>
      </div>
    </div>
  );
}

function DriveMonitorScreen({
  showRefLine = true, profileMaxSpeed = null,
  // 枠3に全体進捗の図を出すか（既定は showRefLine と同じ。シーケンスは走行中の手順がモードのときだけ図を出す）
  showProgressFigure = showRefLine,
  driveStartPath = '/api/v1/drive/start', driveStartBody = null,
  // 計画の事前作成つきの開始（自動運転画面だけ）。{kind, id, limit_s?}。null なら従来の開始（学習画面など）
  prepareBody = null,
  driveStopPath = '/api/v1/drive/stop',
  confirmStartMessage = '開始しますか？', resultPanel = null,
  progressLabel = null,
  // この画面が扱うジョブの種類（learning / mode_drive / schedule。配列も可）。別の種類のジョブが走っているときは開始を止める
  jobKind = null,
  // シーケンスの画面: 段の切れ目（ジョブが無い間）もシーケンスが動いていれば「実行中」とみなす
  seqMode = false,
  busyLabel = null, stopLabel = '走行終了', confirmStopMessage = '走行を終了しますか？',
  startedToastMessage = '走行を開始しました',
  startDisabledReason = null,
  // シーケンスの開始前の枠3・基準の線に使う（選んだシーケンスの ID。走っていない間だけ使う）
  seqPreviewId = null,
  // 学習の画面だけ: 中断・終了のあとも、最後の走行の線を次のジョブが始まるまで残す
  keepTraceAfterStop = false,
  // 状態の行の「区間」に出す文字（学習の画面は今の段の名前。無ければ走行ログの区間名）
  sectionLabel = null,
}) {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, realtimeData, realtimeBuf, activeModeId, activeModeName, activeModeKind, robotState, setNavLock, setNav, recheckSelection } = useContext(window.AppContext);
  const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH, Box, Btn } = window;

  const [modeDetail, setModeDetail] = useState(null);
  const [nowS, setNowS] = useState(0);
  const [confirmStop, setConfirmStop] = useState(false);
  const [confirmStart, setConfirmStart] = useState(false);
  const [prepResp, setPrepResp] = useState(null);       // POST /drive/prepare の応答（WebSocket の prepare が届くまでの代わり）
  const [startingGo, setStartingGo] = useState(false);  // 「はい」を押してから prepare が waiting / null になるまで
  // この画面が計画の準備を始めて、まだ「はい／いいえ」に答えていない（準備中〜確認ポップアップ）
  const [prepOwned, setPrepOwned] = useState(false);
  const prepOwnedRef = useRef(false);                   // 画面を離れるときの取り消し用（上と同じ値）
  const [prepErr, setPrepErr] = useState(null);         // 準備に失敗した理由（次の開始まで状態の行に残す）
  const wasWaitingRef = useRef(false);
  const [isDriving, setIsDriving] = useState(false);
  const driveStartTimeRef = useRef(null);
  const pausedRef = useRef(false);
  // モード走行の時刻は、ランナーのモードの時刻（mode_time_s）だけで進める（段 A・A'）。
  //   モード開始前（準備・走行前チェック）は経過 0 のまま止め、モードが終わった後（停車まで減速など）は最後の時刻で止める
  const modeJobIdRef = useRef(null);   // 今見ているジョブ（変わったら下の印を戻す）
  const modeSeenRef = useRef(false);   // このジョブで mode_time_s が一度でも来たか
  const lastModeTRef = useRef(null);   // 最後に来た mode_time_s
  const frozenRef = useRef(null);      // モード終了後に止めておく時刻（止めていなければ null）
  const preStopRef = useRef({ jobId: null, section: null, ref: null, dev: null });   // 停止の流れ（STOPPING）の前の最後の区間・基準・偏差
  const [frozenS, setFrozenS] = useState(null);
  // 実測・ペダルの点（受け取ったときの経過秒 t を 1 回だけ決めて持つ。一時停止中も過去の点がずれない）
  const ptsRef = useRef({ jobId: null, lastTs: 0, list: [] });
  const keptTraceRef = useRef(null);   // keepTraceAfterStop: 走行中の最後の線（{ jobId, nowS, speed, accel, brake }）

  // 実行中のジョブ（WebSocket の job。終わると null）。この画面のジョブか・別のジョブかを分ける
  const liveJob = realtimeData?.job ?? null;
  const seqActive = seqMode && realtimeData?.sequence?.state === 'running';
  const ownKinds = jobKind ? [].concat(jobKind) : null;
  const ownJobNow = !!liveJob && liveJob.kind !== 'interactive' && (!ownKinds || ownKinds.includes(liveJob.kind));
  const ownJob = ownJobNow || seqActive;
  pausedRef.current = !!liveJob?.paused;
  const otherJob = !!liveJob && !ownJob;
  // 計画の事前作成（WebSocket の prepare）。waiting＝「はい」の後で計画がまだ終わっていない（この画面が待っている）
  const prepare = realtimeData?.prepare ?? null;
  const prepWaiting = !!prepareBody && prepare?.state === 'waiting';
  // この画面のポップアップが無いのに計画の準備中（別の画面が開いている）
  const otherPrep = !!prepareBody && !!prepare && prepare.state !== 'waiting' && prepare.state !== 'error'
    && !prepOwned && !startingGo;
  // 準備中（計画づくり）と、計画ができて確認ポップアップを出している間
  const prepNow = prepResp && !prepare ? prepResp : prepare;
  const prepReady = prepOwned && prepNow?.state === 'ready';
  const busy = ownJob || prepWaiting;
  // 初期化ページの初期化が済んでいないと、モード走行・学習は始められない（D-15c。サーバーも 409 で断る）。
  // シーケンスはモードの段があるときだけ断られるので、画面では止めずサーバーの案内に任せる
  const needsInit = !seqMode && ownKinds != null && ownKinds.length === 1
    && (ownKinds[0] === 'learning' || ownKinds[0] === 'mode_drive');
  const notInitialized = needsInit && realtimeData?.initialized === false;
  // モード走行・スケジュール（シーケンスの各段を含む）は、時刻をモードの時刻（mode_time_s）に限る。学習運転だけ従来どおり t_s。
  //   ジョブ単位で決める（走っているジョブの種類。無いときは画面の種類）。準備中（計画作り）は経過 0 のまま止める
  const modeTimed = liveJob
    ? (liveJob.kind === 'mode_drive' || liveJob.kind === 'schedule')
    : (ownKinds != null && ownKinds.every(k => k === 'mode_drive' || k === 'schedule'));

  // ジョブの実行中（初期化・走行・終了処理を含む）は他ページへの離脱をロック
  useEffect(() => {
    setNavLock(busy || prepOwned);
    return () => setNavLock(false);
  }, [busy, prepOwned]);

  // 画面を離れるとき、準備中・ポップアップが開いたまま（未回答）なら準備を取り消す（ベストエフォート）
  useEffect(() => () => {
    if (prepOwnedRef.current && !window.__drvViewer) {
      try { fetch('/api/v1/drive/prepare/cancel', { method: 'POST', keepalive: true, headers: window.drvControlHeaders() }); } catch (e) { /* 無視 */ }
    }
  }, []);

  // 「はい」の後の経過: waiting か null になったら「はい直後」の印を戻す。waiting 中に error になったら 1 回だけ知らせる
  useEffect(() => {
    if (!prepareBody) return;
    if (prepare?.state === 'waiting') {
      wasWaitingRef.current = true;
      setStartingGo(false);
    } else if (!prepare) {
      wasWaitingRef.current = false;
      setStartingGo(false);
    } else if (prepare.state === 'error' && wasWaitingRef.current) {
      wasWaitingRef.current = false;
      setStartingGo(false);
      window.showToast(`計画を作れませんでした: ${prepare.message ?? ''}`, 'error');
    }
  }, [prepare?.state]);

  // 準備に失敗したとき: 1 回だけ知らせ、準備を取り消して通常の画面に戻す（理由は状態の行に残す）
  useEffect(() => {
    if (!prepareBody || !prepOwnedRef.current) return;
    if (prepare?.state !== 'error') return;
    const msg = prepare.message ?? '';
    prepOwnedRef.current = false;
    setPrepOwned(false);
    setPrepResp(null);
    setPrepErr(msg);
    window.showToast(`準備できませんでした: ${msg}`, 'error');
    apiFetch('POST', '/api/v1/drive/prepare/cancel');
  }, [prepare?.state, prepOwned]);

  // 走行中（RUNNING）のみ経過時間を進める。グラフの横スクロールを滑らかにするため、
  // 整数秒の setInterval ではなく requestAnimationFrame で小数秒（nowS）を更新する。
  useEffect(() => {
    const driving = robotState === 'RUNNING';
    if (!driving) {
      driveStartTimeRef.current = null;
      modeSeenRef.current = false;
      lastModeTRef.current = null;
      frozenRef.current = null;
      setFrozenS(null);
      setIsDriving(false);
      setNowS(0);
      return;
    }
    // モード時刻で動くジョブは、モードの時刻が来るまで（下の置き直しで）起点を置かない
    if (!driveStartTimeRef.current && !modeTimed) {
      driveStartTimeRef.current = Date.now();
      setIsDriving(true);
    }
    let raf;
    let last = Date.now();
    const tick = () => {
      const now = Date.now();
      // 一時停止中はモードの時刻が止まるので、起点を進めて経過秒を止める
      if (driveStartTimeRef.current && pausedRef.current) driveStartTimeRef.current += now - last;
      last = now;
      if (driveStartTimeRef.current) setNowS((now - driveStartTimeRef.current) / 1000);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [robotState, modeTimed]);

  // モード走行は、ランナーの sample のモードの時刻（mode_time_s）に合わせて起点を置き直す。
  // RUNNING には計画を作る時間（数分）も入るので、壁時計だけだと経過・進捗・グラフの位置がその分ずれる（2026-11-16 段9）。
  // 時刻が飛んだとき（D 待ちでモードを止めた等）だけ置き直し、普段は壁時計でなめらかに進める。
  useEffect(() => {
    if (modeTimed) {
      const jid = liveJob?.job_id ?? null;
      if (modeJobIdRef.current !== jid) {
        modeJobIdRef.current = jid;
        modeSeenRef.current = false;
        lastModeTRef.current = null;
        frozenRef.current = null;
        setFrozenS(null);
        driveStartTimeRef.current = null;   // 別のジョブ（シーケンスの次の段など）は、モードの時刻が来るまで経過 0
        setIsDriving(false);
        setNowS(0);
      }
      if (robotState !== 'RUNNING') return;
      const m = liveJob?.sample?.mode_time_s;
      if (m == null) {
        // 開始前は何もしない（経過 0・起点なし）。モードが終わった後は最後の時刻を覚えておく（経過・赤線だけが使う。横軸は壁時計で進める）
        // 中断（stop_requested）後の停止の流れは覚えない
        if (!liveJob?.stop_requested && modeSeenRef.current && frozenRef.current == null && lastModeTRef.current != null) {
          frozenRef.current = lastModeTRef.current;
          setFrozenS(lastModeTRef.current);
        }
        return;
      }
      modeSeenRef.current = true;
      lastModeTRef.current = m;
      if (frozenRef.current != null) { frozenRef.current = null; setFrozenS(null); }
      const anchorM = Date.now() - m * 1000;
      if (!driveStartTimeRef.current || liveJob?.paused || Math.abs(driveStartTimeRef.current - anchorM) > 300) {
        driveStartTimeRef.current = anchorM;
        setIsDriving(true);
      }
      return;
    }
    // 学習運転はモードの時刻が無いので、ランナーの経過（t_s。走行前チェックから）に合わせる
    const mt = liveJob?.sample?.mode_time_s ?? liveJob?.sample?.t_s;
    if (robotState !== 'RUNNING' || mt == null) return;
    const anchor = Date.now() - mt * 1000;
    if (!driveStartTimeRef.current || liveJob?.paused || Math.abs(driveStartTimeRef.current - anchor) > 300) {
      driveStartTimeRef.current = anchor;
      setIsDriving(true);
    }
  }, [realtimeData, robotState]);

  // 表示用の整数秒（mm:ss ラベルは秒単位で十分）
  // モード時刻のジョブ（走行モード・スケジュール・シーケンスの段）は、経過と枠3の赤線を次の時刻で止める
  //   (1) モードが終わった後は最後のモードの時刻（frozenS）、(2) 走行終了を押した後はその時の nowS。先に来た方
  const stopHoldRef = useRef({ jobId: null, s: null });
  const holdJobId = liveJob?.job_id ?? null;
  if (stopHoldRef.current.jobId !== holdJobId) stopHoldRef.current = { jobId: holdJobId, s: null };
  if (modeTimed && liveJob?.stop_requested) {
    if (stopHoldRef.current.s == null) stopHoldRef.current.s = nowS;
  } else {
    stopHoldRef.current.s = null;
  }
  const holdCands = modeTimed ? [frozenS, stopHoldRef.current.s].filter(v => v != null) : [];
  const shownNowS = holdCands.length > 0 ? Math.min(nowS, ...holdCands) : nowS;
  const elapsed = Math.floor(shownNowS);

  // 走っているモード走行のジョブがあれば、画面の選択より優先してそのモードの基準を読む（段 B）
  const jobModeId = liveJob?.kind === 'mode_drive' ? (liveJob.mode_id ?? null) : null;
  // シーケンスの開始前: 最初のモードの段の基準車速で枠3を描く（そのモードの段を走り出したときと同じ図。
  //   開始で図の形が変わらないようにする。CLAUDE.md「フロントエンド」）
  const [seqPreview, setSeqPreview] = useState(null);
  const previewActive = !!seqPreviewId && !liveJob && !seqActive;
  useEffect(() => {
    if (!seqPreviewId) { setSeqPreview(null); return; }
    let alive = true;
    (async () => {
      const seq = await apiFetch('GET', `/api/v1/sequences/${seqPreviewId}`);
      if (!alive || !seq) { if (alive) setSeqPreview(null); return; }
      const steps = seq.steps ?? [];
      const firstMode = steps.find(st => st.kind === 'mode' && !st.missing);
      const mode = firstMode ? await apiFetch('GET', `/api/v1/modes/${firstMode.ref_id}`) : null;
      if (!alive) return;
      const ref = mode?.reference_speed ?? [];
      const first = steps[0];
      setSeqPreview({ id: seqPreviewId,
        segs: ref.length > 0 ? [ref] : [],
        totalS: ref.length > 0 ? ref[ref.length - 1].time_s : 0,
        maxSpd: ref.reduce((m, p) => Math.max(m, p.speed_kmh), 0),
        firstModeId: first && first.kind === 'mode' && !first.missing ? first.ref_id : null });
    })();
    return () => { alive = false; };
  }, [seqPreviewId]);
  const previewOk = previewActive && seqPreview && seqPreview.id === seqPreviewId;
  const refModeId = jobModeId ?? (activeModeKind === 'mode' ? activeModeId : null)
    ?? (previewOk ? seqPreview.firstModeId : null);
  useEffect(() => {
    if (refModeId) {
      setModeDetail(prev => (prev && prev.id === refModeId ? prev : null));
      apiFetch('GET', `/api/v1/modes/${refModeId}`).then(d => {
        if (d) setModeDetail(d);
        else if (refModeId === activeModeId) recheckSelection?.();   // 見つからない（404）なら選択を外す（段 D）
      });
    } else {
      setModeDetail(null);
    }
  }, [refModeId]);

  // 走っているスケジュールの総時間（経過の「/」の右に出す。シーケンスのスケジュールの段も同じ）
  const jobScheduleId = liveJob?.kind === 'schedule' ? (liveJob.schedule_id ?? null) : null;
  const [scheduleTotalS, setScheduleTotalS] = useState(null);
  useEffect(() => {
    if (!jobScheduleId) { setScheduleTotalS(null); return; }
    let alive = true;
    apiFetch('GET', `/api/v1/schedules/${jobScheduleId}`).then(d => {
      if (alive) setScheduleTotalS(d && d.total_duration > 0 ? d.total_duration : null);
    });
    return () => { alive = false; };
  }, [jobScheduleId]);

  // 開始: 確認ポップアップ → POST。409/422/503 などは apiFetch が「何が起きたか・次にすること」を出す
  async function handleStart() {
    if (!prepareBody) { setConfirmStart(true); return; }
    // 計画の事前作成を始め、「準備中…」を出す。計画ができたら確認ポップアップを出す（失敗は apiFetch が知らせる）
    setPrepErr(null);
    setPrepResp(null);
    prepOwnedRef.current = true;
    setPrepOwned(true);
    const r = await apiFetch('POST', '/api/v1/drive/prepare', prepareBody);
    if (!r) { prepOwnedRef.current = false; setPrepOwned(false); return; }
    setPrepResp(r);
  }

  async function handleConfirmStartNo() {
    prepOwnedRef.current = false;
    setPrepOwned(false);
    setPrepResp(null);
    if (prepareBody) await apiFetch('POST', '/api/v1/drive/prepare/cancel');
  }

  async function handlePrepareGo() {
    prepOwnedRef.current = false;
    setPrepOwned(false);
    setPrepResp(null);
    setStartingGo(true);
    const r = await apiFetch('POST', '/api/v1/drive/prepare/go');
    if (!r) { setStartingGo(false); return; }
    if (r.state === 'waiting') window.showToast('計画ができしだい開始します', 'success');
    else window.showToast(startedToastMessage, 'success');
  }

  async function handleConfirmStartYes() {
    if (prepareBody) { await handlePrepareGo(); return; }
    setConfirmStart(false);
    const r = await apiFetch('POST', driveStartPath, driveStartBody);
    if (r) window.showToast(startedToastMessage, 'success');
  }

  async function handleStop() {
    // 計画の作成待ち（ジョブはまだ無い）の停止は準備の取り消し
    if (prepWaiting && !liveJob) {
      const rc = await apiFetch('POST', '/api/v1/drive/prepare/cancel');
      setConfirmStop(false);
      if (rc) window.showToast('計画の作成を取り消しました', 'success');
      return;
    }
    const r = await apiFetch('POST', driveStopPath);
    setConfirmStop(false);
    if (r) window.showToast('停止を要求しました（原点復帰などの終了処理に少しかかります）', 'success');
  }

  // 一時停止 / 再開（モード走行だけ。ランナーへ SIGUSR1 / SIGUSR2）
  async function handlePauseToggle() {
    const r = await apiFetch('POST', liveJob?.paused ? '/api/v1/drive/resume' : '/api/v1/drive/pause');
    if (r) window.showToast(r.paused ? '一時停止を要求しました' : '再開を要求しました', 'success');
  }

  const buf = realtimeBuf.current;
  const driveStart = driveStartTimeRef.current;
  // axis 2 (開度) は走行後のバッファデータのみ。
  //   点は受け取ったときの経過秒 t を 1 回だけ決めて持つ（ts − driveStart で毎回計算し直すと、一時停止中に左へ流れる）。
  //   t は sample.mode_time_s があればそれ、無ければ（STOPPING・学習運転など）壁時計 (ts − 起点)/1000
  const pts = ptsRef.current;
  const ptsJobId = liveJob?.job_id ?? null;
  if (pts.jobId !== ptsJobId || !driveStart) { pts.jobId = ptsJobId; pts.list = []; pts.lastTs = 0; }
  if (driveStart) {
    const fresh = [];
    for (let i = buf.length - 1; i >= 0 && buf[i].ts > pts.lastTs; i--) fresh.push(buf[i]);
    fresh.reverse();
    for (const d of fresh) {
      pts.lastTs = d.ts;
      if (d.ts < driveStart) continue;
      if (d.job?.job_id != null && liveJob?.job_id != null && d.job.job_id !== liveJob.job_id) continue;
      const m = d.job?.sample?.mode_time_s;
      pts.list.push({ ...d, t: m != null ? m : (d.ts - driveStart) / 1000 });
    }
    if (pts.list.length > 600) pts.list.splice(0, pts.list.length - 600);
  }
  const recent = driveStart ? pts.list.slice(-300) : [];

  // Graph constants
  const VW = 900, PL = 50, PR = 16, PH1 = 120, PH2 = 120, PH3 = 60;
  const PW = VW - PL - PR;
  const G_PAD_T = 8, G_PAD_B = 2;

  function toX(i, len) { return PL + (i / Math.max(1, len - 1)) * PW; }
  function toXFull(frac) { return PL + frac * PW; }
  function toY(val, valMax, height, padT = G_PAD_T, padB = G_PAD_B) {
    return (height - padB) - (Math.min(val, valMax) / valMax) * (height - padT - padB);
  }

  function polyline(pts, color, strokeW = 2, dasharray) {
    if (pts.length < 2) return null;
    const d = pts.map((p, i) => `${i === 0 ? 'M' : 'L'}${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ');
    return <path d={d} fill="none" stroke={color} strokeWidth={strokeW} strokeDasharray={dasharray} strokeLinejoin="round" strokeLinecap="round" />;
  }

  // 現在位置ポインタ: 中央(frac=0.5)に固定し、現在のセンサ値で上下にのみ動く。
  // 軌跡(polyline)は左へ流れ、その先端にこの●が常に乗る（走行前=0でも表示）。
  // SVG は preserveAspectRatio="none" で非等比に伸縮するため、円を <circle> で描くと
  // グラフの縦横比に応じて楕円に歪む（軸3を持たない学習運転ページで顕著）。
  // 固定 px サイズの HTML 要素をパーセント位置で重ねることで、常に真円を保つ。
  function marker(val, valMax, height, color) {
    return <div style={{
      position: 'absolute',
      left: `${(toXFull(0.5) / VW) * 100}%`,
      top: `${(toY(val, valMax, height) / height) * 100}%`,
      width: 12, height: 12, boxSizing: 'border-box',
      borderRadius: '50%', background: color, border: `1.5px solid ${PAPER}`,
      transform: 'translate(-50%, -50%)', pointerEvents: 'none',
    }} />;
  }

  // 全体進捗: profile data
  // showRefLine=false（学習運転・スケジュール）では基準を使わない（選択中モードの全体時間を出さない）
  const refProfile = showRefLine ? (modeDetail?.reference_speed ?? []) : [];
  const totalDurS = refProfile.length > 0
    ? refProfile[refProfile.length - 1].time_s
    : (jobScheduleId && scheduleTotalS ? Math.floor(scheduleTotalS) : 0);

  // 枠3の図用の基準（速度グラフの基準線・経過の合計とは別。シーケンスは走行中がモード手順で、読み込み済みの詳細がそのモードのときだけ）
  const figProfile = showRefLine
    ? refProfile
    : (showProgressFigure && liveJob?.kind === 'mode_drive' && modeDetail && modeDetail.id === liveJob.mode_id
        ? (modeDetail.reference_speed ?? []) : []);
  // シーケンスの開始前は最初のモードの段の図。走行中は今の段のモードの図
  const figSegs = figProfile.length > 0 ? [figProfile] : (previewOk && showProgressFigure ? seqPreview.segs : []);
  const figTotalS = figProfile.length > 0 ? figProfile[figProfile.length - 1].time_s
    : (figSegs.length > 0 ? seqPreview.totalS : 0);
  // 開始前のシーケンスの先頭の段がモードなら、上の図に基準の線を出す（走行モードの開始前と同じ）
  const preRefProfile = (!showRefLine && previewOk && seqPreview.firstModeId && modeDetail
    && modeDetail.id === seqPreview.firstModeId) ? (modeDetail.reference_speed ?? []) : [];
  // シーケンスの走行中は、走っている段がモードで詳細が読み込み済みなら、その基準を 1 軸目の線に使う
  const liveRefProfile = (!showRefLine && seqMode) ? figProfile : [];
  const refLineProfile = showRefLine ? refProfile : (liveRefProfile.length > 0 ? liveRefProfile : preRefProfile);
  const profileSpeedMax = refLineProfile.reduce((m, p) => Math.max(m, p.speed_kmh), 0);
  const maxSpeed = Math.max(100, Math.ceil(Math.max(profileSpeedMax, profileMaxSpeed ?? 0) / 20) * 20);
  // 枠3の図の縦の上限。走行モードの画面と同じ式を、図に描くモード（figProfile）の基準で計算する
  //   （シーケンスは refProfile が空なので maxSpeed を使うと 100 固定になり、走行モードと形が変わる）
  const figSpeedMax = figProfile.length > 0 ? figProfile.reduce((m, p) => Math.max(m, p.speed_kmh), 0)
    : (figSegs.length > 0 ? seqPreview.maxSpd : 0);
  const figMaxSpeed = Math.max(100, Math.ceil(Math.max(figSpeedMax, profileMaxSpeed ?? 0) / 20) * 20);

  function getRefSpeedAtTime(elapsedS) {
    const refProfile = refLineProfile;
    if (refProfile.length === 0 || elapsedS < 0) return null;
    if (elapsedS <= refProfile[0].time_s) return refProfile[0].speed_kmh;
    const last = refProfile[refProfile.length - 1];
    if (elapsedS >= last.time_s) return last.speed_kmh;

    // 制御側(drive_loop.py の _ref_speed_at)と同じ線形補間で滑らかに表示する
    let lo = 0, hi = refProfile.length - 1;
    while (lo < hi - 1) {
      const mid = (lo + hi) >> 1;
      if (refProfile[mid].time_s <= elapsedS) lo = mid; else hi = mid;
    }
    const p0 = refProfile[lo];
    const p1 = refProfile[hi];
    const span = p1.time_s - p0.time_s;
    const frac = span > 0 ? (elapsedS - p0.time_s) / span : 0;
    return p0.speed_kmh + (p1.speed_kmh - p0.speed_kmh) * frac;
  }

  // Axis 1: 30秒ウィンドウ・中央固定プレイヘッド（時間軸基準）
  // 現在時刻(nowS)を常にグラフ中央(frac=0.5)に固定し、波形は左へ流れる。
  // 左半分=過去(実測+基準)、中央=現在、右半分=未来(基準の先読み)。
  const WINDOW_S = 30;
  const HALF_S = WINDOW_S / 2;
  // 中断・終了のあと（keepTraceAfterStop）は、最後の走行の線とその時刻の窓をそのまま見せる
  const keptView = keepTraceAfterStop && !isDriving && keptTraceRef.current
    && (!liveJob || liveJob.job_id === keptTraceRef.current.jobId) ? keptTraceRef.current : null;
  const viewNowS = keptView ? keptView.nowS : nowS;
  const windowStartS = viewNowS - HALF_S;   // クランプしない（now を常に中央に保つ）。走行前は nowS=0 → [-15, +15]
  const windowEndS   = viewNowS + HALF_S;

  const toFrac = elapsedS => (elapsedS - windowStartS) / WINDOW_S;

  // 基準車速: プロファイルから直接サンプリング（モードロード済なら常に表示）
  const speedRef_pts = refLineProfile.length > 0
    ? Array.from({ length: 150 }, (_, i) => {
        const frac  = i / 149;
        const t     = windowStartS + frac * WINDOW_S;
        return { x: toXFull(frac), y: toY(getRefSpeedAtTime(t) ?? 0, maxSpeed, PH1) };
      })
    : [];

  // 実測の軌跡（車速・開度）。サーバー（ws.py broadcast_loop）は値が変わらない間は WebSocket を送らないので、
  // 点が来ない間＝値は直前のまま。ボタン押下中（ペダル・車速が一定）などに線が中央の●の手前で途切れないよう、
  //   ・点の間が空いたら、次の点の直前まで直前の値で横に引く（斜めの線にしない）
  //   ・最後の点から現在時刻（中央の●）まで、最後の値で横に引く
  // （2026-11-20 ProblemReport_20261120「軌跡が途切れる」）
  const HOLD_GAP_S = 0.25;   // WS は 0.1 s 周期。これより空いたら「送られなかった＝値は同じ」とみなす
  function trace(key, max, h) {
    if (!isDriving || !driveStart) return [];
    const pts = [];
    let prev = null;
    const push = (t, v) => {
      const frac = toFrac(t);
      if (frac >= 0 && frac <= 1) pts.push({ x: toXFull(frac), y: toY(v, max, h) });
    };
    for (const d of recent) {
      const t = d.t;
      if (prev && t - prev.t > HOLD_GAP_S) push(t - 0.1, prev.v);
      push(t, d[key]);
      prev = { t, v: d[key] };
    }
    if (prev && nowS > prev.t) push(nowS, prev.v);
    return pts;
  }

  // 実車速: 同一時間軸に位置合わせ（走行中のみ）。最新点は中央(frac=0.5)に張り付く
  const speedAct_pts = keptView ? keptView.speed : trace('actual_speed_kmh', maxSpeed, PH1);

  // Axis 2: openings — axis 1 と共通の時間軸
  const accelPts = keptView ? keptView.accel : trace('accel_opening', 100, PH2);
  const brakePts = keptView ? keptView.brake : trace('brake_opening', 100, PH2);
  if (keepTraceAfterStop && isDriving && driveStart) {
    keptTraceRef.current = { jobId: liveJob?.job_id ?? null, nowS, speed: speedAct_pts, accel: accelPts, brake: brakePts };
  }

  // T4: マーカーと進捗は走行中のみ
  // Axis 3: PAD_T=8, PAD_B=8 → speed=0 が baseline(y=PH3-8=52)に一致
  const G3_PAD_T = 8, G3_PAD_B = 8;
  const profileSegs = figSegs.map(seg => seg.map(p => ({
    x: toXFull(figTotalS > 0 ? p.time_s / figTotalS : 0),
    y: toY(p.speed_kmh, figMaxSpeed, PH3, G3_PAD_T, G3_PAD_B),
  })));
  const progressFrac = isDriving && figTotalS > 0 ? Math.min(1, shownNowS / figTotalS) : 0;
  const markerX = toXFull(progressFrac);

  const fmt = s => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;

  // 目盛りの数字・単位は SVG の外（HTML の絶対配置）に出す。SVG は preserveAspectRatio="none" で
  // 非等比に伸縮するため、<text> を中に置くと文字が縦長につぶれる。線だけ SVG に残す。
  // 親要素は position: relative（枠内の div）。位置は目盛りの y を高さに対する割合で指定する。
  function GraphSvg({ height, children, yMax, unit, unitLabel, padT = G_PAD_T, padB = G_PAD_B }) {
    const ticks = [0, 0.25, 0.5, 0.75, 1].map(f => ({
      v: Math.round(f * yMax),
      y: (height - padB) - f * (height - padT - padB),
    }));
    return (
      <>
        <svg viewBox={`0 0 ${VW} ${height}`} width="100%" height="100%" preserveAspectRatio="none" style={{ display: 'block' }}>
          {ticks.map(({ v, y }) => (
            <line key={v} x1={PL} y1={y} x2={PL + PW} y2={y} stroke={HATCH} strokeWidth="1" strokeDasharray="3 4" />
          ))}
          {children}
        </svg>
        {unitLabel && (
          <div style={{ position: 'absolute', left: 2, top: 0, fontSize: 10, lineHeight: 1, color: INK_SOFT, pointerEvents: 'none' }}>{unitLabel}</div>
        )}
        {ticks.map(({ v, y }, i) => (
          <div key={v} style={{
            position: 'absolute', left: 0, width: `calc(${(PL / VW) * 100}% - 4px)`,
            top: `${(y / height) * 100}%`, textAlign: 'right',
            fontSize: 11, lineHeight: 1, whiteSpace: 'nowrap', color: INK, pointerEvents: 'none',
            // 最下段（0）は線の上側に置き、それ以外は線の中央に合わせる
            transform: i === 0 ? 'translateY(-110%)' : 'translateY(-50%)',
          }}>{v}{unit}</div>
        ))}
      </>
    );
  }

  // 一時停止はモード走行のジョブだけ（学習・スケジュールでは出さない）
  const canPause = !!liveJob && liveJob.kind === 'mode_drive' && (!ownKinds || ownKinds.includes('mode_drive'));
  // 状態の 1 行: 状態 ｜ 区間 ｜ 経過/全体 ｜ 最大|偏差| ｜ p95
  const dev = liveJob?.dev_stats ?? null;
  const smp = liveJob?.sample ?? null;
  const planWaitingOnly = prepWaiting && !liveJob;   // 計画の作成待ち（ジョブはまだ始まっていない）
  const statusText = planWaitingOnly
    ? `計画を作成中（${prepare.done ?? 0}/${prepare.total ?? 0}）`
    : (prepOwned && !liveJob)
    ? '準備中'
    : (prepErr != null && !liveJob)
    ? `準備できませんでした: ${prepErr}`
    : liveJob
    ? (liveJob.paused ? '一時停止中' : window.jobPhaseShort(liveJob))
    : (robotState ?? '—');
  const isStoppingFlow = liveJob?.sample?.section === 'STOPPING';
  if (preStopRef.current.jobId !== (liveJob?.job_id ?? null)) preStopRef.current = { jobId: liveJob?.job_id ?? null, section: null, ref: null, dev: null };
  const sectionTextNow = smp ? (sectionLabel || smp.pattern || window.SECTION_LABEL?.[smp.section] || smp.section || '—') : '—';
  if (liveJob && smp && !isStoppingFlow) preStopRef.current.section = sectionTextNow;
  const sectionText = isStoppingFlow ? (preStopRef.current.section ?? '—') : sectionTextNow;
  const timeText = (isDriving && !planWaitingOnly)
    ? (totalDurS > 0 ? `${fmt(elapsed)}/${fmt(totalDurS)}` : `${fmt(elapsed)}/—`)
    : '—';
  const statusLine = [
    `状態 ${statusText}`,
    `区間 ${sectionText}`,
    `経過 ${timeText}`,
    `最大|偏差| ${dev ? dev.max.toFixed(2) : '—'}`,
    `p95 ${dev ? dev.p95.toFixed(2) : '—'}`,
  ].join(' ｜ ');

  // ボタン押下の矩形（ペダルの図の時間軸。start_s/end_s はサンプルの mode_time_s ?? t_s と同じ軸）。
  // 押し始めで 0 → BUTTON_PRESSED_LEVEL に立ち、押している間は横に伸び、離して 0 に落ちる。押している最中は現在時刻まで伸ばす
  const buttonRects = (isDriving && liveJob?.button_events ? liveJob.button_events : []).map((e, i) => {
    const xs = toXFull(toFrac(e.start_s));
    const xe = Math.max(xs, toXFull(toFrac(e.end_s ?? nowS)));   // 逆向きの箱は描かない（幅 0）
    if (xe < PL || xs > PL + PW) return null;
    const x0 = Math.max(PL, xs), x1 = Math.min(PL + PW, xe);
    const y0 = toY(0, 100, PH2), y1 = toY(BUTTON_PRESSED_LEVEL, 100, PH2);
    return {
      key: `${e.button}-${e.start_s}-${i}`, color: BUTTON_COLORS[e.button] ?? INK_SOFT, label: BUTTON_LABELS[e.button] ?? e.button,
      d: `M${xs.toFixed(1)},${y0.toFixed(1)} L${xs.toFixed(1)},${y1.toFixed(1)} L${xe.toFixed(1)},${y1.toFixed(1)} L${xe.toFixed(1)},${y0.toFixed(1)}`,
      cx: (x0 + x1) / 2, topY: y1, clipL: xs < PL, clipR: xe > PL + PW,
    };
  }).filter(Boolean);

  const rd = realtimeData;
  if (liveJob && smp && !isStoppingFlow) {
    const okRef = typeof rd?.ref_speed_kmh === 'number' && typeof rd?.actual_speed_kmh === 'number';
    preStopRef.current.ref = okRef ? rd.ref_speed_kmh : null;
    preStopRef.current.dev = okRef ? rd.actual_speed_kmh - rd.ref_speed_kmh : null;
  }

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 6, flex: 1, minHeight: 0 }}>

      {/* 枠1: speed graph — 車速とペダルは同じ高さ */}
      <Box style={{ padding: '6px 0 2px', flex: '1 1 0', minHeight: 0, display: 'flex', flexDirection: 'column' }}>
        <div style={{ flex: 1, minHeight: 0, position: 'relative' }}>
          <GraphSvg height={PH1} yMax={maxSpeed} unit="" unitLabel="km/h">
            {polyline(speedRef_pts, INK_SOFT, 1.5, '4 3')}
            {polyline(speedAct_pts, '#c8922a', 2.2)}
          </GraphSvg>
          {marker(rd.actual_speed_kmh, maxSpeed, PH1, '#c8922a')}
        </div>
        <div style={{ display: 'flex', gap: 20, fontSize: 12, color: INK_SOFT, paddingLeft: PL, paddingBottom: 4, alignItems: 'center' }}>
          {(showRefLine || preRefProfile.length > 0 || refLineProfile.length > 0) && (
            <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
              <svg width="28" height="10" style={{ flexShrink: 0 }}>
                <line x1="0" y1="5" x2="28" y2="5" stroke={INK_SOFT} strokeWidth="1.5" strokeDasharray="4 3" />
              </svg>
              基準
            </span>
          )}
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#c8922a" strokeWidth="2.2" />
            </svg>
            実測
          </span>
        </div>
      </Box>

      {/* 枠2: openings graph */}
      <Box style={{ padding: '6px 0 2px', flex: '1 1 0', minHeight: 0, display: 'flex', flexDirection: 'column' }}>
        <div style={{ flex: 1, minHeight: 0, position: 'relative' }}>
          <GraphSvg height={PH2} yMax={100} unit="%">
            {/* ボタンの帯は描画範囲（縦軸より右）で切り落とす。左端より前に始まった帯は左端から見える分だけ描く */}
            <defs><clipPath id="buttonBandClip"><rect x={PL} y={0} width={PW} height={PH2} /></clipPath></defs>
            <g clipPath="url(#buttonBandClip)">
              {buttonRects.map(r => (
                <path key={r.key} d={r.d} fill={r.color} fillOpacity="0.28" stroke={r.color} strokeWidth="1.5" strokeLinejoin="round" />
              ))}
            </g>
            {polyline(accelPts, '#78c8f0', 2.2)}
            {polyline(brakePts, '#f07070', 2.2)}
          </GraphSvg>
          {buttonRects.map(r => (
            <span key={`${r.key}-l`} style={{
              // 帯が描画範囲の端で切れているときは、ラベルを範囲の内側に寄せる（縦軸の目盛りに重ねない）
              position: 'absolute', top: `${(r.topY / PH2) * 100}%`,
              left: `${((r.clipL ? PL + 2 : r.clipR ? PL + PW - 2 : r.cx) / VW) * 100}%`,
              transform: `translate(${r.clipL ? '0' : r.clipR ? '-100%' : '-50%'}, -105%)`, fontSize: 11, lineHeight: 1, color: r.color,
              fontWeight: 700, whiteSpace: 'nowrap', pointerEvents: 'none',
            }}>{r.label}</span>
          ))}
          {marker(rd.accel_opening, 100, PH2, '#78c8f0')}
          {marker(rd.brake_opening, 100, PH2, '#f07070')}
        </div>
        {/* 共通時間軸ラベル（5秒ごと）。目盛りの線は SVG、数字は伸びないよう HTML で重ねる */}
        {(() => {
          const first = Math.max(0, Math.ceil(windowStartS / 5) * 5);
          const ticks = [];
          for (let t = first; t <= windowEndS; t += 5) ticks.push({ t, x: toXFull((t - windowStartS) / WINDOW_S) });
          const pct = x => `${(x / VW) * 100}%`;
          return (
            <div style={{ position: 'relative', height: 18 }}>
              <svg viewBox={`0 0 ${VW} 18`} width="100%" height="18" preserveAspectRatio="none" style={{ display: 'block' }}>
                <line x1={PL} y1={0} x2={PL + PW} y2={0} stroke={HATCH} strokeWidth="0.5" />
                {ticks.map(({ t, x }) => (
                  <line key={t} x1={x} y1={0} x2={x} y2={4} stroke={HATCH} strokeWidth="1" />
                ))}
              </svg>
              {ticks.map(({ t, x }) => (
                <span key={t} style={{ position: 'absolute', top: 5, left: pct(x), transform: 'translateX(-50%)', fontSize: 11, lineHeight: 1, color: INK_SOFT, pointerEvents: 'none' }}>{t}</span>
              ))}
              <span style={{ position: 'absolute', top: 5, left: pct(PL + PW + 8), fontSize: 11, lineHeight: 1, color: INK_SOFT, pointerEvents: 'none' }}>s</span>
            </div>
          );
        })()}
        <div style={{ display: 'flex', gap: 20, fontSize: 12, color: INK_SOFT, paddingLeft: PL, paddingBottom: 4, alignItems: 'center' }}>
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#f07070" strokeWidth="2.2" />
            </svg>
            ブレーキ
          </span>
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#78c8f0" strokeWidth="2.2" />
            </svg>
            アクセル
          </span>
          {['start', 'P', 'N', 'D'].map(k => (
            <span key={k} style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
              <svg width="14" height="10" style={{ flexShrink: 0 }}>
                <rect x="1" y="1" width="12" height="8" fill={BUTTON_COLORS[k]} fillOpacity="0.4" stroke={BUTTON_COLORS[k]} strokeWidth="1.2" />
              </svg>
              {BUTTON_LABELS[k]}
            </span>
          ))}
        </div>
      </Box>

      {/* 枠3: 進捗（高さ 0.5）— モード（シーケンスのモードの段も）: 全体進捗の図。図が無いとき: 段階表示・結果（resultPanel）
          【固定ルール】図は「走行モード」の画面と同じに描く（幅いっぱい・縦は figMaxSpeed・横に何も並べない）。
          図を出している間は resultPanel を並べない。シーケンスの進み具合は枠3ではなく画面上の見出し行に出す（AutoDriveScreen）。
          シーケンスで図の幅が縮む・形が変わる不具合を 3 回直している（2026-11-20）。変えるときは CLAUDE.md の「フロントエンド」節を読むこと */}
      <Box style={{ padding: '4px 10px', flex: '0.5 1 0', minHeight: 0, minWidth: 0, display: 'flex', gap: 12 }}>
        {figSegs.length > 0 ? (
          <div style={{ flex: '1 1 100%', minWidth: 0, display: 'flex', flexDirection: 'column' }}>
            <div style={{ flex: 1, minHeight: 0 }}>
              <svg viewBox={`0 0 ${VW} ${PH3}`} width="100%" height="100%" preserveAspectRatio="none" style={{ display: 'block' }}>
                <line x1={PL} y1={PH3 - G3_PAD_B} x2={PL + PW} y2={PH3 - G3_PAD_B} stroke={HATCH} strokeWidth="0.8" />
                {profileSegs.map((pts, si) => {
                  const d = pts.map((p, i) => `${i === 0 ? 'M' : 'L'}${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ');
                  return <path key={si} d={d} fill="none" stroke={INK_SOFT} strokeWidth="1.5" strokeDasharray="4 3" />;
                })}
                {isDriving && figTotalS > 0 && (
                  <line x1={markerX} y1={4} x2={markerX} y2={PH3 - 4} stroke="#f07070" strokeWidth="2" />
                )}
              </svg>
            </div>
          </div>
        ) : resultPanel ? (
          <div style={{ flex: '1 1 100%', minWidth: 0, minHeight: 0, overflowY: 'auto', fontSize: 13 }}>
            {resultPanel}
          </div>
        ) : <div style={{ flex: 1 }} />}
      </Box>

      {/* 枠4: 情報1行（高さ 0.5）— 左: 実車速・開度 | 中: 状態の 1 行 | 右: ボタン */}
      <div style={{ flex: '0.5 1 0', minHeight: 0, display: 'flex', gap: 8, alignItems: 'stretch', overflow: 'hidden' }}>
        <Box style={{ display: 'flex', gap: 8, alignItems: 'center', justifyContent: 'center', padding: '2px 8px', minHeight: 0, flexShrink: 0 }}>
          <div style={{ display: 'flex', alignItems: 'baseline', gap: 6, whiteSpace: 'nowrap' }}>
            <span style={{ fontSize: 12, color: INK_SOFT, letterSpacing: 1 }}>実車速</span>
            <span style={{ fontSize: 'clamp(22px, 3.6vh, 36px)', fontWeight: 700, lineHeight: 1, fontFamily: 'inherit' }}>
              {(typeof rd.actual_speed_kmh === 'number' ? rd.actual_speed_kmh : 0).toFixed(1)}
            </span>
            <span style={{ fontSize: 12, color: INK_SOFT }}>km/h</span>
          </div>
          <div style={{ fontSize: 12, color: INK_SOFT, whiteSpace: 'nowrap', lineHeight: 1.5 }}>
            {showRefLine && (
              isStoppingFlow
                ? (preStopRef.current.ref != null
                    ? `基準 ${preStopRef.current.ref.toFixed(1)} / 偏差 ${preStopRef.current.dev.toFixed(2)}`
                    : '基準 — / 偏差 —')
                : (rd.ref_speed_kmh ?? null) !== null
                ? `基準 ${rd.ref_speed_kmh.toFixed(1)} / 偏差 ${(rd.actual_speed_kmh - rd.ref_speed_kmh).toFixed(2)}`
                : '基準 — / 偏差 —'
            )}
            {showRefLine && <br />}
            {`アクセル ${rd.accel_opening.toFixed(1)}% / ブレーキ ${rd.brake_opening.toFixed(1)}%`}
          </div>
        </Box>
        <Box style={{ flex: 1, minWidth: 0, minHeight: 0, padding: '2px 12px', display: 'flex', alignItems: 'center' }}>
          <div style={{ minWidth: 0, fontSize: 'clamp(10px, 1.4vh, 15px)', whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis', color: INK }}>
            {statusLine}
          </div>
        </Box>
        <div style={{ display: 'flex', flexDirection: 'row', gap: 8, flex: '0 0 clamp(250px, 22%, 420px)', minHeight: 0 }}>
          {busy ? (
            <>
              {canPause && (
                <Btn big style={{ flex: 1, padding: '4px 10px', borderColor: '#b89030', color: '#f0c84a' }} disabled={!!liveJob?.stop_requested} onClick={handlePauseToggle}>
                  {liveJob?.paused ? '再開' : '一時停止'}
                </Btn>
              )}
              <Btn danger big style={{ flex: 1, padding: '4px 10px' }} disabled={!!liveJob?.stop_requested} onClick={() => setConfirmStop(true)}>
                {liveJob?.stop_requested ? '終了処理中' : stopLabel}
              </Btn>
            </>
          ) : otherJob ? (
            <Btn big disabled style={{ flex: 1, padding: '4px 10px' }}>他のジョブが実行中</Btn>
          ) : prepOwned ? (
            <Btn key="preparing" big disabled style={{ flex: 1, padding: '4px 10px' }}>準備中…</Btn>
          ) : otherPrep ? (
            <Btn big disabled style={{ flex: 1, padding: '4px 10px' }}>準備中です</Btn>
          ) : notInitialized ? (
            <>
              <Btn big disabled style={{ flex: 1, padding: '4px 10px' }}>開始</Btn>
              <Btn big primary style={{ flex: 1, padding: '4px 10px' }} onClick={() => setNav('init')}>初期化ページへ</Btn>
            </>
          ) : startDisabledReason ? (
            <Btn big disabled style={{ flex: 1, padding: '4px 10px' }}>開始（準備中）</Btn>
          ) : robotState !== 'READY' ? (
            // READY（待機）以外で「開始」すると 409 になるため押せなくする（非常停止中は画面が別になる）
            <Btn big disabled style={{ flex: 1, padding: '4px 10px' }}>開始できません（{robotState}）</Btn>
          ) : (
            <Btn key="start" big style={{ flex: 1, padding: '4px 10px', borderColor: '#3c8c3c', background: '#0e220e', color: '#68d468' }} onClick={handleStart}>開始</Btn>
          )}
        </div>
      </div>

      {confirmStop && React.createElement(window.ConfirmStopPopup, {
        message: confirmStopMessage,
        onYes: handleStop,
        onNo: () => setConfirmStop(false),
      })}

      {confirmStart && !prepareBody && React.createElement(window.ConfirmStopPopup, {
        message: confirmStartMessage,
        onYes: handleConfirmStartYes,
        onNo: () => setConfirmStart(false),
      })}

      {prepReady && !startingGo && (
        <PrepareConfirmPopup message={confirmStartMessage}
          onYes={handleConfirmStartYes} onNo={handleConfirmStartNo} />
      )}
    </div>
  );
}

// 終わったシーケンス（GET /api/v1/drive/status の last_sequence）。実行中のシーケンスが終わった瞬間に取りに行く
function useLastSequence(liveSeq) {
  const { useState, useEffect, useRef } = React;
  const [last, setLast] = useState(null);
  const prevRef = useRef(null);
  async function load() {
    try {
      const res = await fetch('/api/v1/drive/status');
      if (res.ok) { const j = await res.json(); setLast(j.last_sequence ?? null); }
    } catch (e) { /* 次の機会に */ }
  }
  useEffect(() => { load(); }, []);
  const liveKey = liveSeq?.state === 'running' ? liveSeq.sequence_id + liveSeq.started_at : null;
  useEffect(() => {
    if (prevRef.current && !liveKey) { load(); setTimeout(load, 800); setTimeout(load, 2500); }
    prevRef.current = liveKey;
  }, [liveKey]);
  return last;
}

const SEQ_STATE_INFO = {
  completed: { level: 'ok', label: '完了しました' },
  stopped: { level: 'warn', label: '停止しました' },
  error: { level: 'ng', label: '途中で止まりました' },
  emergency: { level: 'ng', label: '非常停止で止まりました' },
};
const SEQ_TINT = {
  ok:   { fg: '#68d468', bg: '#0e220e', border: '#2a6c2a' },
  warn: { fg: '#f0c84a', bg: '#221c08', border: '#b89030' },
  ng:   { fg: '#f07070', bg: '#220e0e', border: '#943030' },
};

// シーケンスの進み具合（「何段目/全体・何回目」）。seq は job_manager.SequenceRun.to_dict
function SequenceProgress({ seq, inline = false }) {
  const { INK, INK_SOFT } = window;
  const kind = seq.kind === 'schedule' ? 'スケジュール' : '走行モード';
  const Tag = inline ? 'span' : 'div';
  return (
    <Tag style={{ fontSize: 12, color: INK_SOFT, lineHeight: 1.5 }}>
      <b style={{ color: INK }}>シーケンス「{seq.name}」</b>
      {'　'}全体 <b style={{ color: INK }}>{seq.cycle}/{seq.cycles}</b> 周目
      {'　'}<b style={{ color: INK }}>{seq.step_no}/{seq.steps_total}</b> 段目（{kind}「{seq.ref_name}」{seq.step_repeat}/{seq.step_repeats} 回目）
      {'　'}通算 <b style={{ color: INK }}>{seq.run_no}/{seq.runs_total}</b> 回
    </Tag>
  );
}

function SequenceResultBanner({ seq, onDismiss, onLogs }) {
  const { INK, INK_SOFT, Btn } = window;
  const info = SEQ_STATE_INFO[seq.state] ?? { level: 'ng', label: seq.state };
  const t = SEQ_TINT[info.level];
  const done = (seq.results || []).filter(r => r.final_status === 'completed').length;
  return (
    <div style={{ border: `1.3px solid ${t.border}`, background: t.bg, borderRadius: 4, padding: '4px 10px', display: 'flex', gap: 10, alignItems: 'flex-start' }}>
      <div style={{ flex: 1, minWidth: 0, fontSize: 12.5, lineHeight: 1.5 }}>
        <div style={{ color: t.fg, fontWeight: 700 }}>シーケンス「{seq.name}」: {info.label}（{done}/{seq.runs_total} 回が完了）</div>
        {seq.message && <div style={{ color: INK }}>{seq.message}</div>}
        {info.level !== 'ok' && <div style={{ color: INK_SOFT }}>次にすること: 原因を進行のメッセージ（ログ画面）で確認し、必要なら最初からやり直してください（残りの段は走っていません）。</div>}
      </div>
      <div style={{ display: 'flex', gap: 6, flexShrink: 0 }}>
        {onLogs && <Btn onClick={onLogs}>ログへ</Btn>}
        {onDismiss && <Btn ghost onClick={onDismiss}>閉じる</Btn>}
      </div>
    </div>
  );
}

// 自動運転（RPD D-15）: 走行モード・タイムスケジュール・シーケンスを選んで開始。プラン・一時停止は無い。
//   モード: 常に手順24（スタート・D・N のボタンを自動で押して走る）
//   タイムスケジュール（段6）: ペダル開度を時刻で再生する開ループ。ボタンはボタン名で指定したイベントだけ押す
//   シーケンス（段6）: 段（モード・スケジュール）を順に、繰り返し回数どおり 1 回ずつ走らせる。どれかが失敗したら残りは走らせない
function AutoDriveScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeProfileId, activeModeId: selModeId, activeModeName: selModeName, activeModeKind: selModeKind, setNav, realtimeData } = useContext(window.AppContext);
  const { ValidationPopup, Note, INK, INK_SOFT, INK_MUTE, Pill } = window;

  const [popup, setPopup] = useState(null);
  const [profile, setProfile] = useState(null);
  const [dismissedJobId, setDismissedJobId] = useState(null);
  const [dismissedSeq, setDismissedSeq] = useState(null);
  // 走っているジョブ・シーケンスがあれば、画面の選択（ブラウザの localStorage）より優先してそれを表示・使う（段 B）
  const runJob = realtimeData?.job ?? null;
  const runSeq = realtimeData?.sequence?.state === 'running' ? realtimeData.sequence : null;
  let activeModeKind = selModeKind, activeModeId = selModeId, activeModeName = selModeName;
  let followsRunning = false;
  if (runSeq) {
    activeModeKind = 'sequence'; activeModeId = runSeq.sequence_id ?? selModeId; activeModeName = runSeq.name ?? selModeName;
    followsRunning = true;
  } else if (runJob?.kind === 'mode_drive' && runJob.mode_id) {
    activeModeKind = 'mode'; activeModeId = runJob.mode_id; activeModeName = runJob.mode_name ?? selModeName;
    followsRunning = true;
  } else if (runJob?.kind === 'schedule' && runJob.schedule_id) {
    activeModeKind = 'schedule'; activeModeId = runJob.schedule_id; activeModeName = runJob.schedule_name ?? selModeName;
    followsRunning = true;
  }
  const isSchedule = activeModeKind === 'schedule';
  const isSequence = activeModeKind === 'sequence';
  const lastJob = window.useLastJob(realtimeData?.job ?? null);
  const lastSeq = useLastSequence(realtimeData?.sequence ?? null);

  // 開発用: URL に ?dev_limit_s=60 を付けると、モード走行（シーケンスのモードの段を含む）を先頭 60 秒で打ち切る（API の limit_s。動作確認用）
  const devLimitS = (() => {
    try {
      const v = parseFloat(new URLSearchParams(location.search).get('dev_limit_s'));
      return Number.isFinite(v) && v > 0 ? v : null;
    } catch { return null; }
  })();

  useEffect(() => {
    setPopup(null);
    if (!activeProfileId) { setPopup('no_profile'); return; }
    // 車の表示（上の行）はモードの選択が無くても出す（走っているジョブを開いたとき「車: —」にならないように）
    apiFetch('GET', `/api/v1/profiles/${activeProfileId}`).then(p => {
      if (!p) return;
      setProfile(p);
      if (!selModeId) setPopup('no_mode');
      else if (!(p.min_accel_measured && p.min_brake_measured)) setPopup('no_min');
      // タイムスケジュールは学習の結果（pkl）を使わない開ループなので、学習済みでなくても走らせられる
      else if (!p.learned && !isSchedule) setPopup('no_learned');
    });
  }, [activeProfileId, selModeId]);

  const POPUP_CONFIG = {
    no_profile: { message: '車を選択してください',                       actionLabel: '車両プロファイルへ',   nav: 'profiles' },
    no_min:     { message: 'この車のペダルの min が未測定です',           actionLabel: 'キャリブレーションへ', nav: 'calibration' },
    no_learned: { message: 'この車は学習運転が済んでいません',           actionLabel: '学習運転へ',           nav: 'learning' },
    no_mode:    { message: '走行モード・スケジュール・シーケンスを選択してください', actionLabel: '走行モードへ', nav: 'modes' },
  };
  // 走行中（この画面のジョブ・シーケンス）は開始の前提の窓を出さない
  // WebSocket の最初の値が来るまで（initialized が undefined の間）は、走っているジョブがあるか分からないので窓を出さない
  const wsReady = realtimeData?.initialized !== undefined;
  const cfg = popup && wsReady && !followsRunning ? POPUP_CONFIG[popup] : null;

  const liveJob = realtimeData?.job ?? null;
  const liveSeq = realtimeData?.sequence ?? null;
  const jobKinds = isSchedule ? ['schedule'] : isSequence ? ['mode_drive', 'schedule'] : ['mode_drive'];
  const ownKinds = jobKinds;
  const showResult = !isSequence && !liveJob && lastJob && ownKinds.includes(lastJob.kind) && lastJob.job_id !== dismissedJobId && (lastJob.exit_code != null || lastJob.result);
  const showSeqResult = isSequence && !liveSeq && lastSeq && lastSeq.state !== 'running' && lastSeq.state !== 'stopped' && lastSeq.started_at !== dismissedSeq;
  const summary = showSeqResult ? (
      <SequenceResultBanner seq={lastSeq} onDismiss={() => setDismissedSeq(lastSeq.started_at)} onLogs={() => setNav('logs')} />
    ) : null;   // 走り終わった結果の帯は 3 段目に出さない（結果はログページで見る。ユーザー決定 2026-11-16）

  return (
    <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column', gap: 6 }}>
      <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
        {isSequence ? (
          <DriveMonitorScreen showRefLine={false} showProgressFigure={true} jobKind={jobKinds} seqMode={true}
            seqPreviewId={activeModeId}
            driveStartPath="/api/v1/drive/sequence/start"
            driveStartBody={activeModeId ? { sequence_id: activeModeId, ...(devLimitS ? { limit_s: devLimitS } : {}) } : null}
            prepareBody={activeModeId ? { kind: 'sequence', id: activeModeId, limit_s: devLimitS || null } : null}
            driveStopPath="/api/v1/drive/sequence/stop"
            confirmStartMessage={`シーケンス「${activeModeName ?? ''}」を開始しますか？`}
            confirmStopMessage="走行を終了しますか？（走っている段を止め、残りの段は走らせません）"
            startedToastMessage="シーケンスを開始しました"
            progressLabel={activeModeName}
            resultPanel={summary} />
        ) : isSchedule ? (
          <DriveMonitorScreen showRefLine={false} jobKind="schedule"
            driveStartPath="/api/v1/drive/schedule/start"
            driveStartBody={activeModeId ? { schedule_id: activeModeId } : null}
            prepareBody={activeModeId ? { kind: 'schedule', id: activeModeId, limit_s: null } : null}
            driveStopPath="/api/v1/drive/schedule/stop"
            confirmStartMessage={`スケジュール「${activeModeName ?? ''}」を開始しますか？`}
            progressLabel={activeModeName}
            resultPanel={summary} />
        ) : (
          <DriveMonitorScreen jobKind="mode_drive" progressLabel={activeModeName}
            driveStartPath="/api/v1/drive/start"
            driveStartBody={activeModeId ? { mode_id: activeModeId, ...(devLimitS ? { limit_s: devLimitS } : {}) } : null}
            prepareBody={activeModeId ? { kind: 'mode', id: activeModeId, limit_s: devLimitS || null } : null}
            driveStopPath="/api/v1/drive/stop"
            confirmStartMessage={`走行モード「${activeModeName ?? '走行'}」を開始しますか？`}
            resultPanel={summary} />
        )}
        {cfg && (
          <ValidationPopup message={cfg.message} actionLabel={cfg.actionLabel} onAction={() => setNav(cfg.nav)} />
        )}
      </div>
    </div>
  );
}

window.DriveMonitorScreen = DriveMonitorScreen;
window.AutoDriveScreen = AutoDriveScreen;
