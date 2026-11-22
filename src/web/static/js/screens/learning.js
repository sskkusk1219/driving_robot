// ── 学習運転（RPD D-14） ─────────────────────────────────────
// 学習はいつも全部学習:
//   学習（手順 2 全部）: Ready・D → ペダル探索 → G ランプ走行 → モデル作成 → 学習結果チェック（→ 補正表の学習）→ N
// 学習結果チェックが NG（終了コード 8）のときは車両 yaml を書き換えず、理由と「やり直す」を出す。
// 画面の部品は自動運転と同じ DriveMonitorScreen（グラフ・状態の 1 行。一時停止ボタンは出さない）を使う。
// 段階表示・終了コードの意味は job-utils.js。

const LEARNING_LABEL = '学習運転';

function LearningScreen() {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, activeProfileId, setNav, realtimeData } = useContext(window.AppContext);
  const { ValidationPopup, Btn, Pill, INK, INK_SOFT, INK_MUTE, HATCH } = window;

  const [popup, setPopup] = useState(null);
  const [profile, setProfile] = useState(null);
  const [dismissedJobId, setDismissedJobId] = useState(null);

  const liveJob = realtimeData?.job ?? null;
  const learningRunning = !!liveJob && liveJob.kind === 'learning';
  const lastJob = window.useLastJob(liveJob);
  const lines = window.useDriveLogLines();

  async function loadProfile() {
    if (!activeProfileId) return;
    const p = await apiFetch('GET', `/api/v1/profiles/${activeProfileId}`);
    if (p) setProfile(p);
    return p;
  }
  useEffect(() => {
    if (!activeProfileId) { setPopup('no_profile'); return; }
    loadProfile().then(p => {
      if (p && !(p.min_accel_measured && p.min_brake_measured)) setPopup('no_min');
    });
  }, []);
  // ジョブが終わったら（学習済みか・pkl の日時が変わったので）車の状態を読み直す
  const liveId = liveJob?.job_id ?? null;
  useEffect(() => { if (!liveId) loadProfile(); }, [liveId]);

  // 表示するジョブ: 実行中のもの、なければ直前の学習ジョブ
  const showLast = !liveJob && lastJob && lastJob.kind === 'learning' && lastJob.job_id !== dismissedJobId;
  const stageJob = learningRunning ? liveJob : (showLast ? lastJob : null);
  // 段階表示の種類: 過去の部分学習のジョブは steps から読み分ける（新しく始めるのは full だけ）
  const stageKind = stageJob ? window.learningKindOf(stageJob) : 'full';
  const rawStage = window.deriveStages(stageKind, stageJob, lines);
  // 段は戻さない: 進行のメッセージは 500 行で流れる・画面を途中で開くと無いので、ジョブごとに今までの最大を覚える
  // （チェック NG＝終了コード 8 のときは deriveStages が合わせた check をそのまま使う）
  const maxStageRef = useRef({ jobId: null, current: -1 });
  if (maxStageRef.current.jobId !== (stageJob?.job_id ?? null)) maxStageRef.current = { jobId: stageJob?.job_id ?? null, current: -1 };
  const ngCheck = rawStage.state === 'ng' && (stageJob?.exit_code ?? stageJob?.result?.exit_code) === 8;
  if (!ngCheck) maxStageRef.current.current = Math.max(maxStageRef.current.current, rawStage.current);
  const stage = ngCheck ? rawStage : { ...rawStage, current: Math.max(rawStage.current, maxStageRef.current.current) };

  const resultPanel = (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 5 }}>
      <window.StageRail defs={stage.defs} current={stage.current} state={stage.state} />
      {learningRunning && (
        <div style={{ fontSize: 12, color: INK_SOFT }}>
          {window.PHASE_LABEL?.[liveJob.phase] ?? liveJob.phase}
          {liveJob.sample?.section ? `　（${stage.defs[stage.current]?.label ?? window.SECTION_LABEL?.[liveJob.sample.section] ?? liveJob.sample.section}）` : ''}
          {liveJob.step_title && liveJob.phase === 'step'
            ? `　${liveJob.step === 2 ? 'ペダル探索 → パターン走行 → 順モデル作成' : liveJob.step_title}` : ''}
        </div>
      )}
      {showLast && (
        <window.JobResultBanner job={lastJob} compact
          onRetry={() => retryLast()} onDismiss={() => setDismissedJobId(lastJob.job_id)} onLogs={() => setNav('logs')} />
      )}
    </div>
  );

  async function retryLast() {
    setDismissedJobId(lastJob.job_id);
    const r = await apiFetch('POST', '/api/v1/drive/learning/start', { kind: 'full' });
    if (r) window.showToast('学習運転をやり直します', 'success');
  }

  const POPUP_CONFIG = {
    no_profile: { message: '車を選択してください', actionLabel: '車両プロファイルへ', nav: 'profiles' },
    no_min:     { message: 'この車のペダルの min が未測定です', actionLabel: 'キャリブレーションへ', nav: 'calibration' },
  };
  const cfg = popup ? POPUP_CONFIG[popup] : null;

  return (
    <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column', gap: 6 }}>
      <div style={{ display: 'flex', gap: 8, alignItems: 'center', flexWrap: 'wrap' }}>
        <div style={{ flex: 1 }} />
        {profile && (
          <span style={{ fontSize: 12, color: INK_SOFT }}>
            車: <b style={{ color: INK }}>{profile.name}</b>　{profile.learned ? <Pill accent="READY" style={{ fontSize: 11 }}>学習済み</Pill> : <Pill style={{ fontSize: 11 }}>未学習</Pill>}
          </span>
        )}
      </div>

      <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
        {React.createElement(window.DriveMonitorScreen, {
          showRefLine: false,
          jobKind: 'learning',
          progressLabel: LEARNING_LABEL,
          profileMaxSpeed: profile?.max_speed_kmh ?? null,
          driveStartPath: '/api/v1/drive/learning/start',
          driveStartBody: { kind: 'full' },
          driveStopPath: '/api/v1/drive/stop',
          confirmStartMessage: `${LEARNING_LABEL}を開始しますか？`,
          startedToastMessage: '学習運転を開始しました',
          stopLabel: '中断',
          confirmStopMessage: '学習運転を中断しますか？（原点へ戻して止まります）',
          resultPanel,
        })}
        {cfg && (
          <ValidationPopup message={cfg.message} actionLabel={cfg.actionLabel} onAction={() => setNav(cfg.nav)} />
        )}
      </div>
    </div>
  );
}

window.LearningScreen = LearningScreen;
