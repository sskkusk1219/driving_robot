// ── Learning drive screen ─────────────────────────────────
// Reuses DriveMonitorScreen。「開始」ボタンは学習運転単体ではなく学習サイクル
// 全体（学習運転→訓練→PID適合→再学習→PID適合）をサーバー側オーケストレータ
// （LearningCycleOrchestrator）に委譲する。自動運転と同じ arm → 確認ポップアップ →
// start のフロー（driveArmPath/driveCancelPath）を使い、ポップアップ表示前に
// 停車ブレーキ踏込・走行前チェック・車速0確認を完了させる。
// 進捗は WebSocket の cycle_progress で配信され、resultPanel と busyLabel に表示する。
// busy（cycleActive）中はフェーズが READY に戻る訓練段階でも「開始」を再表示
// させず、常に中断ボタンを出す（DriveMonitorScreen 側で robotState より busy を優先）。
// 旧実装が行っていたクライアント側の自動チェーン（robotState 監視での
// train→refine 連続発火）は削除済み: 残すとオーケストレーション中の READY 遷移
// （フェーズごとに複数回発生）のたびに二重発火してしまうため。

function LearningScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeProfileId, setNav, realtimeData } = useContext(window.AppContext);
  const { ValidationPopup } = window;

  const [popup, setPopup] = useState(null);
  const [profileMaxSpeed, setProfileMaxSpeed] = useState(null);
  const [profileMaxDecelG, setProfileMaxDecelG] = useState(null);

  useEffect(() => {
    if (!activeProfileId) {
      setPopup('no_profile');
      return;
    }
    apiFetch('GET', `/api/v1/profiles/${activeProfileId}`).then(p => {
      if (!p) return;
      if (!p.calibration?.is_valid) setPopup('no_calib');
      if (p.max_speed) setProfileMaxSpeed(p.max_speed);
      if (p.max_decel_g != null) setProfileMaxDecelG(p.max_decel_g);
    });
  }, []);

  const cycleProgress = realtimeData?.cycle_progress ?? null;
  const cyclePhase = cycleProgress?.phase ?? 'IDLE';
  const cycleActive = !['IDLE', 'COMPLETED', 'ERROR', 'ABORTED'].includes(cyclePhase);

  async function handleAbortCycle() {
    const res = await apiFetch('POST', '/api/v1/drive/learning-cycle/abort');
    if (res) window.showToast('学習サイクルの中断を要求しました', 'info');
  }

  const POPUP_CONFIG = {
    no_profile: { message: '車両プロファイルを選択してください', actionLabel: 'プロファイルへ', nav: 'profiles' },
    no_calib:   { message: 'キャリブレーションデータがありません', actionLabel: 'キャリブレーションへ', nav: 'calibration' },
  };

  const cfg = popup ? POPUP_CONFIG[popup] : null;
  const fmt = (m) => (m == null ? '—' : m.toFixed(3));

  // 規定パターンの主要速度（0→0.6→0.8→0.3→0×max_speed）と上限Gの文字情報。
  // グラフへの基準パターン描画はしない（学習ページは実車速のみ表示）。
  const patternText = profileMaxSpeed
    ? `規定パターン: 0 → ${Math.round(0.6 * profileMaxSpeed)} → ${Math.round(0.8 * profileMaxSpeed)} → ${Math.round(0.3 * profileMaxSpeed)} → 0 km/h`
      + (profileMaxDecelG != null ? `（最大減速 ${profileMaxDecelG.toFixed(2)}G 以内）` : '')
    : null;

  const PHASE_LABEL = {
    IDLE: '待機中',
    ARMING: '学習運転を準備中…',
    LEARNING: '学習運転を実行中…',
    TRAINING_1: '運転モデルを学習中(1段目)…',
    REFINE_1: 'PID適合を実行中(1段目)…',
    TRAINING_2: 'サイクル全ログで再学習中(2段目)…',
    REFINE_2: 'PID適合を実行中(2段目)…',
    VERIFY: '検証走行でKPIを確認中…',
    COMPLETED: '学習サイクルが完了しました',
    ERROR: '学習サイクルでエラーが発生しました',
    ABORTED: '学習サイクルを中断しました',
  };

  // 学習サイクルの進捗を DriveMonitorScreen の resultPanel 枠に表示する。
  // 状態遷移でフレームの高さが変わらないよう、中身の有無に関わらず同じ要素を返す。
  const resultPanel = (
    <div style={{ fontSize: 12, color: INK }}>
      {!cycleProgress || cyclePhase === 'IDLE' ? (
        <div style={{ color: INK_MUTE }}>—</div>
      ) : (
        <div>
          <div style={{ fontWeight: 700 }}>
            {PHASE_LABEL[cyclePhase] ?? cyclePhase}
            {cycleProgress.run_total > 0 && `（${cycleProgress.run_index}/${cycleProgress.run_total}回）`}
          </div>
          {cycleProgress.message && (
            <div style={{ color: INK_MUTE, marginTop: 2 }}>{cycleProgress.message}</div>
          )}
          {cycleProgress.best_cost != null && (
            <div style={{ color: INK_MUTE, marginTop: 2 }}>
              最良コスト {fmt(cycleProgress.best_cost)}
              {cycleProgress.best_pid_preview_s != null && ` / PID先読み補償 ${cycleProgress.best_pid_preview_s.toFixed(2)}s`}
            </div>
          )}
          {patternText && (cyclePhase === 'REFINE_1' || cyclePhase === 'REFINE_2') && (
            <div style={{ color: INK_MUTE, marginTop: 2 }}>{patternText}</div>
          )}
        </div>
      )}
    </div>
  );

  return (
    <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
      {React.createElement(window.DriveMonitorScreen, {
        showModeAxis: false,
        profileMaxSpeed,
        screenTitle: '学習運転',
        driveStartPath: '/api/v1/drive/learning-cycle/start',
        driveStartBody: {},
        driveArmPath: '/api/v1/drive/learning-cycle/arm',
        driveCancelPath: '/api/v1/drive/learning-cycle/cancel',
        confirmStartMessage: '開始しますか？',
        startedToastMessage: '学習サイクルを開始しました',
        busy: cycleActive,
        busyLabel: PHASE_LABEL[cyclePhase] ?? cyclePhase,
        onAbort: handleAbortCycle,
        resultPanel,
      })}

      {cfg && (
        <ValidationPopup
          message={cfg.message}
          actionLabel={cfg.actionLabel}
          onAction={() => setNav(cfg.nav)}
        />
      )}
    </div>
  );
}

window.LearningScreen = LearningScreen;
