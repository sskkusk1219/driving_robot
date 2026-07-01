// ── Learning drive screen ─────────────────────────────────
// Reuses DriveMonitorScreen.
// 学習運転が正常終了（RUNNING→READY）したら自動で次を一気通貫実行する:
//   1. 開度パターン走行（このページの DriveMonitorScreen）
//   2. モデル学習（/drive/learning/train）
//   3. PID 初期値設定（train 内で同定・SIMC）
//   4. 規定パターンを走行しながら PID 最適化（/drive/pid-tune/refine）
// 4 は内部で RUNNING→READY を最大15回繰り返すため、パイプライン実行中は
// robotState 監視による再トリガーを busyRef で抑止する。

function LearningScreen() {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, activeProfileId, setNav, robotState } = useContext(window.AppContext);
  const { ValidationPopup } = window;

  const [popup, setPopup] = useState(null);
  const [profileMaxSpeed, setProfileMaxSpeed] = useState(null);
  const [profileMaxDecelG, setProfileMaxDecelG] = useState(null);
  const [phase, setPhase] = useState(null);    // null | 'training' | 'optimizing'
  const [result, setResult] = useState(null);   // 学習（モデル＋PID初期値）結果
  const [tuneResult, setTuneResult] = useState(null);  // PID 最適化（絞り込み）結果
  const wasRunningRef = useRef(false);
  const busyRef = useRef(false);  // パイプライン実行中は学習再トリガーを抑止

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

  // 学習運転が正常終了（RUNNING→READY）したタイミングでパイプラインを実行する。
  // 非常停止（EMERGENCY 等）ではトリガーしない。パイプライン実行中（ステップ4の
  // 反復走行を含む）は busyRef で再トリガーを抑止する。
  useEffect(() => {
    if (busyRef.current) return;
    if (robotState === 'RUNNING') {
      wasRunningRef.current = true;
      return;
    }
    if (robotState === 'READY' && wasRunningRef.current) {
      wasRunningRef.current = false;
      runLearningPipeline();
    } else if (robotState !== 'READY') {
      // EMERGENCY/PRE_CHECK 等に遷移した場合はトリガーしない
      wasRunningRef.current = false;
    }
  }, [robotState]);

  // ステップ2-3（学習・モデル作成・PID初期値）→ ステップ4（規定パターン走行でPID最適化）。
  async function runLearningPipeline() {
    if (!activeProfileId) return;
    busyRef.current = true;
    try {
      // ステップ2-3: モデル学習＋PID初期値
      setPhase('training');
      setResult(null);
      setTuneResult(null);
      const res = await apiFetch('POST', '/api/v1/drive/learning/train', { profile_id: activeProfileId });
      if (!res) return;  // 失敗（ログ不足等）は apiFetch がエラートースト
      setResult(res);
      window.showToast('運転モデルを学習し、PID初期値を設定しました', 'success');

      // ステップ4: モデル＋PID初期値で規定パターンを走行し PID 最適化（閉ループ絞り込み）
      setPhase('optimizing');
      const tune = await apiFetch('POST', '/api/v1/drive/pid-tune/refine', {
        profile_id: activeProfileId, max_runs: 15,
      });
      if (tune) {
        setTuneResult(tune);
        window.showToast('PID最適化が完了しました', 'success');
      }
    } finally {
      setPhase(null);
      busyRef.current = false;
    }
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

  return (
    <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
      {React.createElement(window.DriveMonitorScreen, {
        showModeAxis: false,
        profileMaxSpeed,
        screenTitle: '学習運転',
        driveStartPath: '/api/v1/drive/learning/start',
        driveArmPath: '/api/v1/drive/learning/arm',
        driveCancelPath: '/api/v1/drive/learning/cancel',
        confirmStartMessage: '学習運転を開始しますか？',
      })}

      {/* 自動パイプラインのステータス（手動ボタンは廃止。終了時に自動実行される） */}
      {(phase || result || tuneResult) && (
        <div style={{
          flexShrink: 0, marginTop: 10, padding: '10px 14px',
          border: `1.5px solid ${INK}`, borderRadius: 6, background: PAPER_2,
          fontSize: 13, color: INK,
        }}>
          {phase === 'training' && <div>運転モデルを学習中…</div>}

          {result && (
            <div>
              <div>運転モデルを更新しました: <code>{result.model_path?.split('/').pop()}</code></div>
              <div style={{ color: INK_MUTE, marginTop: 2 }}>
                加速 MAE {fmt(result.metrics?.accel?.mae)} / RMSE {fmt(result.metrics?.accel?.rmse)} / R² {fmt(result.metrics?.accel?.r2)}（{result.metrics?.accel?.n ?? 0}点）
              </div>
              <div style={{ color: INK_MUTE, marginTop: 1 }}>
                減速 MAE {fmt(result.metrics?.brake?.mae)} / RMSE {fmt(result.metrics?.brake?.rmse)} / R² {fmt(result.metrics?.brake?.r2)}（{result.metrics?.brake?.n ?? 0}点）
              </div>
              {result.pid_gains && (
                <div style={{ color: INK_MUTE, marginTop: 4 }}>
                  PID初期値 Kp {fmt(result.pid_gains.kp)} / Ki {fmt(result.pid_gains.ki)} / Kd {fmt(result.pid_gains.kd)}
                  {!result.pid_auto_tuned && '（区間不足のため既存値を維持）'}
                </div>
              )}
            </div>
          )}

          {phase === 'optimizing' && (
            <div style={{ marginTop: result ? 6 : 0 }}>
              <div>PID最適化中…（規定パターンを走行・最大15回）</div>
              {patternText && <div style={{ color: INK_MUTE, marginTop: 2 }}>{patternText}</div>}
            </div>
          )}

          {tuneResult && (
            <div style={{ color: INK_MUTE, marginTop: 6 }}>
              <span style={{
                display: 'inline-block', marginRight: 6, padding: '0 6px',
                border: '1px solid #3c8c3c', borderRadius: 3, color: '#68d468', fontSize: 11,
              }}>PID最適化</span>
              Kp {fmt(tuneResult.pid_gains?.kp)} / Ki {fmt(tuneResult.pid_gains?.ki)} / Kd {fmt(tuneResult.pid_gains?.kd)}
              （{tuneResult.history?.length ?? 0}回走行・最良コスト {fmt(tuneResult.best_cost)}）
            </div>
          )}
        </div>
      )}

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
