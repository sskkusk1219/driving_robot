// ── Learning drive screen ─────────────────────────────────
// Reuses DriveMonitorScreen.
// 学習運転が正常終了（RUNNING→READY）したら自動でモデル学習を実行し、
// プロファイルに紐付ける（手動の「学習」ボタンは廃止）。

function LearningScreen() {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, activeProfileId, setNav, robotState } = useContext(window.AppContext);
  const { ValidationPopup } = window;

  const [popup, setPopup] = useState(null);
  const [profileMaxSpeed, setProfileMaxSpeed] = useState(null);
  const [training, setTraining] = useState(false);
  const [result, setResult] = useState(null);
  const wasRunningRef = useRef(false);

  useEffect(() => {
    if (!activeProfileId) {
      setPopup('no_profile');
      return;
    }
    apiFetch('GET', `/api/v1/profiles/${activeProfileId}`).then(p => {
      if (!p) return;
      if (!p.calibration?.is_valid) setPopup('no_calib');
      if (p.max_speed) setProfileMaxSpeed(p.max_speed);
    });
  }, []);

  // 学習運転が正常終了（RUNNING→READY）したタイミングで自動学習を実行する。
  // 非常停止（EMERGENCY 等）では学習しない。
  useEffect(() => {
    if (robotState === 'RUNNING') {
      wasRunningRef.current = true;
      return;
    }
    if (robotState === 'READY' && wasRunningRef.current) {
      wasRunningRef.current = false;
      autoTrain();
    } else if (robotState !== 'READY') {
      // EMERGENCY/PRE_CHECK 等に遷移した場合は学習をトリガーしない
      wasRunningRef.current = false;
    }
  }, [robotState]);

  async function autoTrain() {
    if (!activeProfileId) return;
    setTraining(true);
    const res = await apiFetch('POST', '/api/v1/drive/learning/train', { profile_id: activeProfileId });
    setTraining(false);
    if (res) {
      setResult(res);
      window.showToast('運転モデルを学習し、プロファイルに紐付けました', 'success');
    }
    // 失敗時（ログ不足など）は apiFetch がエラートーストを表示する
  }

  const POPUP_CONFIG = {
    no_profile: { message: '車両プロファイルを選択してください', actionLabel: 'プロファイルへ', nav: 'profiles' },
    no_calib:   { message: 'キャリブレーションデータがありません', actionLabel: 'キャリブレーションへ', nav: 'calibration' },
  };

  const cfg = popup ? POPUP_CONFIG[popup] : null;
  const fmt = (m) => (m == null ? '—' : m.toFixed(3));

  return (
    <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
      {React.createElement(window.DriveMonitorScreen, {
        showModeAxis: false,
        profileMaxSpeed,
        screenTitle: '学習運転',
        driveStartPath: '/api/v1/drive/learning/start',
      })}

      {/* 自動学習ステータス（手動ボタンは廃止。終了時に自動実行される） */}
      {(training || result) && (
        <div style={{
          flexShrink: 0, marginTop: 10, padding: '10px 14px',
          border: `1.5px solid ${INK}`, borderRadius: 6, background: PAPER_2,
          fontSize: 13, color: INK,
        }}>
          {training && <div>運転モデルを学習中…</div>}
          {!training && result && (
            <div>
              <div>運転モデルを更新しました: <code>{result.model_path?.split('/').pop()}</code></div>
              <div style={{ color: INK_MUTE, marginTop: 2 }}>
                加速 MAE {fmt(result.metrics?.accel?.mae)}（{result.metrics?.accel?.n ?? 0}点） /
                減速 MAE {fmt(result.metrics?.brake?.mae)}（{result.metrics?.brake?.n ?? 0}点）
              </div>
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
