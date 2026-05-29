// ── Learning drive screen ─────────────────────────────────
// Reuses DriveMonitorScreen with showPause=false

function LearningScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeProfileId, setNav } = useContext(window.AppContext);
  const { ValidationPopup } = window;

  const [popup, setPopup] = useState(null);
  const [profileMaxSpeed, setProfileMaxSpeed] = useState(null);

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

  const POPUP_CONFIG = {
    no_profile: { message: '車両プロファイルを選択してください', actionLabel: 'プロファイルへ', nav: 'profiles' },
    no_calib:   { message: 'キャリブレーションデータがありません', actionLabel: 'キャリブレーションへ', nav: 'calibration' },
  };

  const cfg = popup ? POPUP_CONFIG[popup] : null;

  return (
    <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
      {React.createElement(window.DriveMonitorScreen, {
        showPause: false,
        showModeAxis: false,
        profileMaxSpeed,
        screenTitle: '学習運転',
        driveStartPath: '/api/v1/drive/learning/start',
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
