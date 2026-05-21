// ── Learning drive screen ─────────────────────────────────
// Reuses DriveMonitorScreen with showPause=false

function LearningScreen() {
  return React.createElement(window.DriveMonitorScreen, { showPause: false, screenTitle: '学習運転' });
}

window.LearningScreen = LearningScreen;
