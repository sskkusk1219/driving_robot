// ── Schedule & Sequence screens (Post-MVP placeholder) ────

function ScheduleScreen() {
  const { Box, H2, Note } = window;
  return React.createElement('div', { style: { padding: 32 } },
    React.createElement(H2, null, 'タイムスケジュール'),
    React.createElement(Box, { style: { padding: 32, marginTop: 20, textAlign: 'center' } },
      React.createElement(Note, null, 'タイムスケジュール機能は Post-MVP で実装予定です。'),
    ),
  );
}

function SequenceScreen() {
  const { Box, H2, Note } = window;
  return React.createElement('div', { style: { padding: 32 } },
    React.createElement(H2, null, 'シーケンス'),
    React.createElement(Box, { style: { padding: 32, marginTop: 20, textAlign: 'center' } },
      React.createElement(Note, null, 'シーケンス機能は Post-MVP で実装予定です。'),
    ),
  );
}

window.ScheduleScreen = ScheduleScreen;
window.SequenceScreen = SequenceScreen;
