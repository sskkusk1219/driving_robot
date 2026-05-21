// ── System state screens ──────────────────────────────────
// Matches wireframe: missing-system-states.jsx

const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH, Box, Btn, Pill, H2, Note } = window;
const { useContext } = React;

// ── Step icon helper ─────────────────────────────────────
function StepIcon({ st, i }) {
  const iconColor = st === 'done' ? '#3f6b3f' : st === 'now' ? '#2f5780' : INK_MUTE;
  const icon = st === 'done' ? '✓' : st === 'now' ? '⟳' : '—';
  const bg   = st === 'done' ? '#dfeadc' : st === 'now' ? '#dde6f0' : 'transparent';
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
function BootingScreen() {
  const checks = [
    ['ttyUSB0 (アクセル / Modbus RTU)', 'done', 'pymodbus connect OK · 38400bps'],
    ['ttyUSB1 (ブレーキ / Modbus RTU)', 'done', 'pymodbus connect OK · 38400bps'],
    ['CAN / Kvaser USB-CAN',            'now',  '/dev/usbcanII0 確認中...'],
    ['PostgreSQL (ローカル)',            'wait', '—'],
    ['GPIO17 非常停止スイッチ',          'wait', '—'],
    ['GPIO27 AC UPS 接点出力',          'wait', '—'],
  ];

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 28 }}>
      <div style={{ textAlign: 'center' }}>
        <div style={{ fontSize: 40, fontWeight: 800, letterSpacing: 1 }}>Driving_Robot</div>
        <div style={{ fontSize: 18, color: INK_SOFT, marginTop: 4 }}>システムを起動しています — 通信確認中...</div>
      </div>

      <Box style={{ width: 620, padding: '20px 24px' }} label="通信確認">
        <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
          {checks.map(([label, st, sub]) => (
            <div key={label} style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
              <StepIcon st={st} />
              <div style={{ flex: 1 }}>
                <div style={{ fontSize: 16, fontWeight: st === 'now' ? 700 : 400 }}>{label}</div>
                <div style={{ fontSize: 12, color: INK_SOFT, fontFamily: 'monospace' }}>{sub}</div>
              </div>
              {st === 'now' && <div style={{ fontSize: 13, color: '#2f5780', fontStyle: 'italic' }}>確認中...</div>}
            </div>
          ))}
        </div>
      </Box>

      <Note>通信確認が完了すると STANDBY 状態に移行し、「初期化」ボタンが有効になります。</Note>
    </div>
  );
}

// ── ERROR ────────────────────────────────────────────────
function ErrorScreen() {
  const { apiFetch } = useContext(window.AppContext);

  const errors = [
    ['ttyUSB0 (アクセル)', true,  'pymodbus connect OK'],
    ['ttyUSB1 (ブレーキ)', false, "ConnectionError: [Errno 2] No such file or directory: '/dev/ttyUSB1'"],
    ['CAN / Kvaser',      false, 'CANLIBError: No Kvaser device found — usbcanII0 not present'],
    ['PostgreSQL',        true,  'connected'],
  ];
  const failedItems = errors.filter(e => !e[1]);

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
            <div style={{ fontSize: 26, fontWeight: 800, color: '#5e1414' }}>通信エラー — 起動できません</div>
            <div style={{ fontSize: 15, color: '#8a3030', marginTop: 4 }}>
              {failedItems.length}つの通信が失敗しました。接続を確認してから再試行してください。
            </div>
          </div>
        </div>

        <div style={{ display: 'flex', flexDirection: 'column', gap: 8, marginBottom: 18 }}>
          {errors.map(([label, ok, msg]) => (
            <div key={label} style={{
              display: 'flex', gap: 10, alignItems: 'flex-start',
              padding: '10px 12px',
              border: `1.2px solid ${ok ? '#3f6b3f' : '#a23232'}`,
              background: ok ? '#dfeadc' : '#fdf0ee',
              borderRadius: 4,
            }}>
              <div style={{ fontSize: 16, color: ok ? '#22421f' : '#a23232', fontWeight: 700, flexShrink: 0, marginTop: 1 }}>
                {ok ? '✓' : '✗'}
              </div>
              <div style={{ flex: 1 }}>
                <div style={{ fontSize: 15, fontWeight: ok ? 400 : 700, color: ok ? '#22421f' : '#5e1414' }}>{label}</div>
                <div style={{ fontSize: 12, fontFamily: 'monospace', color: ok ? '#3f6b3f' : '#8a3030', marginTop: 2 }}>{msg}</div>
              </div>
            </div>
          ))}
        </div>

        <Box label="確認事項" style={{ padding: '12px 14px', marginBottom: 16 }}>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 6, fontSize: 14 }}>
            <div>• <b>ttyUSB1</b>: ブレーキ用 USB-RS485 ケーブルが接続されているか確認 (<code>ls /dev/ttyUSB*</code>)</div>
            <div>• <b>Kvaser CAN</b>: Kvaser USB-CAN が接続されているか確認 (<code>listChannels</code>)、ドライバが有効か (<code>sudo modprobe usbcanII</code>)</div>
            <div>• udev rules が設定されている場合、デバイス名が変わっていないか確認</div>
          </div>
        </Box>

        <div style={{ display: 'flex', gap: 10, justifyContent: 'flex-end' }}>
          <Btn>ログを確認</Btn>
          <Btn primary big onClick={handleRetry}>⟳ 再試行</Btn>
        </div>
      </Box>
    </div>
  );
}

// ── AC電源断 ──────────────────────────────────────────────
function AcPowerLossScreen() {
  const steps = [
    ['AC断検知 (GPIO27 LOW)',             'done', '14:22:03.012  GPIO27 → LOW (バッテリー運転中)'],
    ['制御ループ停止 / 走行セッション終了', 'done', '14:22:03.014  session → status=emergency'],
    ['原点復帰 (両軸 asyncio.gather)',     'now',  '14:22:03.180  HEND 待機中... (両軸並列)'],
    ['走行ログ PostgreSQL フラッシュ',    'wait', '—'],
    ['PostgreSQL 正常終了',              'wait', '—'],
    ['Raspberry Pi シャットダウン',       'wait', '—'],
  ];
  const elapsed = 0.4;

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 20 }}>
      <div style={{
        width: 680, background: '#f1d8d2',
        border: '2.5px solid #a23232', borderRadius: 8,
        padding: '16px 24px', display: 'flex', alignItems: 'center', gap: 16,
      }}>
        <div style={{ fontSize: 40, color: '#a23232' }}>⚡</div>
        <div>
          <div style={{ fontSize: 26, fontWeight: 800, color: '#5e1414' }}>AC電源断を検知しました</div>
          <div style={{ fontSize: 15, color: '#8a3030' }}>
            UPS バッテリー給電中 — 安全停止シーケンスを実行しています。操作しないでください。
          </div>
        </div>
        <div style={{ marginLeft: 'auto', textAlign: 'right' }}>
          <div style={{ fontSize: 12, color: INK_SOFT }}>経過</div>
          <div style={{ fontSize: 28, fontFamily: 'monospace', fontWeight: 700, color: '#5e1414' }}>{elapsed.toFixed(1)}s</div>
          <div style={{ fontSize: 12, color: '#8a3030' }}>残り ~29s</div>
        </div>
      </div>

      <Box style={{ width: 680, padding: '18px 22px' }} label="安全停止シーケンス (5ステップ / 30秒以内)">
        <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
          {steps.map(([label, st, ts], i) => {
            const iconColor = st === 'done' ? '#3f6b3f' : st === 'now' ? '#a23232' : INK_MUTE;
            const icon = st === 'done' ? '✓' : st === 'now' ? '⟳' : `${i + 1}`;
            return (
              <div key={label} style={{
                display: 'flex', alignItems: 'center', gap: 12,
                padding: '8px 10px',
                background: st === 'now' ? '#fdf0ee' : 'transparent',
                border: st === 'now' ? '1.5px solid #a23232' : '1px solid transparent',
                borderRadius: 4,
              }}>
                <div style={{
                  width: 28, height: 28, borderRadius: '50%',
                  border: `1.6px solid ${iconColor}`,
                  background: st === 'done' ? '#dfeadc' : st === 'now' ? '#f1d8d2' : 'transparent',
                  display: 'flex', alignItems: 'center', justifyContent: 'center',
                  fontSize: 13, fontWeight: 700, color: iconColor, flexShrink: 0,
                }}>{icon}</div>
                <div style={{ flex: 1 }}>
                  <div style={{ fontSize: 15, fontWeight: st === 'now' ? 700 : 400, color: st === 'wait' ? INK_MUTE : INK }}>{label}</div>
                  <div style={{ fontSize: 11, fontFamily: 'monospace', color: st === 'wait' ? INK_MUTE : INK_SOFT }}>{ts}</div>
                </div>
                {st === 'now' && <div style={{ fontSize: 13, color: '#a23232', fontWeight: 700 }}>実行中...</div>}
              </div>
            );
          })}
        </div>
        <div style={{ marginTop: 14 }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 12, color: INK_SOFT, marginBottom: 4, fontFamily: 'monospace' }}>
            <span>0s (AC断)</span>
            <span style={{ color: '#a23232' }}>▼ 現在 {elapsed.toFixed(1)}s</span>
            <span>30s (タイムリミット)</span>
          </div>
          <div style={{ height: 10, border: `1.4px solid ${INK}`, position: 'relative', background: PAPER }}>
            <div style={{ position: 'absolute', left: 0, top: 0, bottom: 0, width: `${(elapsed / 30) * 100}%`, background: '#a23232', opacity: 0.7 }} />
          </div>
        </div>
      </Box>

      <Note style={{ width: 680 }}>
        AC UPS のバッテリーで Raspberry Pi (5V) と P-CON-CB (24V) を同時給電中。シャットダウン完了後に自動で電源が切れます。
      </Note>
    </div>
  );
}

// ── 自動停止 — 逸脱超過 ──────────────────────────────────
function AutoStopDeviationScreen() {
  const { realtimeData, apiFetch, setNav } = useContext(window.AppContext);

  async function handleReset() {
    const r = await apiFetch('POST', '/api/v1/drive/reset-emergency');
    if (r) { window.showToast('EMERGENCY をリセットしました', 'success'); setNav('init'); }
  }

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 20 }}>
      <div style={{
        border: '2.5px solid #a23232', background: '#f1d8d2',
        padding: '22px 40px', borderRadius: 8, textAlign: 'center',
        transform: 'rotate(-0.4deg)', boxShadow: '4px 4px 0 rgba(0,0,0,0.15)',
        width: 600,
      }}>
        <div style={{ fontSize: 38, color: '#a23232', marginBottom: 6 }}>⚠</div>
        <div style={{ fontSize: 32, fontWeight: 800, color: '#5e1414' }}>自動停止 — 逸脱超過</div>
        <div style={{ fontSize: 17, color: '#8a3030', marginTop: 8, lineHeight: 1.5 }}>
          基準車速からの逸脱が設定閾値を超えたため<br/>自動停止し、原点復帰しました。
        </div>
      </div>

      <Box style={{ width: 600, padding: '16px 20px' }} label="停止詳細">
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 12, marginBottom: 14 }}>
          {[
            ['基準車速',    `${(realtimeData.ref_speed_kmh ?? 0).toFixed(1)} km/h`],
            ['実車速',      `${(realtimeData.actual_speed_kmh ?? 0).toFixed(1)} km/h`],
            ['逸脱量',      `${((realtimeData.actual_speed_kmh ?? 0) - (realtimeData.ref_speed_kmh ?? 0)).toFixed(2)} km/h`],
            ['逸脱閾値',    '±2.0 km/h / 4.0 s'],
          ].map(([l, v]) => (
            <div key={l} style={{ fontSize: 14 }}>
              <div style={{ color: INK_SOFT }}>{l}</div>
              <div style={{ fontFamily: 'monospace', fontWeight: 700 }}>{v}</div>
            </div>
          ))}
        </div>
      </Box>

      <div style={{ display: 'flex', gap: 14 }}>
        <Btn big>ログを確認</Btn>
        <Btn primary big onClick={handleReset}>EMERGENCY リセット → READY へ</Btn>
      </div>
    </div>
  );
}

// ── 自動停止 — 過電流 ────────────────────────────────────
function AutoStopOvercurrentScreen() {
  const { realtimeData, apiFetch, setNav } = useContext(window.AppContext);

  async function handleReset() {
    const r = await apiFetch('POST', '/api/v1/drive/reset-emergency');
    if (r) { window.showToast('EMERGENCY をリセットしました', 'success'); setNav('init'); }
  }

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 20 }}>
      <div style={{
        border: '2.5px solid #a23232', background: '#f1d8d2',
        padding: '22px 40px', borderRadius: 8, textAlign: 'center',
        transform: 'rotate(-0.4deg)', boxShadow: '4px 4px 0 rgba(0,0,0,0.15)',
        width: 600,
      }}>
        <div style={{ fontSize: 38, color: '#a23232', marginBottom: 6 }}>⚡</div>
        <div style={{ fontSize: 32, fontWeight: 800, color: '#5e1414' }}>自動停止 — 過電流検知</div>
        <div style={{ fontSize: 17, color: '#8a3030', marginTop: 8, lineHeight: 1.5 }}>
          アクセルアクチュエータの電流値が上限を超えたため<br/>緊急停止しました。機械的な干渉がないか確認してください。
        </div>
      </div>

      <Box style={{ width: 600, padding: '16px 20px' }} label="停止詳細">
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 12 }}>
          {[
            ['停止理由',  '過電流 (SafetyMonitor)'],
            ['検知軸',    'アクセル (ttyUSB0)'],
            ['アクセル電流', `${(realtimeData.accel_current_ma ?? 0).toFixed(0)} mA`],
            ['ブレーキ電流', `${(realtimeData.brake_current_ma ?? 0).toFixed(0)} mA`],
            ['閾値',      'OVERCURRENT_LIMIT = 3,000 mA'],
            ['状態',      '緊急停止 → 原点復帰完了'],
          ].map(([l, v]) => (
            <div key={l} style={{ fontSize: 14 }}>
              <div style={{ color: INK_SOFT }}>{l}</div>
              <div style={{ fontFamily: 'monospace', fontWeight: 700 }}>{v}</div>
            </div>
          ))}
        </div>
      </Box>

      <Note style={{ width: 600 }}>
        過電流はペダルへの機械的干渉またはアクチュエータ故障が原因の場合があります。物理的な異常を確認してからリセットしてください。
      </Note>

      <div style={{ display: 'flex', gap: 14 }}>
        <Btn big>ログを確認</Btn>
        <Btn primary big onClick={handleReset}>EMERGENCY リセット → READY へ</Btn>
      </div>
    </div>
  );
}

// ── 自動停止 — CAN タイムアウト ──────────────────────────
function AutoStopCanTimeoutScreen() {
  const { apiFetch, setNav } = useContext(window.AppContext);

  async function handleReset() {
    const r = await apiFetch('POST', '/api/v1/drive/reset-emergency');
    if (r) { window.showToast('EMERGENCY をリセットしました', 'success'); setNav('init'); }
  }

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 20 }}>
      <div style={{
        border: '2.5px solid #a23232', background: '#f1d8d2',
        padding: '22px 40px', borderRadius: 8, textAlign: 'center',
        transform: 'rotate(-0.4deg)', boxShadow: '4px 4px 0 rgba(0,0,0,0.15)',
        width: 600,
      }}>
        <div style={{ fontSize: 38, color: '#a23232', marginBottom: 6 }}>📡</div>
        <div style={{ fontSize: 32, fontWeight: 800, color: '#5e1414' }}>自動停止 — 車速信号タイムアウト</div>
        <div style={{ fontSize: 17, color: '#8a3030', marginTop: 8, lineHeight: 1.5 }}>
          シャシダイナモからのCAN車速信号が途絶えたため<br/>走行を停止し、原点復帰しました。
        </div>
      </div>

      <Box style={{ width: 600, padding: '16px 20px' }} label="停止詳細">
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 12 }}>
          {[
            ['停止理由',      'CAN受信タイムアウト'],
            ['タイムアウト閾値', '100 ms'],
            ['インターフェース', '/dev/usbcanII0'],
            ['状態',          '走行停止 → 原点復帰完了'],
          ].map(([l, v]) => (
            <div key={l} style={{ fontSize: 14 }}>
              <div style={{ color: INK_SOFT }}>{l}</div>
              <div style={{ fontFamily: 'monospace', fontWeight: 700 }}>{v}</div>
            </div>
          ))}
        </div>
      </Box>

      <Box style={{ width: 600, padding: '12px 16px' }} label="確認事項">
        <div style={{ fontSize: 14, lineHeight: 1.9 }}>
          <div>• Kvaser USB-CAN のケーブル接続を確認</div>
          <div>• シャシダイナモ側の CAN 送信が停止していないか確認</div>
          <div>• 終端抵抗: CANH-CANL 間が約 60Ω か確認（120Ω × 2並列）</div>
          <div>• <code>listChannels</code> で /dev/usbcanII0 が認識されているか確認</div>
        </div>
      </Box>

      <div style={{ display: 'flex', gap: 14 }}>
        <Btn big>ログを確認</Btn>
        <Btn primary big onClick={handleReset}>EMERGENCY リセット → READY へ</Btn>
      </div>
    </div>
  );
}

// ── 走行前チェック NG ─────────────────────────────────────
function PreCheckNGScreen({ details = [] }) {
  const { setNav } = useContext(window.AppContext);

  const checks = details.length > 0
    ? details.map(d => ({ label: d, ok: false, detail: d, fix: null }))
    : [
        { label: 'ttyUSB0 (アクセル)', ok: true,  detail: 'pymodbus connect OK', fix: null },
        { label: 'ttyUSB1 (ブレーキ)', ok: true,  detail: 'pymodbus connect OK', fix: null },
        { label: 'CAN (シャシダイナモ)', ok: true, detail: '/dev/usbcanII0 ready', fix: null },
        { label: 'サーボ状態',         ok: true,  detail: 'サーボON確認 / アラームなし', fix: null },
        { label: 'キャリブレーション', ok: false, detail: 'キャリブレーションデータが未設定です',
          fix: 'キャリブレーション画面でアクセル/ブレーキの ZERO・FULL 位置を設定してください。', nav: 'calibration' },
        { label: 'プロファイル選択',   ok: true,  detail: 'プロファイル選択済み', fix: null },
        { label: 'UPS残量',           ok: false, detail: 'UPS残量が低下しています (20% 以上必要)',
          fix: 'UPS を AC 電源に接続し、残量が 20% を超えるまで充電してから再試行してください。', nav: null },
        { label: 'アクチュエータ位置', ok: true,  detail: 'acc: 0 pulse / brk: 0 pulse (±10 以内)', fix: null },
      ];

  const failCount = checks.filter(c => !c.ok).length;

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: 16 }}>
      <div style={{
        display: 'flex', alignItems: 'center', gap: 14,
        padding: '14px 20px',
        border: '2px solid #a23232', background: '#fdf0ee', borderRadius: 6,
      }}>
        <div style={{ fontSize: 28, color: '#a23232' }}>✗</div>
        <div>
          <div style={{ fontSize: 22, fontWeight: 800, color: '#5e1414' }}>
            走行前チェック NG — {failCount}項目が未通過
          </div>
          <div style={{ fontSize: 14, color: '#8a3030' }}>
            すべての項目が OK になるまで走行を開始できません。
          </div>
        </div>
        <div style={{ flex: 1 }} />
        <Btn big>⟳ 再チェック</Btn>
      </div>

      <div style={{ display: 'flex', gap: 16, flex: 1, minHeight: 0 }}>
        <Box label="チェック結果" style={{ flex: 1, padding: '14px 16px', display: 'flex', flexDirection: 'column', gap: 8, overflow: 'auto' }}>
          {checks.map(({ label, ok, detail }) => (
            <div key={label} style={{
              display: 'flex', alignItems: 'center', gap: 10,
              padding: '8px 10px',
              border: `1.2px solid ${ok ? '#3f6b3f' : '#a23232'}`,
              background: ok ? '#dfeadc' : '#fdf0ee',
              borderRadius: 4,
            }}>
              <div style={{ fontSize: 18, color: ok ? '#22421f' : '#a23232', fontWeight: 700, width: 20, textAlign: 'center', flexShrink: 0 }}>
                {ok ? '✓' : '✗'}
              </div>
              <div style={{ flex: 1 }}>
                <div style={{ fontSize: 15, fontWeight: ok ? 400 : 700, color: ok ? '#22421f' : '#5e1414' }}>{label}</div>
                <div style={{ fontSize: 12, fontFamily: 'monospace', color: ok ? '#3f6b3f' : '#8a3030' }}>{detail}</div>
              </div>
              <div style={{ fontSize: 13, color: ok ? '#3f6b3f' : '#a23232', fontWeight: 700 }}>{ok ? 'OK' : 'NG'}</div>
            </div>
          ))}
        </Box>

        <div style={{ width: 340, display: 'flex', flexDirection: 'column', gap: 12 }}>
          <Box label="解決が必要な項目" style={{ padding: '14px 16px', flex: 1, overflow: 'auto' }}>
            {checks.filter(c => !c.ok).map(({ label, fix, nav }) => (
              <div key={label} style={{ marginBottom: 18 }}>
                <div style={{ display: 'flex', gap: 8, alignItems: 'center', marginBottom: 6 }}>
                  <div style={{ fontSize: 14, color: '#a23232' }}>✗</div>
                  <div style={{ fontSize: 16, fontWeight: 700, color: '#5e1414' }}>{label}</div>
                </div>
                <div style={{ fontSize: 14, color: INK_SOFT, lineHeight: 1.6, paddingLeft: 22 }}>{fix}</div>
                {nav && (
                  <div style={{ paddingLeft: 22, marginTop: 8 }}>
                    <Btn onClick={() => setNav(nav)}>{label}画面へ →</Btn>
                  </div>
                )}
              </div>
            ))}
          </Box>
          <Note>UPS残量は充電中でも確認できます。キャリブレーションは毎回の車両セット後に必要です。</Note>
        </div>
      </div>
    </div>
  );
}

// ── EMERGENCY リセット — InitB ────────────────────────────
function EmergencyResetScreen() {
  const { apiFetch, setNav } = useContext(window.AppContext);

  const STOP_LOG = [
    { label: 'GPIO 割り込み検知',                   note: '非常停止スイッチ検知' },
    { label: '制御ループ停止',                      note: '制御ループを即時停止' },
    { label: '原点復帰 (両軸 asyncio.gather)',       note: 'HEND=1 確認済み' },
    { label: 'セッション終了 (status=emergency)',     note: 'PostgreSQL 書き込み完了' },
    { label: 'ログフラッシュ',                       note: 'ファイル保存完了' },
  ];

  async function handleReset() {
    const r = await apiFetch('POST', '/api/v1/drive/reset-emergency');
    if (r) {
      window.showToast('非常停止をリセットしました', 'success');
      setNav('init');
    }
  }

  return (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: 24 }}>
      <div style={{
        border: '3px solid #a23232', background: '#f1d8d2',
        padding: '24px 40px', borderRadius: 8,
        textAlign: 'center', transform: 'rotate(-0.6deg)',
        boxShadow: '4px 4px 0 rgba(0,0,0,0.15)',
      }}>
        <div style={{ fontSize: 38, fontWeight: 800, color: '#5e1414', letterSpacing: 1 }}>
          EMERGENCY
        </div>
        <div style={{ fontSize: 16, color: '#5e1414', marginTop: 4 }}>
          非常停止スイッチ (操作エリア) が押されました
        </div>
      </div>

      <Box style={{ width: 600, padding: 18 }}>
        <H2 sub="停止シーケンスが完了しました">停止シーケンスの記録</H2>
        {STOP_LOG.map(({ label, note }) => (
          <div key={label} style={{
            display: 'flex', alignItems: 'center', gap: 10,
            padding: '6px 0', fontSize: 14,
          }}>
            <span style={{ color: '#3f6b3f', fontWeight: 700 }}>✓</span>
            <span style={{ flex: 1 }}>{label}</span>
            <span style={{ fontSize: 11, fontFamily: 'monospace', color: INK_SOFT }}>{note}</span>
          </div>
        ))}
      </Box>

      <div style={{ display: 'flex', gap: 14 }}>
        <Btn big onClick={() => setNav('logs')}>ログを確認</Btn>
        <Btn primary big onClick={handleReset}>
          EMERGENCY をリセット → READY へ
        </Btn>
      </div>
    </div>
  );
}

Object.assign(window, {
  BootingScreen,
  ErrorScreen,
  AcPowerLossScreen,
  AutoStopDeviationScreen,
  AutoStopOvercurrentScreen,
  AutoStopCanTimeoutScreen,
  PreCheckNGScreen,
  EmergencyResetScreen,
});
