// ── Errors screen（RPD_Phase_20261120_ErrorLog）─────────────────────────
// 左: エラーのできごと一覧（新しい順。日時・赤/黄・きっかけ・題名・回数。確認済みは薄く）。上に絞り込みと一括操作。
// 右: 選んだ 1 件の詳細（題名・日時・きっかけ・本文・関連・直前の進行メッセージ）。確認済み・削除・セッションのログへ。
// サーバーが error_events に記録したものを GET /api/v1/errors で読む。

const ERR_SOURCE_LABEL = {
  job: 'ジョブ',
  emergency: '非常停止',
  idle: '待機中',
  prepare: '計画の準備',
  sequence: 'シーケンス',
  server: 'サーバー',
  browser: '画面',
};

const ERR_JOB_KIND_LABEL = {
  learning: '学習運転',
  mode_drive: '自動運転（モード走行）',
  schedule: '自動運転（スケジュール）',
  interactive: '対話ジョブ',
  init_only: '初期化',
  ign_off: 'IGN-OFF',
};

const ERR_PAGE = 50;
const ERR_RED = '#e06666';
const ERR_YELLOW = '#f0c84a';

function errLevelColor(level) { return level === 'ERROR' ? ERR_RED : ERR_YELLOW; }
function errLevelLabel(level) { return level === 'ERROR' ? 'エラー' : '警告'; }
function errFmt(t) { return t ? new Date(t).toLocaleString('ja-JP') : ''; }

function ErrorsScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, setNav } = useContext(window.AppContext);
  const { Btn, Note, ConfirmStopPopup, INK, PAPER_2, HATCH } = window;

  const [items, setItems] = useState(null);          // null = 読み込み中
  const [total, setTotal] = useState(0);
  const [level, setLevel] = useState('');           // '' = すべて / 'ERROR' / 'WARNING'
  const [unackOnly, setUnackOnly] = useState(false);
  const [selectedId, setSelectedId] = useState(null);
  const [confirm, setConfirm] = useState(null);      // { message, onYes }

  function query(offset) {
    const p = new URLSearchParams({ limit: String(ERR_PAGE), offset: String(offset) });
    if (level) p.set('level', level);
    if (unackOnly) p.set('unacknowledged', 'true');
    return `/api/v1/errors?${p.toString()}`;
  }

  async function load() {
    const d = await apiFetch('GET', query(0));
    if (d) { setItems(d.items); setTotal(d.total); } else if (items === null) { setItems([]); }
  }
  async function loadMore() {
    const d = await apiFetch('GET', query(items.length));
    if (d) { setItems(prev => [...prev, ...d.items]); setTotal(d.total); }
  }
  useEffect(() => { load(); }, [level, unackOnly]);

  async function acknowledgeAll() {
    const r = await apiFetch('POST', '/api/v1/errors/acknowledge-all');
    if (r) load();
  }
  async function deleteAcknowledged() {
    setConfirm(null);
    const r = await apiFetch('DELETE', '/api/v1/errors/acknowledged');
    if (r) { setSelectedId(null); load(); }
  }

  const rowStyle = on => ({
    padding: '8px 12px', borderRadius: 4, cursor: 'pointer',
    background: on ? 'rgba(200,146,42,0.14)' : PAPER_2,
    border: `${on ? 2 : 1}px solid ${on ? INK : HATCH}`,
  });
  const filterBtn = (label, on, onClick) => (
    <Btn primary={on} onClick={onClick}>{label}</Btn>
  );

  return (
    <div style={{ display: 'flex', gap: 12, flex: 1, minHeight: 0, position: 'relative' }}>
      <div style={{ flex: '0 0 clamp(240px, 26%, 340px)', minHeight: 0, display: 'flex', flexDirection: 'column', gap: 8 }}>
        <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap', flexShrink: 0 }}>
          {filterBtn('すべて', level === '', () => setLevel(''))}
          {filterBtn('エラー', level === 'ERROR', () => setLevel('ERROR'))}
          {filterBtn('警告', level === 'WARNING', () => setLevel('WARNING'))}
          {filterBtn('未確認のみ', unackOnly, () => setUnackOnly(v => !v))}
        </div>
        <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap', flexShrink: 0 }}>
          <Btn onClick={acknowledgeAll}>すべて確認済みにする</Btn>
          <Btn danger onClick={() => setConfirm({ message: '確認済みのエラーをすべて削除しますか？', onYes: deleteAcknowledged })}>確認済みを削除</Btn>
        </div>
        {items === null
          ? <div style={{ fontSize: 13, color: '#888' }}>読み込み中…</div>
          : items.length === 0
            ? <Note>エラーの記録がありません。</Note>
            : (
              <div style={{ display: 'flex', flexDirection: 'column', gap: 6, overflowY: 'auto', flex: 1, minHeight: 0 }}>
                {items.map(e => (
                  <div key={e.id} onClick={() => setSelectedId(e.id)} style={{ ...rowStyle(selectedId === e.id), opacity: e.acknowledged_at ? 0.5 : 1 }}>
                    <div style={{ fontSize: 11, color: '#888' }}>
                      <span style={{ color: errLevelColor(e.level), fontWeight: 700 }}>{errLevelLabel(e.level)}</span>
                      {`　${ERR_SOURCE_LABEL[e.source] ?? e.source}　${errFmt(e.last_at)}`}
                      {e.count > 1 ? `　×${e.count}` : ''}
                    </div>
                    <div style={{ fontWeight: 700, fontSize: 13, lineHeight: 1.35, wordBreak: 'break-all' }}>{e.title}</div>
                  </div>
                ))}
                {items.length < total && <Btn onClick={loadMore}>さらに読み込む（{items.length} / {total} 件）</Btn>}
              </div>
            )}
      </div>
      <div style={{ flex: 1, minWidth: 0, minHeight: 0, display: 'flex', flexDirection: 'column',
                    border: `1px solid ${HATCH}`, borderRadius: 4, background: PAPER_2, padding: '8px 12px' }}>
        {selectedId !== null
          ? <ErrorDetail key={selectedId} eventId={selectedId} setConfirm={setConfirm}
              onChanged={load} onDeleted={() => { setSelectedId(null); load(); }}
              onOpenSession={sid => {
                try { localStorage.setItem('drv_logs_open_session', sid); } catch (e) { /* 保存できなければログページを開くだけ */ }
                setNav('logs');
              }} />
          : <div style={{ fontSize: 13, color: '#888' }}>左の一覧からエラーを選んでください。</div>}
      </div>
      {confirm && <ConfirmStopPopup message={confirm.message} onYes={confirm.onYes} onNo={() => setConfirm(null)} />}
    </div>
  );
}

function ErrorDetail({ eventId, setConfirm, onChanged, onDeleted, onOpenSession }) {
  const { useState, useEffect, useContext } = React;
  const { apiFetch } = useContext(window.AppContext);
  const { Btn, INK, INK_SOFT, INK_MUTE, HATCH, PAPER } = window;
  const [e, setE] = useState(null);   // null = 読み込み中

  useEffect(() => {
    let alive = true;
    apiFetch('GET', `/api/v1/errors/${eventId}`).then(d => { if (alive && d) setE(d); });
    return () => { alive = false; };
  }, [eventId]);

  if (e === null) return <div style={{ fontSize: 13, color: '#888' }}>読み込み中…</div>;

  async function toggleAck() {
    const r = await apiFetch('POST', `/api/v1/errors/${e.id}/${e.acknowledged_at ? 'unacknowledge' : 'acknowledge'}`);
    if (r) { setE(r); onChanged(); }
  }
  async function remove() {
    setConfirm(null);
    const r = await apiFetch('DELETE', `/api/v1/errors/${e.id}`);
    if (r) onDeleted();
  }

  const msgs = (e.context && e.context.messages) || [];
  const colorOf = lv => (lv === 'ERROR' || lv === 'CRITICAL' ? '#f07070' : lv === 'WARNING' ? '#f0c84a' : INK);
  const related = [
    e.job_kind ? ['ジョブの種類', ERR_JOB_KIND_LABEL[e.job_kind] ?? e.job_kind] : null,
    e.job_id ? ['ジョブ ID', e.job_id] : null,
    e.vehicle_id ? ['車', e.vehicle_id] : null,
    e.session_id ? ['セッション ID', e.session_id] : null,
    e.sequence_name ? ['シーケンス', e.sequence_name] : null,
  ].filter(Boolean);
  const timeOf = m => { try { return new Date(m.time).toLocaleTimeString('ja-JP'); } catch (x) { return ''; } };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0, gap: 8 }}>
      <div style={{ flexShrink: 0 }}>
        <div style={{ fontSize: 16, fontWeight: 700, wordBreak: 'break-all' }}>
          <span style={{ color: errLevelColor(e.level), marginRight: 8 }}>{errLevelLabel(e.level)}</span>{e.title}
        </div>
        <div style={{ fontSize: 12, color: INK_SOFT, marginTop: 2 }}>
          {`${ERR_SOURCE_LABEL[e.source] ?? e.source}　最初 ${errFmt(e.first_at)}　最後 ${errFmt(e.last_at)}　${e.count} 回`}
          {e.acknowledged_at ? `　確認済み ${errFmt(e.acknowledged_at)}` : ''}
        </div>
        <div style={{ display: 'flex', gap: 6, marginTop: 6, flexWrap: 'wrap' }}>
          <Btn onClick={toggleAck}>{e.acknowledged_at ? '確認済みを戻す' : '確認済みにする'}</Btn>
          <Btn danger onClick={() => setConfirm({ message: 'このエラーを削除しますか？', onYes: remove })}>削除</Btn>
          {e.session_id && <Btn onClick={() => onOpenSession(e.session_id)}>このセッションのログを開く</Btn>}
        </div>
      </div>
      <div style={{ flex: 1, minHeight: 0, overflowY: 'auto', display: 'flex', flexDirection: 'column', gap: 8 }}>
        <div>
          <div style={{ fontSize: 13, fontWeight: 700, marginBottom: 4 }}>本文</div>
          <div style={{ border: `1px solid ${HATCH}`, borderRadius: 3, padding: '6px 10px', background: PAPER, fontSize: 13, lineHeight: 1.5, whiteSpace: 'pre-wrap', wordBreak: 'break-all' }}>
            {e.detail || '（本文なし）'}
          </div>
        </div>
        {related.length > 0 && (
          <div>
            <div style={{ fontSize: 13, fontWeight: 700, marginBottom: 4 }}>関連</div>
            <div style={{ fontSize: 13, lineHeight: 1.6 }}>
              {related.map(([k, v]) => (
                <div key={k}><span style={{ color: INK_SOFT, display: 'inline-block', minWidth: 110 }}>{k}</span>{v}</div>
              ))}
            </div>
          </div>
        )}
        <div style={{ display: 'flex', flexDirection: 'column', minHeight: 120 }}>
          <div style={{ fontSize: 13, fontWeight: 700, marginBottom: 4 }}>
            直前の進行メッセージ<span style={{ fontWeight: 400, color: INK_SOFT, marginLeft: 8 }}>{msgs.length} 行</span>
          </div>
          <div style={{ border: `1px solid ${HATCH}`, borderRadius: 3, padding: '6px 10px', background: PAPER,
                        fontFamily: 'ui-monospace, Menlo, Consolas, monospace', fontSize: 12, lineHeight: 1.45 }}>
            {msgs.length === 0
              ? <div style={{ color: INK_MUTE }}>メッセージはありません</div>
              : msgs.map((m, i) => (
                <div key={i} style={{ whiteSpace: 'pre-wrap', wordBreak: 'break-all', color: colorOf(m.level) }}>
                  {`[${timeOf(m)}] ${m.text}`}
                </div>
              ))}
          </div>
        </div>
      </div>
    </div>
  );
}

window.ErrorsScreen = ErrorsScreen;
