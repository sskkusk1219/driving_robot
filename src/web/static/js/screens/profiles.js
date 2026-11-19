// ── 車両プロファイル（RPD D-12） ────────────────────────────
// 車は vehicles/<車ID>/vehicle.yaml。ID はフォルダ名（API は /api/v1/profiles/*）。
//   人が決める値（名前・最高速・最大減速G・min/max 開度・停止判定・運転指標用）→ 編集できる
//   学習の結果（日時・モデル・クリープ車速・停車保持の開度）→ 読むだけ（学習運転が書く）
// ボタンの設定は共通設定（config/robot_common.yaml）へ移した（段11）。

// 一覧の列幅（ヘッダと各行で同じ値。fr は minmax(0, …)、操作（4 ボタン）は固定 px）
const PROFILE_LIST_COLS = ['minmax(0, 2fr)', 'minmax(0, 1.2fr)', 'minmax(0, 1fr)', 'minmax(0, 0.9fr)', 'minmax(0, 0.7fr)', '290px'];

// 画面に出す vehicle.* の数値欄（キー, ラベル）。edit のときだけ出すものは別
const VEHICLE_BASIC_FIELDS = [
  ['max_speed_kmh', '最高速 [km/h]'],
  ['max_decel_g', '最大減速G [G]'],
];
const VEHICLE_MAX_FIELDS = [
  ['max_accel_opening_pct', 'アクセル max [%]'],
  ['max_brake_opening_pct', 'ブレーキ max [%]'],
];
const VEHICLE_MIN_FIELDS = [
  ['min_accel_opening_pct', 'アクセル min [%]'],
  ['min_brake_opening_pct', 'ブレーキ min [%]'],
];
const VEHICLE_STOP_FIELDS = [
  ['stop_deviation_threshold_kmh', '異常停止の偏差しきい値 [km/h]'],
  ['stop_deviation_duration_s', 'しきい値が続いたら停止 [s]'],
];
const VEHICLE_ROADLOAD_FIELDS = [
  ['test_mass_kg', '試験質量 [kg]'],
  ['road_load_a0_n', '走行抵抗 a0 [N]'],
  ['road_load_a1_n_per_kmh', 'a1 [N/(km/h)]'],
  ['road_load_a2_n_per_kmh2', 'a2 [N/(km/h)^2]'],
];
function fmtNum(v) { return v == null ? '' : String(v); }

function fmtFeedforward(v) {
  if (Array.isArray(v)) return v.length ? `${v.length} 点` : '（空）';
  if (v == null) return '—';
  return String(v);
}

function ProfilesScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeProfileId, setActiveProfileId, activeProfileName, setActiveProfileName, setNav } = useContext(window.AppContext);
  const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH, Box, Btn, Pill, H2, Row, RowActions, ConfirmStopPopup } = window;

  const [profiles, setProfiles] = useState([]);
  const [mode, setMode] = useState('list'); // list | create | edit
  const [editTarget, setEditTarget] = useState(null);
  const [search, setSearch] = useState('');
  const [sortKey, setSortKey] = useState(null);  // 'name' | 'max_speed_kmh'
  const [sortAsc, setSortAsc] = useState(true);
  const [editingNameId, setEditingNameId] = useState(null);
  const [editingNameValue, setEditingNameValue] = useState('');
  const [confirmDelete, setConfirmDelete] = useState(null);

  useEffect(() => { loadProfiles(); }, []);

  async function loadProfiles() {
    const data = await apiFetch('GET', '/api/v1/profiles/');
    if (data) setProfiles(data);
  }

  async function handleSelect(p) {
    const r = await apiFetch('POST', '/api/v1/drive/select-profile', { profile_id: p.id });
    if (!r) return;
    setActiveProfileId(p.id);
    setActiveProfileName(p.name);
    window.showToast(`「${p.name}」を選択しました`, 'success');
    loadProfiles();
  }

  function startEditName(p) { setEditingNameId(p.id); setEditingNameValue(p.name); }
  function cancelEditName() { setEditingNameId(null); setEditingNameValue(''); }

  async function handleSaveName(p) {
    const trimmed = editingNameValue.trim();
    if (!trimmed) { window.showToast('名前を入力してください', 'error'); return; }
    if (trimmed === p.name) { cancelEditName(); return; }
    const r = await apiFetch('PUT', `/api/v1/profiles/${p.id}`, { name: trimmed });
    if (r) {
      window.showToast('名前を更新しました', 'success');
      if (p.id === activeProfileId) setActiveProfileName(trimmed);
      cancelEditName();
      loadProfiles();
    }
  }

  async function handleCopy(p) {
    // 学習の値（pkl・feedforward）はコピーしない。min は仮値として引き継ぐ（新しい車では未測定から始まる）
    const vehicleKeys = [...VEHICLE_BASIC_FIELDS, ...VEHICLE_MAX_FIELDS, ...VEHICLE_MIN_FIELDS, ...VEHICLE_STOP_FIELDS, ...VEHICLE_ROADLOAD_FIELDS].map(f => f[0]);
    const payload = { name: `${p.name} のコピー` };
    for (const k of vehicleKeys) if (p[k] != null) payload[k] = p[k];
    if (p.road_load_source) payload.road_load_source = p.road_load_source;
    const r = await apiFetch('POST', '/api/v1/profiles/', payload);
    if (r) {
      window.showToast(`「${r.name}」を作成しました（学習・min の測定はこれからです）`, 'success');
      loadProfiles();
    }
  }

  async function doDelete(p) {
    const r = await apiFetch('DELETE', `/api/v1/profiles/${p.id}`);
    setConfirmDelete(null);
    if (r !== null) {
      window.showToast('車を削除しました', 'success');
      if (p.id === activeProfileId) { setActiveProfileId(null); setActiveProfileName(null); }
      setMode('list');
      setEditTarget(null);
      loadProfiles();
    }
  }

  const deletePopup = confirmDelete && (
    <ConfirmStopPopup
      message={`「${confirmDelete.name}」を削除しますか？（学習結果・学習データも消えます）`}
      onYes={() => doDelete(confirmDelete)}
      onNo={() => setConfirmDelete(null)}
    />
  );

  if (mode === 'create') {
    return (
      <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 }}>
        <ProfileForm
          onSave={async payload => {
            const r = await apiFetch('POST', '/api/v1/profiles/', payload);
            if (!r) return;
            window.showToast(`「${r.name}」を作成しました。次は min・max の測定です`, 'success');
            // 作った車をそのまま選ぶ（案内の続き: ② min を測る → ③ 学習）
            const s = await apiFetch('POST', '/api/v1/drive/select-profile', { profile_id: r.id });
            if (s) { setActiveProfileId(r.id); setActiveProfileName(r.name); }
            setMode('list');
            loadProfiles();
          }}
          onCancel={() => setMode('list')}
        />
      </div>
    );
  }

  if (mode === 'edit' && editTarget) {
    return (
      <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 }}>
        <ProfileForm
          key={`${editTarget.id}:${editTarget.updated_at ?? ''}`}
          initial={editTarget}
          onSave={async payload => {
            if (Object.keys(payload).length === 0) {
              window.showToast('変更はありません', 'info');
              return;
            }
            const r = await apiFetch('PUT', `/api/v1/profiles/${editTarget.id}`, payload);
            if (r) {
              window.showToast(`「${r.name}」を更新しました`, 'success');
              if (r.id === activeProfileId) setActiveProfileName(r.name);
              setEditTarget(r);   // 保存後も画面に残る（min の測定済み印など、保存後の値を見直せる）
              loadProfiles();
            }
          }}
          onCancel={() => { setMode('list'); setEditTarget(null); loadProfiles(); }}
          onDelete={() => setConfirmDelete(editTarget)}
          onNav={setNav}
        />
        {deletePopup}
      </div>
    );
  }

  // ── 一覧 ────────────────────────────────────────────────────
  const activeProfile = profiles.find(p => p.id === activeProfileId);

  function handleSortClick(key) {
    if (sortKey === key) { setSortAsc(a => !a); } else { setSortKey(key); setSortAsc(true); }
  }
  function sortIcon(key) {
    if (sortKey !== key) return ' ↕';
    return sortAsc ? ' ↑' : ' ↓';
  }

  const q = search.trim().toLowerCase();
  const filtered = profiles.filter(p => p.name.toLowerCase().includes(q));
  const sorted = sortKey
    ? [...filtered].sort((a, b) => {
        const va = a[sortKey], vb = b[sortKey];
        const cmp = typeof va === 'string' ? va.localeCompare(vb, 'ja') : (va ?? 0) - (vb ?? 0);
        return sortAsc ? cmp : -cmp;
      })
    : filtered;

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 10, height: '100%' }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
        <div style={{ flex: 1 }} />
        <input
          type="text" value={search} onChange={e => setSearch(e.target.value)} placeholder="名前で検索…"
          style={{ padding: '6px 12px', fontSize: 14, border: `1.3px solid ${INK}`, borderRadius: 4,
                   fontFamily: 'inherit', background: PAPER, color: INK, outline: 'none', width: 200 }}
        />
        <Btn primary big onClick={() => setMode('create')}>＋ 新規作成</Btn>
      </div>

      <Box style={{ padding: 0, display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 }}>
        <div style={{
          display: 'grid', gridTemplateColumns: PROFILE_LIST_COLS.join(' '),
          borderBottom: `1px solid ${INK}`, padding: '10px 14px', background: PAPER_2,
          fontSize: 14, fontWeight: 700, flexShrink: 0,
        }}>
          <div onClick={() => handleSortClick('name')} style={{ cursor: 'pointer', userSelect: 'none' }}>{'名前' + sortIcon('name')}</div>
          <div>最大開度 (acc/brk)</div>
          <div onClick={() => handleSortClick('max_speed_kmh')} style={{ cursor: 'pointer', userSelect: 'none' }}>{'最高車速' + sortIcon('max_speed_kmh')}</div>
          <div>ペダル min</div>
          <div>学習</div>
          <div>操作</div>
        </div>

        <div style={{ overflowY: 'auto', flex: 1, minHeight: 0 }}>
          {sorted.length === 0
            ? <div style={{ padding: '20px 14px', color: INK_SOFT, fontSize: 14 }}>
                {profiles.length === 0 ? '車がありません。「＋ 新規作成」で登録してください。' : `「${search}」に一致する車がありません。`}
              </div>
            : sorted.map(p => {
                const isActive = p.id === activeProfileId;
                const minOk = p.min_accel_measured && p.min_brake_measured;
                const nameCell = editingNameId === p.id
                  ? (
                    <div style={{ display: 'flex', gap: 4, alignItems: 'center' }}>
                      <input
                        autoFocus value={editingNameValue} onChange={e => setEditingNameValue(e.target.value)}
                        onKeyDown={e => { if (e.key === 'Enter') handleSaveName(p); if (e.key === 'Escape') cancelEditName(); }}
                        style={{ fontSize: 14, padding: '2px 6px', border: `1.3px solid ${INK}`, borderRadius: 3,
                                 fontFamily: 'inherit', background: PAPER, color: INK, outline: 'none', width: '100%' }}
                      />
                      <span onClick={() => handleSaveName(p)} title="保存" style={{ cursor: 'pointer', fontSize: 16, userSelect: 'none' }}>✓</span>
                      <span onClick={cancelEditName} title="キャンセル" style={{ cursor: 'pointer', fontSize: 16, userSelect: 'none' }}>✗</span>
                    </div>
                  ) : (
                    <div style={{ display: 'flex', gap: 6, alignItems: 'center' }}>
                      <b>{p.name}</b>
                      {p.error && <Pill accent="ERROR" style={{ fontSize: 11 }}>yaml 異常</Pill>}
                      <span onClick={() => startEditName(p)} title="名前を変更" style={{ cursor: 'pointer', opacity: 0.5, fontSize: 13, userSelect: 'none' }}>✎</span>
                    </div>
                  );
                return (
                  <Row
                    key={p.id}
                    cells={[
                      [nameCell, PROFILE_LIST_COLS[0]],
                      [p.max_accel_opening_pct == null ? '—' : `${p.max_accel_opening_pct} / ${p.max_brake_opening_pct} %`, PROFILE_LIST_COLS[1], 'mono'],
                      [p.max_speed_kmh == null ? '—' : `${p.max_speed_kmh} km/h`, PROFILE_LIST_COLS[2], 'mono'],
                      [minOk ? <Pill accent="READY">測定済み</Pill>
                             : <Pill accent="PAUSED">{p.min_accel_measured || p.min_brake_measured ? '片方未測定' : '未測定'}</Pill>, PROFILE_LIST_COLS[3]],
                      [p.learned ? <Pill accent="READY">あり</Pill> : <Pill>なし</Pill>, PROFILE_LIST_COLS[4]],
                      [<RowActions
                         isActive={isActive}
                         onSelect={() => handleSelect(p)}
                         onEdit={() => { setEditTarget(p); setMode('edit'); }}
                         onCopy={() => handleCopy(p)}
                         onDelete={() => setConfirmDelete(p)}
                       />, PROFILE_LIST_COLS[5]],
                    ]}
                    style={{ padding: '12px 14px', background: isActive ? '#201e16' : 'transparent', borderBottom: `1px dashed ${INK_MUTE}` }}
                  />
                );
              })}
        </div>
      </Box>

      <div style={{ display: 'flex', gap: 12, fontSize: 13, color: INK_SOFT }}>
        {activeProfile ? <span>選択中: <b>{activeProfile.name}</b></span> : <span>(未選択)</span>}
        <div style={{ flex: 1 }} />
        <span>{q ? `${sorted.length} / ${profiles.length}件` : `${profiles.length}件`}</span>
      </div>
      {deletePopup}
    </div>
  );
}

// ── ProfileForm — 新規作成・編集共用（1 ページに収まる） ────────────────────
// 新規作成: 基本（名前・最高速・最大減速G）とペダル max（min は未測定の仮値で作られる）
// 編集: 基本／ペダル min・max／停止判定／運転指標用／学習の結果（読むだけ）。
//   保存では「変えた値だけ」送る（min を送ると測定済みになるため、触っていない min は送らない）
function ProfileForm({ initial, onSave, onCancel, onDelete, onNav }) {
  const { useState } = React;
  const { INK, INK_SOFT, INK_MUTE, HATCH, Box, Btn, Pill, H2, Input } = window;
  const isEdit = !!initial;

  const initialForm = () => {
    const f = { name: initial?.name ?? '' };
    const all = [...VEHICLE_BASIC_FIELDS, ...VEHICLE_MAX_FIELDS, ...VEHICLE_MIN_FIELDS, ...VEHICLE_STOP_FIELDS, ...VEHICLE_ROADLOAD_FIELDS];
    for (const [k] of all) f[k] = fmtNum(initial?.[k]);
    if (!isEdit) { f.max_speed_kmh = '140'; f.max_decel_g = '0.4'; f.max_accel_opening_pct = '80'; f.max_brake_opening_pct = '80'; }
    f.road_load_source = initial?.road_load_source ?? '';
    return f;
  };
  const [form, setForm] = useState(initialForm);
  const [base] = useState(initialForm);   // 変えた値だけ送るための元の値
  const set = (k, v) => setForm(f => ({ ...f, [k]: v }));

  async function handleSave() {
    if (!form.name.trim()) { window.showToast('名前を入力してください', 'error'); return; }
    const numeric = (key) => {
      const raw = String(form[key]).trim();
      if (raw === '') return undefined;
      const n = Number(raw);
      return Number.isFinite(n) ? n : NaN;
    };
    const bad = [];
    const payload = {};
    const consider = (key) => {
      const changed = String(form[key]).trim() !== String(base[key]).trim();
      if (isEdit && !changed) return;
      const v = numeric(key);
      if (v === undefined) return;       // 空欄は送らない（値を消すことはしない）
      if (Number.isNaN(v)) { bad.push(key); return; }
      payload[key] = v;
    };
    const vehicleKeys = [...VEHICLE_BASIC_FIELDS, ...VEHICLE_MAX_FIELDS];
    if (isEdit) vehicleKeys.push(...VEHICLE_STOP_FIELDS, ...VEHICLE_ROADLOAD_FIELDS, ...VEHICLE_MIN_FIELDS);   // 新規作成では min は仮値のまま（送らない）
    for (const [k] of vehicleKeys) consider(k);
    if (isEdit) {
      if (form.name.trim() !== base.name) payload.name = form.name.trim();
      if (form.road_load_source !== base.road_load_source && form.road_load_source.trim()) payload.road_load_source = form.road_load_source.trim();
    } else {
      payload.name = form.name.trim();
    }
    // 画面で先に見る: min は max より小さく（サーバーの 422 と同じ条件）
    const effective = (key) => (payload[key] !== undefined ? payload[key] : Number(String(base[key]).trim()));
    for (const [minKey, maxKey, label] of [
      ['min_accel_opening_pct', 'max_accel_opening_pct', 'アクセル'],
      ['min_brake_opening_pct', 'max_brake_opening_pct', 'ブレーキ'],
    ]) {
      if (payload[minKey] === undefined && payload[maxKey] === undefined) continue;
      const mn = effective(minKey), mx = effective(maxKey);
      if (Number.isFinite(mn) && Number.isFinite(mx) && mn >= mx) {
        window.showToast(`${label} max は ${label} min（${mn}%）より大きくしてください`, 'error');
        return;
      }
    }
    if (bad.length) { window.showToast(`数値として読めない欄があります: ${bad.join('、')}\n次にすること: 数字だけで入力し直してください。`, 'error'); return; }
    await onSave(payload);
  }

  // 保存後（initial が更新されたとき）に、フォームの「元の値」と測定済みの印が古くならないよう、親が key で作り直す
  const field = (label, key, opts = {}) => (
    <Input key={key} label={label} value={String(form[key])} onChange={v => set(key, v)} type="text" mono={opts.mono ?? true} width="100%" />
  );
  const grid2 = { display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 8 };
  const boxStyle = { padding: '12px 14px', display: 'flex', flexDirection: 'column', gap: 8, minHeight: 0, minWidth: 0 };

  const minAccelOk = !!initial?.min_accel_measured;
  const minBrakeOk = !!initial?.min_brake_measured;
  const ff = initial?.feedforward ?? {};
  const readonlyRow = (label, value, title) => (
    <div style={{ display: 'flex', justifyContent: 'space-between', gap: 12, fontSize: 13, color: INK_SOFT, minWidth: 0 }}>
      <span style={{ flexShrink: 0 }}>{label}</span>
      <span title={title ?? String(value)} style={{ color: INK, fontFamily: "'JetBrains Mono', monospace", overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap', minWidth: 0 }}>{value}</span>
    </div>
  );
  const fmtVal = v => (v == null ? '—' : String(v));

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 10, height: '100%', minHeight: 0 }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 12, flexShrink: 0 }}>
        <Btn onClick={onCancel}>← 一覧に戻る</Btn>
        <H2>{isEdit ? `車の編集: ${initial.name}` : '新規作成'}</H2>
      </div>

      <div style={{
        flex: 1, minHeight: 0, display: 'grid', gap: 12, alignContent: 'start',
        gridTemplateColumns: isEdit ? 'repeat(3, minmax(0, 1fr))' : 'repeat(2, minmax(0, 1fr))',
        gridTemplateRows: isEdit ? 'repeat(2, minmax(0, 1fr))' : 'auto',
      }}>
        <Box label="基本" style={boxStyle}>
          {field('名前', 'name', { mono: false })}
          <div style={grid2}>{VEHICLE_BASIC_FIELDS.map(([k, label]) => field(label, k))}</div>
        </Box>

        <Box label={isEdit ? 'ペダル min・max' : 'ペダル max'} style={boxStyle}>
          {isEdit && (
            <div style={{ display: 'flex', gap: 8, alignItems: 'center', fontSize: 13, color: INK_SOFT }}>
              <span>アクセル</span>{minAccelOk ? <Pill accent="READY">測定済み</Pill> : <Pill accent="PAUSED">未測定</Pill>}
              <span>ブレーキ</span>{minBrakeOk ? <Pill accent="READY">測定済み</Pill> : <Pill accent="PAUSED">未測定</Pill>}
            </div>
          )}
          {isEdit && <div style={grid2}>{VEHICLE_MIN_FIELDS.map(([k, label]) => field(label, k))}</div>}
          <div style={grid2}>{VEHICLE_MAX_FIELDS.map(([k, label]) => field(label, k))}</div>
        </Box>

        {isEdit && (
          <Box label="停止判定" style={boxStyle}>
            {VEHICLE_STOP_FIELDS.map(([k, label]) => field(label, k))}
          </Box>
        )}

        {isEdit && (
          <Box label="運転指標用" style={boxStyle}>
            <div style={grid2}>{VEHICLE_ROADLOAD_FIELDS.map(([k, label]) => field(label, k))}</div>
            {field('走行抵抗の出どころ', 'road_load_source', { mono: false })}
          </Box>
        )}

        {isEdit && (
          <Box label="学習の結果（読むだけ）" style={{ ...boxStyle, gridColumn: 'span 2' }}>
            <div style={{ display: 'flex', gap: 10, alignItems: 'center' }}>
              {initial.learned ? <Pill accent="READY">学習済み</Pill> : <Pill>未学習</Pill>}
            </div>
            {readonlyRow('学習日時', initial.learned_at ? new Date(initial.learned_at).toLocaleString('ja-JP') : '—')}
            {readonlyRow('使っているモデル', initial.model_path ?? '—')}
            {readonlyRow('クリープ車速 [km/h]', fmtVal(ff.creep_speed_kmh))}
            {readonlyRow('停車保持の開度 [%]（min からの量）', fmtVal(ff.stop_brake_offset_pct))}
          </Box>
        )}
      </div>

      <div style={{ display: 'flex', gap: 12, justifyContent: 'flex-end', flexShrink: 0 }}>
        {isEdit && <Btn danger onClick={onDelete}>削除</Btn>}
        <Btn onClick={onCancel}>{isEdit ? '一覧に戻る' : 'キャンセル'}</Btn>
        <Btn primary big onClick={handleSave}>{isEdit ? '保存' : '作成'}</Btn>
      </div>
    </div>
  );
}

window.ProfilesScreen = ProfilesScreen;
