// ── ジョブまわりの共通部品（段5） ───────────────────────────────
// ・describeApiError: API の 409/422/423/503 などの detail を「何が起きたか・次に何をするか」にする
// ・EXIT_CODE_INFO / describeJobResult: ランナーの終了コードの意味（src/robot/README.md の表）
// ・useDriveLogLines: GET /api/v1/drive/log（進行のメッセージ）を 1 秒ごとに取る
// ・useLastJob: 実行中のジョブが終わったあとの最後のジョブ（GET /api/v1/drive/status の last_job）
// ・deriveStages / StageRail: 学習運転の段階表示（手順2: ペダル探索・低速の保持・停止手前・コーストダウン・G ランプ・遅れの測定・モデル作成・チェック）
// ・JobResultBanner: 終わったジョブの結果（終了コードの意味・理由・次の操作）

// ── API エラー → 「何が起きたか」「次にすること」 ───────────────────────────
function normalizeDetail(detail, fallback) {
  if (detail == null || detail === '') return fallback || '';
  if (typeof detail === 'string') return detail;
  // FastAPI の入力検証（422）は [{loc, msg}] の配列
  if (Array.isArray(detail)) {
    return detail.map(d => {
      if (d && typeof d === 'object') {
        const loc = Array.isArray(d.loc) ? d.loc.filter(x => x !== 'body').join('.') : '';
        return loc ? `${loc}: ${d.msg ?? ''}` : String(d.msg ?? '');
      }
      return String(d);
    }).join(' / ');
  }
  if (typeof detail === 'object') return detail.message ?? JSON.stringify(detail);
  return String(detail);
}

// 車両プロファイルの 422 の文に出る内部キー名を、画面のラベルへ直す
const PROFILE_KEY_LABELS = [
  ['min_accel_opening_pct', 'アクセル min'],
  ['max_accel_opening_pct', 'アクセル max'],
  ['min_brake_opening_pct', 'ブレーキ min'],
  ['max_brake_opening_pct', 'ブレーキ max'],
  ['max_speed_kmh', '最高速'],
  ['max_decel_g', '最大減速G'],
  ['stop_deviation_threshold_kmh', '異常停止の偏差しきい値'],
  ['stop_deviation_duration_s', 'しきい値が続いたら停止'],
  ['test_mass_kg', '試験質量'],
  ['road_load_a0_n', '走行抵抗 a0'],
  ['road_load_a1_n_per_kmh', '走行抵抗 a1'],
  ['road_load_a2_n_per_kmh2', '走行抵抗 a2'],
];
function translateProfileKeys(text) {
  let t = String(text);
  for (const [k, label] of PROFILE_KEY_LABELS) t = t.split(`vehicle.${k}`).join(label).split(k).join(label);
  return t;
}

function describeApiError(status, detail, statusText) {
  let text = normalizeDetail(detail, statusText);
  if (status === 422) text = translateProfileKeys(text);
  let next = '';
  switch (status) {
    case 409:
      if (/初期化が済んでいません/.test(text)) next = '「初期化」ページで「初期化を実行」を押し、終わってから、もう一度操作してください。';
      else if (/車が選ばれて/.test(text)) next = '「車両プロファイル」で車を選んでから、もう一度操作してください。';
      else if (/非常停止/.test(text)) next = '非常停止を解除してから（非常停止の画面の解除ボタン）、もう一度操作してください。';
      else if (/実行中|終了処理中/.test(text)) next = '実行中のジョブが終わるまで待つか、停止してから、もう一度操作してください。';
      else if (/既にあります|同じ名前|重複/.test(text)) next = '別の名前（ID）にしてください。';
      else if (/初期化/.test(text)) next = '進行のメッセージ（ログ）で原因を確認し、通信やペダルの接続を見てからやり直してください。';
      else next = '今の状態を確認してから、もう一度操作してください。';
      break;
    case 422:
      next = '入力した値を直して、もう一度保存してください（理由は上のとおりです）。何も保存されていません。';
      break;
    case 423:
      next = '操作エリアの非常停止スイッチが押されたままです。スイッチを戻してから、もう一度解除してください。';
      break;
    case 503:
      next = 'データベース（PostgreSQL）やランナーを使えません。記録できないまま走らせないため、開始を止めています。サーバーの状態を確認してください。';
      break;
    case 501:
      next = 'この機能は準備中です（次の段で実装します）。';
      break;
    case 404:
      next = '一覧を開き直して、対象が残っているか確認してください。';
      break;
    default:
      next = status >= 500 ? 'サーバーのログを確認してください。直らなければサーバーを再起動してください。' : '';
  }
  return { what: text || `エラー ${status}`, next };
}
window.describeApiError = describeApiError;

// ── 終了コードの意味（ランナー。src/robot/README.md「終了コード」） ─────────────
const EXIT_CODE_INFO = {
  0:   { label: '成功', level: 'ok', next: '' },
  2:   { label: '設定エラー', level: 'ng', what: '車両の設定が実行条件を満たしていません。', next: '「車両プロファイル」で値を確認して直してから、やり直してください。' },
  3:   { label: '未実装の手順', level: 'ng', what: '未実装の手順に到達しました。', next: 'サーバーを管理している人に連絡してください。' },
  4:   { label: '初期化エラー', level: 'ng', what: '初期化（通信確認・アラームリセット・サーボON・原点復帰）で失敗しました。', next: 'ペダルのアクチュエータと CAN の接続・電源を確認し、「初期化」画面からやり直してください。' },
  5:   { label: '走行エラー', level: 'ng', what: '走行中にエラーで止まりました。ペダルは原点へ戻っています。', next: '進行のメッセージ（ログ）で原因を確認してから、やり直してください。' },
  6:   { label: 'モデル作成エラー', level: 'ng', what: '学習データからのモデル作成に失敗しました。', next: '進行のメッセージ（ログ）で原因を確認し、学習運転をやり直してください。' },
  7:   { label: '走行前チェックエラー', level: 'ng', what: '走行前チェック（ペダル・車速の確認）に通りませんでした。', next: 'ペダルの取り付けと車速信号を確認し、やり直してください。' },
  8:   { label: '学習結果チェック NG', level: 'ng', what: '学習結果のチェックが NG でした。車両 yaml は書き換えていません（学習前の状態のままです）。', next: '理由を確認し、「やり直す」で学習運転をもう一度走らせてください。' },
  130: { label: '中止（SIGINT）', level: 'warn', what: '利用者の操作で中止しました。終了処理（原点復帰など）は済んでいます。', next: '続けるには、もう一度始めてください。' },
  143: { label: '中止（SIGTERM）', level: 'warn', what: 'サーバー側の指示で中止しました。終了処理は済んでいます。', next: '続けるには、もう一度始めてください。' },
};
window.EXIT_CODE_INFO = EXIT_CODE_INFO;

const JOB_KIND_LABEL = {
  learning: '学習運転', mode_drive: '自動運転', schedule: 'スケジュール走行', interactive: '対話操作', init_only: '初期化', ign_off: 'IGN-OFF',
};
window.JOB_KIND_LABEL = JOB_KIND_LABEL;

// 終わったジョブ（last_job）→ {level: ok|warn|ng, title, what, next, reason}
function describeJobResult(job) {
  if (!job) return null;
  const code = job.exit_code ?? job.result?.exit_code ?? null;
  const info = code != null ? EXIT_CODE_INFO[code] : null;
  const kind = JOB_KIND_LABEL[job.kind] ?? job.kind;
  const reason = job.result?.error ?? null;
  if (job.emergency) {
    return {
      level: 'ng', title: `${kind}: 非常停止で止まりました`,
      what: job.brake_kept
        ? '非常停止が押されましたが、停車を確認できませんでした。ブレーキを最大開度で踏んだまま終えています（サーボ ON のまま。原点復帰・IGN-OFF はしていません）。'
        : '非常停止が押されたため止まりました。減速して停車 → N → スタート 10 秒（IGN-OFF）→ 原点復帰の流れは済んでいます。',
      next: '非常停止を解除してから、もう一度始めてください。', reason, code,
    };
  }
  if (code === 0) {
    return { level: 'ok', title: `${kind}: 完了しました`, what: '', next: '', reason: null, code };
  }
  if (info) {
    return {
      level: info.level, title: `${kind}: ${info.label}（終了コード ${code}）`,
      what: info.what, next: info.next, reason, code,
    };
  }
  return {
    level: 'ng', title: `${kind}: 終了コード ${code ?? '不明'}`,
    what: '想定外の終了コードで終わりました。', next: '進行のメッセージ（ログ）を確認してください。', reason, code,
  };
}
window.describeJobResult = describeJobResult;

const LEVEL_TINT = {
  ok:   { fg: '#68d468', bg: '#0e220e', border: '#2a6c2a' },
  warn: { fg: '#f0c84a', bg: '#221c08', border: '#b89030' },
  ng:   { fg: '#f07070', bg: '#220e0e', border: '#943030' },
};

// 走行・学習がエラー・中断で終わって車が Ready のまま残ったときの注意と、IGN-OFF だけを押すジョブ（手順 25）のボタン（D-15b）。
// job.needs_ign_off（サーバーが、手順 21・22・24 を始めたのに手順 25 まで行かなかったジョブに付ける）のときだけ出す。
function IgnOffNotice({ job, compact = false }) {
  const { useContext, useState } = React;
  const { apiFetch } = useContext(window.AppContext);
  const { INK, INK_SOFT, Btn } = window;
  const [busy, setBusy] = useState(false);
  if (!job?.needs_ign_off) return null;
  async function ignOff() {
    setBusy(true);
    const r = await apiFetch('POST', '/api/v1/drive/ign-off');
    setBusy(false);
    if (r) window.showToast('IGN-OFF のジョブを始めました', 'success');
  }
  return (
    <div style={{ border: '1.3px solid #b89030', background: '#221c08', borderRadius: 4, padding: compact ? '4px 10px' : '6px 12px',
                  display: 'flex', gap: 10, alignItems: 'center' }}>
      <div style={{ flex: 1, fontSize: 12.5, lineHeight: 1.5 }}>
        <div style={{ color: '#f0c84a', fontWeight: 700 }}>車は Ready のまま</div>
        <div style={{ color: INK }}>車は Ready のままです。次の走行の前に IGN-OFF にしてください（下のボタン）。</div>
        <div style={{ color: INK_SOFT }}>車が止まっていることを確かめてから押します（動いていれば押しません）。</div>
      </div>
      <Btn primary disabled={busy} onClick={ignOff}>IGN-OFF にする</Btn>
    </div>
  );
}
window.IgnOffNotice = IgnOffNotice;

// 終わったジョブの結果の帯。onRetry があれば「やり直す」ボタンを出す。
function JobResultBanner({ job, onRetry, onDismiss, onLogs, compact = false }) {
  const { INK, INK_SOFT, INK_MUTE, Btn } = window;
  const r = describeJobResult(job);
  if (!r) return null;
  const t = LEVEL_TINT[r.level] ?? LEVEL_TINT.ng;
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
    <div style={{ border: `1.3px solid ${t.border}`, background: t.bg, borderRadius: 4, padding: compact ? '4px 10px' : '6px 12px',
                  display: 'flex', gap: 10, alignItems: 'flex-start' }}>
      <div style={{ flex: 1, minWidth: 0, fontSize: 12.5, lineHeight: 1.5 }}>
        <div style={{ color: t.fg, fontWeight: 700 }}>{r.title}</div>
        {r.what && <div style={{ color: INK }}>{r.what}</div>}
        {r.reason && r.level !== 'ok' && (
          <pre style={{ margin: '3px 0', padding: '3px 6px', maxHeight: 70, overflow: 'auto', whiteSpace: 'pre-wrap', wordBreak: 'break-all',
                        background: 'rgba(0,0,0,0.25)', color: INK_SOFT, fontSize: 11.5, fontFamily: 'ui-monospace, Menlo, Consolas, monospace' }}>{r.reason}</pre>
        )}
        {r.next && <div style={{ color: INK_SOFT }}>次にすること: {r.next}</div>}
      </div>
      <div style={{ display: 'flex', gap: 6, flexShrink: 0 }}>
        {onRetry && r.level !== 'ok' && <Btn primary onClick={onRetry}>やり直す</Btn>}
        {onLogs && <Btn onClick={onLogs}>ログへ</Btn>}
        {onDismiss && <Btn ghost onClick={onDismiss}>閉じる</Btn>}
      </div>
    </div>
    <IgnOffNotice job={job} compact={compact} />
    </div>
  );
}
window.JobResultBanner = JobResultBanner;

// ── 進行のメッセージ（GET /api/v1/drive/log）を 1 秒ごとに取る ──────────────────
function useDriveLogLines(maxKeep = 500) {
  const { useState, useEffect, useRef } = React;
  const [lines, setLines] = useState([]);
  const lastSeqRef = useRef(0);
  useEffect(() => {
    let alive = true;
    let timer = null;
    async function poll() {
      try {
        const q = lastSeqRef.current > 0 ? `?after=${lastSeqRef.current}` : '';
        const res = await fetch(`/api/v1/drive/log${q}`);
        if (res.ok && alive) {
          const j = await res.json();
          if (j.last_seq < lastSeqRef.current) {
            lastSeqRef.current = 0;   // サーバー再起動などで連番が巻き戻った: 取り直す
            setLines([]);
          } else {
            if (j.lines.length > 0) setLines(prev => prev.concat(j.lines).slice(-maxKeep));
            lastSeqRef.current = j.last_seq;
          }
        }
      } catch (e) { /* 通信失敗は次回に再試行（トーストは出さない） */ }
      if (alive) timer = setTimeout(poll, 1000);
    }
    poll();
    return () => { alive = false; if (timer) clearTimeout(timer); };
  }, []);
  return lines;
}
window.useDriveLogLines = useDriveLogLines;

// ── 最後のジョブ（実行中のジョブが終わったあとの結果） ─────────────────────────
// WebSocket の job は終わると null になる。終わった結果は GET /drive/status の last_job にある。
function useLastJob(liveJob) {
  const { useState, useEffect, useRef } = React;
  const [lastJob, setLastJob] = useState(null);
  const prevIdRef = useRef(null);
  async function load() {
    try {
      const res = await fetch('/api/v1/drive/status');
      if (res.ok) {
        const j = await res.json();
        setLastJob(j.last_job ?? null);
        return j;
      }
    } catch (e) { /* 次の機会に */ }
    return null;
  }
  useEffect(() => { load(); }, []);
  const liveId = liveJob?.job_id ?? null;
  useEffect(() => {
    // 実行中 → 終了（job が null になった）瞬間に、結果を取りに行く（終了処理の書き込みを待って 2 回）
    if (prevIdRef.current && !liveId) {
      load();
      const t1 = setTimeout(load, 800);
      const t2 = setTimeout(load, 2500);
      prevIdRef.current = liveId;
      return () => { clearTimeout(t1); clearTimeout(t2); };
    }
    prevIdRef.current = liveId;
  }, [liveId]);
  return lastJob;
}
window.useLastJob = useLastJob;

// ── 学習運転の段階表示 ───────────────────────────────────────────
// 手順2（full）の段階: 準備 → ペダル探索(2-0) → 低速の保持(4 点) → 停止手前の測定 → コーストダウン → G ランプ走行 → 遅れの測定(2-1) → モデル作成(2-2) → 学習結果チェック
// ジョブの phase・step と、走行ログの section（PEDAL_SEARCH など）・phase（LOW_HOLD・HOLD_STEP）・pattern（COAST_DOWN）から決める。
// 停止手前・遅れの測定・G ランプは pattern が同じ G_RAMP なので、パターン開始のメッセージ（"── パターン n/N: …"）でも見分ける。
// モデル作成・チェックは走行ログの行が無いので、進行のメッセージ（"2-2." の行・チェック表の見出し）で見分ける。
const SECTION_LABEL = {
  PRE_DRIVE_CHECK: '走行前チェック',
  PEDAL_SEARCH: 'ペダル探索',
  PATTERN_DRIVE: 'G ランプ走行（パターン走行）',
  FEL_DRIVE: '学習用パターン走行（補正表）',
  MODE_DRIVE: 'モード走行',
  DECEL_TO_STOP: '停車まで減速',
  EXCITE: '加振走行',
};
window.SECTION_LABEL = SECTION_LABEL;

const PHASE_LABEL = {
  starting: 'ランナーを起動中', init: '初期化中（通信確認・サーボON・原点復帰）', prepare: '準備中（モード・モデルの読み込み）',
  pre_check: '走行前チェック中', step: '実行中', step_end: '手順の終わり', shutdown: '終了処理中（原点復帰・サーボOFF・切断）',
  ready: 'Ready にしています（ブレーキを踏んでスタート短押し）', d: 'D レンジに入れています', n: '終わりの操作（ブレーキを踏んで N）',
  done: '完了', interactive: 'コマンド待ち',
  emergency: '非常停止の流れ（減速して停車 → N → スタート 10 秒（IGN-OFF）→ 原点復帰）',
};
window.PHASE_LABEL = PHASE_LABEL;

// 非常停止の流れの今の段（state phase=emergency の stage）の呼び名
const EMERGENCY_STAGE_LABEL = {
  start: '異常の確認中', decel: '0.2G で停車させています', n_press: 'N を押して停車を確かめています',
  release: 'ブレーキを離しています', ign_off: 'スタートを押しました（IGN-OFF）', home: '原点復帰中',
  done: '流れは完了', brake_kept: '止まらず、ブレーキを最大開度で踏んだまま', buttons_unavailable: 'ボタンを押せません（停車保持のまま）', can_lost: 'CAN を読めず、ブレーキを刻んで踏んでいます',
  brake_fault: 'ブレーキ軸の異常: 原点復帰してスタートを押しています', accel_fault: 'アクセル軸の異常: アクセルを原点へ戻しています',
  error: '流れが途中で失敗しました（終了処理へ進みます）',
};
window.EMERGENCY_STAGE_LABEL = EMERGENCY_STAGE_LABEL;

const STG_CHK  = { key: 'chk',   label: '走行前チェック', hint: '通信・車速 0・ブレーキ停止の確認', sections: ['PRE_DRIVE_CHECK'] };
const STG_RD   = { key: 'rd',    label: 'Ready・D',       hint: 'ブレーキを踏んでスタート短押し → D', sections: [], marker: /\[学習\] Ready のため/ };
const STG_N    = { key: 'n',     label: 'N',              hint: 'ブレーキを踏んで N（学習のあとの操作）', sections: [], marker: /\[学習\] 終わり/ };
const STG_OFF  = { key: 'off',   label: 'IGN-OFF',        hint: '停車を確かめてスタート短押し', sections: [] };
const LEARNING_HEAD = [STG_CHK, STG_RD];  // 初期化（手順 1）は初期化ページだけ・21・22 は外した（D-15c、2026-11-16）
const LEARNING_TAIL = [STG_N, STG_OFF];

// 流れ（D-15c）: 走行前チェック → Ready・D → 学習 → N → IGN-OFF(25)。初期化は初期化ページで済ませておく
const LEARNING_STAGES = {
  full: [
    ...LEARNING_HEAD,
    { key: 'pedal', label: 'ペダル探索',     hint: '2-0 クリープ車速・停車保持の開度', sections: ['PEDAL_SEARCH'], marker: /2-0\./ },
    { key: 'hold',     label: '低速の保持（4 点）', hint: '2-0 クリープより下の 4 点の車速を保つ', sections: [], phases: ['LOW_HOLD'], marker: /4 点の保持:/ },
    { key: 'approach', label: '停止手前の測定',   hint: '2-1 一定 G で停車まで下りる', sections: ['PATTERN_DRIVE'], marker: /── パターン \d+\/\d+: 停止の手前の測定/ },
    { key: 'coast',    label: 'コーストダウン',   hint: '2-1 加速して惰行で減速', sections: [], patternRe: /COAST_DOWN/, marker: /── パターン \d+\/\d+: COAST_DOWN/ },
    { key: 'ramp',     label: 'G ランプ走行',     hint: '2-1 狙いの G で加速・減速', sections: [], marker: /── パターン \d+\/\d+: G ランプ/ },
    { key: 'lag',      label: '遅れの測定',       hint: '2-1 定速から開度をステップ → 緩減速で停車', sections: ['DECEL_TO_STOP'], phases: ['HOLD_STEP'], marker: /── パターン \d+\/\d+: 遅れの測定/ },
    { key: 'model', label: 'モデル作成',     hint: '2-2 物理定数の推定・FF モデル（順モデル）', sections: [], marker: /2-2\./ },
    { key: 'check', label: '学習結果チェック', hint: '2-2 のチェック（NG なら yaml を書かない）', sections: [], marker: /id \| 名前 \| 判定|学習結果のチェック/ },
    ...LEARNING_TAIL,
  ],
  fel: [
    ...LEARNING_HEAD,
    { key: 'fel',   label: '学習用パターン走行', hint: 'FEL', sections: ['FEL_DRIVE'] },
    { key: 'save',  label: '補正表の保存',   hint: '今の pkl に補正表を付け直す', sections: [], after: true },
    ...LEARNING_TAIL,
  ],
  hold: [
    ...LEARNING_HEAD,
    { key: 'pedal', label: 'ペダル探索',     hint: 'ペダル探索', sections: ['PEDAL_SEARCH'] },
    { key: 'hold',  label: '保持の点を走る', hint: '4 点の保持', sections: ['PATTERN_DRIVE'] },
    { key: 'save',  label: '別名で保存',     hint: '今の pkl に保持の開度を付ける', sections: [], after: true },
    ...LEARNING_TAIL,
  ],
  stop: [
    ...LEARNING_HEAD,
    { key: 'stop',  label: '停止の手前の測定', hint: '一定 G で停車まで 4 本', sections: ['PATTERN_DRIVE', 'DECEL_TO_STOP'] },
    { key: 'save',  label: '別名で保存',     hint: '停止側の表を作り直す', sections: [], after: true },
    ...LEARNING_TAIL,
  ],
};
window.LEARNING_STAGES = LEARNING_STAGES;

const LEARNING_KIND_BY_STEPS = {
  '2,25': 'full', '12,25': 'fel', '13,25': 'hold', '14,25': 'stop',
  '1,2,25': 'full', '1,12,25': 'fel', '1,13,25': 'hold', '1,14,25': 'stop',  // 旧（初期化を各ジョブで行っていた）
  '1,21,22,2,25': 'full', '1,21,22,12,25': 'fel', '1,21,22,13,25': 'hold', '1,21,22,14,25': 'stop',  // 旧（21・22 を外す前）
  '1,2': 'full', '1,12': 'fel', '1,13': 'hold', '1,14': 'stop',  // 旧（通しの流れの前）
};

// job（実行中 or 最後の）・メッセージの行 → {stages, current(index), state: running|ok|ng|stopped}
function deriveStages(kind, job, lines) {
  const defs = LEARNING_STAGES[kind] ?? LEARNING_STAGES.full;
  if (!job) return { defs, current: -1, state: 'idle' };
  const finished = job.phase === 'done' || job.ended_at != null || job.exit_code != null;
  const sec = job.sample?.section ?? null;
  let idx = 0;
  const at = key => defs.findIndex(d => d.key === key);
  const bump = key => { const i = at(key); if (i >= 0) idx = Math.max(idx, i); };
  // 流れの段（手順番号・phase から）。走行前チェック → Ready・D → 学習 → N → 25
  if (job.phase === 'pre_check') bump('chk');
  if (job.phase === 'ready' || job.phase === 'd') bump('rd');
  const secIdx = sec ? defs.findIndex(d => d.sections.includes(sec)) : -1;
  const phs = job.sample?.phase ?? null;
  const pat = job.sample?.pattern ?? null;
  const phIdx = phs ? defs.findIndex(d => d.phases?.includes(phs)) : -1;
  const patIdx = pat ? defs.findIndex(d => d.patternRe?.test(pat)) : -1;
  if (job.step === 2 || (job.step >= 12 && job.step <= 14)) {
    // 段は進むだけで戻らない（section・phase・pattern のどれか進んだ方）
    idx = Math.max(idx, secIdx, phIdx, patIdx);
  }
  if (job.phase === 'n') bump('n');
  if (job.step === 25) bump('off');
  // 進行のメッセージ（このジョブの分だけ）
  const t0 = job.started_at ? Date.parse(job.started_at) - 5000 : 0;
  const mine = (lines || []).filter(l => {
    const t = Date.parse(l.time);
    return Number.isNaN(t) || t >= t0;
  });
  for (const l of mine) {
    defs.forEach((d, i) => { if (d.marker && d.marker.test(l.message)) idx = Math.max(idx, i); });
  }
  // 走行の手順が終わったあと（保存など）: 手順の走行ログが止まり、最後の段が「保存」なら進める
  const last = defs.findIndex(d => d.after);  // 「保存」の段（N の段はその後ろ）
  if (last >= 0 && mine.some(l => /保存しました|書き換えません|付けました|別名/.test(l.message)) && idx >= last - 1) idx = Math.max(idx, last);
  let state = 'running';
  if (finished) {
    const code = job.exit_code ?? job.result?.exit_code;
    state = code === 0 ? 'ok' : (code === 130 || code === 143 || job.stop_requested ? 'stopped' : 'ng');
  }
  // 手順2 でチェック NG（終了コード 8）→ 止まった段はチェック
  if (state === 'ng' && (job.exit_code ?? job.result?.exit_code) === 8) {
    const ci = defs.findIndex(d => d.key === 'check');
    if (ci >= 0) idx = ci;
  }
  return { defs, current: idx, state };
}
window.deriveStages = deriveStages;
window.learningKindOf = job => LEARNING_KIND_BY_STEPS[job?.steps] ?? 'full';

function StageRail({ defs, current, state }) {
  const { INK, INK_SOFT, INK_MUTE, HATCH } = window;
  return (
    <div style={{ display: 'flex', gap: 4, alignItems: 'stretch', flexWrap: 'wrap' }}>
      {defs.map((d, i) => {
        const done = state === 'ok' || (current >= 0 && i < current);
        const now = !done && i === current;
        let color = INK_MUTE, border = HATCH, bg = 'transparent', mark = `${i + 1}`;
        if (done) { color = '#68d468'; border = '#2a6c2a'; bg = '#0e1e0e'; mark = '✓'; }
        else if (now && state === 'running') { color = '#78c8f0'; border = '#3880b8'; bg = '#0e1c2a'; mark = '▶'; }
        else if (now && state === 'ng') { color = '#f07070'; border = '#943030'; bg = '#220e0e'; mark = '✗'; }
        else if (now && state === 'stopped') { color = '#f0c84a'; border = '#b89030'; bg = '#221c08'; mark = '■'; }
        return (
          <div key={d.key} title={d.hint} style={{ border: `1.2px solid ${border}`, background: bg, color, borderRadius: 4,
                                                   padding: '2px 8px', fontSize: 12, fontWeight: now ? 700 : 400, whiteSpace: 'nowrap' }}>
            <span style={{ marginRight: 5 }}>{mark}</span>{d.label}
          </div>
        );
      })}
    </div>
  );
}
window.StageRail = StageRail;
