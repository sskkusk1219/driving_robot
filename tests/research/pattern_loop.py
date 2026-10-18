"""手順 2-1 の閉ループパターン走行を実行する、研究用の自前状態機械。

ProblemReport_20260910 の遵守事項「`/src` の本番環境のコードを実行しないこと。実行環境は
すべて `tests/` に記載すること」に対応する。以前は `src.domain.control.learning_loop.LearningLoop`
をサブクラス化してそのまま実行していたが、それは「引用」ではなく本番コードの実行に当たるため、
アルゴリズムだけを移植した自前実装に置き換えた。

引用元（アルゴリズムを移植。クラス自体は import・実行しない）:
    src/domain/control/learning_loop.py … フェーズ遷移（MEASURE/DRIVE_ACCEL/COAST/DRIVE_BRAKE/
        BRAKE_HOLD/CRUISE_TRIM）、包絡線ガバナ（上限G厳守）、ランプ、オーバースピード回復、
        非常停止判定（CAN取得失敗・アクチュエータ通信失敗・過電流）のロジック。
    src/domain/safety_monitor.py … `check_overcurrent`（`current_ma > limit` の比較のみ）。

本番との違い（研究用にシンプルにしたところ）:
    - `CycleLoopBase`（本番の共通基盤）が持つ絶対時刻グリッドスケジューリング・
      ウォッチドッグ（サイクル未完了の連続監視）・DBログ書き込みバックログ管理は持たない。
      研究ハーネスは単発の短い走行のみを行うため、固定周期の単純な asyncio ループで足りる。
    - `SafetyMonitor` クラスは使わず、過電流しきい値との比較を直接行う（同じロジック）。
    - `RealtimeSnapshot`（WebSocket 配信用のキャッシュ）は持たない。研究ハーネスは配信しない。
    - サンプルごとのログフックは `on_sample` コールバックとして最初から持ち、
      本番の `_enqueue_log_write` をサブクラスでオーバーライドする形は取らない。

2026-09-13 A6（KAIZEN 表5-5 順3。report20260912_KAIZEN_process2,3.md）で本番から変えたところ:
    - ガバナーの解除: 本番は頭打ちをフェーズが変わるまで戻さず、実機では DRIVE_BRAKE が 8〜10%
      （不感帯未満）・DRIVE_ACCEL が 19〜22% に張り付いて「打ち切りに張り付く」原因になっていた。
      加減速が上限 × gov_release_frac を下回ったら gov_raise_step_pct ずつ戻し、
      指令に届いたら解除する。
    - 停車復帰を全運転パターンの後に置く: ACCEL_SWEEP だけでなく BRAKE_HOLD・COAST_DOWN・
      CRUISE_TRIM も、終わって停車していなければ DRIVE_BRAKE で停車させてから次へ進む
      （次の段を必ず停車から始める）。目標は ACCEL_SWEEP がリセットブレーキ、他は停車保持開度。
    - DRIVE_BRAKE の打ち切りは「過ぎたら次へ進む」ではなく
      「過ぎても停車しなければ中断する安全上限」。
    - ブレーキのランプはフェーズに入ったときの開度から始める
      （BRAKE_HOLD の保持開度から 0% に抜けない）。
    - 走行中の安全網（axis_safety.py: アラーム確認・電流ゼロの継続）を入れ、
      中断理由を abort_reason に残す。

2026-09-13 A2・A5（KAIZEN 表5-5 順4）で足したところ:
    - 目標車速つきパターン `SpeedTargetPattern`: 加速を cap ではなく accel_target_kmh で終える
      （BRAKE_HOLD を 60 km/h から保持する段に使う。src の LearningPattern は変えない）。

2026-09-13 A3・A4（KAIZEN 表5-5 順5）で足したところ:
    - トリム階段 `TrimStairPattern`（kind は CRUISE_TRIM のまま）: accel_target_kmh まで加速したら、
      trim_steps_pct をこの順に step_hold_s ずつ保持し、終わったら停車復帰する。
      cap に達したら次の（低い）段へ下げ、最後の段は cap に達しても保持を続ける
      （最高速を超えたら今と同じ回復に入る）。通常の CRUISE_TRIM は今と同じく cap で終える。
    - A4（20 km/h から高ブレーキで停車）は SpeedTargetPattern の BRAKE_HOLD をそのまま使う。

2026-09-13 A7（KAIZEN 表5-5 順6）で変えたところ:
    - 各軸の電流の読み取り（CNOW）を、0x9000 から 14 レジスタのまとめ読み（`read_monitor`）に
      替えた。実位置・ステータスも同じ 1 回で取れる。過電流・安全網の判定は電流だけを見る（中身は
      変えない）。
    - `on_sample` に両軸のモニターと、その周期の処理時間 cycle_ms（車速を読む直前から
      まとめ読みの後まで）を足して渡す。

2026-09-14 定速階段（段2）で足したところ:
    背景（docs/memo.md「応答遅れの実測 → 高速域の偏りの原因 → 定速階段＋走行抵抗式（C6）の計画」）:
    手順2に「定速保持」の状態が 50 km/h 以上で 1 行も無く、FF モデルの定速開度の予測が
    60〜80 km/h で浅く（遅れ）・130 km/h で深く（出すぎ）なる原因になっていた。手順3では
    指令→実開度の遅れがほぼ無い（|実−指令| p95 0.06〜0.07%）と実測済みなので、階段の開度は
    低ゲイン・レート制限の緩い PI で「実開度 ≈ 指令」を保ったまま定常開度を記録すればよい。
    - トリム階段と同じ流儀の `CruiseStairPattern`（kind は CRUISE_TRIM のまま）を追加:
      `hold_speeds_kmh`（昇順。例 30〜130 km/h を 10 km/h 刻み）へ、専用フェーズ
      `_Phase.CRUISE_HOLD` で 1 本ずつ弱い PI 保持する。DRIVE_ACCEL は最初の車速
      （`hold_speeds_kmh[0]`）で終える（`_accel_target_kmh`）。
    - CRUISE_HOLD の開度は速度偏差 [km/h] → 開度 [%] の PI（`_cruise_hold_opening`）。
      アンチワインドアップ（[不感帯, max_accel_opening] で頭打ちの方向へはこれ以上積分しない）・
      1 周期あたりの変化量を `max_rate_pct_per_s × interval_s` に制限（実開度が指令に追従できる
      速さに保つ）。初期開度は「アクセル不感帯 + initial_offset_pct」（DRIVE_ACCEL の加速用開度
      は定速に対して大きすぎ、そこから始めるとレート制限で最初の車速に間に合わないため）。
      積分・開度は車速をまたいでも引き継ぎ、段の変わり目で跳躍させない。
    - 前進判定（`_advance_cruise_hold`）: |車速−目標| ≤ settle_tol_kmh が settle_s 続いたら
      保持タイマーを開始し、hold_s 経過で次の車速へ（積分は引き継ぐ・settle/hold タイマーだけ
      リセット）。step_timeout_s を過ぎたら保持できていなくても次へ進む。最後の車速の後は
      `_finish_pattern`（停車復帰）。最高速超え・低速側（coast_down_stop_speed_kmh 以下）は
      トリム階段と同じ回復（前者は DRIVE_BRAKE の速度超過回復、後者は停車復帰）。
    - `_move_duration` は CRUISE_HOLD も DRIVE_ACCEL と同じ短い一手（pedal_step_time_s）にし、
      実開度が指令に追従できるようにする。G ガバナー（`_update_governor`）は CRUISE_HOLD に
      適用しない（緩い PI・レート制限そのものが穏やかなので、ガバナーによる頭打ちは不要と判断）。

2026-09-17 クリープ発進・クリープ域ブレーキ保持（段1。ProblemReport_20260916 課題#2）で足したところ:
    背景（docs/Problem/ProblemReport_20260916.md）: 本番のクリープ加速率 creep_rate_kmhs=0.19 が
    実測クリープ加速（中央値 2.39 km/h/s）と 12 倍ずれていた。原因は「ペダルオフ・低速・加速中」の
    全サンプルの中央値を取る既存推定（手順2がクリープ安定まで待つため母集団が定常側に偏る）。
    - `CreepLaunchPattern`（kind は CREEP_SETTLE を流用）を追加: 専用フェーズ `_Phase.CREEP_LAUNCH`
      で両ペダル 0%（完全解放）を指令し、`target_kmh` 到達または `timeout_s` の打ち切りまでクリープ
      のみで自走させる（`_advance_creep_launch`）。到達後は `hold_after` で分岐:
      False（クリープ発進）は通常の停車復帰（`_finish_pattern` → `_Phase.DRIVE_BRAKE`）、
      True（クリープ域ブレーキ保持）は `brake_opening`（不感帯 + offset）を保持する
      既存の `_Phase.BRAKE_HOLD` へ直接入る。
    - `_move_duration` は変更しない（CREEP_LAUNCH は COAST と同じく既定の pedal_release_time_s）。
      G ガバナー（`_update_governor`）も適用しない（両ペダル 0% で開度指令が無いため不要）。

2026-09-19 低開度階段（段3-1。ProblemReport_20260919 候補(c)）で足したところ:
    背景（docs/Problem/ProblemReport_20260919.md 6章）: 手順2 に「不感帯の直上を細かく刻んで
    一定保持する」パターンが 1 本も無く、5〜24 km/h × 不感帯 +0〜2% の学習行が 5.4 秒しか
    無かった（既存の ACCEL_DEADBAND_PROBE は無ランプ・昇順で停車に戻らずつながっており、
    開度と車速が一体で動くため定常判定をほとんど通らない）。
    - `LowOpenStairPattern`（`TrimStairPattern` を継承。kind は CRUISE_TRIM のまま）を追加:
      停車から `accel_target_kmh`（= 1 段目と同じ開度で終える低い車速）まで加速し、
      `trim_steps_pct` を `step_hold_s` ずつ一定保持して、その開度固有の平衡車速へ収束させる。
      `TrimStairPattern` の機構（`_command_openings` の CRUISE_TRIM 分岐・
      `_phase_after_accel`・`_finish_pattern` 等）をそのまま使うが、低速域が運転域の
      真ん中に来るため、低速終了の判定だけ `min_speed_kmh`（既定 2.0）に差し替える
      （`_advance_trim_stair`。トリム階段の `coast_down_stop_speed_kmh`=5.0 のままだと
      1 段目で即終了してしまう）。

2026-09-25 段4（ProblemReport_20260925）で削除したところ:
    手順2 のパターン列を格子ステップ走行に置き換えたため、上記の履歴のうち次の機構はコードから
    消した（履歴は経緯として残す）: 目標車速つき `SpeedTargetPattern`・トリム階段・定速階段・
    低開度階段（`TrimStairPattern`・`CruiseStairPattern`・`LowOpenStairPattern`）と、それらの
    前進判定・PI。ACCEL_SWEEP・BRAKE_HOLD（固定ブレーキ保持）・CRUISE_TRIM・不感帯プローブの各
    `PatternKind` も無い。残るのは COAST_DOWN・クリープ発進（CreepLaunchPattern。ブレーキ保持は
    `_Phase.BRAKE_HOLD`）・格子ステップ走行。`_Phase.CRUISE_HOLD` は格子ステーションの PI 保持が
    使う（CSV の phase 名を段3 と揃えるため名前は据え置き）。

2026-09-28 段7c（ProblemReport_20260925 段7。実機 20260928_035806 で見つかった不具合の修正）で
変えたところ:
    背景: 15 km/h ステーションで、強い減速の助走のため 25 km/h へ GRID_RETURN に入ったが、
    アクセル指令 8.11%（釣り合う車速 21.2 km/h）で 190 s 止まった。打ち切りが効かないバグと、
    感度の初期値（未測定のアクセルの感度を仮定の 2.0 のまま使う）が浅すぎたことが重なった。
    - GRID_RETURN の打ち切り（バグ修正）: `_advance_grid_hold` の打ち切り処理を
      `_grid_handle_settle_timeout` に切り出し、`_advance_grid_return` でも
      `now − _grid_settle_started ≥ settle_timeout_s` を判定する（今までは CRUISE_HOLD 側にしか
      無く、GRID_RETURN では永久に待ち続けていた）。
    - GRID_RETURN の開度: 踏み始めは従来の式（u0 + (狙い − a_hold) / g_accel）のまま、以降は毎周期
      `_step_grid_return` が直近 `coast_accel_slope_window_s` の実測加速度と
      `grid_return_accel_kmhs` の差に応じて開度を刻み足す（G 校正の加速 `_step_coast_accel` と
      同じ式。新しい調整値は増やさない）。
    - G 校正の加速（`_step_coast_accel`）: 開度が目標 G ± 押し込み余裕で止まっている瞬間しか
      点を記録していなかったため、40〜110 km/h 帯では開度が動き続けてアクセルの点が 1 つも
      取れず、上限開度表のアクセル列が全帯「—」になっていた。開度がゆっくり動いている
      （`gov_lead_s` 前の窓の振れが `coast_accel_record_max_change_pct` 以下）間も 1 s に 1 点
      記録するようにした（`_ca_slow_move_opening`）。**段7d で「振れが小さい間だけ」の条件は
      やめ、実開度＋遅れずらしの平均で常に記録する方式に置き換えた（下記）**。
    - 格子ステップの最初の感度: そのステーションでまだ測っていないペダルの最初の開度は、感度の
      仮定値（2.0）ではなく `glimit`（上限 G の予測マップ）にあるその車速帯の一番効いた実測点から
      引く（`GridPlanner` の `gain_fn`。`GLimitMap.strongest_point`）。HOLD・COAST（惰行）の実測
      直後に 1 度だけ種をまき、以降は従来どおり実測で更新する。発進・停車セル（`begin_launch`）は
      今のまま（直前のステーションの感度を引き継ぐ）。
    - コーストダウンの終了: `coast_down_stop_speed_kmh`（固定 5.0）ではなく、クリープ平衡車速
      （`feedforward.creep_speed_kmh`。2-0 実測）を基準にする。惰行は平衡へ漸近するだけで
      ちょうど届く保証がないため、平衡以下に達するか、クリープの影響が残る車速
      （平衡 + `grid_settle_tol_kmh`）以下で車速の傾きが `creep_launch_settle_kmhs` 未満の状態が
      `creep_launch_settle_s` 続いたら終える（`_coast_creep_settled`。停車ステップの
      `_grid_creep_settled` と同じ判定・同じ調整値、`_grid_samples` の代わりに毎周期の
      `_speed_hist` を使う）。`coast_down_stop_speed_kmh` の他の使い道（G 校正のブレーキ減速の
      終了・停車復帰の切替・格子ステップの底・発進セル）はブレーキを踏む区間でクリープに
      左右されないため変えない。

2026-09-28 段7d（ProblemReport_20260925 段7。実機 20260928_050944 で見つかった不具合の修正）で
変えたところ:
    背景: 段7c 後の G 校正の上限開度表で、アクセル上限が 0〜10 km/h で 227.3%（バグ）、
    70〜110 km/h が 60.5% で平ら（点が取れず借用）、10〜30 km/h が実測より高め（危険側）だった。
    - 偽の点（0〜10 km/h の 227.3%）: 前のパターン（コーストダウン加速）の指令開度 70% が
      `_ca_opening` に残ったまま、次の G 校正の踏み始め（接近ランプ中）に開度の履歴へ積まれ、
      踏み始め直後に (70%, 実測は低加速度) という偽の点になっていた。実開度で記録するように
      すれば、接近ランプ中の実際の位置（0% 付近）が積まれるので直る。`_enter_phase` でも
      `_ca_opening_hist` を空にする（保険）。
    - 記録は「開度が止まっている／振れが小さい」の条件をやめ、実開度（`monitor_accel` から。
      指令ではなく、低速の踏み込みで指令より遅れる実際の位置）を、遅れ `gov_lead_s`（0.6 s。
      ブレーキ指令から G の山までの遅れの実測値の流用）ぶん過去へずらした窓で平均し、今の G と
      対にして 1 s に 1 点記録する（`_ca_lagged_opening`）。開度が動き続けていても対応関係は
      崩れないため、70〜110 km/h も含め全帯で点が取れる。
    - 上限開度表（`GLimitMap.table`）: 予測開度が 100% を超える帯は数値の代わりに
      「100%(届かない)」と表示する（実際に機構の上限 G まで踏めないという意味。上限としての
      動きは変えない）。

2026-10 段2k 段E（ProblemReport_20261001）で削除したところ:
    手順2 のパターン列は G ランプ走行（`GRampPattern`）に置き換わったため、格子ステップ走行・
    通し掃引・G 校正の機構（`GridStationPattern`・`GridLaunchPattern`・`GridSweepPattern`、
    フェーズ CRUISE_HOLD・HOLD_STEP・GRID_RETURN・CALIB_BRAKE・SWEEP_UP・SWEEP、`PatternKind` の
    GRID_STEP・GRID_LAUNCH・G_CALIB・GRID_SWEEP、grid_planner.py ほか）をコードから消した。
    上の段7c・7d の履歴のうち、これらに触れる箇所は経緯として残してある。残るのは
    COAST_DOWN・クリープ発進・クリープ域ブレーキ保持・G ランプ走行・停車復帰（高い車速からは
    `_step_gprop_brake` の G 比例ブレーキ）。

2026-10-08 段2l-6（ProblemReport_20261001）で変えたところ:
    不感帯の測定をやめ、手で測った「ペダルに触れた位置」min（vehicle.min_*_opening_pct。
    プロファイルの feedforward_params.accel_deadband_pct・brake_deadband_pct に入る。名前は
    不感帯のまま、意味は u の原点）を待機位置かつ u の原点にした。
    - 待機位置: 「不感帯 − standby_margin_pct」をやめ min そのもの（`pedal_standby`）。ラッチ後の
      ブレーキは u ≤ 0 でブレーキ ＝ min ＋ |u|（下限 min・u の上限 0）。
    - 「踏んでいない」の判定に幅 PEDAL_OFF_TOL_PCT（待機位置がしきい値ちょうどになり、実位置の
      ずれで行が割れるため）。G ランプの実測点を貯める条件に使う。
    - G ランプの最初の開度の見積もりは外挿しない（`_glimit_open_for(extrapolate=False)`）。
    - RAMP_UP の発進（`_rp_launch`）: 停車保持開度から止まった開度（2-0 の stop_confirm_pct）まで
      戻し、そこから毎周期「実測 G ＜ 狙い（保持マージンの外）」のときだけブレーキを下げる。
      単調に下がり、踏み直さない。min に着いたらアクセルの u 制御（u ≥ 0）に渡す。
    - RAMP_DOWN は 5 km/h（coast_down_stop_speed_kmh）ではなく停車（STOP_SPEED_KMH）まで続ける。

2026-10-08 段2l-6b（ProblemReport_20261001。ユーザー決定）で変えたところ:
    コーストダウンの停車復帰（DRIVE_BRAKE）の境目を車ごとの値にした。クリープ車速は車によって
    違うので、固定の `coast_down_stop_speed_kmh`（5.0）ではなく「クリープ車速 + 落ち着きの許容幅」
    （`_creep_ceiling_kmh(grid_settle_tol_kmh)`）より上は G 比例ブレーキ、以下（クリープ域）は
    今のブレーキ開度から停車保持開度まで一定の速さで踏み上げる（`_db_creep_press`。速さは
    `_step_gprop_brake` の 1 刻み/待ちと同じ。単調に上げ、途中で下げない）。次のパターンへ進むのは
    停車かつ停車保持開度に届いてから、`brake_stop_timeout_s` も届いてから数える。G ランプなど
    他のパターンの停車復帰・速度超過の回復（overspeed_recovery）は従来どおり。

2026-10-09 段2l-8（ProblemReport_20261001。ユーザー決定）で変えたところ:
    背景: 手順2 実機（drive_log_real_20261009_042625）で、G ランプの減速中にブレーキが約 2.5 秒
    周期でばたついた。原因は「実測 G（1 秒の傾き。平均 0.5 秒古い）と狙いの差 × 固定ゲイン」で u を
    動かしていたこと。
    - G ランプの u を「基準車速への追従」で動かす。基準車速 v_ref は段ごとに 0 km/h から始まり
      0 km/h で終わる 1 本の続いた線（狙いの加速度 g_ref の積み上げ。上端で g_ref を +G → −G へ
      ramp_turn_time_s かけて変える）。偏差 e ＝ v_ref − 実車速。
    - 式は 1 つ（増分形）: Δu ＝ Δu_表 ＋ Kp × (e − e_前) ＋ (Kp ÷ Ti) × e × dt。
      （Δu_表 は 2026-10-10 段2l-10 で削除。下の段2l-10 を参照）
      Kp ＝ 1 ÷（感度 × T）、T ＝ ramp_settle_time_s、Ti ＝ RAMP_TI_RATIO × T。感度が無い
      ペダルは e の向きへアクチュエータの最小刻み 1 個 / 周期（いちばん弱い踏み方）。
      固定ゲイン ramp_rate_gain・保持幅は廃止。
    - 発進（停車保持ブレーキを離す場面）も同じ式。u を負（ブレーキ）から始める。RAMP_UP では
      u ＜ 0 の間は離す向きにだけ動き、u ≥ 0 になったら下限 0（ブレーキへ戻らない）。
    - 段の頭・減速への切替で見積もりを u に代入する処理をやめた（u は常に連続）。
    - u が上下限・ガバナーで止められた周期は、偏差が広がらないよう v_ref を実車速側へ引き戻す。
    - 表の水準への寄せ（ユーザー決定 2026-10-09。段2l-10 で 3 か所に一般化）:
      Δu_表 は表の「変化」だけなので、段の頭では表の
      水準が u に入らず、高い G の段で立ち上がりが遅れた。RAMP_UP でアクセルに移った周期に 1 回
      だけ、表の開度と今の u の差 gap を覚え、段の G で決まる時間（2026-10-10 段2l-11。
      `_ramp_launch_blend_time_s` ＝ (G − 発進の G) ÷ ramp_launch_g_rate_gps、最短
      ramp_launch_blend_min_s。弱い段 0.5 秒・0.35G で 2 秒。寄せ先は段の G ＝
      pattern.target_g）かけて等速で足す
      （一気に代入しない）。表に点が無ければ寄せない。RAMP_DOWN に入ったら寄せ残りは捨てる。
    - 実測 G の計算・上限 G マップへの点の追加・結果の表は残す（制御には使わない。
      2026-10-12 段2l-12 の「戻す条件」だけは実測の加速度を使う）。
    - 動き出しの置き直し（ユーザー決定 2026-10-09）: 発進でブレーキを離しきるまで基準だけが先へ進み
      大きな偏差が残るので、RAMP_UP で実車速が初めて STOP_SPEED_KMH を超えた周期に 1 回だけ基準を
      実車速へ置き直す（その周期は開度を動かさない。折り返しでは置き直さない）。
    - 感度が無い周期は、基準が実車速より cap ＝ 狙いの G × T（T ＝ ramp_settle_time_s）を超えて
      先に行かないよう引き戻す（感度が取れて Ki に切り替わったときの積み上がりを穏やかにする）。

2026-10-09 段2l-9（ProblemReport_20261001。ユーザー決定）で変えたところ:
    背景: シャシダイナモ上で、タイヤが回り始める瞬間と止まる瞬間にいきなり狙いの G を要求すると
    スリップや揺れが大きく危険。狙いの加速度 g_ref の作り方だけを変える（ペダルを動かす式・
    置き直し・頭打ちの引き戻し・感度なしの cap・結果の表は変えない）。段2l-8 の「踏み始めに
    丸みを入れない」はこれで置き換える。
    - 発進: 動き出し（置き直した）周期から ramp_launch_time_s かけて g_min → +G（直線。
      g_min ＝ min(ramp_edge_g, G)。0.05G の下げ止め）。置き直した周期は g_min。
      動き出す前は +G のまま（基準を進めて偏差でブレーキを離すため）。
    - 切り替え: 折り返した時点の g_ref（発進の途中で折り返しても飛ばない）から、
      ramp_turn_time_s かけて -G へ。
    - 停止: -max(g_min, G × min(1, √(v_ref ÷ v_stop)))。
      v_stop ＝ G × G_TO_KMHS × ramp_stop_time_s ÷ 2
      （-G → 0 を停止の時間で直線に変え、ちょうど 0 km/h に着く線を車速で書き直したもの。
      時刻でなく基準車速から決める）。g_ref ＝ max(切り替え, 停止) で弱いほうを使う。
    - ramp_edge_g ＝ 0 なら 0 から・0 まで（下げ止め前の挙動）。
    - 3 つの時間が 0 なら丸めない（発進は最初から +G、切り替えは即 -G、停止は -G のまま）。
    - `_ramp_control` は「置き直しの判定 → g_ref → 基準を進める → 表を引く」の順。置き直しの周期は
      g_ref ＝ g_min で表を引き、次の周期の比較元にする（次の周期に開度が飛ばない）。
      （表を引く部分は段2l-10 で削除。置き直しの周期の偏差の項は 0 のまま）

2026-10-10 段2l-10（ProblemReport_20261001。ユーザー決定）で変えたところ:
    背景: 実機 041505（手順2）で、(1) 表の項（毎周期、車速 10 km/h 幅の帯ごとの別々の表から引いた
    開度の前周期との差）が帯の境目で +1〜+3.7% 跳ね、折り返し〜ブレーキ最大の間にブレーキ側へ
    −21〜−37% 入って 0.5〜0.8G の減速が出た（要る量は −3〜−5%。折り返し時点に 1 回だけ引いた値は
    9.0〜15.3% で、跳ねは「毎周期の差」が作っていた）。(2) 切り替え 1 秒は車の反応（約 1.4 秒）より
    短い。
    (3) 指令 60% のとき実開度 37% など、アクチュエータが指令に遅れた（送り方と周期は別に直した）。
    - 表の項（`_rp_table_prev`・Δu_表・毎周期の表引き）を削除。Δu ＝ Kp × (e′ − e′_前) ＋
      (Kp ÷ Ti) × e′ × dt ＋ 寄せ。表は「寄せ」にだけ使い、1 回だけ引いて決めた時間で等速に足す
      （`_ramp_blend_start(target_u, u, time_s)`。差は符号つき、残りを超えない）。3 か所:
      発進でアクセルに移った周期（時間 0.5 秒・増やす向きだけ。今のまま）、保持の頭（その車速で
      0G のアクセル開度へ。ramp_turn_time_s）、減速の頭（惰行より緩ければアクセル側の値、
      ブレーキ側は
      自分の車速帯で内挿できるときだけ `GLimitMap.interp_pct`、無ければ u ＝ 0 へ離すだけ。
      ramp_turn_time_s。u ≤ 0 でその規則 3 なら寄せない）。
    - 偏差の不感帯 ramp_dev_deadband_kmh（既定 0.5 km/h）: e′ ＝ |e| ≤ 不感帯なら 0、外側は
      e − 不感帯 × sign(e)（境目で連続）。車速の細かい揺れで開度を動かさない。感度が無いペダルも
      e′ の向きへ最小刻み 1 個（e′ ＝ 0 なら動かさない）。使わない所: RAMP_DOWN で基準が
      STOP_SPEED_KMH 未満（使うと実車速 0.5 km/h 以下で踏み増さず止まらない）、発進の置き直しの前。
      e′_前 は前周期の
      生の e を今周期の不感帯で引き直す（有無が切り替わる周期に開度が飛ばない）。
    - 区間 RAMP_HOLD を追加（RAMP_UP → RAMP_HOLD → RAMP_DOWN）。基準車速が
      「上端 − その時点の g_ref ×
      G_TO_KMHS × ramp_turn_time_s ÷ 2」に着いたら入る（+G → 0 の間に進むぶんを引き、基準が上端で
      ちょうど止まる）。g_ref は入った時点の値 → 0 を ramp_turn_time_s で直線、その後 0。
      ramp_turn_time_s ＋ ramp_hold_time_s 経ったら RAMP_DOWN（0 → −G を ramp_turn_time_s で直線）。
      既存の安全網（実車速が上端に着いた・時間切れ）は RAMP_HOLD へ進む。RAMP_HOLD 中に実車速が
      上端の頭打ちを skip_consecutive_required 周期続けて超えたら RAMP_DOWN へ。u の範囲・点を貯める
      規則は RAMP_UP と同じ。結果の表の G 中央値は RAMP_UP・RAMP_DOWN だけ、保持秒（hold_s）は
      別の列。
    - ア: 今使っているペダルの |指令 − 実開度| が ramp_actuator_lag_tol_pct を超え、さらに離れる
      向きに指令を動かす周期は、基準車速を実車速側へ戻す（開度 u は止めない）。
      実開度が読めていないときは
      働かない。
    - 設定: ramp_turn_time_s 1.0 → 2.0、ramp_hold_time_s 7.0、ramp_dev_deadband_kmh 0.5、
      ramp_actuator_lag_tol_pct 2.0（いずれもユーザー決定・仮の値。次の手順2 実機の後に決め直す）。

2026-10-10 段2l-11 B（ProblemReport_20261001。ユーザー決定）で変えたところ:
    背景: 実機 063905 で、上端 30 km/h の 0.35G は基準が狙いの G にいる時間が計算上 0.9 秒しか
    なく、基準車速で RAMP_HOLD へ入る判定では車が基準より 5〜7 km/h 遅れて上端に着く前に
    アクセルを戻していた。減速の頭では「アクセル 32% → ブレーキ 11.6%」の合計を等速に動かし、
    ブレーキのぶんが最後の約 0.4 秒に集中した。
    - 上端（ramp_planner の `min_top_kmh`）: モードから決めた上端と「最小の上端」の大きいほう。
    - RAMP_UP → RAMP_HOLD は実車速で判定する: 実車速 ≥ 上端 − max(0, 実測の加速度) ×
      ramp_turn_time_s ÷ 2（実測の加速度は `_rp_hist` の傾き）。基準車速での判定は外した。
      安全網（実車速が上端の頭打ちに着いた・時間切れ）はそのまま。
    - 切り替えた周期に基準車速 v_ref ＝ 実車速・e_前 ＝ 0（発進の置き直しと同じ扱い。Kp・Ki の
      式は偏差の「変化分」なので開度は飛ばない）。RAMP_HOLD の g_ref の始まり `_rp_hold_g0` は
      実測の加速度 ÷ G_TO_KMHS を 0〜G に丸めた値（測れていなければ従来どおり入った時点の
      g_ref）。RAMP_HOLD → RAMP_DOWN の周期にも v_ref ＝ 実車速・e_前 ＝ 0。
    - 減速の頭の寄せを 2 つに分ける（`_ramp_descent_accel_target`・`_ramp_descent_brake_target`）。
      1 つ目（RAMP_DOWN に入った周期）: 狙いが惰行より緩くアクセル側の値が引けるときはその値へ
      ramp_turn_time_s で 1 回（2 つ目は無し）。それ以外で u ＞ 0 なら u ＝ 0 へ。u ≤ 0 なら
      1 つ目は無し。2 つ目（ブレーキ側。段ごとに 1 回）: 1 つ目の寄せが終わった周期か u が 0 以下に
      なった周期の早いほうで、ブレーキの見積もり（自分の車速帯で内挿できるときだけ）へ、その周期の
      u からの差を ramp_turn_time_s で。寄せ先が今の u より浅い（ブレーキを緩める向き）ときと、
      引けないときは寄せない。u ≤ 0 で RAMP_DOWN に入った段は 2 つ目も無し。

2026-10-12 段2l-12（ProblemReport_20261001。ユーザー決定）で変えたところ:
    背景: 実機 114602 で、0.045G の減速のブレーキが ±0.2% で 1.38 回/秒往復し、減速が
    0.023〜0.068G で揺れた。偏差の符号が車速の細かい揺れで入れ替わるたびに踏む・戻すを繰り返す
    ため（原因は推測）。目的は表を貯めることで、基準どおりに走ることではない。
    - 片側調整（learning.ramp_one_sided。False で段2l-11 までの両方向）: RAMP_UP（動き出しの
      置き直しの後）は車が基準より遅いときだけ u を増やし、RAMP_DOWN（停止の直前より前）は
      車が基準より速いときだけ u を減らす（`_ramp_one_sided_dir`）。行き過ぎ（RAMP_UP で実車速 ＞
      基準、RAMP_DOWN で実車速 ＜ 基準）の周期は開度を動かさず基準を実車速に置き直す
      （e ＝ e_前 ＝ 0）。RAMP_HOLD・置き直しの前後の周期そのもの・停止の直前
      （基準 ≤ v_stop か STOP_SPEED_KMH 未満）は両方向のまま。
    - 戻す条件: 実測の加速度（`_rp_hist` の傾き。`_step_ramp` が `_ramp_control` の前に計算して
      渡す。測れていなければ戻さない）の順方向の G が段の上端（`_ramp_upper_g` ＝ ramp_g_levels の
      隣の強い段との中間。いちばん強い段は無し）を超えた周期は、偏差の項の代わりに逆方向へ
      Kp ×（実測の加速度 − 狙いの加速度）× dt（感度なしは最小刻み 1 個）動かす。
      上端を下回れば止まる。寄せ・ガバナー・上下限・`blocked`・cap は変えない。

2026-10-13 段2o-2（ProblemReport_20261001。ユーザー決定）で足したところ:
    背景: 実機の段2n で、G ランプに「ほぼ定速（±0.01G 以内）」を受け持つ段が無く、WLTP が長くいる
    定速付近の表の点が貯まらなかった。いちばん弱い段（ramp_g_levels の最小）の上り・下りの途中で、
    決めた車速ごとに定速を保持する。
    - `GRampPattern.stations_kmh`（空なら従来どおり。
      `ramp_planner.plan_cruise_stations` が決める）。
      新フェーズ `_Phase.RAMP_CRUISE`（CSV の phase 列に出る）。
    - 入り方: 上り（RAMP_UP）は 実車速 ≥ 保持点 − max(0, 実測の加速度) × ramp_turn_time_s ÷ 2、
      下り（RAMP_DOWN）は 実車速 ≤ 保持点 ＋ max(0, −実測の加速度) × ramp_turn_time_s ÷ 2 で、
      その方向でまだ保持していない点。複数当てはまるときは車速に近い 1 点だけ保持し、残りは
      とばす。上端の判定（RAMP_HOLD へ）が先。入った周期は v_ref ＝ 実車速・e_前 ＝ 0・寄せ残りを
      捨てる（u ≥ 0 のときだけ、その車速の 0G のアクセル開度へ ramp_turn_time_s で寄せる）。
      g_ref は 入った時点の実測（±G に丸める）→ 0 を ramp_turn_time_s で直線、その後 0。
    - 保持中は偏差の項を両方向で使う（`_ramp_one_sided_dir` は None）。u の下限は、ブレーキ側
      （u ＜ 0）で入ったときは今の u（離す向きだけ。`_rp_accel_reached` で下限 0 にすると 1 周期で
      ブレーキが抜ける）、u ≥ 0 なら 0。上限はアクセルの上限（ラッチは外す）。
    - 出方: 実測の加速度（`_rp_hist` の傾き）が ±ramp_cruise_band_g 以内だった周期の時間の合計が
      ramp_cruise_hold_s に達したら出る。入ってから ramp_turn_time_s ＋ ramp_cruise_timeout_s
      で打ち切り（その点は保持済み）。切り替え直後の加速度が測れない周期（None）は数えない。
      上りへ戻るときは `_enter_phase` を通さない（`_ramp_up_begin` が状態を作り直すため）。
      フェーズと g_ref の起点（0 → +G を ramp_turn_time_s。`_rp_resume_started`）だけ変え、
      `_rp_hist` も残す。下りへ戻るときは `_ramp_switch_down`（`_rp_down_started` は最初の 1 回）。
    - 上り・下りの時間切れは保持の時間を除いた通算で数える（`_ramp_leg_elapsed`）。
      `RampResult` の up_s・hold_s・down_s は区間を切り替えるたびに積算した値、`cruise_s`
      （途中の保持の合計）・`cruise_n`（入った回数）・`cruise_done`（合計が hold_s に達した回数）
      を足した。

2026-10-14 段2o-4: 途中の保持を上りだけにできる（`GRampPattern.stations_up_only`。
    `ramp_planner` の `execution_order()` の 4 つ目）。True の段は RAMP_DOWN で保持に入らない
    （上りの入り方・出方は段2o-2 のまま）。

2026-10-13 段2o-2（ペダルの遅れの測定）:
    背景: 順モデルの表の遅れ L が既定値 0.05 秒のまま（実測は約 0.2 秒）。G ランプは開度が
    ゆっくりしか動かず遅れが測れない。G ランプの後ろに、上端 30・60 km/h の段
    （`GRampPattern.lag_step`）を足し、定速の保持の後に開度を固定でステップさせる。
    - 上端の保持（RAMP_HOLD）の出方を途中の保持と同じにする（±ramp_cruise_band_g が合計
      ramp_cruise_hold_s、または ramp_turn_time_s ＋ ramp_cruise_timeout_s で打ち切り）。
    - 出たら `_Phase.HOLD_STEP`（CSV の phase 名は `pedal_lag.py` が読む名前と同じ）。開度 u を
      固定で持ち（偏差の項なし・基準は進めない・過渡の点は上限 G マップに貯めない）、1 つの開度
      につき ramp_lag_step_hold_s 秒、順に 5 つ: u_hold ＋ Δa（アクセル踏み増し）→ u_hold →
      0（両ペダル待機位置）→ −Δb（ブレーキ踏み増し）→ 0。Δa ＝ 表の +ramp_lag_step_g の開度 −
      u_hold（引けなければ ramp_lag_step_g ÷ 感度。どちらも無ければこの段は飛ばす）、Δb ＝ 表
      の −ramp_lag_step_g のブレーキ開度 − min（引けなければ感度から。無ければブレーキの 2 つを
      飛ばす）。開度の上下限・ガバナー・最高速超えの安全網はそのまま。終わったら
      `_ramp_switch_down`。`_rp_hist` はフェーズの切り替えで空にしない（`_enter_phase` を
      通さない）。

2026-10-14 段2o-5: 下り（RAMP_DOWN）の途中に「戻し」を入れる（`GRampPattern.release_stations`。
    `ramp_planner` の `execution_order()` の 5 つ目。いちばん弱い段以外）。背景: モードの減速の
    3 割強は走行中のブレーキの戻しだが、手順2 の減速は停車まで踏み続けるので戻し中の効きの行が
    1 つも無く、ブレーキ側の「実測 − モデル」が戻し中に +0.56〜+0.83 km/h/s 弱かった。
    - 新フェーズ `_Phase.RAMP_RELEASE`（CSV の phase 列に出る）。入り方: RAMP_DOWN でブレーキの
      ラッチ後（`_rp_brake_latched`）、実車速 ≤ まだ使っていない戻す車速で、u ＜ 0（ブレーキを
      踏んでいる）。u ≥ 0 ならその点は飛ばす（`release_skipped`）。入った時点の u が u_peak。
    - 流れ: learning.ramp_release_fracs の各割合 f について、u を直前の値 → f × u_peak へ
      ramp_release_move_s で直線に動かし、ramp_release_hold_s の対応する秒だけ保持する。最後の
      0 は待機位置（アクセル 0・ブレーキ min）。制御の項（Kp・Ki・ガバナーの踏み増し）は使わない。
    - G を守る手当て: HOLD_STEP と同じく `_step_ramp` で早く返す（`_ramp_control` を呼ばない・
      上限 G のマップ `glimit` に貯めない・`_rp_down_g` にも入れない）。出るとき（流れの最後、または
      実車速 ≤ ramp_release_min_kmh ＝ クリープ ＋ 5 の安全網）は `_ramp_release_exit` で
      RAMP_DOWN へ戻し、g_ref を 0 → −G へ ramp_turn_time_s で直線、v_ref ＝ 実車速、
      e_前 ＝ 0（取り返しに行かない）、u はその時点の値（流れの最後は待機位置 0）から
      片側調整で踏み直す。
    - 戻しに使った時間は下りの時間切れから除く（`_rp_down_prior_s` に戻しの前までを足す。
      `_enter_phase` で経過が 0 に戻るので、戻しの間は数えない）。結果に戻しの秒・入った回数・
      飛ばした回数を持つ（`RampResult.release_s`・`release_n`・`release_skipped`）。
"""

from __future__ import annotations

import asyncio
import math
import statistics
import time
from collections import deque
from collections.abc import Awaitable, Callable, Sequence
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Protocol

import numpy as np

from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet
from tests.research.g_limit import PEDAL_ACCEL, PEDAL_BRAKE, GLimitMap
from tests.research.play_onset import coast_fn_from_curve, onset_from_series
from tests.research.research_types import (
    ACTUATOR_PULSE_MAX,
    G_TO_KMHS,
    PEDAL_OFF_TOL_PCT,
    VEHICLE_STOP_SPEED_KMH,
    DriveLogData,
    LearningPattern,
    PatternKind,
    VehicleProfile,
    clamp_opening,
    enforce_pedal_exclusion,
    opening_to_position,
    position_to_opening,
)
from tests.research.term import say


def fit_slope(times_s: Sequence[float], speeds_kmh: Sequence[float]) -> float:
    """車速 [km/h] の時間 [s] に対する最小二乗の傾き [km/h/s]。2 点未満・時間幅 0 なら 0。"""
    if len(times_s) < 2 or times_s[-1] - times_s[0] <= 0.0:
        return 0.0
    slope, _ = np.polyfit(np.asarray(times_s, dtype=float), np.asarray(speeds_kmh, dtype=float), 1)
    return float(slope)


# 既定の周期 [s]。実走では呼び出し側が yaml の control.loop_interval_ms（50 ms）を
# interval_s で渡す（2026-10-10 段2l-10。手順2 も手順3・本番と同じ周期にそろえた）。
# これは渡されないときだけの既定値
PATTERN_LOOP_INTERVAL_S: float = 0.1
# 「周期の数」で書いた設定（skip_consecutive_required）が、何秒の周期を前提にした数か。
# 周期が変わっても秒としての長さを保つため、実際の周期との比で数を換算する
PATTERN_LOOP_REFERENCE_INTERVAL_S: float = 0.1
# 位置を動かす指令の後、読み取りの前に待つ秒数。手順3（mode_drive.AXIS_PRE_READ_DELAY_S）と同じ値。
# mode_drive からの import は循環の恐れがあるので、ここに同名の定数を持つ
AXIS_PRE_READ_DELAY_S: float = 0.002
# G ランプの送り方: 周期のこの割合で動ききる加減速にする（手順3 の AXIS_SMOOTH_DUTY と同じ）
AXIS_SMOOTH_DUTY: float = 0.8
STOP_SPEED_KMH: float = VEHICLE_STOP_SPEED_KMH
# G ランプ 1 区間（加速または減速）の打ち切り = 見込み秒数（上端 ÷ 狙いの加速度）× 係数 + 余裕。
# 仮値（データの根拠なし）。狙いの G に届かず上端に着けない・クリープ域まで下りられない場合の
# 安全網で、通常の走行には効かない。2l の実機で実際の所要秒数を見て決め直す
RAMP_LEG_TIMEOUT_FACTOR: float = 3.0
RAMP_LEG_TIMEOUT_MARGIN_S: float = 10.0
# 2026-10-10 段2l-11: 発進の寄せの時間は定数をやめ、段の G で決める（`_ramp_launch_blend_time_s`）
# （旧 RAMP_TABLE_BLEND_TIME_S 0.5 秒。短いと G が一気に上がり先読みガバナーが働いた）
# 積分時間 Ti ＝ これ × T（T ＝ ramp_settle_time_s）。偏差が行き過ぎずに収まる条件（減衰係数 1）から
# 出る比で、変えない想定
RAMP_TI_RATIO: float = 4.0
# キャリブレーションが無いときのアクチュエータ最小刻み 1 個ぶんの開度 [%]。ストローク
# ACTUATOR_PULSE_MAX パルスが 100% なので 1 パルス ＝ 100 ÷ 9500 ≒ 0.0105%
RAMP_MIN_STEP_FALLBACK_PCT: float = 100.0 / ACTUATOR_PULSE_MAX


@dataclass(frozen=True)
class CreepLaunchPattern(LearningPattern):
    """クリープ発進・クリープ域ブレーキ保持（段1。ProblemReport_20260916 課題#2。研究側の追加）。

    両ペダル 0%（完全解放）でクリープ自走し、専用フェーズ `_Phase.CREEP_LAUNCH` で `target_kmh`
    到達または `timeout_s` の打ち切りを待つ。到達後の扱いは `hold_after`:
      - False（クリープ発進）: 通常の停車復帰（`_Phase.DRIVE_BRAKE`、停車保持開度）。
      - True（クリープ域ブレーキ保持）: `brake_opening`（不感帯 + offset。呼び出し側で
        max_brake_opening にクランプ済み）を保持して停車させる（既存の `_Phase.BRAKE_HOLD`）。

    kind は CREEP_SETTLE を流用する（両ペダル解放でクリープを測る、という意味は共通）。CSV の
    pattern 列表示にのみ影響し、`_advance` の CREEP_SETTLE 専用分岐は `_Phase.MEASURE` でしか
    見ない（本パターンの初期フェーズは CREEP_LAUNCH）ため衝突しない。
    """

    target_kmh: float = 0.0
    timeout_s: float = 20.0
    hold_after: bool = False


@dataclass(frozen=True)
class GRampPattern(LearningPattern):
    """G ランプ走行 1 本（段2k 段B。kind は G_RAMP）。

    停車から `+target_g` を狙って `top_kmh` まで加速（RAMP_UP）→ `-target_g` を狙って減速
    （RAMP_DOWN）→ クリープ域まで下りたら通常の停車復帰。開度は固定値を持たず、走行中に
    実測の G を見て「効く開度 u」を動かして作る（`accel_opening` 等は使わない）。
    """

    target_g: float = 0.0  # 狙いの G（正。加速は +、減速は − に使う）
    top_kmh: float = 0.0  # 上端の車速 [km/h]
    # 2026-10-13 段2o-2: 上り・下りの途中で定速を保持する車速 [km/h]（昇順。空なら従来どおり）
    stations_kmh: tuple[float, ...] = ()
    # 2026-10-14 段2o-4: True なら途中の保持は上り（RAMP_UP）だけ。下りでは入らない
    stations_up_only: bool = False
    # 2026-10-13 段2o-2: True なら上端で保持した後に開度のステップを入れる（ペダルの遅れの測定）
    lag_step: bool = False
    # 2026-10-14 段2o-5: 下りで山からブレーキを離す「戻し」の車速 [km/h]（昇順。空なら従来どおり）
    release_stations: tuple[float, ...] = ()


@dataclass(frozen=True)
class RampResult:
    """G ランプ 1 本の結果（段ごとの表示用。`PatternLoop.ramp_results`）。"""

    pattern_idx: int
    target_g: float
    top_kmh: float
    up_s: float  # RAMP_UP の所要秒数
    down_s: float  # RAMP_DOWN の所要秒数
    accel_g_median: float | None  # 加速側の実測 G の中央値（取れなければ None）
    decel_g_median: float | None  # 減速側の実測 G の中央値（正。取れなければ None）
    max_speed_kmh: float  # 到達した最高車速
    # RAMP_HOLD（最高速での保持。切り替え +G → 0 を含む）の所要秒数。2026-10-10 段2l-10。
    # 合計秒 ＝ up_s ＋ hold_s ＋ down_s（既定 0 は保持なしの結果）
    hold_s: float = 0.0
    # 2026-10-13 段2o-2: 途中の定速保持（RAMP_CRUISE）の合計秒数・入った回数・合計が
    # ramp_cruise_hold_s に達した回数。up_s・hold_s・down_s は保持の時間を含まない積算値。
    cruise_s: float = 0.0
    cruise_n: int = 0
    cruise_done: int = 0
    # 2026-10-14 段2o-5: 下りの戻し（RAMP_RELEASE）の合計秒数・入った回数・飛ばした回数（u ≥ 0 で
    # 着いた点、安全網の車速より下で着いた点、一度に飛び越えた点）。down_s は戻しの時間を含まない
    release_s: float = 0.0
    release_n: int = 0
    release_skipped: int = 0
    # 2026-10-16 段2o-6: この段の終わりに求めた効き始め（絶対開度 [%]。求まっていなければ None。
    # 前の段の値を引き継いだ場合はその値）と、アクセルのステップの起点が効き始めより浅かった累計回数
    accel_onset_pct: float | None = None
    brake_onset_pct: float | None = None
    step_accel_below_onset_n: int = 0


@dataclass(frozen=True)
class RampStatus:
    """G ランプ中の表示用の状態（`PatternLoop.ramp_status`）。Kp・Ki は直近に計算した値
    （感度がまだ無いペダルは 0）。"""

    ref_kmh: float
    accel_kp: float
    accel_ki: float
    brake_kp: float
    brake_ki: float


class _Phase(Enum):
    MEASURE = auto()
    DRIVE_ACCEL = auto()
    COAST = auto()
    DRIVE_BRAKE = auto()
    BRAKE_HOLD = auto()
    CREEP_LAUNCH = auto()  # クリープ発進・クリープ域ブレーキ保持（段1）。両ペダル 0% で自走
    RAMP_UP = auto()  # G ランプ走行（段2k 段B）: 狙いの G で上端の車速まで加速
    # G ランプ走行（段2l-10）: 上端の手前で +G → 0 に切り替え、上端で保持する（RAMP_UP の次）
    RAMP_HOLD = auto()
    RAMP_DOWN = auto()  # G ランプ走行（段2k 段B）: 同じ G で減速してクリープ域まで下りる
    # G ランプ走行（段2o-2）: 上り・下りの途中で決めた車速の定速を保持する（RAMP_UP/DOWN の途中）
    RAMP_CRUISE = auto()
    # G ランプ走行（段2o-2）: 遅れの測定。上端の保持の後、開度を固定で 5 つステップさせる
    # （CSV の phase 名 `HOLD_STEP` は pedal_lag.py が読む名前と同じ）
    HOLD_STEP = auto()
    # G ランプ走行（段2o-5）: 下りの途中でブレーキを山から離す戻し（RAMP_DOWN の途中）
    RAMP_RELEASE = auto()
    DONE = auto()


# 発進のゆっくり離し（段2o-10 A）の間の CSV の phase 名。内部のフェーズは RAMP_UP のまま
# （`_rp_lr_active` が立っている間だけ名前を差し替える）
RAMP_LAUNCH_RELEASE_PHASE = "RAMP_LAUNCH_RELEASE"

# G ランプの区間（順に RAMP_UP → RAMP_HOLD → RAMP_DOWN。2026-10-10 段2l-10 で RAMP_HOLD を追加）
_RAMP_PHASES = (
    _Phase.RAMP_UP, _Phase.RAMP_CRUISE, _Phase.RAMP_HOLD, _Phase.HOLD_STEP, _Phase.RAMP_DOWN,
    _Phase.RAMP_RELEASE,
)


@dataclass
class PatternLoopConfig:
    """開ループ実行のタイミング・ランプ。`LearningLoopConfig` と同じフィールド名。

    既定値も本番と同じ。ただし A6 で accel_full_range_timeout_s（20 → 60s）・brake_stop_timeout_s
    （10 → 60s、意味も「停車しなければ中断する安全上限」）を変え、gov_release_frac・
    gov_raise_step_pct（ガバナーの解除）を足した。
    """

    brake_stop_timeout_s: float = field(default=60.0)
    brake_hold_timeout_s: float = field(default=20.0)
    accel_ramp_time_s: float = field(default=1.5)
    brake_ramp_time_s: float = field(default=1.5)
    pedal_release_time_s: float = field(default=0.4)
    pedal_step_time_s: float = field(default=0.2)
    skip_consecutive_required: int = field(default=2)
    g_smoothing_window_s: float = field(default=0.4)
    g_limit_frac: float = field(default=0.98)
    gov_reduce_step_pct: float = field(default=2.0)
    gov_release_frac: float = field(default=0.7)  # 加減速が上限のこの割合を下回ったら頭打ちを戻す
    gov_raise_step_pct: float = field(default=0.5)  # 戻すときの 1 周期あたりの量 [%]
    accel_speed_cap_frac: float = field(default=0.98)
    accel_full_range_timeout_s: float = field(default=60.0)
    overspeed_lead_s: float = field(default=1.2)
    overspeed_recovery_brake_pct: float = field(default=30.0)
    coast_down_stop_speed_kmh: float = field(default=5.0)
    coast_timeout_s: float = field(default=90.0)
    # 段7c: コーストダウンの終了判定に使う「落ち着いた」許容幅（learning.grid_settle_tol_kmh から
    # pattern_drive.py が渡す。既定値は config.LearningSection と揃えている）
    grid_settle_tol_kmh: float = field(default=1.0)
    creep_settle_min_s: float = field(default=13.0)
    creep_settle_stable_tol_kmhs: float = field(default=0.3)
    creep_settle_stable_duration_s: float = field(default=2.0)
    creep_settle_timeout_s: float = field(default=25.0)
    # クリープ発進（段1b。CreepLaunchPattern）の平衡到達判定。learning.creep_launch_settle_* から
    # pattern_drive.py が渡す（既定値はここと config.py の LearningSection とで揃えている）
    creep_launch_settle_kmhs: float = field(default=0.1)
    creep_launch_settle_s: float = field(default=2.0)
    # 2026-09-17（誤判定修正）: 停車保持解放直後は車速0・傾き0で
    # abs(accel_kmhs) < creep_launch_settle_kmhs が最初から成立してしまい、車が動き出す前に
    # 「平衡到達」と誤判定していた。車速がこの値以上になるまで安定カウントを積まない
    # （本質的なガード）。learning.creep_launch_min_speed_kmh から渡す
    creep_launch_min_speed_kmh: float = field(default=1.0)
    # elapsed がこの値以上になるまで平衡到達で終了しない（_advance_creep_settle の
    # creep_settle_min_s と同じ流儀の最短時間ガード）。learning.creep_launch_settle_min_s から渡す
    creep_launch_settle_min_s: float = field(default=5.0)
    # コーストダウンの加速（DRIVE_ACCEL）。目標 G との差に比例した速さでアクセルを踏み進める
    # （G に余裕があればぐっと踏み、目標に近づくほどゆっくり、超えたら戻す）。
    # 上限は pattern.accel_opening（COAST_DOWN_ACCEL_PCT。頭打ち）。
    # target/margin/slope_window/approach_margin は decel_stop セクションと同じ値
    # （pattern_drive.py が渡す。既定値は config.DecelStopSection と揃えている）
    coast_accel_target_g: float = field(default=0.2)
    coast_accel_press_margin_g: float = field(default=0.02)  # 目標 ± これ の間は開度を保持
    coast_accel_slope_window_s: float = field(default=1.0)  # 加速度（最小二乗の傾き）の窓 [s]
    coast_accel_approach_margin_pct: float = field(default=1.0)  # min − これ まで先に上げる
    # 踏む速さ [%/s] = これ ×（目標 G − 今の G）。半分データ由来・半分仮の値:
    # 感度 0.004〜0.005 G/%・遅れ約 1s（実機 110427）から、応答の時定数（1/(これ×感度) ≒ 2〜2.5s）
    # が遅れの 2 倍以上になる値。大きいほど早く届くが 0.2G を超えやすい。実機で決め直す
    # 踏み増す向きの指令は「実開度 + ramp_actuator_lag_tol_pct」までに抑える（2026-10-13。
    # 実機 090916 で指令が実開度より 20% 以上先行し 0.34G になった。戻す向きには掛けない）
    coast_accel_rate_gain: float = field(default=100.0)
    # 高い車速からの停車復帰の G 比例ブレーキ（DRIVE_BRAKE。`_step_gprop_brake`）。
    # stop_decel.decelerate_to_stop（手順2 終了時の緩減速）と同じ刻み方:
    # dwell_s ごとに直近 slope_window_s の減速 G を見て、目標 − press_margin 未満なら
    # step_mm 踏み増し、release_above_g 超なら step_mm 戻す。目標・余裕・窓・接近位置は
    # coast_accel_* を共用（decel_stop と同じ値）。pattern_drive.py が decel_stop から渡す
    calib_release_above_g: float = field(default=0.3)
    calib_step_mm: float = field(default=0.5)
    calib_dwell_s: float = field(default=1.0)
    # 惰行に入ってからこの時間は減速を数えない（ペダルを離してから車が応える遅れ。実機で
    # 指令→G の山まで約 0.6s、離す動作 0.4s）。惰行の減速 → 上限 G の予測の基準点に使う
    coast_measure_delay_s: float = field(default=1.5)
    # 段6b・門②: 上限 G を先読みして緩める。G の見込み = 今の G + G の増える速さ × gov_lead_s。
    # gov_lead_s 0.6 s は実機（20260926 の 890.7→891.3 s）で「ブレーキ指令から G の山まで」の遅れ。
    # 見込みが上限 × gov_soft_frac に届いたら踏み増しを止め、上限（× g_limit_frac）に届いたら
    # 従来どおり 1 周期ごとに下げる。gov_soft_frac 0.85 は**仮の値**（0.85 × 0.4G × 0.98 ≒ 0.33G）。
    # 実機の G の最大で決め直す
    gov_lead_s: float = field(default=0.6)
    gov_soft_frac: float = field(default=0.85)
    # 待機位置を使うか。待機位置 ＝ ペダルに触れた位置 min（プロファイルの
    # feedforward_params.*_deadband_pct。名前は不感帯のまま値は u の原点）。(1) G ランプの減速で
    # ブレーキの領域に入ったあと、ブレーキの下限を min にする（0% へ戻さない。2026-10-07 段2l-3）。
    # (2) 周期の出力で両ペダルの指令に min を下限として掛ける（使わない側を 0% にせず待機位置に
    # 置く。手順3 の mode_drive と同じ形。2026-10-07 段2l-4）。False なら (2) は掛けない
    # （今までどおり 0%）。2026-10-08 段2l-6: 旧 standby_margin_pct（不感帯 − 余裕）を置き換えた
    pedal_standby: bool = field(default=False)
    # G ランプの発進の最初のブレーキ開度 = 2-0 で測った止まった開度（stop_confirm_pct）。
    # None なら停車保持開度から始める。2026-10-08 段2l-6
    stop_confirm_pct: float | None = field(default=None)
    # G ランプの発進の離し方（段2o-10 A）。① 停車保持開度 → 止まる点 ＋ margin を fast_rate で一定に
    # 離し、② そこから slow_rate で車が動き出す（creep_launch_min_speed_kmh 以上）まで離す。
    # slow_rate は発進ごとに slow_rates を順に使う。動き出したら今の制御（偏差で離す）へ。
    # slow_rates が空なら使わない（今までどおり偏差だけで離す）。learning.ramp_launch_* から渡す
    ramp_launch_fast_rate_pctps: float = field(default=2.0)
    ramp_launch_release_margin_pct: float = field(default=2.0)
    ramp_launch_slow_rates_pctps: tuple[float, ...] = field(default=())
    # コーストダウン加速の車速の上端 [km/h]（None なら車両の最高速 × accel_speed_cap_frac のまま）。
    # 選んだモードの最高車速 + 余裕（learning.ramp_top_margin_kmh）。2026-10-07 段2l-4
    top_speed_cap_kmh: float | None = field(default=None)
    gov_rate_window_s: float = field(default=0.5)  # G の増える速さを見る窓 [s]
    # G ランプ走行（段2k 段B。GRampPattern）。learning.ramp_settle_time_s から
    # pattern_drive.py が渡す。偏差を何秒で詰めるか。Kp ＝ 1 ÷（走行中に測った感度 × これ）、
    # Ki ＝ Kp ÷（RAMP_TI_RATIO × これ）。アクセル・ブレーキ共通（ユーザー決定 2026-10-09:
    # 最初は共通）。2.0 秒は半分データ由来（変更前の式と同じ強さになる値）
    ramp_settle_time_s: float = field(default=2.0)
    # 狙いの加速度 g_ref を立ち上げる・切り替える・戻す時間 [s]（2026-10-09 段2l-9）。
    # learning.ramp_launch_time_s / ramp_turn_time_s / ramp_stop_time_s から渡す。0 なら丸めない
    ramp_launch_time_s: float = field(default=1.0)  # 発進: 動き出した周期から 0 → +G
    # 切り替え: +G → 0（保持の頭）と 0 → -G（減速の頭）の各片道。2026-10-10 段2l-10: 1.0 → 2.0
    ramp_turn_time_s: float = field(default=2.0)
    ramp_stop_time_s: float = field(default=1.0)  # 停止: -G → -ramp_edge_g（基準車速から決める）
    # 発進の始まりと停止の終わりの狙いの G [G]。0 なら 0 から・0 まで（旧）。狙いの G がこれより
    # 小さい段は丸めない（learning.ramp_edge_g）
    ramp_edge_g: float = field(default=0.05)
    # 2026-10-10 段2l-11: 発進の寄せの時間を段の G で変える（ユーザー決定）。時間 ＝
    # (G − min(ramp_edge_g, G)) ÷ ramp_launch_g_rate_gps、ただし ramp_launch_blend_min_s より
    # 短くしない（`_ramp_launch_blend_time_s`）。どの段でも G の上がる速さが同じになる。
    # rate が 0 以下なら全段 ramp_turn_time_s
    ramp_launch_g_rate_gps: float = field(default=0.15)
    ramp_launch_blend_min_s: float = field(default=0.5)
    # 2026-10-12 段2l-12: G ランプを踏み増し側だけで調整する（ユーザー決定）。False なら両方向
    # （段2l-11 までの動き）。上端は ramp_g_levels（learning.ramp_g_levels から渡す）の隣の強い段
    # との中間で決まる（`_ramp_upper_g`）。空なら戻す条件は働かない
    ramp_one_sided: bool = field(default=True)
    ramp_g_levels: tuple[float, ...] = field(default=())
    # 2026-10-10 段2l-10。learning.ramp_hold_time_s / ramp_dev_deadband_kmh /
    # ramp_actuator_lag_tol_pct から渡す（根拠は
    # config_testVehicle.yaml のコメント）。
    # 保持: +G → 0 の切り替えが終わってから 0 → -G が始まるまで [s]（仮の値）
    ramp_hold_time_s: float = field(default=7.0)
    # 偏差の不感帯 [km/h]（仮の値。車速の細かい揺れで開度を動かさない）
    ramp_dev_deadband_kmh: float = field(default=0.5)
    # 指令と実開度のずれがこれを超え、さらに離れる向きに指令を動かす周期は基準を待たせる [%]
    # （仮の値。表の開度のマス幅 2%）
    ramp_actuator_lag_tol_pct: float = field(default=2.0)
    # 2026-10-13 段2o-2: いちばん弱い段の途中の定速保持（learning.ramp_cruise_* から渡す。根拠は
    # config_testVehicle.yaml のコメント）。実測の加速度が ±band_g 以内の時間の合計が hold_s で
    # 次へ。入ってから ramp_turn_time_s ＋ timeout_s で打ち切り
    ramp_cruise_band_g: float = field(default=0.005)
    ramp_cruise_hold_s: float = field(default=7.0)
    ramp_cruise_timeout_s: float = field(default=15.0)
    # 2026-10-13 段2o-2: 遅れの測定のステップ（`GRampPattern.lag_step`）。learning.ramp_lag_step_g /
    # ramp_lag_step_hold_s から渡す。1 つの開度を ramp_lag_step_hold_s 秒保持し、踏み増しの大きさは
    # ramp_lag_step_g の加速度が出る開度の差
    ramp_lag_step_g: float = field(default=0.05)
    ramp_lag_step_hold_s: float = field(default=2.5)
    # 2026-10-14 段2o-5: 下りの戻し（`GRampPattern.release_stations`）。learning.ramp_release_*
    # から渡す。fracs が空なら戻しを入れない。min_kmh ＝ クリープ ＋ 5 km/h（planner と同じ。
    # 戻しの流れの途中でこれ以下になったら打ち切る安全網。0 なら停車だけ）
    ramp_release_fracs: tuple[float, ...] = field(default=())
    ramp_release_move_s: float = field(default=1.0)
    ramp_release_hold_s: tuple[float, ...] = field(default=())
    ramp_release_min_kmh: float = field(default=0.0)
    # 2026-10-16 段2o-6: false なら戻しに入らない（learning.ramp_release_enabled から渡す）
    ramp_release_enabled: bool = field(default=True)


class ActuatorDriverProtocol(Protocol):
    async def move_to_position(
        self, pos: int, *, smooth_over_s: float | None = None
    ) -> None: ...

    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None: ...

    async def read_monitor(self) -> AxisMonitor: ...

    async def is_alarm_active(self) -> bool: ...


class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...


# on_sample(データ, パターン番号, 段, ガバナー作動, アラーム(アクセル), アラーム(ブレーキ),
#           モニター(アクセル), モニター(ブレーキ), 周期の処理時間 [ms])
OnSample = Callable[
    [DriveLogData, int, str, bool, bool | None, bool | None, AxisMonitor, AxisMonitor, float],
    None,
]


class PatternLoop:
    """開度パターンを開ループ実行し、1 サイクルごとに `on_sample` でログへ渡す研究用ループ。

    `src/domain/control/learning_loop.py::LearningLoop` のフェーズ遷移・ガバナ・非常停止判定を
    移植したもの（アルゴリズムは変更しない）。
    """

    def __init__(
        self,
        *,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        profile: VehicleProfile,
        patterns: list[LearningPattern],
        overcurrent_limit_ma: float,
        on_complete: Callable[[], Awaitable[None]],
        on_emergency: Callable[[], Awaitable[None]],
        on_sample: OnSample,
        config: PatternLoopConfig | None = None,
        interval_s: float = PATTERN_LOOP_INTERVAL_S,
    ) -> None:
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._profile = profile
        self._patterns = patterns
        self._overcurrent_limit_ma = overcurrent_limit_ma
        self._on_complete = on_complete
        self._on_emergency = on_emergency
        self._on_sample = on_sample
        self._config = config if config is not None else PatternLoopConfig()
        self._interval_s = interval_s
        # skip_consecutive_required は「0.1 秒周期での周期数」。実周期に直した必要連続回数
        self._skip_required = max(
            1,
            round(
                self._config.skip_consecutive_required
                * PATTERN_LOOP_REFERENCE_INTERVAL_S
                / interval_s
            ),
        )
        # 2026-10-10 段2l-11: ガバナーの下げ・戻しの刻みも「0.1 秒周期での 1 周期ぶん」。
        # 実周期に直して 1 秒あたりの量を周期によらず同じにする（_skip_required と同じ考え方）
        interval_ratio = interval_s / PATTERN_LOOP_REFERENCE_INTERVAL_S
        self._gov_reduce_step = self._config.gov_reduce_step_pct * interval_ratio
        self._gov_raise_step = self._config.gov_raise_step_pct * interval_ratio

        self._max_decel_kmhs = max(profile.max_decel_g * G_TO_KMHS, 0.0)
        self._g_limit_kmhs = self._max_decel_kmhs * self._config.g_limit_frac
        self._accel_speed_cap = profile.max_speed * self._config.accel_speed_cap_frac
        if self._config.top_speed_cap_kmh is not None:  # 2026-10-07 段2l-4
            self._accel_speed_cap = min(self._accel_speed_cap, self._config.top_speed_cap_kmh)

        self._running = False
        self._cycle_task: asyncio.Task[None] | None = None
        self.abort_reason: str | None = None  # 非常停止した理由（正常終了なら None）
        # 状態機械（同期）が決めた中断。周期の終わりで処理する
        self._pending_abort: str | None = None
        self._safety = AxisSafetyNet(interval_s)
        self._cycles = 0
        self._started_at: float | None = None

        self._pattern_idx = 0
        self._phase = self._initial_phase(0)
        self._phase_started_at: float | None = None
        self._prev_speed: float | None = None
        self._prev_time: float | None = None
        self._skip_count = 0
        self._stable_count = 0
        self._accel_gov_cap: float | None = None
        self._brake_gov_cap: float | None = None
        self._overspeed_recovery = False
        # コーストダウン加速の G 比例制御（DRIVE_ACCEL）。_enter_phase で作り直す
        self._ca_t0: float | None = None  # 接近ランプの開始時刻。None ならまだ始めていない
        self._ca_approach_pct = 0.0
        self._ca_opening = 0.0  # 接近後の指令開度（ガバナー適用前）
        self._ca_last_t: float | None = None  # 直近に開度を更新した時刻
        self._ca_hist: deque[tuple[float, float]] = deque()  # 加速度の窓 (時刻, 車速)
        # 段7d: アクセルの点を記録するための実開度の履歴 (時刻, 開度)。_ca_lagged_opening が
        # 遅れを見込んだ窓の代表開度を作るのに使う
        self._ca_opening_hist: deque[tuple[float, float]] = deque()
        # 段7d: 毎周期の実開度（monitor から。_execute_one_cycle で更新）。モニタが無い経路
        # （_advance 直叩きのテスト等）では None のまま（_step_coast_accel が _ca_opening で代用）
        self._accel_actual_pct: float | None = None
        # 高い車速からの停車復帰の G 比例ブレーキ（DRIVE_BRAKE）の状態。_enter_phase で作り直す
        self._cb_t0: float | None = None  # 接近ランプの開始時刻
        self._cb_approach_pct = 0.0
        self._cb_opening = 0.0  # 接近後の指令開度（ガバナー適用前）
        self._cb_next_t: float | None = None  # 次に開度を決める時刻。None ならまだ接近中
        self._cb_hist: deque[tuple[float, float]] = deque()  # 減速度の窓 (時刻, 車速)
        # 上限 G に届く開度の予測マップ（段6a）。コーストダウン・G ランプの実測で育てる
    # （G ランプの最初の開度の見積もりに使う）
        self.glimit = GLimitMap()
        # 停車復帰（DRIVE_BRAKE）を G 比例で行うか（高い車速から入ったとき。_enter_phase で決める）
        self._db_gprop = False
        # コーストダウンの停車復帰のクリープ域で、停車保持開度まで徐々に踏むか（段2l-6b）
        self._db_creep_press = False
        self._db_press_reached_at: float | None = None  # 停車保持開度に届いた時刻
        # 2026-10-19 段2s-2: 踏み上げ中に停車した（立つと保持開度へすぐ指令。フェーズ毎に下ろす）
        self._db_stopped = False
        self._cb_start_pct = 0.0  # G 比例ブレーキの最初の開度（接近ランプの目標）
        # G ランプ走行（段2k 段B）。RAMP_UP に入るたびに _enter_phase が作り直す
        self._rp_u = 0.0  # 効く開度 u [%]（正がアクセル・負がブレーキ。RAMP_UP の頭で作り直す）
        # この減速の区間で一度でもブレーキ側（u ＜ 0）に入ったか（2026-10-07 段2l-2）。立つと
        # アクセルへ戻らない（惰行の減速は車速が下がるほど弱くなるので、戻る必要がない）
        self._rp_brake_latched = False
        # この RAMP_UP で一度でも u ≥ 0（アクセル側）になったか。立つと下限 0（ブレーキへ
        # 戻らない）。立つ前（u ＜ 0 ＝ 発進でブレーキを離している間）は今の u より下げない
        # （踏み直さない）
        self._rp_accel_reached = False
        # 基準車速への追従（2026-10-09 段2l-8）。RAMP_UP の頭で作り直し、RAMP_DOWN へ引き継ぐ
        self._rp_vref = 0.0  # 基準車速 v_ref [km/h]
        self._rp_turn_started: float | None = None  # 折り返し（g_ref を +G → −G）を始めた時刻
        # 動き出し（置き直し）の時刻（発進の g_ref を 0 → +G で数える起点）と、折り返した時点の
        # g_ref（切り替えの起点）。None の間は +G 扱い（2026-10-09 段2l-9）
        self._rp_launch_started: float | None = None
        # 発進のゆっくり離し（段2o-10 A。`_ramp_up_begin` が作り直す。通し番号だけ持ち越す）
        self._rp_launch_seq = 0
        self._rp_lr_active = False
        self._rp_lr_t: float | None = None
        self._rp_lr_target_u = 0.0
        self._rp_lr_slow = 0.0
        self._rp_lr_slow_t0: float | None = None
        self._rp_lr_limit_s = 0.0
        self._rp_turn_g0: float | None = None
        self._rp_ctl_t: float | None = None  # 直近に u・v_ref を更新した時刻（折り返しをまたぐ）
        self._rp_e_prev: float | None = None  # 前周期の偏差 e [km/h]（生の値。不感帯は引かない）
        # 保持（RAMP_HOLD。2026-10-10 段2l-10）: 入った時刻と、入った時点の g_ref（→ 0 の起点）、
        # 実車速が exit_speed を超え続けた周期数（安全網）
        self._rp_hold_started = 0.0
        self._rp_hold_g0 = 0.0
        self._rp_hold_over_count = 0
        # 表の開度への寄せ（2026-10-09。段2l-10 で 3 か所に一般化: 発進でアクセルに移った周期・
        # 保持の頭・減速の頭）。決めた周期に表を 1 回だけ引き、決めた時間で等速に足す
        self._rp_blend_decided = False  # 発進の寄せを決めた（寄せ無しに決めた場合も立つ）
        self._rp_blend_gap = 0.0  # 寄せる差の合計 [%]（符号つき）
        self._rp_blend_left = 0.0  # 寄せ残り [%]（符号つき）
        self._rp_blend_time = 0.0  # 寄せにかける時間 [s]
        self._rp_blend_time_left = 0.0  # 寄せ残り時間 [s]
        # 減速の頭のブレーキ側の寄せ（2 つ目）が待ち（2026-10-10 段2l-11）
        self._rp_down_brake_blend_pending = False
        # 今の寄せが「アクセルを離す（u → 0）」ものか。u が 0 以下になったら残りを捨てる（段2l-11）
        self._rp_blend_release = False
        # ペダル別に直近に計算した (Kp, Ki)。感度が無いペダルは (0, 0)。表示用
        self._rp_gains: dict[str, tuple[float, float]] = {
            PEDAL_ACCEL: (0.0, 0.0), PEDAL_BRAKE: (0.0, 0.0)
        }
        self._rp_last_t: float | None = None  # 直近に u を更新した時刻
        self._rp_hist: deque[tuple[float, float]] = deque()  # 加速度の窓 (時刻, 車速)
        # 実開度の履歴 (時刻, アクセル, ブレーキ)。遅れを見込んだ窓の代表開度を作る
        self._rp_open_hist: deque[tuple[float, float, float]] = deque()
        self._rp_record_t = -1e9  # 上限 G マップに最後に足した時刻（1 s に 1 点）
        self._rp_up_started = 0.0
        self._rp_down_started = 0.0
        self._rp_down_begun = False  # RAMP_DOWN に最初に入った時刻を記録したか（段2o-2）
        self._rp_max_speed = 0.0
        # 途中の定速保持（RAMP_CRUISE。2026-10-13 段2o-2）。RAMP_UP の頭で作り直す
        self._rp_cruise_done_up: set[int] = set()  # 上りで保持済み（とばした分を含む）の点の番号
        self._rp_cruise_done_down: set[int] = set()
        self._rp_cruise_dir = 1  # 入った方向（+1 上り・-1 下り）
        self._rp_cruise_started = 0.0
        self._rp_cruise_last_t = 0.0
        self._rp_cruise_in_band_s = 0.0  # 加速度が ±band_g 以内だった時間の合計 [s]
        self._rp_cruise_g0 = 0.0  # 入った時点の g_ref（→ 0 の起点）
        self._rp_resume_started: float | None = None  # 保持から上りへ戻した時刻（0 → +G の起点）
        self._rp_up_cruise_s = 0.0  # 上りの区間の中で保持に使った時間（時間切れから除く）
        self._rp_down_prior_s = 0.0  # 下りの区間の保持の前までに走った時間（時間切れに足す）
        self._rp_cruise_n = 0
        self._rp_cruise_done_n = 0
        # 遅れの測定のステップ（HOLD_STEP）: 開度 u の列・今のステップの番号・始めた時刻
        self._rp_step_targets: list[float] = []
        self._rp_step_idx = 0
        self._rp_step_t0 = 0.0
        # 結果用の積算（区間を切り替えるたびに足す）。キーは up / cruise / hold / down
        self._rp_acc: dict[str, float] = {
            "up": 0.0, "cruise": 0.0, "hold": 0.0, "down": 0.0, "release": 0.0,
        }
        # 下りの戻し（RAMP_RELEASE。2026-10-14 段2o-5）。RAMP_UP の頭で作り直す
        self._rp_release_done: set[int] = set()  # 戻し済み（飛ばした分を含む）の点の番号
        self._rp_release_t0 = 0.0  # 戻しに入った時刻
        self._rp_release_u_peak = 0.0  # 入った時点の u（山の開度。負）
        self._rp_release_n = 0
        self._rp_release_skipped = 0
        self._rp_seg_t = 0.0  # 今の区間の積算の起点
        self._rp_up_g: list[float] = []  # 加速側の実測 G（符号つき。窓が溜まってから）
        self._rp_down_g: list[float] = []  # 減速側の実測 G（符号つき）
        self.ramp_results: list[RampResult] = []  # G ランプ 1 本ごとの結果
        # 2026-10-16 段2o-6: 効き始め（ペダルの遊びの終わり）。G ランプ中の
        # (時刻, 車速, アクセル実開度, ブレーキ実開度) を HOLD_STEP 以外の全周期で貯め（パターンを
        # またいで残す）、G ランプが 1 本終わるたびに折れ点で求め直す。値は絶対開度 [%]。
        # 求まっていなければ辞書に無い
        self._onset_buf: list[tuple[float, float, float, float]] = []
        self._onset_pct: dict[str, float] = {}
        # 段ごとの移り変わり（pattern_idx, アクセルの効き始め, ブレーキの効き始め。None は求まらず）
        self.onset_history: list[tuple[int, float | None, float | None]] = []
        self.step_accel_below_onset_n = 0  # アクセルのステップの起点が効き始めより浅かった回数
        self._brake_actual_pct: float | None = None  # 毎周期のブレーキ実開度（モニターから）
        self._accel_hist: deque[tuple[float, float]] = deque()  # (時刻, 平滑化した加速度)
        self._accel_rate = 0.0  # 加速度の変化の速さ [km/h/s²]（先読みガバナー用）
        self._ca_record_t = -1e9  # 加速側の点を最後に足した時刻（1 s に 1 点）
        self._speed_hist: deque[tuple[float, float]] = deque()
        # コーストダウンの終了判定専用の履歴（段2l-11）。_speed_hist は _smoothed_accel が
        # g_smoothing_window_s（0.4 秒）で刈るので、creep_launch_settle_s（2 秒）ぶんが溜まらない
        self._coast_hist: deque[tuple[float, float]] = deque()
        self._last_speed = 0.0  # 定速階段の PI が使う直近の車速（_execute_one_cycle で更新）
        self._accel_pos_cmd = 0
        self._brake_pos_cmd = 0
        self._current_accel_opening = 0.0
        self._current_brake_opening = 0.0
        self._accel_request = 0.0  # ガバナーをかける前の指令開度（解除の判定に使う）
        self._brake_request = 0.0
        self._brake_ramp_from = 0.0  # ブレーキのランプを始める開度（フェーズに入ったときの開度）
        # 停車復帰の目標開度 = 停車保持開度
        self._stop_return_brake_pct = min(
            self._profile.feedforward_params.stop_brake_opening_pct,
            self._profile.max_brake_opening,
        )

        calib = self._profile.calibration
        if calib is not None:
            self._accel_pos_cmd = calib.accel_zero_pos
            self._brake_pos_cmd = opening_to_position(
                self._stop_return_brake_pct, calib.brake_zero_pos, calib.brake_full_pos
            )
        if self._phase is _Phase.RAMP_UP:  # 最初が G ランプのとき（_enter_phase を通らない）
            self._ramp_up_begin()

    # ── ライフサイクル ─────────────────────────────────────────────
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._cycle_task = asyncio.ensure_future(self._run())

    def stop(self) -> None:
        self._running = False

    async def stop_and_join(self, timeout_s: float = 2.0) -> None:
        self.stop()
        task = self._cycle_task
        if task is None or task.done() or task is asyncio.current_task():
            return
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout_s)
        except TimeoutError:
            task.cancel()
        except Exception:
            pass

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        next_tick = loop.time()
        try:
            while self._running:
                await self._execute_one_cycle()
                if not self._running:
                    return
                next_tick += self._interval_s
                delay = max(0.0, next_tick - loop.time())
                await asyncio.sleep(delay)
        except Exception as exc:
            await self._abort_emergency(f"パターン走行ループで例外（{type(exc).__name__}: {exc}）")

    # ── 1 サイクル ─────────────────────────────────────────────────
    async def _execute_one_cycle(self) -> None:
        if not self._running:
            return

        loop = asyncio.get_running_loop()
        now = loop.time()
        cycle_started = time.perf_counter()

        if self._pattern_idx >= len(self._patterns):
            self._running = False
            await self._on_complete()
            return

        pattern = self._patterns[self._pattern_idx]
        if self._phase_started_at is None:
            self._phase_started_at = now
        if self._started_at is None:
            self._started_at = now

        try:
            speed = await self._can_reader.read_speed()
        except Exception as exc:
            await self._abort_emergency(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）")
            return

        self._last_speed = speed
        self._speed_hist.append((now, speed))
        accel_kmhs = self._smoothed_accel(now)
        self._update_accel_rate(now, accel_kmhs)

        self._update_governor(accel_kmhs)
        accel_opening, brake_opening = self._command_openings(pattern, now)
        accel_opening, brake_opening = enforce_pedal_exclusion(accel_opening, brake_opening)
        self._current_accel_opening = accel_opening
        self._current_brake_opening = brake_opening
        # 2026-10-07 段2l-4: 出力だけ待機位置を下限にする（手順3 mode_drive と同じ形）。内部状態
        # （_current_*・ガバナー・停車復帰）は掛ける前の値のまま。
        # pedal_standby が False なら掛けない。
        # 2026-10-08 段2l-6: 待機位置は min（ペダルに触れた位置）そのもの
        if self._config.pedal_standby:
            ff = self._profile.feedforward_params
            accel_opening = max(accel_opening, ff.accel_deadband_pct)
            brake_opening = max(brake_opening, ff.brake_deadband_pct)

        calib = self._profile.calibration
        if calib is None:
            await self._abort_emergency("車両プロファイルにキャリブレーションがありません")
            return

        accel_pos = opening_to_position(accel_opening, calib.accel_zero_pos, calib.accel_full_pos)
        brake_pos = opening_to_position(brake_opening, calib.brake_zero_pos, calib.brake_full_pos)

        if not self._running:
            return

        try:
            monitor_accel, monitor_brake = await self._drive_or_sense(accel_pos, brake_pos)
        except Exception as exc:
            await self._abort_emergency(
                f"アクチュエータ通信に失敗しました（{type(exc).__name__}: {exc}）"
            )
            return

        cycle_ms = 1000.0 * (time.perf_counter() - cycle_started)
        accel_current, brake_current = monitor_accel.current_ma, monitor_brake.current_ma
        # 段7d: 上限 G の予測マップ用に、指令ではなく実際の開度を積む（低速の踏み込みは遅れる）
        self._accel_actual_pct = position_to_opening(
            monitor_accel.position_pulse, calib.accel_zero_pos, calib.accel_full_pos
        )
        self._brake_actual_pct = position_to_opening(
            monitor_brake.position_pulse, calib.brake_zero_pos, calib.brake_full_pos
        )
        t = now - self._started_at
        for label, current in (("アクセル", accel_current), ("ブレーキ", brake_current)):
            if current > self._overcurrent_limit_ma:
                await self._abort_emergency(
                    f"{label}軸が過電流です"
                    f"（{current:.0f} mA > {self._overcurrent_limit_ma:.0f} mA、t={t:.1f}s）"
                )
                return
        alarm_accel, alarm_brake = await self._safety.poll_alarms(
            self._cycles, self._accel_driver, self._brake_driver
        )
        self._cycles += 1
        reason = self._safety.check(
            t=t, accel_pos=accel_pos, brake_pos=brake_pos,
            accel_current=accel_current, brake_current=brake_current,
            alarm_accel=alarm_accel, alarm_brake=alarm_brake,
        )
        if reason is not None:
            await self._abort_emergency(reason)
            return

        # 2026-10-09 段2l-8: G ランプ中は基準車速 v_ref を CSV の ref_speed_kmh に書く（偏差列は
        # drive_log が実 − 基準で作る）。学習データの読み込み（fwd_data ほか）は基準列を見ない
        ramp_status = self.ramp_status()
        self._on_sample(
            DriveLogData(
                ref_speed_kmh=None if ramp_status is None else ramp_status.ref_kmh,
                actual_speed_kmh=speed,
                accel_opening=accel_opening,
                brake_opening=brake_opening,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=accel_current,
                brake_current=brake_current,
            ),
            self._pattern_idx,
            RAMP_LAUNCH_RELEASE_PHASE if self._rp_lr_active else self._phase.name,
            self._governor_limiting(),
            alarm_accel,
            alarm_brake,
            monitor_accel,
            monitor_brake,
            cycle_ms,
        )

        self._advance(pattern, speed, accel_kmhs, now)
        if self._pending_abort is not None:
            await self._abort_emergency(self._pending_abort)
            return

        self._prev_speed = speed
        self._prev_time = now

    # ── 状態機械 ───────────────────────────────────────────────────
    def _initial_phase(self, idx: int) -> _Phase:
        if idx >= len(self._patterns):
            return _Phase.DONE
        if isinstance(self._patterns[idx], CreepLaunchPattern):
            return _Phase.CREEP_LAUNCH
        if isinstance(self._patterns[idx], GRampPattern):
            return _Phase.RAMP_UP  # 停車保持から。発進の段階でブレーキを離し、min で u の制御へ
        if self._patterns[idx].kind is PatternKind.COAST_DOWN:
            return _Phase.DRIVE_ACCEL
        return _Phase.MEASURE

    def _command_openings(self, pattern: LearningPattern, now: float) -> tuple[float, float]:
        assert self._phase_started_at is not None
        elapsed = max(0.0, now - self._phase_started_at)
        if self._phase in _RAMP_PHASES:
            return self._ramp_command()
        if self._phase is _Phase.DRIVE_ACCEL:
            ramped = self._coast_accel_opening(pattern, now)
            self._accel_request = ramped
            if self._accel_gov_cap is not None:
                ramped = min(ramped, self._accel_gov_cap)
            return clamp_opening(ramped, self._profile.max_accel_opening), 0.0
        if self._phase is _Phase.COAST:
            return 0.0, 0.0
        if self._phase is _Phase.CREEP_LAUNCH:
            return 0.0, 0.0  # 両ペダル完全解放（クリープのみで自走）
        if self._phase is _Phase.DRIVE_BRAKE and self._db_gprop:
            opening = self._calib_brake_opening(now)
            self._brake_request = opening
            if self._brake_gov_cap is not None:
                opening = min(opening, self._brake_gov_cap)
            return 0.0, clamp_opening(opening, self._profile.max_brake_opening)
        if self._phase is _Phase.DRIVE_BRAKE and self._db_creep_press:
            # 段2l-6b: 今の開度から停車保持開度まで一定の速さで踏み上げる（下げない）
            rate = self._stop_return_press_rate_pctps()
            assert rate is not None
            target_brake = self._stop_return_brake_pct
            # 2026-10-19 段2s-2: 起点は max(今の開度, ブレーキ min)。min の手前を踏み上げの速さで
            # 待たない。停車した後は速さを待たずに停車保持開度へすぐ指令する（単調は守る）
            start = max(self._brake_ramp_from, self._profile.feedforward_params.brake_deadband_pct)
            if self._db_stopped:
                ramped = target_brake
            else:
                ramped = min(target_brake, start + rate * elapsed)
            ramped = max(ramped, min(start, target_brake))
            self._brake_request = ramped
            if self._brake_gov_cap is not None:
                ramped = min(ramped, self._brake_gov_cap)
            return 0.0, clamp_opening(ramped, self._profile.max_brake_opening)
        if self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD):
            target_brake = pattern.brake_opening
            if self._phase is _Phase.DRIVE_BRAKE:
                target_brake = self._stop_return_brake_pct  # 停車復帰（A6）
                if self._overspeed_recovery:
                    target_brake = max(target_brake, self._config.overspeed_recovery_brake_pct)
            start = self._brake_ramp_from
            frac = self._ramp_fraction(elapsed, self._config.brake_ramp_time_s)
            ramped = start + (target_brake - start) * frac
            self._brake_request = ramped
            if self._brake_gov_cap is not None:
                ramped = min(ramped, self._brake_gov_cap)
            return 0.0, clamp_opening(ramped, self._profile.max_brake_opening)
        return pattern.accel_opening, pattern.brake_opening

    @staticmethod
    def _ramp_fraction(elapsed: float, ramp_time_s: float) -> float:
        if ramp_time_s <= 0.0:
            return 1.0
        return min(1.0, max(0.0, elapsed) / ramp_time_s)

    def _advance(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, now: float
    ) -> None:
        assert self._phase_started_at is not None
        elapsed = now - self._phase_started_at

        if self._phase is _Phase.DRIVE_ACCEL:
            self._advance_drive_accel(pattern, speed, accel_kmhs, elapsed, now)
            return
        if self._phase is _Phase.COAST:
            self._advance_coast(speed, accel_kmhs, elapsed, now)
            return
        if self._phase is _Phase.CREEP_LAUNCH:
            self._advance_creep_launch(pattern, speed, accel_kmhs, elapsed, now)
            return
        if self._phase in _RAMP_PHASES:
            assert isinstance(pattern, GRampPattern)
            self._advance_ramp(pattern, speed, elapsed, now)
            return
        if self._phase is _Phase.DRIVE_BRAKE:
            if self._db_gprop:
                self._step_gprop_brake(speed, now)
                if speed <= self._stop_return_low_speed_kmh():
                    # 低速まで下りた。あとは停車保持開度へ（今の開度から。コーストダウンは
                    # 徐々に踏む、それ以外は従来どおり brake_ramp_time_s のランプ）
                    self._db_gprop = False
                    self._db_creep_press = self._creep_press_applies()
                    self._brake_ramp_from = self._current_brake_opening
                    self._phase_started_at = now
                    elapsed = 0.0
            if self._db_creep_press:
                if speed <= STOP_SPEED_KMH:
                    self._db_stopped = True  # 2026-10-19 段2s-2: 停車したら保持開度へすぐ指令
                # 段2l-6b: 停車かつ停車保持開度に届いてから次へ。時間切れも届いてから数える
                # （踏み上げに 30 秒以上かかるため、届く前には中断しない）
                if (
                    self._db_press_reached_at is None
                    and self._brake_request >= self._stop_return_brake_pct - 1e-9
                ):
                    self._db_press_reached_at = now
                if self._db_press_reached_at is None:
                    return
                if speed <= STOP_SPEED_KMH:
                    self._advance_pattern(now)
                    return
                elapsed = now - self._db_press_reached_at
            if speed <= STOP_SPEED_KMH:
                self._advance_pattern(now)
            elif elapsed >= self._config.brake_stop_timeout_s:
                self._pending_abort = (
                    f"停車復帰（DRIVE_BRAKE）で brake_stop_timeout_s="
                    f"{self._config.brake_stop_timeout_s:g}s 以内に停車しません"
                    f"（パターン {self._pattern_idx + 1}:{pattern.kind.name}、"
                    f"車速 {speed:.1f} km/h、"
                    f"ブレーキ {self._current_brake_opening:.2f}%）"
                )
            return
        if self._phase is _Phase.BRAKE_HOLD:
            if speed <= STOP_SPEED_KMH:
                self._advance_pattern(now)
            elif elapsed >= self._config.brake_hold_timeout_s:
                self._finish_pattern(speed, now)
            return
        if self._phase is _Phase.MEASURE and pattern.kind is PatternKind.CREEP_SETTLE:
            self._advance_creep_settle(elapsed, accel_kmhs, now)
            return
        if self._phase is _Phase.MEASURE:
            if elapsed >= pattern.hold_duration_s:
                self._advance_pattern(now)
            return

    def _coast_accel_opening(self, pattern: LearningPattern, now: float) -> float:
        """コーストダウン加速の指令開度（ガバナー適用前）。

        最初は 0 から「min − approach_margin」まで accel_ramp_time_s で上げる（遊びの中では
        加速が出ない）。以降の開度は `_step_coast_accel` が G との差に応じて動かす。
        """
        cfg = self._config
        if self._ca_t0 is None:
            ff = self._profile.feedforward_params
            self._ca_approach_pct = min(
                pattern.accel_opening,
                max(0.0, ff.accel_deadband_pct - cfg.coast_accel_approach_margin_pct),
            )
            self._ca_t0 = now
        elapsed = now - self._ca_t0
        if elapsed < cfg.accel_ramp_time_s:
            return self._ca_approach_pct * self._ramp_fraction(elapsed, cfg.accel_ramp_time_s)
        if self._ca_last_t is None:  # 接近が済んだ。ここから G 比例で踏み進める
            self._ca_opening = self._ca_approach_pct
            self._ca_last_t = now
        return self._ca_opening

    def _step_coast_accel(self, pattern: LearningPattern, speed: float, now: float) -> None:
        """毎周期、直近 slope_window_s の加速度 G から開度を動かす。

        踏む速さ [%/s] = rate_gain ×（目標 G − 今の G）。目標 ± press_margin_g の間は保持。
        範囲は [接近位置, pattern.accel_opening]（G が高ければ同じ式で戻る）。
        踏み増す向きだけ、指令は「実開度 + ramp_actuator_lag_tol_pct」までに抑える
        （実開度が追いつくのを待つ。2026-10-13。実開度が無い経路は制限なし）。
        """
        cfg = self._config
        hist = self._ca_hist
        hist.append((now, speed))
        while hist and now - hist[0][0] > cfg.coast_accel_slope_window_s:
            hist.popleft()
        # 段7d: 実開度の履歴（_ca_lagged_opening が遅れを見込んだ窓の代表開度を作るのに使う）。
        # モニタが無い経路（_advance 直叩きのテスト等）は _accel_actual_pct が None のまま
        # なので、この phase の指令値 _ca_opening で代用する
        actual = self._accel_actual_pct if self._accel_actual_pct is not None else self._ca_opening
        self._ca_opening_hist.append((now, actual))
        max_age = cfg.coast_accel_slope_window_s + cfg.gov_lead_s + 0.5
        while self._ca_opening_hist and now - self._ca_opening_hist[0][0] > max_age:
            self._ca_opening_hist.popleft()
        if self._ca_last_t is None:
            return  # 接近ランプ中
        dt = now - self._ca_last_t
        self._ca_last_t = now
        accel_g = fit_slope([t for t, _ in hist], [v for _, v in hist]) / G_TO_KMHS
        margin = cfg.coast_accel_target_g - accel_g
        # 記録は 1 s に 1 点。開度が動いていても（40〜110 km/h は G が目標に届かず開度が
        # 上がり続け、以前の「止まっている間しか記録しない」条件では 1 点も取れなかった）、
        # 遅れ gov_lead_s（＝指令から G の山までの遅れ。ガバナーの先読みと同じ値）ぶん過去へ
        # ずらした窓の実開度の平均を、今の G と対にして記録する（段7d）。振れの条件は無い
        # （実開度で記録するので、指令が動いていても対応関係は崩れない）
        lagged = self._ca_lagged_opening(now)
        if lagged is not None and now - self._ca_record_t >= 1.0:
            self._ca_record_t = now
            mean_v = sum(v for _, v in hist) / len(hist)
            self.glimit.add(PEDAL_ACCEL, mean_v, lagged, accel_g * G_TO_KMHS)
        if abs(margin) <= cfg.coast_accel_press_margin_g:
            return
        opening = self._ca_opening + cfg.coast_accel_rate_gain * margin * dt
        opening = min(pattern.accel_opening, max(self._ca_approach_pct, opening))
        if opening > self._ca_opening and self._accel_actual_pct is not None:
            # 2026-10-13: 実機 090916 で実開度は約 8 %/s、指令は最大 20 %/s で進み、指令が
            # 実開度より 20% 以上先行 → G が目標に届いた後も実開度が上がり続けて 0.34G。
            # 踏み増しは「実開度 + 許容ずれ」までに抑える（実開度が追いつくのを待つ）。
            # 戻す向きには掛けず、今の指令を強制的に下げることもしない。許容ずれは G ランプと
            # 同じ ramp_actuator_lag_tol_pct（仮の値。ユーザー決定の流用）
            ceiling = max(self._ca_opening, self._accel_actual_pct + cfg.ramp_actuator_lag_tol_pct)
            opening = min(opening, ceiling)
        self._ca_opening = opening

    def _ca_lagged_opening(self, now: float) -> float | None:
        """遅れ gov_lead_s を見込んだ窓 `[now − gov_lead_s − slope_window_s, now − gov_lead_s]`

        の実開度の平均（段7d。窓に 2 点未満なら None）。今の G はこの窓の開度が作ったもの、
        という遅れの見込み（`gov_lead_s` はブレーキ指令から G の山までの遅れの実測値の流用）。
        """
        cfg = self._config
        lo = now - cfg.gov_lead_s - cfg.coast_accel_slope_window_s
        hi = now - cfg.gov_lead_s
        openings = [o for t, o in self._ca_opening_hist if lo <= t <= hi]
        if len(openings) < 2:
            return None
        return sum(openings) / len(openings)

    def _advance_drive_accel(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, elapsed: float, now: float
    ) -> None:
        cfg = self._config
        self._step_coast_accel(pattern, speed, now)
        if speed > self._profile.max_speed:
            self._skip_count += 1
        else:
            self._skip_count = 0
        if self._skip_count >= self._skip_required:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return

        # コーストダウン: cap の手前（加速度 × 先読み）まで上げたら惰行へ。
        # accel_full_range_timeout_s は「加速に使う時間の上限」で、cap 未達でもここで惰行へ進む
        # （目標 G が出ないほどペダルが利かない場合に、加速だけで走り続けないための打ち切り）
        exit_speed = self._accel_speed_cap - max(0.0, accel_kmhs) * cfg.overspeed_lead_s
        if speed >= exit_speed or elapsed >= cfg.accel_full_range_timeout_s:
            self._enter_phase(_Phase.COAST, now)

    # ── G ランプ走行（段2k 段B）─────────────────────────────────────
    def _ramp_u_limits(self) -> tuple[float, float]:
        """効く開度 u の範囲 (下限＝ブレーキ側, 上限＝アクセル側)。

        2026-10-07 段2l-2: RAMP_UP は、一度アクセル側（u ≥ 0）に入ったあとは下限 0（ブレーキ側へ
        行かない。クリープ域で狙いより強く加速しても両ペダル min のままクリープに任せる）。
        RAMP_DOWN で一度ブレーキ側に入ったあと（`_rp_brake_latched`）は上限 0（アクセルへ戻らない。
        惰行の減速は車速が下がるほど弱くなるので、一度ブレーキが要る領域に入ったらその先で
        アクセルが要ることはない。ユーザー決定）。
        2026-10-08 段2l-6: 待機位置が min そのものになったので、ラッチ後の上限は素直に 0
        （ブレーキ ＝ min ＋ |u|。下限 min）。
        2026-10-09 段2l-8: 発進（RAMP_UP の頭で u ＜ 0 ＝ ブレーキを離している間）は下限を今の u に
        する（離す向きにしか動かさない＝踏み直さない）。u ≥ 0 になったら下限 0。
        """
        ff = self._profile.feedforward_params
        lo = -max(0.0, self._profile.max_brake_opening - ff.brake_deadband_pct)
        hi = max(0.0, self._profile.max_accel_opening - ff.accel_deadband_pct)
        if self._phase in (_Phase.RAMP_UP, _Phase.RAMP_HOLD):
            # 2026-10-10 段2l-10: RAMP_HOLD も RAMP_UP と同じ（アクセルに入った後は下限 0。
            # 発進が終わらないまま来たときは今の u から離す向きだけ）
            lo = 0.0 if self._rp_accel_reached else min(0.0, self._rp_u)
        elif self._phase is _Phase.RAMP_CRUISE:
            # 2026-10-13 段2o-2: 保持の下限は、ブレーキ側（u ＜ 0）で入ったときは今の u
            # （離す向きだけ）、
            # 0 以上なら 0。下りのラッチ（上限 0）は外してアクセルで保つ。`_rp_accel_reached` を
            # 見て下限 0 にすると、ブレーキを踏んだまま入ったとき 1 周期でブレーキが抜ける
            lo = min(0.0, self._rp_u)
        elif (self._phase is _Phase.RAMP_DOWN and self._rp_brake_latched) or (
            self._phase is _Phase.RAMP_RELEASE
        ):
            hi = 0.0  # 戻し（段2o-5）はブレーキ側（u ≤ 0）だけ。u は `_ramp_release_u` が決める
        return lo, hi

    def _ramp_on_brake(self) -> bool:
        """G ランプの u がブレーキ側を指すか（RAMP_DOWN のラッチ後、または u ＜ 0）。"""
        if self._phase is _Phase.RAMP_DOWN and self._rp_brake_latched:
            return True
        if self._phase is _Phase.RAMP_RELEASE:
            return True  # 戻しの間はブレーキの領域（u ＝ 0 でも待機位置 ＝ min）
        return self._rp_u < 0.0

    def _ramp_estimate_u(self, speed: float, target_g: float) -> float | None:
        """`speed` で `target_g`（符号つき。加速 +・減速 −）を出すと予測される u。無ければ None。

        実測の（車速・開度・加速度）を持つ `self.glimit` から引く。減速で狙いが惰行より緩い
        （その車速帯の惰行の減速の中央値より小さい）ときはアクセルを少し踏んだ側（u ＞ 0）、
        それ以外の減速はブレーキ側（u ＜ 0）の予測を使う。

        2026-10-08 段2l-6: 狙いが貯めた点のどれより強いときは割線の外挿をせず、いちばん効いた
        点の開度を返す（`extrapolate=False`）。補間は変わらない。
        """
        ff = self._profile.feedforward_params
        target_kmhs = target_g * G_TO_KMHS
        if target_kmhs >= 0.0:
            opening = self._glimit_open_for(PEDAL_ACCEL, speed, target_kmhs, extrapolate=False)
            return None if opening is None else opening - ff.accel_deadband_pct
        coast = self.glimit.coast_decel_kmhs(speed)
        if coast is not None and -target_kmhs < coast:
            # 惰行より緩い減速: アクセル側の負の加速度として引く（惰行の点も使う）
            opening = self._glimit_open_for(PEDAL_ACCEL, speed, target_kmhs, extrapolate=False)
            return None if opening is None else opening - ff.accel_deadband_pct
        opening = self._glimit_open_for(PEDAL_BRAKE, speed, -target_kmhs, extrapolate=False)
        return None if opening is None else -(opening - ff.brake_deadband_pct)

    def _ramp_command(self) -> tuple[float, float]:
        """G ランプの指令開度 (アクセル, ブレーキ)。u ＞ 0 はアクセル ＝ min ＋ u、
        u ＜ 0 はブレーキ ＝ min ＋ |u|、u ＝ 0 は両方 0（待機位置の下限は出力側で掛ける。
        `fwd_data.effective_opening` の逆）。

        2026-10-07 段2l-3: RAMP_DOWN のラッチ後（ブレーキの領域）は u の符号に関係なく
        アクセル ＝ 0、ブレーキ ＝ min − u（u は 0 以下なのでブレーキ ≧ min。0% へ戻さない）。
        0% ↔ 不感帯の飛びでペダルの移動 約 0.7 秒の間に踏みすぎて往復した（実機 151515）。

        2026-10-09 段2l-8: 発進（停車保持ブレーキを離す場面）も u で表す（RAMP_UP の頭の u は
        「今のブレーキ指令開度 − min」の負の値。`_ramp_up_begin`）。専用の発進の段階は無い。
        """
        ff = self._profile.feedforward_params
        u = self._rp_u
        # 2026-10-14 段2o-5: RAMP_RELEASE もラッチ後の RAMP_DOWN と同じ（u ＝ 0 でブレーキ ＝ min。
        # 0% へ飛ばさない）
        if (self._phase is _Phase.RAMP_DOWN and self._rp_brake_latched) or (
            self._phase is _Phase.RAMP_RELEASE
        ):
            accel = 0.0
            brake = ff.brake_deadband_pct - u
        else:
            accel = ff.accel_deadband_pct + u if u > 0.0 else 0.0
            brake = ff.brake_deadband_pct - u if u < 0.0 else 0.0
        self._accel_request = accel
        self._brake_request = brake
        if accel <= 0.0:
            self._accel_gov_cap = None  # 使っていないペダルの古い頭打ちは持ち越さない
        if brake <= 0.0:
            self._brake_gov_cap = None
        if self._accel_gov_cap is not None:
            accel = min(accel, self._accel_gov_cap)
        if self._brake_gov_cap is not None:
            brake = min(brake, self._brake_gov_cap)
        return (
            clamp_opening(accel, self._profile.max_accel_opening),
            clamp_opening(brake, self._profile.max_brake_opening),
        )

    def _ramp_up_begin(self) -> None:
        """G ランプ 1 本の始まり（RAMP_UP に入った瞬間。ブレーキを離す前から）の状態を作る。

        基準車速 v_ref ＝ 0 km/h・折り返し前・動き出し前（`_rp_launch_started`・`_rp_turn_g0` は
        None。2026-10-09 段2l-9）。u は「ブレーキを止まった開度（無ければ停車保持
        開度）に置いた指令 − min」の負の値（開始開度の決め方は 2026-10-08 段2l-6 から踏襲:
        今のブレーキ位置がそれより低いとき、前の G ランプの減速で止まった開度に届く前に停車
        したなどは、踏み直さずに今の位置から始める）。この u から偏差で離していく
        （`_step_ramp`）。
        """
        self._rp_vref = 0.0
        self._rp_turn_started = None
        self._rp_launch_started = None  # 動き出し（置き直し）の時刻。None の間は +G（段2l-9）
        self._rp_turn_g0 = None  # 折り返した時点の g_ref。None なら +G 扱い（段2l-9）
        self._rp_ctl_t = None
        self._rp_e_prev = None
        self._rp_hold_started = 0.0
        self._rp_hold_g0 = 0.0
        self._rp_hold_over_count = 0
        self._rp_vref_anchored = False  # 動き出しで基準車速を実車速へ置き直したか（段ごとに 1 回）
        self._rp_blend_decided = False
        self._rp_blend_gap = self._rp_blend_left = self._rp_blend_time_left = 0.0
        self._rp_blend_time = 0.0
        self._rp_down_brake_blend_pending = False
        self._rp_gains = {PEDAL_ACCEL: (0.0, 0.0), PEDAL_BRAKE: (0.0, 0.0)}
        self._rp_brake_latched = False
        # 途中の定速保持の状態を作り直す（2026-10-13 段2o-2）
        self._rp_cruise_done_up = set()
        self._rp_cruise_done_down = set()
        self._rp_resume_started = None
        self._rp_up_cruise_s = 0.0
        self._rp_down_prior_s = 0.0
        self._rp_down_begun = False
        self._rp_cruise_n = 0
        self._rp_cruise_done_n = 0
        self._rp_acc = {"up": 0.0, "cruise": 0.0, "hold": 0.0, "down": 0.0, "release": 0.0}
        self._rp_release_done = set()
        self._rp_release_n = 0
        self._rp_release_skipped = 0
        start = self._config.stop_confirm_pct
        if start is None or self._ramp_launch_release_enabled():
            # 段2o-10 A: ゆっくり離すときは停車保持開度から始め、止まる点は ① の終点に使う
            start = self._stop_return_brake_pct
        calib = self._profile.calibration
        if calib is not None:
            start = min(
                start,
                position_to_opening(
                    self._brake_pos_cmd, calib.brake_zero_pos, calib.brake_full_pos
                ),
            )
        floor = self._profile.feedforward_params.brake_deadband_pct
        self._rp_u = -(max(floor, min(start, self._profile.max_brake_opening)) - floor)
        self._rp_accel_reached = self._rp_u >= 0.0
        self._ramp_launch_release_begin(floor)

    def _ramp_launch_release_enabled(self) -> bool:
        cfg = self._config
        return bool(cfg.ramp_launch_slow_rates_pctps) and cfg.ramp_launch_fast_rate_pctps > 0.0

    def _ramp_launch_release_begin(self, floor: float) -> None:
        """発進のゆっくり離し（段2o-10 A）の用意。① 止まる点 ＋ margin まで fast_rate、② そこから
        slow_rate（発進ごとに配列を順に）。止まる点が None なら停車保持開度
        （`_stop_return_brake_pct`）。
        u が 0 以上（最初からアクセル側）・slow_rates が空・fast_rate が 0 以下なら使わない。"""
        cfg = self._config
        self._rp_lr_active = False
        self._rp_lr_t = None
        self._rp_lr_slow_t0 = None
        rates = cfg.ramp_launch_slow_rates_pctps
        if not rates:
            return
        slow = rates[self._rp_launch_seq % len(rates)]
        self._rp_launch_seq += 1
        if self._rp_u >= 0.0 or slow <= 0.0 or cfg.ramp_launch_fast_rate_pctps <= 0.0:
            return
        stop = cfg.stop_confirm_pct
        if stop is None:
            stop = self._stop_return_brake_pct
        target_brake = min(
            stop + cfg.ramp_launch_release_margin_pct, self._profile.max_brake_opening
        )
        self._rp_lr_target_u = -(max(floor, target_brake) - floor)
        self._rp_lr_slow = slow
        # ② が長すぎるときの上限: (止まる点 ＋ margin − min) ÷ slow_rate ＋ 5 秒
        self._rp_lr_limit_s = -self._rp_lr_target_u / slow + 5.0
        self._rp_lr_active = True

    def _ramp_launch_release_step(self, speed: float, now: float) -> bool:
        """発進のゆっくり離しを 1 周期進める。続けるなら True、③（今の制御）へ進むなら False
        （動き出した・min に着いた・② の上限を超えた）。③ へはその周期の u のまま渡す。"""
        cfg = self._config
        last = self._rp_lr_t
        self._rp_lr_t = now
        dt = 0.0 if last is None else min(max(0.0, now - last), 10.0 * self._interval_s)
        if speed >= cfg.creep_launch_min_speed_kmh:
            self._rp_lr_active = False
            return False
        u = self._rp_u
        target = self._rp_lr_target_u
        if u < target:  # ①: 止まる点 ＋ margin まで一定の速さ
            self._rp_u = min(target, u + cfg.ramp_launch_fast_rate_pctps * dt)
            return True
        if self._rp_lr_slow_t0 is None:
            self._rp_lr_slow_t0 = now
        u += self._rp_lr_slow * dt  # ②: ゆっくり
        if u >= 0.0 or now - self._rp_lr_slow_t0 > self._rp_lr_limit_s:
            self._rp_u = min(u, 0.0)
            self._rp_lr_active = False
            return False
        self._rp_u = u
        return True

    def _ramp_gref(self, pattern: GRampPattern, now: float) -> float:
        """狙いの加速度 g_ref [G]（符号つき）。2026-10-09 段2l-9。G ＝ pattern.target_g。

        - RAMP_UP・動き出す前（置き直し前）: +G（基準を進めて偏差でブレーキを離すため）。
        - RAMP_UP・置き直し後: g_min ＋ (G − g_min) × min(1, 動き出してからの経過 ÷
          ramp_launch_time_s)。置き直した周期は g_min。時間が 0 なら最初から +G。
          g_min ＝ min(ramp_edge_g, G)（下げ止め。2026-10-09。0 まで下げると基準が 0 km/h に
          着いたあと表の開度が「減速 0」へ向かいブレーキが緩んで停車が長引く。
          ramp_edge_g ＝ 0 なら 0 から）。
        - RAMP_HOLD（2026-10-10 段2l-10）: 入った時点の g_ref（`_rp_hold_g0`）→ 0 を
          ramp_turn_time_s で直線、その後 0（時間 0 なら最初から 0）。
        - RAMP_DOWN: max(切り替え, 停止)。切り替え ＝ 入った時点の g_ref（保持の後は 0）から −G へ
          ramp_turn_time_s で直線（時間 0・折り返し時刻なしなら −G）。停止 ＝ −max(g_min, G × min(1,
          √(max(0, v_ref) ÷ v_stop)))、v_stop ＝ G × G_TO_KMHS × ramp_stop_time_s ÷ 2
          （−G → 0 を停止の時間で直線に変え、ちょうど 0 km/h に着く線を車速で書いたもの。
          時間 0 なら −G）。v_ref が 0 のとき停止は −g_min（基準は 0 で頭打ち）、残る実車速は
          偏差でブレーキを踏んで止める。
        """
        cfg = self._config
        g = pattern.target_g
        g_min = min(max(0.0, cfg.ramp_edge_g), g)  # 発進の始まり・停止の終わりの下げ止め
        if self._phase in (_Phase.HOLD_STEP, _Phase.RAMP_RELEASE):
            # 開度を固定で持つ（基準は進めない。2026-10-13 段2o-2。戻しも同じ。2026-10-14 段2o-5:
            # 戻しを出るときの `_rp_turn_g0` が 0 になり、g_ref は 0 → −G で始まる）
            return 0.0
        if self._phase is _Phase.RAMP_CRUISE:
            # 2026-10-13 段2o-2: 入った時点の実測（±G）→ 0 を ramp_turn_time_s で直線、その後 0
            if cfg.ramp_turn_time_s <= 0.0:
                return 0.0
            frac = min(1.0, max(0.0, (now - self._rp_cruise_started) / cfg.ramp_turn_time_s))
            return self._rp_cruise_g0 * (1.0 - frac)
        if self._phase is _Phase.RAMP_UP:
            if self._rp_resume_started is not None:
                # 保持から上りへ戻した後: 0 → +G を ramp_turn_time_s で直線（2026-10-13 段2o-2）
                if cfg.ramp_turn_time_s <= 0.0:
                    return g
                frac = (now - self._rp_resume_started) / cfg.ramp_turn_time_s
                return g * min(1.0, max(0.0, frac))
            if self._rp_launch_started is None or cfg.ramp_launch_time_s <= 0.0:
                return g
            frac = (now - self._rp_launch_started) / cfg.ramp_launch_time_s
            return g_min + (g - g_min) * min(1.0, max(0.0, frac))
        if self._phase is _Phase.RAMP_HOLD:
            if cfg.ramp_turn_time_s <= 0.0:
                return 0.0
            frac = min(1.0, max(0.0, (now - self._rp_hold_started) / cfg.ramp_turn_time_s))
            return self._rp_hold_g0 * (1.0 - frac)
        if self._rp_turn_started is None or cfg.ramp_turn_time_s <= 0.0:
            turn = -g
        else:
            frac = min(1.0, max(0.0, (now - self._rp_turn_started) / cfg.ramp_turn_time_s))
            g0 = g if self._rp_turn_g0 is None else self._rp_turn_g0
            turn = g0 + (-g - g0) * frac
        v_stop = g * G_TO_KMHS * cfg.ramp_stop_time_s / 2.0
        if v_stop <= 0.0:
            return turn
        stop = -max(g_min, g * min(1.0, math.sqrt(max(0.0, self._rp_vref) / v_stop)))
        return max(turn, stop)

    def _ramp_upper_g(self, pattern: GRampPattern) -> float | None:
        """段の G の上端 [G]（2026-10-12 段2l-12）。ramp_g_levels の中で target_g より大きい
        最小の段 g_next があれば (target_g ＋ g_next) ÷ 2（`ramp_planner.level_lower_edges` と
        同じ境目）。いちばん強い段・一覧が空なら None（戻さない）。"""
        g = pattern.target_g
        upper = [lv for lv in self._config.ramp_g_levels if lv > g + 1e-9]
        if not upper:
            return None
        return (g + min(upper)) / 2.0

    def _ramp_one_sided_dir(self, pattern: GRampPattern) -> int | None:
        """片側で調整する区間の順方向の符号（2026-10-12 段2l-12）。+1 ＝ u を増やす向き
        （RAMP_UP）、-1 ＝ u を減らす向き（RAMP_DOWN）。片側にしない周期は None。

        - ramp_one_sided が False → None。
        - RAMP_UP で動き出しの置き直しの後（`_rp_vref_anchored`）→ +1。
        - RAMP_DOWN で停止の直前でない → -1。停止の直前 ＝ 基準車速 ≤ v_stop（`_ramp_gref` の
          停止の式と同じ。v_stop ≤ 0 のときはこの判定なし）か 基準車速 ＜ STOP_SPEED_KMH。
        - RAMP_HOLD・それ以外 → None。
        """
        cfg = self._config
        if not cfg.ramp_one_sided:
            return None
        if self._phase is _Phase.RAMP_UP:
            return 1 if self._rp_vref_anchored else None
        if self._phase is _Phase.RAMP_DOWN:
            if self._rp_vref < STOP_SPEED_KMH:
                return None
            v_stop = pattern.target_g * G_TO_KMHS * cfg.ramp_stop_time_s / 2.0
            if v_stop > 0.0 and self._rp_vref <= v_stop:
                return None
            return -1
        return None

    def _ramp_min_step_pct(self, on_brake: bool) -> float:
        """アクチュエータの最小刻み 1 個（1 パルス）ぶんの開度 [%]。キャリブレーションの
        ストローク（zero_pos〜full_pos）から求める。無ければ RAMP_MIN_STEP_FALLBACK_PCT。"""
        calib = self._profile.calibration
        if calib is not None:
            zero, full = (
                (calib.brake_zero_pos, calib.brake_full_pos)
                if on_brake
                else (calib.accel_zero_pos, calib.accel_full_pos)
            )
            span = abs(full - zero)
            if span > 0:
                return 100.0 / span
        return RAMP_MIN_STEP_FALLBACK_PCT

    def ramp_status(self) -> RampStatus | None:
        """表示用: G ランプ中（RAMP_HOLD を含む）の基準車速とペダル別の直近の Kp・Ki。
        G ランプ中でなければ None。"""
        if self._phase not in _RAMP_PHASES:
            return None
        acc, brk = self._rp_gains[PEDAL_ACCEL], self._rp_gains[PEDAL_BRAKE]
        return RampStatus(self._rp_vref, acc[0], acc[1], brk[0], brk[1])

    def _ramp_lagged_openings(self, now: float) -> tuple[float, float] | None:
        """遅れ gov_lead_s を見込んだ窓の実開度の平均 (アクセル, ブレーキ)。2 点未満なら None。"""
        cfg = self._config
        lo = now - cfg.gov_lead_s - cfg.coast_accel_slope_window_s
        hi = now - cfg.gov_lead_s
        rows = [(a, b) for t, a, b in self._rp_open_hist if lo <= t <= hi]
        if len(rows) < 2:
            return None
        return sum(a for a, _ in rows) / len(rows), sum(b for _, b in rows) / len(rows)

    def _ramp_deadband_kmh(self) -> float:
        """今の偏差の不感帯 [km/h]（2026-10-10 段2l-10）。設定値 `ramp_dev_deadband_kmh`。

        使わない所（0 を返す）: (1) RAMP_DOWN で基準車速が STOP_SPEED_KMH 未満（基準が 0 km/h に
        着いた後。使うと実車速 0.5 km/h 以下で踏み増さず止まらない）、(2) RAMP_UP で動き出しの
        置き直しの前（今の発進の動きを変えない）。
        """
        if self._phase is _Phase.RAMP_DOWN and self._rp_vref < STOP_SPEED_KMH:
            return 0.0
        if self._phase is _Phase.RAMP_UP and not self._rp_vref_anchored:
            return 0.0
        return max(0.0, self._config.ramp_dev_deadband_kmh)

    @staticmethod
    def _dead(error: float, deadband: float) -> float:
        """不感帯を引いた偏差 e′。|e| ≤ deadband なら 0、外側は e − deadband × sign(e)（連続）。"""
        if abs(error) <= deadband:
            return 0.0
        return error - math.copysign(deadband, error)

    def _ramp_actuator_lagging(
        self, on_brake: bool, actual_a: float, actual_b: float, new_u: float
    ) -> bool:
        """今使っているペダルの実開度が指令に追いつかず、さらに離れる向きに指令を動かすか
        （2026-10-10 段2l-10 ア）。

        指令 ＝ 今周期に出した指令（`_current_*`。pedal_standby なら出力側と同じく待機位置 min の
        下限を掛ける）、実開度 ＝ 引数。|指令 − 実開度| が `ramp_actuator_lag_tol_pct`
        を超え、かつ new_u が作る次の指令が実開度からさらに離れる向きのとき True。実開度が読めて
        いない（指令値にフォールバックしている）ときは False。
        """
        actual = self._brake_actual_pct if on_brake else self._accel_actual_pct
        if actual is None:
            return False
        ff = self._profile.feedforward_params
        floor = ff.brake_deadband_pct if on_brake else ff.accel_deadband_pct
        cmd = self._current_brake_opening if on_brake else self._current_accel_opening
        if on_brake:
            nxt = floor - min(new_u, 0.0)
        else:
            nxt = floor + new_u if new_u > 0.0 else 0.0
        if self._config.pedal_standby:
            cmd, nxt = max(cmd, floor), max(nxt, floor)
        measured = actual_b if on_brake else actual_a
        lag, lag_next = cmd - measured, nxt - measured
        return (
            abs(lag) > self._config.ramp_actuator_lag_tol_pct
            and lag * lag_next > 0.0
            and abs(lag_next) > abs(lag)
        )

    def _ramp_control(
        self,
        pattern: GRampPattern,
        speed: float,
        now: float,
        actual_a: float,
        actual_b: float,
        actual_accel_kmhs: float | None = None,
    ) -> None:
        """毎周期、基準車速 v_ref を進め、偏差 e ＝ v_ref − 実車速 から u を動かす
        （2026-10-09 段2l-8。2026-10-10 段2l-10 で表の項を削除し不感帯を足した）。

        Δu ＝ Kp × (e′ − e′_前) ＋ (Kp ÷ Ti) × e′ × dt ＋ 寄せ。アクセルもブレーキも、踏む側も
        離す側もこの式だけ（e′ が正なら u を増やす ＝ アクセルを踏む・ブレーキを緩める）。
        - e′ ＝ 不感帯を引いた偏差（`_dead`。|e| ≤ ramp_dev_deadband_kmh なら 0、外側は不感帯ぶん
          内側へ寄せた値で、境目で連続）。使わない所は `_ramp_deadband_kmh`。e′_前 は前周期の生の
          e を今周期の不感帯で引き直すので、不感帯の有無が切り替わる周期に開度が飛ばない。基準の
          引き戻し・cap・`blocked` の「広がらない」判定は生の e のまま。
        - Kp ＝ 1 ÷（感度 × T）。感度は今使っているペダルの走行中の実測
          （`sensitivity_kmhs_per_pct`）。感度が無い（None・0 以下）ときは Kp・Ki を使わず、
          e′ の向きへ最小刻み 1 個 / 周期だけ動かす（e′ ＝ 0 なら動かさない）。
          2026-10-10 段2l-11: 感度は「傾き × 点の開度の幅」が ramp_dev_deadband_kmh ÷
          ramp_settle_time_s 未満の候補を採らない（`min_accel_span_kmhs`。点が近すぎて傾きが 0 に
          近い正の値になり Kp が跳ねるのを防ぐ。不感帯を使わない所でも設定値をそのまま使う）。
        - 停止の直前（段2l-11）: RAMP_DOWN で基準が 0 km/h に着き、実車速が STOP_SPEED_KMH より
          上で、感度があるとき（Kp・Ki を使う周期）は、偏差の項のブレーキを踏み増す向きの大きさを
          最小刻み 1 個 / 周期より小さくしない（偏差が小さいと踏み増しが 0.01 %/秒まで落ちて
          停車まで 20 秒以上かかったため）。ガバナー・上下限はその後に今までどおり。
        - 表（段2l-10 で毎周期の変化の項 Δu_表 は削除）は 1 回だけ引いて、決めた時間で等速に
          足す「寄せ」にだけ使う（`_ramp_blend_start`）。
          発進: RAMP_UP でアクセルに移った周期に 1 回、その車速で段の G（pattern.target_g）が
          出る表の開度と今の u の差（増やす向きだけ）を `_ramp_launch_blend_time_s`（段の G で
          決まる。弱い段 0.5 秒・0.35G で 2 秒）かけて
          （2026-10-10 段2l-11。旧: その周期の g_ref へ全段 0.5 秒。立ち上げの途中で
          アクセルに移っても寄せ先が低くならない）。保持の頭・減速の頭は `_ramp_switch_hold`・
          `_ramp_switch_down`。表が引けないときは寄せない。区間が変わったら寄せ残りは捨てる。
          減速の頭のブレーキ側の寄せ（2 つ目。段2l-11）は、1 つ目の寄せが終わった周期か u が
          0 以下になった周期の早いほうで、その周期の u からの差を決める（ここの末尾）。
        - u が上下限で頭打ちになった、ガバナーで踏み増しを止められた、またはアクチュエータが指令に
          追いつかず指令をさらに離す向きに動かす周期（`_ramp_actuator_lagging`。u は止めない）は、
          偏差がそれ以上広がらないよう v_ref を実車速側へ引き戻す（止めが外れたときに踏みすぎない。
          クリープで狙いより速く加速する区間もこれで待つ）。
        - 動き出しの置き直し（2026-10-09）: RAMP_UP で、実車速が初めて STOP_SPEED_KMH を超えた周期に
          1 回だけ v_ref ＝ 実車速にする（段ごと。`_rp_vref_anchored`）。その周期は偏差の項が 0
          （開度を動かさない）。それまでは従来どおり v_ref を進め、偏差でブレーキを離す。
        - 順番（2026-10-09 段2l-9）: 置き直しの判定（`_rp_launch_started` ＝ now を記録）→ g_ref
          （`_ramp_gref`。置き直しの周期は g_min）→ 基準を進める（置き直しの周期は進めず実車速）→
          偏差の項 → 寄せ。
        - 片側調整（2026-10-12 段2l-12。`_ramp_one_sided_dir` が None でない周期）: 基準を進めた
          後、行き過ぎ（d × e ＜ 0。d ＝ RAMP_UP +1・RAMP_DOWN -1）なら v_ref ＝ 実車速・e_前 ＝ 0
          に置き直す。偏差の項は順方向のぶんだけ使う（du_err ＝ d × max(0, d × du_err)。感度なしの
          最小刻みも逆方向なら 0）。実測の加速度（引数。測れていなければ None）の順方向の G が
          上端（`_ramp_upper_g`）を超えた周期は、偏差の項の代わりに
          du_err ＝ -d × Kp × (d × 実測の加速度 − 狙いの加速度) × dt（感度なしは -d × 最小刻み。
          dt ≤ 0 なら 0）。寄せ・ガバナー・上下限・`blocked`・cap・停止の直前の下限は今のまま。
        - 発進の寄せの間（2026-10-12 段2l-13。RAMP_UP でアクセルに移った後、寄せ残りがある周期）は
          v_ref ＝ 実車速・e_前 ＝ 0（偏差の項は 0。上端超えの戻しだけ残る）。寄せが終わった周期
          から基準を実車速から進める。寄せが無い（表が引けない・差が 0 以下）段は今までどおり。
        - 感度が無い（最小刻みの）周期は、u を動かしたあと |e| が cap ＝ 狙いの G × T を超えたら
          v_ref を実車速から cap の位置へ引き戻す（基準が車より T 秒ぶんより先に行かない）。
        """
        cfg = self._config
        ctl_t = self._rp_ctl_t
        self._rp_ctl_t = now
        # 周期が大きく止まった（通信待ちなど）ときに、基準車速や積分が一気に進まないよう頭打ち
        dt = 0.0 if ctl_t is None else min(max(0.0, now - ctl_t), 10.0 * self._interval_s)
        # 動き出しの置き直し（段ごとに 1 回）: ブレーキを離している間に基準だけが先へ進むので、
        # 実車速が初めて STOP_SPEED_KMH を超えた周期に基準を実車速へ置く
        # （その周期は開度を動かさない）。
        # 発進の g_ref（0 → +G）の数え始めもここ（g_ref より先に判定する。段2l-9）
        anchoring = (
            self._phase is _Phase.RAMP_UP
            and not self._rp_vref_anchored
            and speed > STOP_SPEED_KMH
        )
        if anchoring:
            self._rp_vref_anchored = True
            self._rp_launch_started = now
            self._rp_e_prev = 0.0
        g_ref = self._ramp_gref(pattern, now)
        if anchoring:
            self._rp_vref = speed
        else:
            self._rp_vref = max(0.0, self._rp_vref + g_ref * G_TO_KMHS * dt)
        # 片側調整（2026-10-12 段2l-12）。置き直しの周期そのものは今の動き（開度を動かさない）
        side = None if anchoring else self._ramp_one_sided_dir(pattern)
        reset_e_prev = False
        if side is not None and side * (self._rp_vref - speed) < 0.0:
            # 行き過ぎ（RAMP_UP で実車速 ＞ 基準、RAMP_DOWN で実車速 ＜ 基準）: 基準を実車速へ
            self._rp_vref = speed
            reset_e_prev = True
        # 発進の寄せで表を引く車速（段の頭と、減速の終わりで基準が 0 に止まっている間は実車速）
        table_speed = self._rp_vref if self._rp_vref >= STOP_SPEED_KMH else speed

        # 発進の寄せ。最初から u ≥ 0 で始まった段は最初の制御周期に決める（増やす向きだけ）
        if (
            ctl_t is None and self._phase is _Phase.RAMP_UP and self._rp_accel_reached
            and not self._rp_blend_decided
        ):
            self._ramp_blend_start(
                self._ramp_estimate_u(table_speed, pattern.target_g), self._rp_u,
                self._ramp_launch_blend_time_s(pattern), increase_only=True,
            )
        # 発進の寄せの間は基準を実車速に合わせておく（2026-10-12 段2l-13。ユーザー決定）。
        # 寄せだけで G を立ち上げ、寄せが終わった周期から基準を実車速から進める
        # （実機 060150 の 0.35G の段: ブレーキを離す間に基準が 14 km/h 先へ進み、偏差の項が
        # 寄せに上乗せされて G が設計の約 2 倍の速さで立ち上がり、ガバナーの先読みが働いた）
        if (
            self._phase is _Phase.RAMP_UP and self._rp_accel_reached and not anchoring
            and self._rp_blend_left != 0.0
        ):
            self._rp_vref = speed
            reset_e_prev = True

        e = self._rp_vref - speed
        e_prev = e if self._rp_e_prev is None or reset_e_prev else self._rp_e_prev
        deadband = self._ramp_deadband_kmh()
        e_dead = self._dead(e, deadband)
        e_dead_prev = self._dead(e_prev, deadband)
        du_blend = self._ramp_blend_step(dt)

        # 偏差の項（Kp・Ki は今使っているペダルの感度から）
        on_brake = self._ramp_on_brake()
        pedal = PEDAL_BRAKE if on_brake else PEDAL_ACCEL
        ff = self._profile.feedforward_params
        # 2026-10-10 段2l-11: 傾きを作った点の加速度の差が「不感帯 ÷ T」未満なら感度なし
        min_span = (
            cfg.ramp_dev_deadband_kmh / cfg.ramp_settle_time_s
            if cfg.ramp_settle_time_s > 0.0 else 0.0
        )
        sens = self.glimit.sensitivity_kmhs_per_pct(
            pedal, speed,
            actual_b if on_brake else actual_a,
            ff.brake_deadband_pct if on_brake else ff.accel_deadband_pct,
            min_accel_span_kmhs=min_span,
        )
        no_sens = not (sens is not None and sens > 0.0)
        if not no_sens:
            kp = 1.0 / (sens * cfg.ramp_settle_time_s)
            ki = kp / (RAMP_TI_RATIO * cfg.ramp_settle_time_s)
            du_err = kp * (e_dead - e_dead_prev) + ki * e_dead * dt
            if (
                self._phase is _Phase.RAMP_DOWN and self._rp_vref < STOP_SPEED_KMH
                and speed > STOP_SPEED_KMH and dt > 0.0
            ):
                # 停止の直前（段2l-11）: ブレーキを踏み増す向き（u が減る向き）を最小刻みより
                # 遅くしない
                du_err = min(du_err, -self._ramp_min_step_pct(on_brake))
        else:
            kp = ki = 0.0
            step = self._ramp_min_step_pct(on_brake)
            du_err = 0.0 if e_dead == 0.0 or dt <= 0.0 else math.copysign(step, e_dead)
        if side is not None:
            # 順方向のぶんだけ使う（逆方向は 0）
            du_err = side * max(0.0, side * du_err)
            upper = self._ramp_upper_g(pattern)
            if (
                actual_accel_kmhs is not None and upper is not None
                and side * actual_accel_kmhs / G_TO_KMHS > upper
            ):
                # 実測の G が段の上端を超えた: 逆方向へ戻す（Kp ×（実測 − 狙い）× dt。感度なしは
                # 最小刻み 1 個）
                if dt <= 0.0:
                    du_err = 0.0
                elif no_sens:
                    du_err = -side * self._ramp_min_step_pct(on_brake)
                else:
                    over = side * actual_accel_kmhs - pattern.target_g * G_TO_KMHS
                    du_err = -side * kp * over * dt
        self._rp_gains[pedal] = (kp, ki)

        u = self._rp_u
        new_u = u + du_err + du_blend
        blocked = False
        # ガバナーで削られている間は、削られているペダルをさらに踏み増す向きへ積まない
        # （ラッチ後は u が減る向きが踏み増し。それ以外は |u| が増える向き）
        if self._phase is _Phase.RAMP_DOWN and self._rp_brake_latched:
            pressing_more = new_u < u
        else:
            pressing_more = abs(new_u) > abs(u)
        if self._governor_limiting() and pressing_more:
            new_u = u
            blocked = True
        lo, hi = self._ramp_u_limits()
        clamped = min(hi, max(lo, new_u))
        if clamped != new_u:
            blocked = True
        self._rp_u = clamped
        # アクチュエータが指令に追いつかない間は、開度 u は止めず基準だけ待たせる（段2l-10 ア）
        if self._ramp_actuator_lagging(on_brake, actual_a, actual_b, clamped):
            blocked = True
        if blocked and abs(e) > abs(e_prev):
            # 偏差が前周期より広がらないよう基準を実車速側へ戻す
            self._rp_vref = max(0.0, speed + math.copysign(abs(e_prev), e))
            e = self._rp_vref - speed
        if no_sens:
            # 弱く踏んでいる間は開度がほとんど動かず、基準だけが先に進む。感度が取れて Kp・Ki に
            # 切り替わったとき、Ki が残った偏差を積む速さは Ki×cap ＝（狙いの G に要る開度）÷(4T)
            # [%/s] になり、車に依らない穏やかな速さに収まる。cap は T（ramp_settle_time_s）と
            # 狙いの G から決まり、車に依存する値ではない
            cap = pattern.target_g * G_TO_KMHS * cfg.ramp_settle_time_s
            if abs(e) > cap:
                self._rp_vref = max(0.0, speed + math.copysign(cap, e))
                e = self._rp_vref - speed
        self._rp_e_prev = e

        if (
            self._phase in (_Phase.RAMP_UP, _Phase.RAMP_CRUISE, _Phase.RAMP_HOLD)
            and self._rp_u >= 0.0
        ):
            if (
                self._phase is _Phase.RAMP_UP
                and not self._rp_accel_reached
                and not self._rp_blend_decided
            ):
                self._ramp_blend_start(
                    self._ramp_estimate_u(table_speed, pattern.target_g), self._rp_u,
                    self._ramp_launch_blend_time_s(pattern), increase_only=True,
                )
            self._rp_accel_reached = True
        elif self._phase is _Phase.RAMP_DOWN and not self._rp_brake_latched and self._rp_u < 0.0:
            # ブレーキ側に入った。以後アクセルへ戻らない（上限 0）。u は連続のまま
            self._rp_brake_latched = True
        # アクセルを離す寄せ（u → 0）の残りはブレーキ側へ持ち込まない（2026-10-10 段2l-11。
        # 実機 114602 の 0.125G の段: 偏差の項も u を下げるので寄せの 2 秒より先に u が 0 に着き、
        # 残りの約 0.3 秒ぶん（約 15 %/秒）がそのままブレーキを 4.2 → 9.9% に踏んだ）
        if self._phase is _Phase.RAMP_DOWN and self._rp_blend_release and self._rp_u <= 0.0:
            self._rp_blend_release = False
            self._rp_blend_left = self._rp_blend_time_left = 0.0
        # 減速の頭の 2 つ目の寄せ（ブレーキ側。段ごとに 1 回）: 1 つ目の寄せが終わった周期か
        # u が 0 以下になった周期の早いほう。その周期の u からの差を寄せる（段2l-11）
        if (
            self._phase is _Phase.RAMP_DOWN and self._rp_down_brake_blend_pending
            and (self._rp_blend_left == 0.0 or self._rp_u <= 0.0)
        ):
            self._rp_down_brake_blend_pending = False
            target = self._ramp_descent_brake_target(pattern, speed)
            if target is not None and target < self._rp_u:  # ブレーキを緩める向きには寄せない
                self._ramp_blend_start(target, self._rp_u, cfg.ramp_turn_time_s)

    def _ramp_launch_blend_time_s(self, pattern: GRampPattern) -> float:
        """発進の寄せの時間 [s]（2026-10-10 段2l-11。ユーザー決定）。

        (G − g_min) ÷ ramp_launch_g_rate_gps、ただし ramp_launch_blend_min_s より短くしない
        （g_min ＝ min(ramp_edge_g, G) ＝ 発進の G）。どの段でも G の上がる速さが同じになり、
        弱い段は短く（0.5 秒）、強い段は長く（0.35G で 2 秒）なる。実機 063905 で、0.5 秒の寄せは
        0.08G の段では問題が無く、0.25G・0.35G の段でだけガバナーの先読みが働いたため。
        rate が 0 以下なら全段 ramp_turn_time_s。
        """
        cfg = self._config
        if cfg.ramp_launch_g_rate_gps <= 0.0:
            return cfg.ramp_turn_time_s
        g = pattern.target_g
        g_min = min(max(0.0, cfg.ramp_edge_g), g)
        return max(cfg.ramp_launch_blend_min_s, (g - g_min) / cfg.ramp_launch_g_rate_gps)

    def _ramp_blend_start(
        self, target_u: float | None, u: float, time_s: float, *, increase_only: bool = False
    ) -> None:
        """表の開度への寄せを決める（2026-10-10 段2l-10 で一般化）。

        差 gap ＝ target_u − u（符号つき。`increase_only` なら増やす向きだけ ＝ max(0, gap)）を
        `time_s` かけて毎周期 gap × dt ÷ time_s ずつ、残りを超えない範囲で足す
        （`_ramp_blend_step`）。time_s ≤ 0 なら次の制御周期に全部。target_u が None なら寄せない。
        前の寄せ残りは捨てる。
        """
        self._rp_blend_decided = True
        self._rp_blend_release = False
        self._rp_blend_gap = self._rp_blend_left = 0.0
        self._rp_blend_time = self._rp_blend_time_left = 0.0
        if target_u is None:
            return
        gap = target_u - u
        if increase_only:
            gap = max(0.0, gap)
        if gap == 0.0:
            return
        self._rp_blend_gap = self._rp_blend_left = gap
        self._rp_blend_time = self._rp_blend_time_left = max(0.0, time_s)

    def _ramp_blend_step(self, dt: float) -> float:
        """この周期に u へ足す寄せの量（符号つき）。残りを超えない。時間が尽きたら残りは捨てる。"""
        left = self._rp_blend_left
        if left == 0.0:
            return 0.0
        if self._rp_blend_time <= 0.0:
            self._rp_blend_left = 0.0
            return left
        step = self._rp_blend_gap * dt / self._rp_blend_time
        du = min(left, step) if left > 0.0 else max(left, step)
        self._rp_blend_left -= du
        self._rp_blend_time_left -= dt
        if self._rp_blend_time_left <= 0.0:
            self._rp_blend_left = 0.0
        return du

    def _step_ramp(self, pattern: GRampPattern, speed: float, now: float) -> float | None:
        """毎周期、基準車速への追従で u を動かし（`_ramp_control`）、直近 slope_window_s の
        加速度 G を測る。戻り値は実測 G（窓が溜まる前は None）。

        実測 G は結果の表と、上限 G の予測マップ（アクセル側・ブレーキ側それぞれ。
        `_step_coast_accel` と同じ）に点を足すのに使う。2026-10-12 段2l-12: 傾きは
        `_ramp_control` の前に計算して渡し、片側調整の戻す条件にも使う。
        """
        cfg = self._config
        hist = self._rp_hist
        hist.append((now, speed))
        while hist and now - hist[0][0] > cfg.coast_accel_slope_window_s:
            hist.popleft()
        actual_a = self._accel_actual_pct if self._accel_actual_pct is not None else (
            self._current_accel_opening
        )
        actual_b = self._brake_actual_pct if self._brake_actual_pct is not None else (
            self._current_brake_opening
        )
        self._rp_open_hist.append((now, actual_a, actual_b))
        if self._phase is not _Phase.HOLD_STEP:  # 2026-10-16 段2o-6: 効き始めの折れ点用
            self._onset_buf.append((now, speed, actual_a, actual_b))
        max_age = cfg.coast_accel_slope_window_s + cfg.gov_lead_s + 0.5
        while self._rp_open_hist and now - self._rp_open_hist[0][0] > max_age:
            self._rp_open_hist.popleft()
        last_t = self._rp_last_t
        self._rp_last_t = now
        # 実測の加速度（窓が溜まる前は None）。2026-10-12 段2l-12: 片側調整の戻す条件に使うので
        # `_ramp_control` の前に計算する
        accel_kmhs: float | None = None
        if last_t is not None and len(hist) >= 3:
            accel_kmhs = fit_slope([t for t, _ in hist], [v for _, v in hist])
        if self._rp_lr_active and self._ramp_launch_release_step(speed, now):
            # 段2o-10 A: 発進のゆっくり離しの間は偏差の項・基準を使わない（v_ref は 0 のまま）。
            # 過渡の点は上限 G マップにも結果の G にも入れない
            return None
        if self._phase in (_Phase.HOLD_STEP, _Phase.RAMP_RELEASE):
            # 2026-10-13 段2o-2: 遅れの測定のステップは開度を固定で持つ。偏差の項は使わず、
            # 過渡の点を上限 G マップに貯めない（`_rp_hist` の更新・加速度の測定は上でしている）。
            # 2026-10-14 段2o-5: 下りの戻し（RAMP_RELEASE）も同じ。戻り値が None なので実測 G の
            # 中央値（`_rp_down_g`）にも入らない
            return None if accel_kmhs is None else accel_kmhs / G_TO_KMHS
        self._ramp_control(pattern, speed, now, actual_a, actual_b, accel_kmhs)
        if accel_kmhs is None:
            return None
        measured_g = accel_kmhs / G_TO_KMHS
        lagged = self._ramp_lagged_openings(now)
        if lagged is not None and now - self._rp_record_t >= 1.0:
            lag_a, lag_b = lagged
            mean_v = sum(v for _, v in hist) / len(hist)
            # 2026-10-07 段2l-4: 両ペダルが待機位置にいるので「0 超・0.05 以下」では判定できない。
            # 2026-10-08 段2l-6: 待機位置が min なので、自分が min ＋ 幅 より上・反対側が
            # min ＋ 幅 以下
            # （PEDAL_OFF_TOL_PCT。実位置のずれで行が割れないよう幅を持たせる）
            ffp = self._profile.feedforward_params
            a_floor = ffp.accel_deadband_pct + PEDAL_OFF_TOL_PCT
            b_floor = ffp.brake_deadband_pct + PEDAL_OFF_TOL_PCT
            launching = self._phase in (_Phase.RAMP_UP, _Phase.RAMP_HOLD) and self._rp_u < 0.0
            if lag_a > a_floor and lag_b <= b_floor:
                self._rp_record_t = now
                self.glimit.add(PEDAL_ACCEL, mean_v, lag_a, accel_kmhs)
            elif lag_b > b_floor and lag_a <= a_floor:
                # 発進（ブレーキを離している間）は加速中でブレーキの減速は負になる。
                # 減速が出た点だけ貯める
                if not launching or -accel_kmhs > 0.0:
                    self._rp_record_t = now
                    self.glimit.add(PEDAL_BRAKE, mean_v, lag_b, -accel_kmhs)
        return measured_g

    def _advance_ramp(
        self, pattern: GRampPattern, speed: float, elapsed: float, now: float
    ) -> None:
        """G ランプの前進判定（RAMP_UP → RAMP_HOLD → RAMP_DOWN → 停車復帰）。

        RAMP_UP: 実車速が「上端 − 切り替え中に車が進むぶん」に着いたら RAMP_HOLD へ。
        切り替え中に進むぶん ＝ max(0, 実測の加速度 [km/h/s]) × ramp_turn_time_s ÷ 2（+G → 0 を
        直線で変える間に進む距離）。2026-10-10 段2l-11: 基準車速ではなく実車速で判定する（車は基準
        より遅れるので、基準で判定すると車が上端に着く前にアクセルを戻していた）。
        安全網: 実車速が上端（`top_kmh`。最高速側の安全のため
        accel_speed_cap − 加速度 × overspeed_lead_s でも頭打ち）に着いた、時間切れも RAMP_HOLD へ。
        RAMP_HOLD: 入ってから ramp_turn_time_s ＋ ramp_hold_time_s 経ったら RAMP_DOWN へ。安全網:
        実車速が上の上端を skip_consecutive_required 周期続けて超えたら RAMP_DOWN へ（基準は
        g_ref ＝ 0 で止まっているので通常は起きない）。時間切れは要らない（必ず終わる）。
        RAMP_DOWN: 車速が停車（`STOP_SPEED_KMH` 以下）になったら `_finish_pattern` へ渡す
        （2026-10-08 段2l-6: 旧 coast_down_stop_speed_kmh（5 km/h）から変更。0 km/h まで測る）。
        RAMP_UP・RAMP_DOWN は見込み秒数 × RAMP_LEG_TIMEOUT_FACTOR ＋ RAMP_LEG_TIMEOUT_MARGIN_S で
        打ち切り（狙いの G に届かないときの安全網）。実車速が最高速を超え続けたら全区間で停車復帰。
        """
        cfg = self._config
        measured_g = self._step_ramp(pattern, speed, now)
        self._rp_max_speed = max(self._rp_max_speed, speed)
        if measured_g is not None and elapsed >= cfg.coast_accel_slope_window_s:
            # 結果の表の G 中央値は RAMP_UP・RAMP_DOWN だけ（RAMP_HOLD の G は入れない）
            if self._phase is _Phase.RAMP_UP:
                self._rp_up_g.append(measured_g)
            elif self._phase is _Phase.RAMP_DOWN:
                self._rp_down_g.append(measured_g)
        if speed > self._profile.max_speed:
            self._skip_count += 1
        else:
            self._skip_count = 0
        if self._skip_count >= self._skip_required:
            self._ramp_record_result(pattern, now)
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        leg_s = max(pattern.top_kmh, 0.0) / (max(pattern.target_g, 1e-6) * G_TO_KMHS)  # 0 km/h まで
        timeout_s = leg_s * RAMP_LEG_TIMEOUT_FACTOR + RAMP_LEG_TIMEOUT_MARGIN_S
        hist = self._rp_hist
        # 測れていない（履歴が足りない。フェーズを入り直した直後など）ときは None
        measured_accel = (
            fit_slope([t for t, _ in hist], [v for _, v in hist]) if len(hist) >= 3 else None
        )
        if self._phase is _Phase.RAMP_CRUISE:
            self._advance_ramp_cruise(pattern, speed, now, measured_accel)
            return
        if self._phase is _Phase.HOLD_STEP:
            self._advance_hold_step(speed, now)
            return
        if self._phase is _Phase.RAMP_RELEASE:
            self._advance_ramp_release(speed, now)
            return
        leg_s_elapsed = self._ramp_leg_elapsed(elapsed)  # 保持を除いた通算（段2o-2）
        if self._phase is not _Phase.RAMP_DOWN:
            accel_kmhs = 0.0 if measured_accel is None else measured_accel
            exit_speed = min(
                pattern.top_kmh, self._accel_speed_cap - max(0.0, accel_kmhs) * cfg.overspeed_lead_s
            )
        if self._phase is _Phase.RAMP_UP:
            # 実車速で判定する（2026-10-10 段2l-11）。+G → 0 に変える間に車が進むぶん
            # （実測の加速度 × ramp_turn_time_s ÷ 2）を引いて、車速を上端でほぼ止める
            lead_kmh = max(0.0, accel_kmhs) * cfg.ramp_turn_time_s / 2.0
            # 実車速が上端の手前に着いたら保持へ。上端の頭打ちに着いたとき（安全網）・時間切れもここ
            if (
                speed >= exit_speed or leg_s_elapsed >= timeout_s
                or speed >= pattern.top_kmh - lead_kmh
            ):
                self._ramp_switch_hold(now, speed, measured_accel)
            else:
                self._ramp_cruise_try_enter(pattern, speed, now, measured_accel)
            return
        if self._phase is _Phase.RAMP_HOLD:
            # 保持の安全網（2026-10-10 段2l-11）: 実車速で切り替えるので車は上端の近くに着く。
            # 上端ちょうどを境目にすると少し越えただけで保持を打ち切るため、境目は
            # 「アクセルを戻さなかったら切り替えの間に進む車速」（G × ramp_turn_time_s ÷ 2 ＝ 狙いの
            # 行き過ぎの 2 倍ぶん。時間の設定だけで決まる）だけ上に置く。
            # 最高速の頭打ちは、保持の間は先読み（加速度 × overspeed_lead_s）を引かずそのものと
            # 比べる（2026-10-12 段2l-11。実機 114602 の 0.02G の段: 上端 136.3 ＝ 頭打ちで、
            # 135.6 km/h で保持に入った直後に「頭打ち − 先読み」を超え 0.3 秒で打ち切られた。
            # 保持は加速度が 0 へ向かう区間で、先読みは加速中（RAMP_UP）の行き過ぎを防ぐためのもの）
            hold_limit = min(
                pattern.top_kmh + pattern.target_g * G_TO_KMHS * cfg.ramp_turn_time_s / 2.0,
                self._accel_speed_cap,
            )
            self._rp_hold_over_count = self._rp_hold_over_count + 1 if speed > hold_limit else 0
            if pattern.lag_step and cfg.ramp_cruise_hold_s > 0.0:
                # 2026-10-13 段2o-2: 遅れの測定の段は、保持の出方を途中の保持と同じにする（実測の
                # 加速度が ±ramp_cruise_band_g 以内だった時間の合計が ramp_cruise_hold_s、または
                # ramp_turn_time_s ＋ ramp_cruise_timeout_s で打ち切り）。出たら HOLD_STEP へ
                dt = min(max(0.0, now - self._rp_cruise_last_t), 10.0 * self._interval_s)
                self._rp_cruise_last_t = now
                if (
                    measured_accel is not None
                    and abs(measured_accel) / G_TO_KMHS <= cfg.ramp_cruise_band_g
                ):
                    self._rp_cruise_in_band_s += dt
                if self._rp_hold_over_count >= self._skip_required:
                    self._ramp_switch_down(now, speed)
                elif (
                    self._rp_cruise_in_band_s >= cfg.ramp_cruise_hold_s
                    or elapsed >= cfg.ramp_turn_time_s + cfg.ramp_cruise_timeout_s
                ):
                    if not self._ramp_step_begin(speed, now):
                        self._ramp_switch_down(now, speed)  # ステップが作れない: 飛ばす
                return
            if (
                elapsed >= cfg.ramp_turn_time_s + cfg.ramp_hold_time_s
                or self._rp_hold_over_count >= self._skip_required
            ):
                self._ramp_switch_down(now, speed)
            return
        if speed <= STOP_SPEED_KMH or leg_s_elapsed >= timeout_s:
            self._ramp_record_result(pattern, now)
            self._finish_pattern(speed, now)
            return
        if self._ramp_release_try_enter(pattern, speed, now):
            return
        self._ramp_cruise_try_enter(pattern, speed, now, measured_accel)

    def _ramp_leg_elapsed(self, elapsed: float) -> float:
        """上り・下りの区間の時間切れに使う経過秒（2026-10-13 段2o-2）。`elapsed` はフェーズに
        入り直してからの秒数。上りは保持（RAMP_CRUISE）に使った時間を除き、下りは保持の前まで
        に走った時間を足す（下りは保持から戻るたびに `_enter_phase` で `elapsed` が 0 に戻る）。
        保持が無ければ `elapsed` のまま。"""
        if self._phase is _Phase.RAMP_UP:
            return elapsed - self._rp_up_cruise_s
        if self._phase is _Phase.RAMP_DOWN:
            return elapsed + self._rp_down_prior_s
        return elapsed

    def _ramp_account(self, now: float) -> None:
        """今の区間の経過秒を結果用の積算に足し、起点を now にする（2026-10-13 段2o-2）。
        区間（フェーズ）を切り替える前に呼ぶ。HOLD_STEP は保持（hold）に数える。"""
        key = {
            _Phase.RAMP_UP: "up", _Phase.RAMP_CRUISE: "cruise",
            _Phase.RAMP_HOLD: "hold", _Phase.HOLD_STEP: "hold", _Phase.RAMP_DOWN: "down",
            _Phase.RAMP_RELEASE: "release",
        }.get(self._phase)
        if key is not None:
            self._rp_acc[key] += max(0.0, now - self._rp_seg_t)
        self._rp_seg_t = now

    # ── 途中の定速保持（RAMP_CRUISE。2026-10-13 段2o-2）──────────────────
    def _ramp_cruise_try_enter(
        self, pattern: GRampPattern, speed: float, now: float, accel_kmhs: float | None
    ) -> None:
        """上り（RAMP_UP）・下り（RAMP_DOWN）の途中で、保持する車速に着いたら RAMP_CRUISE へ入る。

        上り: 実車速 ≥ 保持点 − max(0, 実測の加速度) × ramp_turn_time_s ÷ 2（`_advance_ramp` の
        上端の判定と同じ式）で、まだ上りで保持していない点。下り: 実車速 ≤ 保持点 ＋
        max(0, −実測の加速度) × ramp_turn_time_s ÷ 2 で、まだ下りで保持していない点。当てはまる点が
        複数あるときは車速にいちばん近い 1 点だけ保持し、残りはとばす（保持済みにする）。
        保持が無い（stations_kmh が空・ramp_cruise_hold_s が 0 以下）なら何もしない。
        2026-10-14 段2o-4: `stations_up_only` が True なら下りでは何もしない（上りだけ保持する）。
        """
        cfg = self._config
        stations = pattern.stations_kmh
        if not stations or cfg.ramp_cruise_hold_s <= 0.0:
            return
        accel = 0.0 if accel_kmhs is None else accel_kmhs
        up = self._phase is _Phase.RAMP_UP
        if not up and pattern.stations_up_only:
            return
        if up:
            lead = max(0.0, accel) * cfg.ramp_turn_time_s / 2.0
            done = self._rp_cruise_done_up
            cands = [i for i, st in enumerate(stations) if i not in done and speed >= st - lead]
            pick = max(cands) if cands else None
        else:
            lead = max(0.0, -accel) * cfg.ramp_turn_time_s / 2.0
            done = self._rp_cruise_done_down
            cands = [i for i, st in enumerate(stations) if i not in done and speed <= st + lead]
            pick = min(cands) if cands else None
        if pick is None:
            return
        done.update(cands)  # 保持する 1 点と、とばす点を保持済みにする
        self._ramp_cruise_enter(pattern, speed, now, accel_kmhs, 1 if up else -1)

    def _ramp_cruise_enter(
        self, pattern: GRampPattern, speed: float, now: float, accel_kmhs: float | None,
        direction: int,
    ) -> None:
        """RAMP_UP / RAMP_DOWN → RAMP_CRUISE。`_enter_phase` は通さない（`_rp_hist` を残し、
        切り替え直後も実測の加速度が測れるようにする。u・v_ref の連続性は `_ramp_switch_hold`
        と同じ）。

        g_ref の起点は入った時点の実測の加速度（符号つき。±G に丸める。測れていなければ今の
        g_ref）。v_ref ＝ 実車速・e_前 ＝ 0・寄せ残りを捨てる。u ≥ 0 のときだけ、その車速の 0G の
        アクセル開度（負なら 0 に丸める）へ ramp_turn_time_s で寄せる（ブレーキ側で入ったときは
        寄せず、偏差で緩める）。
        """
        cfg = self._config
        g = pattern.target_g
        if accel_kmhs is None:
            g0 = self._ramp_gref(pattern, now)  # 入る前のフェーズとしての値
        else:
            g0 = accel_kmhs / G_TO_KMHS
        self._rp_cruise_g0 = min(g, max(-g, g0))
        self._ramp_account(now)
        if direction < 0:  # 下りの保持の前までに走った時間（時間切れに足す）
            assert self._phase_started_at is not None
            self._rp_down_prior_s += max(0.0, now - self._phase_started_at)
        self._rp_blend_left = self._rp_blend_time_left = 0.0  # 寄せ残りは捨てる
        self._rp_down_brake_blend_pending = False
        self._rp_blend_release = False
        self._rp_vref = speed
        self._rp_e_prev = 0.0
        self._phase = _Phase.RAMP_CRUISE
        self._skip_count = 0
        self._rp_cruise_dir = direction
        self._rp_cruise_started = now
        self._rp_cruise_last_t = now
        self._rp_cruise_in_band_s = 0.0
        self._rp_cruise_n += 1
        if self._rp_u >= 0.0:
            target = self._ramp_estimate_u(speed, 0.0)
            self._ramp_blend_start(
                None if target is None else max(0.0, target), self._rp_u, cfg.ramp_turn_time_s
            )

    def _advance_ramp_cruise(
        self, pattern: GRampPattern, speed: float, now: float, accel_kmhs: float | None
    ) -> None:
        """RAMP_CRUISE の前進判定。実測の加速度が ±ramp_cruise_band_g 以内だった周期の時間を足し、
        合計が ramp_cruise_hold_s に達したら出る。入ってから ramp_turn_time_s ＋
        ramp_cruise_timeout_s で打ち切り。実測の加速度が測れない周期（None）は数えない。
        安全網: 実車速が最高速の頭打ちを超えた、上りで上端に着いた、下りで停車した、も出る。"""
        cfg = self._config
        dt = min(max(0.0, now - self._rp_cruise_last_t), 10.0 * self._interval_s)
        self._rp_cruise_last_t = now
        if accel_kmhs is not None and abs(accel_kmhs) / G_TO_KMHS <= cfg.ramp_cruise_band_g:
            self._rp_cruise_in_band_s += dt
        up = self._rp_cruise_dir > 0
        done = self._rp_cruise_in_band_s >= cfg.ramp_cruise_hold_s
        limit_s = cfg.ramp_turn_time_s + cfg.ramp_cruise_timeout_s
        timed_out = now - self._rp_cruise_started >= limit_s
        unsafe = (
            speed > self._accel_speed_cap
            or (up and speed >= pattern.top_kmh)
            or (not up and speed <= STOP_SPEED_KMH)
        )
        if done or timed_out or unsafe:
            self._ramp_cruise_exit(pattern, speed, now, done)

    def _ramp_cruise_exit(
        self, pattern: GRampPattern, speed: float, now: float, done: bool
    ) -> None:
        """RAMP_CRUISE → 入った方向へ戻る。上りは `_enter_phase` を通さずフェーズと g_ref の起点
        （0 → +G）だけ変え、v_ref ＝ 実車速・e_前 ＝ 0。下りは `_ramp_switch_down`。"""
        self._ramp_account(now)
        if done:
            self._rp_cruise_done_n += 1
        if self._rp_cruise_dir > 0:
            self._rp_up_cruise_s += max(0.0, now - self._rp_cruise_started)
            self._rp_blend_left = self._rp_blend_time_left = 0.0
            self._rp_vref = speed
            self._rp_e_prev = 0.0
            self._phase = _Phase.RAMP_UP
            self._skip_count = 0
            self._rp_resume_started = now
        else:
            self._ramp_switch_down(now, speed)

    def _ramp_switch_hold(
        self, now: float, speed: float, accel_kmhs: float | None = None
    ) -> None:
        """RAMP_UP → RAMP_HOLD（上端の手前。2026-10-10 段2l-10）。u は引き継ぐ。
        g_ref は入った時点の値（`_rp_hold_g0`）→ 0 を ramp_turn_time_s で直線。

        2026-10-10 段2l-11: 基準車速 v_ref ＝ 実車速・e_前 ＝ 0（発進の置き直しと
        同じ扱い。偏差の項は変化分の式なので開度は飛ばない）。`_rp_hold_g0` は実測の加速度
        `accel_kmhs` ÷ G_TO_KMHS を 0〜pattern.target_g に丸めた値（基準が車と同じ速さから 0 へ
        落ち着く）。測れていない（None）ときは今までどおり入った時点の g_ref。

        表の寄せ: その車速で 0G（定速）のアクセル開度（`_ramp_estimate_u(実車速, 0.0)`）へ
        ramp_turn_time_s かけて（負＝ブレーキ側になったら 0 に丸める。保持はアクセル側だけ）。
        表が引けないときは寄せない（偏差だけで戻す）。発進が終わらないまま来た（u ＜ 0）ときも
        寄せない（ブレーキを離している最中で、表の開度へ動かさない）。
        """
        self._ramp_account(now)
        self._rp_blend_left = self._rp_blend_time_left = 0.0  # 寄せ残りは捨てる
        pattern = self._patterns[self._pattern_idx]
        assert isinstance(pattern, GRampPattern)
        if accel_kmhs is None:
            self._rp_hold_g0 = self._ramp_gref(pattern, now)  # RAMP_UP としての値（enter 前に取る）
        else:
            self._rp_hold_g0 = min(pattern.target_g, max(0.0, accel_kmhs / G_TO_KMHS))
        self._rp_vref = speed
        self._rp_e_prev = 0.0
        self._rp_down_brake_blend_pending = False
        self._enter_phase(_Phase.RAMP_HOLD, now)
        self._rp_hold_started = now
        self._rp_hold_over_count = 0
        self._rp_cruise_in_band_s = 0.0  # 遅れの測定の段の保持の出方（段2o-2）
        self._rp_cruise_last_t = now
        if self._rp_u >= 0.0:
            target = self._ramp_estimate_u(speed, 0.0)
            self._ramp_blend_start(
                None if target is None else max(0.0, target), self._rp_u,
                self._config.ramp_turn_time_s,
            )

    # ── 下りの戻し（RAMP_RELEASE。2026-10-14 段2o-5）──────────────────────
    def _ramp_release_try_enter(self, pattern: GRampPattern, speed: float, now: float) -> bool:
        """下り（RAMP_DOWN）で、戻す車速に着いたら RAMP_RELEASE へ入る。入ったら True。

        実車速 ≤ まだ使っていない戻す車速（複数当てはまるときは車速にいちばん近い 1 点だけ
        使い、残りは飛ばした数に数えて済みにする）で、u ＜ 0（ブレーキを踏んでいる）かつ
        ラッチ後。u ≥ 0（惰行・アクセルで −G が出ている）、または着いた車速が安全網
        （ramp_release_min_kmh）以下なら、その点は飛ばす。戻しが無い（点が空・fracs が空）なら
        何もしない。
        """
        cfg = self._config
        if not cfg.ramp_release_enabled:  # 2026-10-16 段2o-6: 戻しなしの比較（③）
            return False
        stations = pattern.release_stations
        if not stations or not cfg.ramp_release_fracs:
            return False
        cands = [
            i for i, st in enumerate(stations)
            if i not in self._rp_release_done and speed <= st
        ]
        if not cands:
            return False
        self._rp_release_done.update(cands)
        pick_ok = (
            self._rp_brake_latched and self._rp_u < 0.0
            and speed > max(cfg.ramp_release_min_kmh, STOP_SPEED_KMH)
        )
        self._rp_release_skipped += len(cands) - (1 if pick_ok else 0)
        if not pick_ok:
            return False
        self._ramp_account(now)
        assert self._phase_started_at is not None
        self._rp_down_prior_s += max(0.0, now - self._phase_started_at)  # 時間切れから戻しを除く
        self._rp_blend_left = self._rp_blend_time_left = 0.0  # 寄せ残りは捨てる
        self._rp_down_brake_blend_pending = False
        self._rp_blend_release = False
        self._rp_release_u_peak = self._rp_u
        self._rp_release_t0 = now
        self._rp_release_n += 1
        self._phase = _Phase.RAMP_RELEASE  # `_enter_phase` は通さない（`_rp_hist` を残す）
        self._skip_count = 0
        return True

    def _ramp_release_total_s(self) -> float:
        """戻しの流れの全体の秒数 ＝ 移動 × 割合の数 ＋ 保持の合計。"""
        cfg = self._config
        return cfg.ramp_release_move_s * len(cfg.ramp_release_fracs) + sum(cfg.ramp_release_hold_s)

    def _ramp_release_u(self, t: float) -> float:
        """戻しに入ってから t 秒後の u。u_peak → f × u_peak を ramp_release_move_s で直線、
        ramp_release_hold_s だけ保持、を割合の数だけ繰り返す。"""
        cfg = self._config
        move = max(cfg.ramp_release_move_s, 1e-6)
        prev = self._rp_release_u_peak
        t_acc = 0.0
        # 2026-10-16 段2o-6: ブレーキの効き始め u_on があり、山がそれより深いときは、割合 f の目標を
        # 「効き始めと山の間」−u_on + f×(u_peak + u_on) にする（f=0 は待機位置のまま）。
        # 遊びの中で戻しが空振りしない。山が効き始めより浅い・u_on が無いときは従来の f×u_peak
        u_on = self._onset_u(PEDAL_BRAKE)
        peak = self._rp_release_u_peak
        use_onset = u_on is not None and abs(peak) > u_on
        for frac, hold in zip(cfg.ramp_release_fracs, cfg.ramp_release_hold_s, strict=False):
            if use_onset and frac > 0.0:
                target = -u_on + frac * (peak + u_on)
            else:
                target = frac * peak
            if t < t_acc + move:
                return prev + (target - prev) * max(0.0, t - t_acc) / move
            t_acc += move
            if t < t_acc + hold:
                return target
            t_acc += hold
            prev = target
        return prev

    def _advance_ramp_release(self, speed: float, now: float) -> None:
        """RAMP_RELEASE の前進。u を流れの時刻から決め（制御の項なし）、流れが終わるか、実車速が
        安全網（ramp_release_min_kmh ＝ クリープ ＋ 5。0 なら停車）以下になったら
        `_ramp_release_exit`。最高速超えの停車復帰は `_advance_ramp` の共通の判定が先。"""
        cfg = self._config
        t = max(0.0, now - self._rp_release_t0)
        if t >= self._ramp_release_total_s() or speed <= max(
            cfg.ramp_release_min_kmh, STOP_SPEED_KMH
        ):
            self._ramp_release_exit(speed, now)
            return
        self._rp_u = self._ramp_release_u(t)

    def _ramp_release_exit(self, speed: float, now: float) -> None:
        """RAMP_RELEASE → RAMP_DOWN。g_ref は 0 → −G を ramp_turn_time_s で直線（`_ramp_switch_down`
        の保持の後と同じ。戻しの間の g_ref は 0 なので `_rp_turn_g0` ＝ 0）、基準車速 v_ref ＝
        実車速、e_前 ＝ 0（取り返しに行かない）、制御の時刻 `_rp_ctl_t` は空に（戻しの間の長い
        間隔で基準・積分が進まない）。u は今の値（流れの最後は待機位置 0）のまま、RAMP_DOWN の
        片側調整（踏み増しの向きだけ）が踏み直す。ブレーキの領域なのでラッチは立てたまま
        （待機位置 ＝ ブレーキ min から 0% へ飛ばさない）。表の寄せは入れない。"""
        self._ramp_account(now)
        self._rp_turn_g0 = 0.0
        self._enter_phase(_Phase.RAMP_DOWN, now)
        self._rp_turn_started = now
        self._rp_vref = speed
        self._rp_e_prev = 0.0
        self._rp_ctl_t = None
        self._rp_brake_latched = True
        self._rp_down_brake_blend_pending = False
        # 2026-10-14 段2o-5（Claude 確認時の変更）: 踏み直しは、戻す直前に −G を出していた開度
        # u_peak へ ramp_turn_time_s で寄せる（g_ref の 0 → −G と同じ時間）。片側調整だけだと
        # スタブで狙いの G の ±20% に戻るまで 3.4〜5.8 秒かかり、下りの −G の時間が減ったため。
        # 値はその車が直前に実測で出していた開度なので車に依存しない。深くする向きだけ寄せる
        self._ramp_blend_start(
            self._rp_release_u_peak, self._rp_u, self._config.ramp_turn_time_s,
        )
        if self._rp_blend_left > 0.0:  # 浅くする向き（u_peak が今より浅い）は寄せない
            self._rp_blend_left = self._rp_blend_time_left = 0.0

    # ── 遅れの測定のステップ（HOLD_STEP。2026-10-13 段2o-2）──────────────
    def _ramp_step_targets(self, speed: float, u_hold: float) -> list[float] | None:
        """HOLD_STEP の開度 u の列（5 つ）。アクセルの踏み増しの大きさが決まらなければ None
        （この段は飛ばす）。ブレーキの大きさが決まらなければ 3 つだけ（ブレーキの 2 つを飛ばす）。

        1: u_hold ＋ Δa（Δa ＝ `_ramp_estimate_u(車速, +ramp_lag_step_g)` − u_hold。表が引けない
        か 0 以下なら ramp_lag_step_g ÷ 感度）、2: u_hold、3: 0（両ペダル待機位置）、
        4: −Δb（Δb ＝ `GLimitMap.interp_pct(ブレーキ, 車速, 惰行の減速 ＋ ramp_lag_step_g, min)` −
        min。引けなければ感度から）、5: 0。開度の上下限は最後に掛ける。

        2026-10-13 段2o-2 実装時の変更（計画からの差）: ブレーキの狙いの減速は「ramp_lag_step_g
        だけ」でなく「その車速の惰行の減速 ＋ ramp_lag_step_g」。表のブレーキの点は惰行を含む
        減速の合計なので、計画どおり ramp_lag_step_g だけで引くと、惰行の減速が 0.05G に近い
        車速（30 km/h など）ではブレーキがほぼ 0（スタブで min から 0.49%）になり、指令の変化の
        下限 1.0% に届かず測れない。惰行の減速が引けないときは ramp_lag_step_g だけ。
        """
        cfg = self._config
        ff = self._profile.feedforward_params
        kmhs = cfg.ramp_lag_step_g * G_TO_KMHS
        min_span = (
            cfg.ramp_dev_deadband_kmh / cfg.ramp_settle_time_s
            if cfg.ramp_settle_time_s > 0.0 else 0.0
        )
        actual_a = self._accel_actual_pct if self._accel_actual_pct is not None else (
            self._current_accel_opening
        )
        actual_b = self._brake_actual_pct if self._brake_actual_pct is not None else (
            self._current_brake_opening
        )
        est = self._ramp_estimate_u(speed, cfg.ramp_lag_step_g)
        delta_a = None if est is None else est - u_hold
        sens = self.glimit.sensitivity_kmhs_per_pct(
            PEDAL_ACCEL, speed, actual_a, ff.accel_deadband_pct, min_accel_span_kmhs=min_span
        )
        by_sens = kmhs / sens if sens is not None and sens > 0.0 else None
        if delta_a is None or delta_a <= 0.0:
            delta_a = by_sens
        elif by_sens is not None:
            # 表と感度の両方が引けるときは小さいほう（表の外れで一気に踏みすぎない）
            delta_a = min(delta_a, by_sens)
        if delta_a is None:
            return None
        lo = -max(0.0, self._profile.max_brake_opening - ff.brake_deadband_pct)
        hi = max(0.0, self._profile.max_accel_opening - ff.accel_deadband_pct)
        targets = [u_hold + delta_a, u_hold, 0.0]
        # 2026-10-16 段2o-6: アクセルの起点 u_hold が効き始めより浅いと、遅れに遊びの通過が混ざる
        u_on_a = self._onset_u(PEDAL_ACCEL)
        if u_on_a is not None and u_hold < u_on_a:
            self.step_accel_below_onset_n += 1
            say(
                f"警告: アクセルのステップの起点 u={u_hold:.2f}% が効き始め u_on={u_on_a:.2f}% より"
                f"浅い（遅れに遊びを通る時間が混ざる。{self.step_accel_below_onset_n} 回目）"
            )
        coast = self.glimit.coast_decel_kmhs(speed)
        brake_kmhs = kmhs + (coast if coast is not None and coast > 0.0 else 0.0)
        opening = self.glimit.interp_pct(PEDAL_BRAKE, speed, brake_kmhs, ff.brake_deadband_pct)
        delta_b = None if opening is None else opening - ff.brake_deadband_pct
        sens = self.glimit.sensitivity_kmhs_per_pct(
            PEDAL_BRAKE, speed, actual_b, ff.brake_deadband_pct, min_accel_span_kmhs=min_span
        )
        # 感度は「1% あたりの減速の増え方」なので、足したいぶん（ramp_lag_step_g）だけで割る
        by_sens = kmhs / sens if sens is not None and sens > 0.0 else None
        if delta_b is None or delta_b <= 0.0:
            delta_b = by_sens
        elif by_sens is not None:
            delta_b = min(delta_b, by_sens)  # 表と感度の両方が引けるときは小さいほう
        if delta_b is not None:
            # 2026-10-16 段2o-6: ブレーキの効き始め u_on があれば、起点を待機位置でなく効き始めに
            # する（0 → −u_on は両方 min からの踏み出し。−u_on ↔ −(u_on+Δb) は遊びを通らない）
            u_on_b = self._onset_u(PEDAL_BRAKE)
            if u_on_b is None:
                targets += [-delta_b, 0.0]
            else:
                targets += [-u_on_b, -(u_on_b + delta_b), -u_on_b, 0.0]
        return [min(hi, max(lo, u)) for u in targets]

    def _ramp_step_begin(self, speed: float, now: float) -> bool:
        """RAMP_HOLD → HOLD_STEP。最初の開度（踏み増し）を入れる。ステップが作れなければ
        False（呼び出し側が `_ramp_switch_down` へ進む）。`_enter_phase` は通さない（`_rp_hist` と
        ガバナーの頭打ちを残す）。"""
        targets = self._ramp_step_targets(speed, self._rp_u)
        if targets is None:
            return False
        self._ramp_account(now)
        self._rp_step_targets = targets
        self._rp_step_idx = 0
        self._rp_step_t0 = now
        self._rp_blend_left = self._rp_blend_time_left = 0.0
        self._rp_down_brake_blend_pending = False
        self._rp_blend_release = False
        self._rp_u = targets[0]
        self._phase = _Phase.HOLD_STEP
        self._skip_count = 0
        return True

    def _advance_hold_step(self, speed: float, now: float) -> None:
        """HOLD_STEP の前進。1 つの開度を ramp_lag_step_hold_s 秒保持して次の開度へ、最後まで
        終わったら `_ramp_switch_down`。安全網: 実車速が最高速の頭打ちを続けて超えた、停車した
        （STOP_SPEED_KMH 以下）も `_ramp_switch_down`（最高速超えの停車復帰は `_advance_ramp`
        の共通の判定が先）。"""
        cfg = self._config
        hold = max(cfg.ramp_lag_step_hold_s, 1e-6)
        idx = int(max(0.0, now - self._rp_step_t0) / hold + 1e-9)
        self._rp_hold_over_count = (
            self._rp_hold_over_count + 1 if speed > self._accel_speed_cap else 0
        )
        if (
            idx >= len(self._rp_step_targets) or speed <= STOP_SPEED_KMH
            or self._rp_hold_over_count >= self._skip_required
        ):
            self._ramp_switch_down(now, speed)
            return
        if idx != self._rp_step_idx:
            self._rp_step_idx = idx
            self._rp_u = self._rp_step_targets[idx]

    def _ramp_descent_accel_target(self, pattern: GRampPattern, speed: float) -> float | None:
        """減速の頭の寄せ（アクセル側）の先。狙いの減速 G×G_TO_KMHS がその車速帯の惰行の減速より
        小さい（惰行より緩い）ときの `_ramp_estimate_u(実車速, −G)`。
        そうでない・引けないときは None。
        """
        coast = self.glimit.coast_decel_kmhs(speed)
        if coast is not None and pattern.target_g * G_TO_KMHS < coast:
            return self._ramp_estimate_u(speed, -pattern.target_g)
        return None

    def _ramp_descent_brake_target(self, pattern: GRampPattern, speed: float) -> float | None:
        """減速の頭の寄せ（ブレーキ側）の先 ＝ −(開度 − ブレーキの min)。

        自分の車速帯の中で内挿できるときだけ（`GLimitMap.interp_pct`）。他の帯の借用・外挿・
        いちばん効いた点は使わない。引けなければ None（偏差の項で踏む）。
        """
        ff = self._profile.feedforward_params
        opening = self.glimit.interp_pct(
            PEDAL_BRAKE, speed, pattern.target_g * G_TO_KMHS, ff.brake_deadband_pct
        )
        if opening is None:
            return None
        return min(0.0, -(opening - ff.brake_deadband_pct))

    def _ramp_switch_down(self, now: float, speed: float | None = None) -> None:
        """RAMP_HOLD → RAMP_DOWN（減速の頭）。u は引き継ぐ（見積もりの代入はしない。
        2026-10-09 段2l-8）。狙いの加速度 g_ref はここから ramp_turn_time_s かけて
        「入った時点の g_ref」→ −G（2026-10-09 段2l-9: その時点の g_ref を `_rp_turn_g0` に
        記録する。保持を終えた後は 0。安全網で保持の途中で来ても g_ref は飛ばない）。
        u が負（発進が終わらないまま来た）ならブレーキ側に入った印を立てる。

        2026-10-10 段2l-11: 基準車速 v_ref ＝ 実車速・e_前 ＝ 0（偏差の項は変化分の式なので開度は
        飛ばない）。

        表の寄せ（段2l-10。段2l-11 で 2 つに分けた）:
        1 つ目（この周期）: 狙いが惰行より緩くアクセル側の値が引けるとき
        （`_ramp_descent_accel_target`）はその値へ ramp_turn_time_s で 1 回（2 つ目は無し）。
        それ以外で u ＞ 0 なら u ＝ 0 へ
        （アクセルを離す）。u ≤ 0 なら 1 つ目は無し。
        2 つ目（ブレーキ側。段ごとに 1 回）は `_ramp_control` が、1 つ目の寄せが終わった周期か
        u が 0 以下になった周期の早いほうで始める（`_rp_down_brake_blend_pending`）。u ≤ 0 で
        入った段（発進が終わらないまま来た）は 2 つ目も無し。"""
        speed = self._last_speed if speed is None else speed
        self._ramp_account(now)
        if not self._rp_down_begun:  # 保持から戻るときは最初の 1 回の時刻を残す（段2o-2）
            self._rp_down_begun = True
            self._rp_down_started = now
        self._rp_blend_left = self._rp_blend_time_left = 0.0  # 寄せ残りは捨てる
        pattern = self._patterns[self._pattern_idx]
        assert isinstance(pattern, GRampPattern)
        self._rp_turn_g0 = self._ramp_gref(pattern, now)  # 保持としての値（enter 前に取る）
        self._enter_phase(_Phase.RAMP_DOWN, now)
        self._rp_turn_started = now
        self._rp_vref = speed
        self._rp_e_prev = 0.0
        self._rp_brake_latched = self._rp_u < 0.0
        self._rp_down_brake_blend_pending = False
        accel_target = self._ramp_descent_accel_target(pattern, speed)
        if accel_target is not None:
            self._ramp_blend_start(accel_target, self._rp_u, self._config.ramp_turn_time_s)
        elif self._rp_u > 0.0:
            self._ramp_blend_start(0.0, self._rp_u, self._config.ramp_turn_time_s)
            self._rp_blend_release = True  # u が 0 以下になったら残りを捨てる
            # 惰行より緩い狙い（アクセル側の値が引けなかった）はブレーキ側の寄せをしない
            coast = self.glimit.coast_decel_kmhs(speed)
            gentle = coast is not None and pattern.target_g * G_TO_KMHS < coast
            self._rp_down_brake_blend_pending = not gentle

    def _ramp_record_result(self, pattern: GRampPattern, now: float) -> None:
        """この G ランプの結果（所要秒数・実測 G の中央値・最高車速）を記録する。"""
        # 加速秒 ＝ RAMP_UP、保持秒 ＝ RAMP_HOLD、減速秒 ＝ RAMP_DOWN（2026-10-10 段2l-10）。
        # 2026-10-13 段2o-2: 途中の保持（RAMP_CRUISE）を挟むと差し引きでは出ないので、区間を切り
        # 替えるたびの積算（`_ramp_account`）にした。保持なしなら従来の差し引きと同じ値
        self._ramp_account(now)
        up_s, hold_s = self._rp_acc["up"], self._rp_acc["hold"]
        down_s, cruise_s = self._rp_acc["down"], self._rp_acc["cruise"]
        self._update_onsets()
        self.ramp_results.append(
            RampResult(
                pattern_idx=self._pattern_idx,
                target_g=pattern.target_g,
                top_kmh=pattern.top_kmh,
                up_s=up_s,
                down_s=down_s,
                accel_g_median=statistics.median(self._rp_up_g) if self._rp_up_g else None,
                decel_g_median=-statistics.median(self._rp_down_g) if self._rp_down_g else None,
                max_speed_kmh=self._rp_max_speed,
                hold_s=hold_s,
                cruise_s=cruise_s,
                cruise_n=self._rp_cruise_n,
                cruise_done=self._rp_cruise_done_n,
                release_s=self._rp_acc["release"],
                release_n=self._rp_release_n,
                release_skipped=self._rp_release_skipped,
                accel_onset_pct=self._onset_pct.get(PEDAL_ACCEL),
                brake_onset_pct=self._onset_pct.get(PEDAL_BRAKE),
                step_accel_below_onset_n=self.step_accel_below_onset_n,
            )
        )

    def _onset_u(self, pedal: str) -> float | None:
        """効き始めの min からの量 u_on [%]。求まっていない・0 以下なら None（従来の動き）。"""
        onset = self._onset_pct.get(pedal)
        if onset is None:
            return None
        ff = self._profile.feedforward_params
        pedal_min = ff.accel_deadband_pct if pedal == PEDAL_ACCEL else ff.brake_deadband_pct
        u_on = onset - pedal_min
        return u_on if u_on > 0.0 else None

    def _update_onsets(self) -> None:
        """G ランプ 1 本が終わるたびに、それまでの全部で効き始めを求め直す（2026-10-16 段2o-6）。

        遅れは 0 として計算する（深い側＝確実に効く側に出る。ユーザー決定）。惰行は 2-1 で貯めた
        `GLimitMap` の惰行カーブ。求まったら次の段から使う（None のときは前の値を残す）。
        """
        speeds, decels = self.glimit.coast_curve()
        if not speeds or len(self._onset_buf) < 3:
            say("効き始め: 惰行カーブまたは時系列が足りず計算しない")
            self.onset_history.append((self._pattern_idx, self._onset_pct.get(PEDAL_ACCEL),
                                       self._onset_pct.get(PEDAL_BRAKE)))
            return
        arr = np.asarray(self._onset_buf, dtype=float)
        coast = coast_fn_from_curve(speeds, decels)
        ff = self._profile.feedforward_params
        amin, bmin = ff.accel_deadband_pct, ff.brake_deadband_pct
        for name, label, u, uo, pmin, omin, sign in (
            (PEDAL_ACCEL, "アクセル", arr[:, 2], arr[:, 3], amin, bmin, 1.0),
            (PEDAL_BRAKE, "ブレーキ", arr[:, 3], arr[:, 2], bmin, amin, -1.0),
        ):
            res = onset_from_series(
                arr[:, 0], arr[:, 1], u, uo,
                pedal_min_pct=pmin, other_min_pct=omin, coast_fn=coast, sign=sign,
            )  # fmt: skip
            if res.onset_pct is not None:
                self._onset_pct[name] = res.onset_pct
            onset_text = "なし" if res.onset_pct is None else f"{res.onset_pct:.2f}%"
            k_text = "-" if res.k is None else f"{res.k:.3f}"
            say(
                f"効き始め[{label}]: {onset_text}（min {pmin:.2f}%）  k={k_text}  "
                f"理由={res.reason or '-'}"
                + ("" if res.onset_pct is not None or name not in self._onset_pct
                   else f"  （前の値 {self._onset_pct[name]:.2f}% を使う）")
            )
        self.onset_history.append(
            (self._pattern_idx, self._onset_pct.get(PEDAL_ACCEL), self._onset_pct.get(PEDAL_BRAKE))
        )

    def _calib_brake_opening(self, now: float) -> float:
        """高い車速からの停車復帰の G 比例ブレーキ開度（ガバナー適用前）。

        最初は 0 から「min − approach_margin」まで brake_ramp_time_s で上げる（遊びの中では
        減速が出ない）。以降の開度は `_step_gprop_brake` が減速 G との差に応じて刻む。
        上限は停車保持開度（停車復帰で毎回踏んでいる実績のある開度）。
        """
        cfg = self._config
        if self._cb_t0 is None:
            ff = self._profile.feedforward_params
            self._cb_approach_pct = min(
                self._stop_return_brake_pct,
                max(0.0, ff.brake_deadband_pct - cfg.coast_accel_approach_margin_pct),
            )
            # コーストダウン・G ランプの実測があれば、その車速で目標 G が出る開度から始める
            # （無ければ min の手前から刻んで上げる）
            self._cb_start_pct = self._cb_approach_pct
            predicted = self._glimit_open_for(
                PEDAL_BRAKE, self._last_speed, cfg.coast_accel_target_g * G_TO_KMHS
            )
            if predicted is not None:
                self._cb_start_pct = min(
                    self._stop_return_brake_pct, max(self._cb_approach_pct, predicted)
                )
            self._cb_t0 = now
        elapsed = now - self._cb_t0
        if elapsed < cfg.brake_ramp_time_s:
            return self._cb_start_pct * self._ramp_fraction(elapsed, cfg.brake_ramp_time_s)
        if self._cb_next_t is None:  # 接近が済んだ。ここから G の刻みで踏み進める
            self._cb_opening = self._cb_start_pct
            self._cb_next_t = now + cfg.calib_dwell_s
        return self._cb_opening

    def _step_gprop_brake(self, speed: float, now: float) -> None:
        """G 比例ブレーキ減速の毎周期の処理（高い車速からの停車復帰）。

        dwell_s ごとに直近 slope_window_s の減速 G から開度を 1 刻み動かし、そのときの
        (開度, 減速) を上限 G の予測マップに足す。

        刻みは 0.5mm（約 0.5%）で、1 刻みの G の変化は 0.003〜0.007G と小さい。指令から G が
        出るまでの遅れがあっても、毎回の点は準定常とみなせる。
        """
        cfg = self._config
        hist = self._cb_hist
        hist.append((now, speed))
        while hist and now - hist[0][0] > cfg.coast_accel_slope_window_s:
            hist.popleft()
        if self._cb_next_t is None or now < self._cb_next_t or len(hist) < 3:
            return
        self._cb_next_t = now + cfg.calib_dwell_s
        decel_kmhs = -fit_slope([t for t, _ in hist], [v for _, v in hist])
        mean_v = sum(v for _, v in hist) / len(hist)
        # 記録するのは実際に出ていた開度（ガバナーで削られていればその値）
        self.glimit.add(PEDAL_BRAKE, mean_v, self._current_brake_opening, decel_kmhs)
        step_pct = cfg.calib_step_mm * 100.0 * 100.0 / ACTUATOR_PULSE_MAX
        press_below = (cfg.coast_accel_target_g - cfg.coast_accel_press_margin_g) * G_TO_KMHS
        if decel_kmhs > cfg.calib_release_above_g * G_TO_KMHS:
            self._cb_opening = max(self._cb_approach_pct, self._cb_opening - step_pct)
        elif decel_kmhs < press_below:
            self._cb_opening = min(self._stop_return_brake_pct, self._cb_opening + step_pct)

    def _glimit_open_for(
        self, pedal: str, speed_kmh: float, target_kmhs: float, *, extrapolate: bool = True
    ) -> float | None:
        """そのペダルが `target_kmhs`（正）を出すと予測される開度（測れていなければ None）。

        `extrapolate=False` は G ランプの最初の開度の見積もり用（狙いが貯めた点のどれより強い
        ときは、いちばん効いた点の開度を返す。2026-10-08 段2l-6）。
        """
        ff = self._profile.feedforward_params
        db = ff.brake_deadband_pct if pedal == PEDAL_BRAKE else ff.accel_deadband_pct
        return self.glimit.cap_pct(pedal, speed_kmh, target_kmhs, db, extrapolate=extrapolate)

    def _advance_coast(self, speed: float, accel_kmhs: float, elapsed: float, now: float) -> None:
        """コーストダウンの惰行の前進判定（段7c: 終了をクリープ平衡車速から決める）。

        以前は固定 `coast_down_stop_speed_kmh`（5.0）で終えていたが、クリープ平衡がそれより
        高い車両では惰行だけで届かず `coast_timeout_s`（90s）まで待ち、低い車両では惰行カーブの
        低速端のデータを取りこぼしていた。惰行は平衡へ漸近するだけでちょうど届く保証がないため、
        平衡以下に達するか、クリープの影響が残る車速（平衡 + `grid_settle_tol_kmh`）以下で
        車速の傾きが落ち着いたら終える（`_coast_creep_settled`）。`coast_down_stop_speed_kmh` は
        他の使い道（G ランプの減速の終了・G ランプなどの停車復帰の切替・超過回復）のまま残す。
        コーストダウンの停車復帰の境目は車ごと（`_stop_return_low_speed_kmh`。2026-10-08
        段2l-6b）で、クリープ域は停車保持開度まで徐々に踏む。
        """
        cfg = self._config
        # 終了判定専用の履歴（2026-10-10 段2l-11）: 毎周期足し、creep_launch_settle_s より
        # 古いものを捨てる。ただし窓の端をまたぐ最後の 1 点（窓ぶん以上前の点）は残す
        # （窓ぶん溜まっているかを `_coast_creep_settled` が先頭の古さで見るため）
        self._coast_hist.append((now, speed))
        while (
            len(self._coast_hist) >= 2
            and now - self._coast_hist[1][0] >= cfg.creep_launch_settle_s
        ):
            self._coast_hist.popleft()
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        if elapsed >= cfg.coast_measure_delay_s and accel_kmhs < 0.0:
            self.glimit.add_coast(speed, -accel_kmhs)  # 上限 G の予測の基準（開度=min の点）
        if elapsed >= cfg.coast_timeout_s:
            self._finish_pattern(speed, now)
            return
        creep = self._profile.feedforward_params.creep_speed_kmh
        if speed <= creep:
            self._finish_pattern(speed, now)
            return
        ceiling = self._creep_ceiling_kmh(cfg.grid_settle_tol_kmh)
        if speed <= ceiling and self._coast_creep_settled(now):
            self._finish_pattern(speed, now)

    def _coast_creep_settled(self, now: float) -> bool:
        """コーストダウンの惰行が、クリープ平衡近くで車速の傾きが落ち着いたか（段7c）。

        クリープ発進の平衡到達と同じ調整値（`creep_launch_settle_*`）で、判定専用の履歴
        `_coast_hist` の傾きを見る。

        2026-10-10 段2l-11: 以前は `_speed_hist` を使っていたが、これは `_smoothed_accel` が
        g_smoothing_window_s（0.4 秒）で刈るので 2 秒ぶんが溜まらず、判定が成立しなかった
        （スタブは 300 秒待ち、実機は 5 km/h ちょうどで終了）。
        """
        cfg = self._config
        window_s = cfg.creep_launch_settle_s
        recent = [(t, v) for t, v in self._coast_hist if t >= now - window_s]
        if len(recent) < 3 or not self._coast_hist or self._coast_hist[0][0] > now - window_s:
            return False
        slope = fit_slope([t for t, _ in recent], [v for _, v in recent])
        return abs(slope) < cfg.creep_launch_settle_kmhs

    def _advance_creep_launch(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, elapsed: float, now: float
    ) -> None:
        """クリープ発進・クリープ域ブレーキ保持（段1b）の前進判定。

        2026-09-17（ProblemReport_20260916 ユーザー決定）: 終了条件を「target_kmh 到達」から
        「平衡到達（加速が止まった）」に変えた。車速の傾きが creep_launch_settle_kmhs 未満の
        状態が creep_launch_settle_s 続いたら平衡到達とみなす（_advance_creep_settle と同じ
        流儀）。target_kmh は安全上限として残す（クリープでこれ以上の車速にはならないはずなので、
        達したら平衡を待たず打ち切る）。timeout_s は従来どおりの打ち切り。

        2026-09-17（誤判定修正）: 停車保持を解放した直後は車速 0・傾き 0 で
        abs(accel_kmhs) < creep_launch_settle_kmhs が最初から成立してしまい、車が動き出す前に
        「平衡到達」と誤判定して即終了していた（実機では解放から動き出しまで約 0.3〜0.4s、その
        間の傾きは 0.17 km/h/s 程度でしきい値 0.1 に近く、確実に誤判定する）。
        _advance_creep_settle の settled 判定（creep_settle_min_s による最短時間ガード）に
        揃えて、2 つのガードを足す:
          - 車速ゲート: speed が creep_launch_min_speed_kmh 以上になるまで安定カウントを
            積まない（動き出す前を「平衡」に含めない、本質的なガード）。
          - 最短時間ガード: elapsed が creep_launch_settle_min_s 以上になるまで終了しない。
        到達後は hold_after=True なら BRAKE_HOLD（brake_opening を保持して停車）、
        False なら通常の停車復帰（_finish_pattern）へ進む。
        """
        assert isinstance(pattern, CreepLaunchPattern)
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        moving = speed >= cfg.creep_launch_min_speed_kmh
        if moving and abs(accel_kmhs) < cfg.creep_launch_settle_kmhs:
            self._stable_count += 1
        else:
            self._stable_count = 0
        stable_long = self._stable_count * self._interval_s >= cfg.creep_launch_settle_s
        settled = elapsed >= cfg.creep_launch_settle_min_s and stable_long
        if settled or speed >= pattern.target_kmh or elapsed >= pattern.timeout_s:
            self._stable_count = 0
            if pattern.hold_after:
                self._enter_phase(_Phase.BRAKE_HOLD, now)
            else:
                self._finish_pattern(speed, now)

    def _advance_creep_settle(self, elapsed: float, accel_kmhs: float, now: float) -> None:
        cfg = self._config
        if abs(accel_kmhs) < cfg.creep_settle_stable_tol_kmhs:
            self._stable_count += 1
        else:
            self._stable_count = 0
        stable_long = self._stable_count * self._interval_s >= cfg.creep_settle_stable_duration_s
        settled = elapsed >= cfg.creep_settle_min_s and stable_long
        if settled or elapsed >= cfg.creep_settle_timeout_s:
            self._stable_count = 0
            self._advance_pattern(now)

    def _creep_press_applies(self) -> bool:
        """停車復帰を「徐々に踏む」にするか（段2l-6b）。コーストダウンで、超過回復でなく、
        踏む速さが決まる（calib_dwell_s > 0）とき。"""
        return (
            self._pattern_idx < len(self._patterns)
            and self._patterns[self._pattern_idx].kind is PatternKind.COAST_DOWN
            and not self._overspeed_recovery
            and self._stop_return_press_rate_pctps() is not None
        )

    def _stop_return_low_speed_kmh(self) -> float:
        """停車復帰で G 比例ブレーキからクリープ域の踏み方へ切り替える車速（段2l-6b）。

        コーストダウンはクリープ車速 + 落ち着きの許容幅（車ごと）。それ以外・超過回復は固定の
        `coast_down_stop_speed_kmh`。
        """
        cfg = self._config
        if (
            self._pattern_idx < len(self._patterns)
            and self._patterns[self._pattern_idx].kind is PatternKind.COAST_DOWN
            and not self._overspeed_recovery
        ):
            return self._creep_ceiling_kmh(cfg.grid_settle_tol_kmh)
        return cfg.coast_down_stop_speed_kmh

    def _stop_return_press_rate_pctps(self) -> float | None:
        """クリープ域で停車保持開度へ踏み上げる速さ [%/s]。`_step_gprop_brake` の 1 刻み/待ちと
        同じ。calib_dwell_s が 0 以下なら None（従来のランプに落とす）。"""
        cfg = self._config
        if cfg.calib_dwell_s <= 0.0:
            return None
        return cfg.calib_step_mm * 100.0 * 100.0 / ACTUATOR_PULSE_MAX / cfg.calib_dwell_s

    def _creep_ceiling_kmh(self, settle_tol_kmh: float) -> float:
        """クリープの影響が残る車速の上端 = クリープ平衡速度 + 落ち着きの許容幅。

        コーストダウンの終了判定（`_advance_coast`）が使う。
        """
        return self._profile.feedforward_params.creep_speed_kmh + settle_tol_kmh

    def _finish_pattern(self, speed: float, now: float) -> None:
        """運転パターンの計測が終わった。停車していなければ停車復帰（DRIVE_BRAKE）に入る（A6）。"""
        if speed > STOP_SPEED_KMH:
            self._enter_phase(_Phase.DRIVE_BRAKE, now)
        else:
            self._advance_pattern(now)

    def _advance_pattern(self, now: float) -> None:
        self._pattern_idx += 1
        self._enter_phase(self._initial_phase(self._pattern_idx), now)

    def _enter_phase(
        self, phase: _Phase, now: float, *, overspeed_recovery: bool = False
    ) -> None:
        self._phase = phase
        self._phase_started_at = now
        self._skip_count = 0
        self._stable_count = 0
        if phase is _Phase.COAST:
            self._coast_hist.clear()  # 段2l-11: コーストダウンの終了判定の履歴は入るたびに空から
        self._accel_gov_cap = None
        self._brake_gov_cap = None
        self._ca_t0 = None
        self._ca_last_t = None
        self._ca_hist.clear()
        self._ca_opening_hist.clear()  # 段7d: 前のパターンの開度を次のパターンに持ち込まない
        self._cb_t0 = None
        self._cb_next_t = None
        self._cb_hist.clear()
        # 停車復帰は、低速（クリープ域）からでなければ G 比例で減速する（段6b）。一気に停車保持
        # 開度へ踏むと、高い車速ではブレーキがよく効いて上限 G を超える（20260926: 134 km/h で
        # 0.30〜0.35G）
        self._overspeed_recovery = overspeed_recovery  # 下の停車復帰の判定が見る
        self._db_gprop = (
            phase is _Phase.DRIVE_BRAKE and self._last_speed > self._stop_return_low_speed_kmh()
        )
        # コーストダウンのクリープ域からの停車復帰は徐々に踏む（段2l-6b）
        self._db_creep_press = (
            phase is _Phase.DRIVE_BRAKE and not self._db_gprop and self._creep_press_applies()
        )
        self._db_press_reached_at = None
        self._db_stopped = False
        self._rp_hist.clear()
        self._rp_open_hist.clear()
        self._rp_last_t = None
        self._rp_record_t = -1e9
        if phase is _Phase.RAMP_UP:  # G ランプ 1 本の始まり: 状態を作り直す
            self._ramp_up_begin()  # 2026-10-09 段2l-8: 基準車速 0 km/h・u ＝ ブレーキを離す側から
            self._rp_up_started = now
            self._rp_hold_started = now
            self._rp_down_started = now
            self._rp_seg_t = now  # 結果用の積算の起点（段2o-2。`_ramp_up_begin` が積算を 0 にする）
            self._rp_max_speed = 0.0
            self._rp_up_g = []
            self._rp_down_g = []
        self._brake_ramp_from = self._current_brake_opening

    def _update_governor(self, accel_kmhs: float) -> None:
        """G ガバナー（段6b で先読みつきに）。加速（DRIVE_ACCEL）・ブレーキ（DRIVE_BRAKE・
        BRAKE_HOLD）・G ランプ（RAMP_UP・RAMP_DOWN）に適用する。
        """
        if self._g_limit_kmhs <= 0.0:
            return
        rate = self._accel_rate
        if self._phase in (_Phase.DRIVE_ACCEL,):
            self._accel_gov_cap = self._next_gov_cap(
                self._accel_gov_cap, accel_kmhs, rate, self._current_accel_opening,
                self._accel_request,
            )
        elif self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD):
            self._brake_gov_cap = self._next_gov_cap(
                self._brake_gov_cap, -accel_kmhs, -rate, self._current_brake_opening,
                self._brake_request,
            )
        elif self._phase in _RAMP_PHASES:
            # G ランプは今使っているペダルだけ頭打ちする
            accel_on, brake_on = self._accel_request > 0.0, self._brake_request > 0.0
            if accel_on:
                self._accel_gov_cap = self._next_gov_cap(
                    self._accel_gov_cap, accel_kmhs, rate, self._current_accel_opening,
                    self._accel_request,
                )
            if brake_on:
                self._brake_gov_cap = self._next_gov_cap(
                    self._brake_gov_cap, -accel_kmhs, -rate, self._current_brake_opening,
                    self._brake_request,
                )

    def _next_gov_cap(
        self, cap: float | None, pedal_accel_kmhs: float, pedal_rate: float, current: float,
        request: float,
    ) -> float | None:
        """頭打ちの次の値（先読みつき）。

        `pedal_accel_kmhs` はそのペダルが出す向きの加速度（ブレーキなら減速度）、`pedal_rate` はその
        増える速さ。見込み = 今 + 増える速さ × gov_lead_s（増えていないときは今のまま）。

        見込み or 今が上限以上 … 最初は現在開度で頭打ち、以降 1 周期ごとに
                                        gov_reduce_step_pct（0.1 秒周期での 1 周期ぶん。
                                        実周期に直した `_gov_reduce_step`）下げる
        見込みが上限 × gov_soft_frac 以上 … 踏み増しを止める（現在開度で頭打ち。すでにあれば保持）
        今が上限 × gov_release_frac 未満（かつ見込みも soft 未満）
                                        … gov_raise_step_pct（同じく実周期に直した
                                        `_gov_raise_step`）ずつ戻し、指令（request）に届いたら解除（A6）
        その間 … 保持
        """
        cfg = self._config
        limit = self._g_limit_kmhs
        predicted = pedal_accel_kmhs + max(0.0, pedal_rate) * cfg.gov_lead_s
        if predicted >= limit or pedal_accel_kmhs >= limit:
            if cap is None:
                return current
            return max(0.0, cap - self._gov_reduce_step)
        if predicted >= limit * cfg.gov_soft_frac:
            return current if cap is None else cap
        if cap is None:
            return None
        if pedal_accel_kmhs < limit * cfg.gov_release_frac:
            raised = cap + self._gov_raise_step
            return None if raised >= request else raised
        return cap

    def _governor_limiting(self) -> bool:
        """この周期の指令がガバナーで削られたか（CSV の governor_active 列）。"""
        if self._phase is _Phase.DRIVE_ACCEL and self._accel_gov_cap is not None:
            return self._accel_gov_cap < self._accel_request
        if self._phase in _RAMP_PHASES:
            accel_limited = (
                self._accel_gov_cap is not None and self._accel_gov_cap < self._accel_request
            )
            brake_limited = (
                self._brake_gov_cap is not None and self._brake_gov_cap < self._brake_request
            )
            return accel_limited or brake_limited
        braking = self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD)
        if braking and self._brake_gov_cap is not None:
            return self._brake_gov_cap < self._brake_request
        return False

    def _update_accel_rate(self, now: float, accel_kmhs: float) -> None:
        """加速度の変化の速さ（直近 gov_rate_window_s の最小二乗の傾き）。先読みガバナー用。"""
        hist = self._accel_hist
        hist.append((now, accel_kmhs))
        while hist and now - hist[0][0] > self._config.gov_rate_window_s:
            hist.popleft()
        self._accel_rate = (
            fit_slope([t for t, _ in hist], [a for _, a in hist]) if len(hist) >= 3 else 0.0
        )

    def _smoothed_accel(self, now: float) -> float:
        hist = self._speed_hist
        win = self._config.g_smoothing_window_s
        while len(hist) >= 2 and now - hist[0][0] > win:
            hist.popleft()
        if len(hist) >= 2:
            t0, v0 = hist[0]
            t1, v1 = hist[-1]
            if t1 - t0 > 0.0:
                return (v1 - v0) / (t1 - t0)
        return self._measured_accel()

    def _measured_accel(self) -> float:
        if self._prev_speed is None or self._prev_time is None:
            return 0.0
        cur = self._speed_hist[-1][1] if self._speed_hist else self._prev_speed
        dt = self._speed_hist[-1][0] - self._prev_time if self._speed_hist else 0.0
        if dt <= 0.0:
            return 0.0
        return (cur - self._prev_speed) / dt

    # ── 補助 ───────────────────────────────────────────────────────
    def _in_ramp(self) -> bool:
        """G ランプの区間か（手順3 と同じ送り方にする区間。RAMP_UP・RAMP_HOLD・RAMP_DOWN）。"""
        return self._phase in _RAMP_PHASES

    def _move_duration(self) -> float:
        if self._phase in (
            _Phase.DRIVE_ACCEL, _Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD,
            *_RAMP_PHASES,
        ):
            return self._config.pedal_step_time_s
        return self._config.pedal_release_time_s

    async def _abort_emergency(self, reason: str) -> None:
        self._running = False
        if self.abort_reason is None:
            self.abort_reason = reason
        await self._on_emergency()

    async def _drive_or_sense(
        self, accel_pos: int, brake_pos: int
    ) -> tuple[AxisMonitor, AxisMonitor]:
        """位置が変わった軸だけ指令し、両軸をまとめ読みする（A7。以前は電流だけ読んでいた）。"""
        dur = self._move_duration()
        a_changed = accel_pos != self._accel_pos_cmd
        b_changed = brake_pos != self._brake_pos_cmd

        async def accel_axis() -> AxisMonitor:
            if a_changed:
                return await self._drive_axis_timed(
                    self._accel_driver, accel_pos, self._accel_pos_cmd, dur
                )
            return await self._accel_driver.read_monitor()

        async def brake_axis() -> AxisMonitor:
            if b_changed:
                return await self._drive_axis_timed(
                    self._brake_driver, brake_pos, self._brake_pos_cmd, dur
                )
            return await self._brake_driver.read_monitor()

        a, b = await asyncio.gather(accel_axis(), brake_axis())
        if a_changed:
            self._accel_pos_cmd = accel_pos
        if b_changed:
            self._brake_pos_cmd = brake_pos
        return a, b

    async def _drive_axis_timed(
        self,
        driver: ActuatorDriverProtocol,
        target_pos: int,
        current_pos: int,
        duration_s: float,
    ) -> AxisMonitor:
        if self._in_ramp():
            # G ランプは手順3 と同じ送り方にする。「0.2 秒かけて動く速度」で送るのに次の指令が
            # 周期ごとに来て、指令が動き続ける間、実開度は指令の約半分の速さでしか動かなかった
            # （実機 041505。指令 60% のとき実開度 37%）。残りの距離を周期の 0.8 倍で動ききる
            # 加減速で送り、遅れた分も次の周期で取り返す。2026-10-10 段2l-10
            await driver.move_to_position(
                target_pos, smooth_over_s=self._interval_s * AXIS_SMOOTH_DUTY
            )
        else:
            await driver.move_to_position_timed(target_pos, current_pos, duration_s)
        # 書き込み直後の読み取りは応答を取りこぼすことがあるので、手順3 と同じく少し待つ
        await asyncio.sleep(AXIS_PRE_READ_DELAY_S)
        return await driver.read_monitor()


__all__ = [
    "AXIS_PRE_READ_DELAY_S",
    "AXIS_SMOOTH_DUTY",
    "PATTERN_LOOP_INTERVAL_S",
    "STOP_SPEED_KMH",
    "ActuatorDriverProtocol",
    "CANReaderProtocol",
    "OnSample",
    "PatternLoop",
    "CreepLaunchPattern",
    "GRampPattern",
    "RampResult",
    "PatternLoopConfig",
]
