# ruff: noqa: E501
"""走行・操作 API。段3（RPD_Phase_20261115_PortResearch）で JobManager（常駐ランナーへのコマンド）に作り替えた。

画面が使っているパスはできるだけ残し、意味を新しくした（一覧は src/robot/README.md の「Web API」の表）。
選んでいる車（``active``）は ``vehicles/.active``（VehicleStore）。ジョブはこの車で走る。
"""

from __future__ import annotations

from typing import Annotated, Any

from fastapi import APIRouter, Depends, HTTPException, Query, Request

from src.app.job_manager import (
    PURPOSE_CALIB,
    PURPOSE_MANUAL,
    JobManager,
    JobStatus,
    PlannedStep,
)
from src.infra.vehicle_store import VehicleStore
from src.models.system_state import RobotState
from src.robot.schedule_spec import check_schedule, schedule_from_dict, schedule_to_dict
from src.web.deps import (
    ModeRepoProtocol,
    ScheduleRepoProtocol,
    SequenceRepoProtocol,
    get_job_manager,
    get_mode_repo,
    get_schedule_repo,
    get_sequence_repo,
    get_vehicle_store,
)
from src.web.schemas import (
    AxisRequest,
    DriveLogResponse,
    JobStartResponse,
    JogRequest,
    JogResponse,
    LearningStartRequest,
    LogLineSchema,
    PrepareRequest,
    SaveMaxRequest,
    SaveMaxResponse,
    SaveMinRequest,
    SaveMinResponse,
    SelectProfileRequest,
    StartDriveRequest,
    StartScheduleRequest,
    StartSequenceRequest,
    SystemStateResponse,
)

router = APIRouter(prefix="/api/v1/drive", tags=["drive"])

Manager = Annotated[JobManager, Depends(get_job_manager)]
Store = Annotated[VehicleStore, Depends(get_vehicle_store)]

#: 開度 100% = 9500 pulse（core の開度の定義）。旧 API のジョグ量（pulse）を % へ直す
PULSE_FULL = 9500.0


def _job_response(status: JobStatus) -> JobStartResponse:
    return JobStartResponse(
        job_id=status.job_id,
        kind=status.spec.kind,
        vehicle_id=status.spec.vehicle_id,
        steps=status.spec.steps,
        run_type=status.spec.run_type,
        session_id=status.session_id,
        started_at=status.started_at,
    )


def _active_vehicle(store: VehicleStore) -> str:
    vehicle_id = store.get_active()
    if vehicle_id is None:
        raise HTTPException(
            status_code=409,
            detail="車が選ばれていません。プロファイル画面で車を選んでから操作してください",
        )
    return vehicle_id


def _devices(request: Request, store: VehicleStore) -> dict[str, bool]:
    """機器の有無（初期化ページの表示用）。ups: 共通設定の checks.init_ups / gpio: 非常停止スイッチの見張りを
    起動しているか（settings.toml の [gpio] enabled・環境変数・実機かスタブか）/ buttons: 車両 yaml の buttons.installed。"""
    from src.robot import config_store  # noqa: PLC0415

    ups = True
    try:
        ups = bool((config_store._load_yaml(store.common_path).get("checks") or {}).get("init_ups", True))  # noqa: SLF001
    except Exception:  # noqa: BLE001 - 表示用。読めなければ「ある」扱い
        ups = True
    buttons = True
    vehicle_id = store.get_active()
    if vehicle_id is not None and store.exists(vehicle_id):
        try:
            buttons = bool(store.read(vehicle_id).buttons.get("installed", True))
        except Exception:  # noqa: BLE001
            buttons = True
    return {
        "ups": ups,
        "gpio": bool(getattr(request.app.state, "gpio_enabled", False)),
        "buttons": buttons,
    }


@router.get("/history")
async def get_history(manager: Manager) -> dict[str, Any]:
    """実行中のジョブの最近の sample（車速・基準・開度。最大 120 s）。画面を読み直したときに軌跡を描き直す用。

    各点の時刻は ``age_ms``（今から何 ms 前か）。ジョブが無ければ samples は空。
    """
    return manager.sample_history()


@router.get("/status", response_model=SystemStateResponse)
async def get_status(request: Request, manager: Manager, store: Store) -> SystemStateResponse:
    snap = manager.snapshot()
    job = snap["job"]
    return SystemStateResponse(
        robot_state=RobotState(snap["robot_state"]),
        active_profile_id=store.get_active(),
        active_session_id=job["session_id"] if job else None,
        last_normal_shutdown=True,
        updated_at=_now(),
        manager_state=snap["state"],
        initialized=snap["initialized"],
        devices=_devices(request, store),
        job=job,
        last_job=snap["last_job"],
        sequence=snap["sequence"],
        last_sequence=snap["last_sequence"],
        prepare=snap["prepare"],
    )


def _now() -> Any:
    from datetime import UTC, datetime  # noqa: PLC0415

    return datetime.now(tz=UTC)


@router.get("/log", response_model=DriveLogResponse)
async def get_drive_log(
    request: Request, after: Annotated[int | None, Query(ge=0)] = None
) -> DriveLogResponse:
    """サーバーログ＋ランナーの進行のメッセージの差分（`seq > after`）を古い順に最大 200 行返す。"""
    buf = getattr(request.app.state, "log_buffer", None)
    if buf is None:
        return DriveLogResponse(lines=[], last_seq=0)
    lines = buf.get_lines(after=after, limit=200)
    last_seq = lines[-1]["seq"] if lines else buf.last_seq
    return DriveLogResponse(lines=[LogLineSchema(**ln) for ln in lines], last_seq=last_seq)


# ── 選んでいる車 ───────────────────────────────────────────────────────


@router.post("/select-profile", status_code=200)
async def select_profile(req: SelectProfileRequest, manager: Manager, store: Store) -> dict[str, str]:
    """走らせる車を選ぶ（vehicles/.active に保存。再起動後も残る）。ジョブ実行中は変えられない。"""
    if manager.is_busy or manager.prepare is not None:
        raise HTTPException(status_code=409, detail="ジョブの実行中・準備中は車を変えられません")
    if not store.exists(req.profile_id):
        raise HTTPException(status_code=404, detail=f"車 {req.profile_id!r} が見つかりません")
    store.set_active(req.profile_id)
    return {"status": "ok", "profile_id": req.profile_id}


# ── 初期化・モード走行・停止 ───────────────────────────────────────────


@router.post("/initialize", response_model=JobStartResponse)
async def initialize(manager: Manager, store: Store) -> JobStartResponse:
    """初期化のジョブ（手順 1: 通信確認 → アラームリセット → サーボON → 原点復帰 ＋ ボタンサーボの確認）。

    常駐ランナーが無ければ起動し、生きていれば init コマンドを送る。手順 1 を行うのはこのジョブだけ。
    成功すると ``initialized`` が true になり（サーボ ON・接続のまま待機）、モード走行・学習を始められる。
    """
    return _job_response(await manager.start_init_only(_active_vehicle(store)))


@router.post("/arm", status_code=200)
async def arm_drive(manager: Manager, store: Store) -> dict[str, str]:
    """（互換）旧「走行準備」。ジョブが走行前チェックまで自分で行うので、始められるかの確認だけ行う。"""
    _active_vehicle(store)
    if manager.is_busy or manager.state == "emergency":
        raise HTTPException(status_code=409, detail="別のジョブの実行中か非常停止中です")
    return {"status": "armed"}


@router.post("/cancel", status_code=200)
async def cancel_drive() -> dict[str, str]:
    """（互換）旧「走行準備の取消」。何もしない。"""
    return {"status": "cancelled"}


@router.post("/start", response_model=JobStartResponse)
async def start_drive(
    req: StartDriveRequest,
    manager: Manager,
    store: Store,
    mode_repo: Annotated[ModeRepoProtocol, Depends(get_mode_repo)],
) -> JobStartResponse:
    """モード走行を始める（常に 24,25。走行前チェック → Ready・D・モード・N → IGN-OFF）。選んでいる車で走る。

    初期化ページの初期化が済んでいないと 409（手順 1 は初期化ページだけで行う。D-15c）。"""
    vehicle_id = _active_vehicle(store)
    mode = await mode_repo.get_by_id(req.mode_id)
    if mode is None:
        raise HTTPException(status_code=404, detail=f"走行モード {req.mode_id!r} が見つかりません")
    return _job_response(
        await manager.start_mode_drive(vehicle_id, mode.id, mode.name, req.limit_s)
    )


@router.post("/ign-off", response_model=JobStartResponse)
async def ign_off(manager: Manager, store: Store) -> JobStartResponse:
    """IGN-OFF だけを押すジョブ（手順 25。接続だけ裏で行う。初期化済みでなくても始められる）。走行・学習がエラー・中断で終わって車が Ready のままのときに使う。

    手順 25 は停車を確かめてからブレーキ ON でスタートを短押しする（動いていれば押さずエラー）。
    """
    return _job_response(await manager.start_ign_off(_active_vehicle(store)))


@router.post("/stop", status_code=200)
async def stop_drive(manager: Manager) -> dict[str, str]:
    """実行中のジョブを止める（stop コマンド。停止の流れまで済ませてランナーは待機へ戻る。非常停止の状態にはならない）。無ければ何もしない。"""
    if not manager.is_busy:
        return {"status": "idle"}
    await manager.stop()
    return {"status": "ok"}


@router.post("/pause", status_code=200)
async def pause_drive(manager: Manager) -> dict[str, Any]:
    """一時停止（ランナーへ SIGUSR1）。モード走行の実行中だけ。それ以外は 409。"""
    status = await manager.pause()
    return {"status": "ok", "paused": status.paused}


@router.post("/resume", status_code=200)
async def resume_drive(manager: Manager) -> dict[str, Any]:
    """再開（ランナーへ SIGUSR2）。一時停止中のモード走行だけ。それ以外は 409。"""
    status = await manager.resume()
    return {"status": "ok", "paused": status.paused}


@router.post("/emergency", status_code=200)
async def emergency_stop(manager: Manager) -> dict[str, str]:
    await manager.emergency()
    return {"status": "ok"}


@router.post("/reset-emergency", status_code=200)
async def reset_emergency(manager: Manager) -> dict[str, str]:
    # EmergencyStillActive（423）・JobConflict（409）は app のハンドラが応答にする
    await manager.reset_emergency()
    return {"status": "ok"}


# ── 学習 ───────────────────────────────────────────────────────────────


@router.post("/learning/start", response_model=JobStartResponse)
async def start_learning(
    manager: Manager, store: Store, req: LearningStartRequest | None = None
) -> JobStartResponse:
    """学習ジョブを始める。kind: full（手順 2）/ fel（12）/ hold（13）/ stop（14）。"""
    kind = (req or LearningStartRequest()).kind
    return _job_response(await manager.start_learning(_active_vehicle(store), kind))


@router.post("/learning-cycle/arm", status_code=200)
async def arm_learning_cycle(manager: Manager, store: Store) -> dict[str, str]:
    """（互換）旧「学習サイクルの準備」。始められるかの確認だけ行う。"""
    return await arm_drive(manager, store)


@router.post("/learning-cycle/cancel", status_code=200)
async def cancel_learning_cycle() -> dict[str, str]:
    """（互換）旧「学習サイクルの準備の取消」。何もしない。"""
    return {"status": "cancelled"}


@router.post("/learning-cycle/start", response_model=JobStartResponse, status_code=202)
async def start_learning_cycle(
    manager: Manager, store: Store, req: LearningStartRequest | None = None
) -> JobStartResponse:
    """学習ジョブを始める（`/learning/start` と同じ。body 省略は full）。"""
    return await start_learning(manager, store, req)


@router.post("/learning-cycle/abort", status_code=200)
async def abort_learning_cycle(manager: Manager) -> dict[str, str]:
    """学習ジョブを中断する（stop コマンド）。実行中のジョブが無ければ 409。"""
    status = manager.current
    if status is None or status.spec.kind != "learning":
        raise HTTPException(status_code=409, detail="実行中の学習ジョブがありません")
    await manager.stop()
    return {"status": "aborting"}


@router.get("/learning-cycle/status")
async def get_learning_cycle_status(manager: Manager) -> dict[str, Any]:
    """学習ジョブの進み具合（手順・phase・最新の sample）。実行中でなければ直前のジョブ。"""
    snap = manager.snapshot()
    return {"state": snap["state"], "job": snap["job"], "last_job": snap["last_job"]}


# ── 手動運転・キャリブレーション（対話ジョブ） ─────────────────────────


async def _command(
    manager: Manager, command: dict[str, Any], *, axis: str | None = None
) -> JogResponse:
    reply = await manager.send_command(command)
    if not reply.ack.get("ok"):
        raise HTTPException(status_code=409, detail=str(reply.ack.get("message") or "失敗しました"))
    st = reply.status or manager.current_interactive_status()
    key = "brake_pulse" if axis == "brake" else "accel_pulse"
    return JogResponse(position=int((st or {}).get(key) or 0))


@router.post("/calib/jog", response_model=JogResponse)
async def calib_jog(req: JogRequest, manager: Manager, store: Store) -> JogResponse:
    """キャリブレーション: 軸をジョグ移動する（今の実開度から ±量）。対話ジョブが無ければ始める（初期化に時間がかかる）。

    ``step`` は旧 API のまま pulse（9500 pulse = 100%）。ランナーへは % で送る。
    """
    await manager.ensure_interactive(_active_vehicle(store), PURPOSE_CALIB)
    return await _command(
        manager,
        {
            "cmd": "jog",
            "axis": req.axis,
            "amount_pct": round(req.step / PULSE_FULL * 100.0, 4),
            "calib": True,  # max を測るため、上限をストロークの端の手前（99%）まで広げる
        },
        axis=req.axis,
    )


@router.post("/calib/home", response_model=JogResponse)
async def calib_home(req: AxisRequest, manager: Manager, store: Store) -> JogResponse:
    """キャリブレーション: 両軸を原点へ戻す（応答の position は req.axis の実位置）。"""
    await manager.ensure_interactive(_active_vehicle(store), PURPOSE_CALIB)
    return await _command(manager, {"cmd": "home"}, axis=req.axis)


@router.post("/calib/save-min", response_model=SaveMinResponse)
async def calib_save_min(req: SaveMinRequest, manager: Manager, store: Store) -> SaveMinResponse:
    """今の実位置を min（ペダルに触れた位置）として車両 yaml へ保存する。保存した軸は「測定済み」になる。"""
    vehicle_id = _active_vehicle(store)
    await manager.ensure_interactive(vehicle_id, PURPOSE_CALIB)
    reply = await manager.send_command({"cmd": "save_min", "axis": req.axis})
    if not reply.ack.get("ok"):
        raise HTTPException(status_code=409, detail=str(reply.ack.get("message") or "保存できません"))
    info = store.read(vehicle_id)
    st = reply.status or {}
    return SaveMinResponse(
        message=str(reply.ack.get("message") or ""),
        accel_pct=st.get("accel_pct"),
        brake_pct=st.get("brake_pct"),
        min_accel_measured=info.min_accel_measured,
        min_brake_measured=info.min_brake_measured,
    )


@router.post("/calib/save-max", response_model=SaveMaxResponse)
async def calib_save_max(req: SaveMaxRequest, manager: Manager, store: Store) -> SaveMaxResponse:
    """今の実位置を max（開度上限）として車両 yaml の vehicle.max_*_opening_pct へ保存する。
    min + 1% 以上・99% 以下でなければ 409。"""
    await manager.ensure_interactive(_active_vehicle(store), PURPOSE_CALIB)
    reply = await manager.send_command({"cmd": "save_max", "axis": req.axis})
    if not reply.ack.get("ok"):
        raise HTTPException(status_code=409, detail=str(reply.ack.get("message") or "保存できません"))
    st = reply.status or {}
    return SaveMaxResponse(
        message=str(reply.ack.get("message") or ""),
        accel_pct=st.get("accel_pct"),
        brake_pct=st.get("brake_pct"),
    )


@router.post("/manual/start", response_model=JobStartResponse)
async def start_manual(manager: Manager, store: Store) -> JobStartResponse:
    """手動運転を始める（対話ジョブ。初期化が済んでコマンド待ちになってから返る）。セッション（manual）を作る。"""
    status = await manager.ensure_interactive(_active_vehicle(store), PURPOSE_MANUAL)
    return _job_response(status)


@router.post("/manual/jog", response_model=JogResponse)
async def manual_jog(req: JogRequest, manager: Manager) -> JogResponse:
    """手動運転中に軸をジョグ移動する（`/calib/jog` と同じ換算）。手動運転が始まっていなければ 409。"""
    _require_interactive(manager, PURPOSE_MANUAL)
    return await _command(
        manager,
        {"cmd": "jog", "axis": req.axis, "amount_pct": round(req.step / PULSE_FULL * 100.0, 4)},
        axis=req.axis,
    )


@router.post("/manual/home", response_model=JogResponse)
async def manual_home(req: AxisRequest, manager: Manager) -> JogResponse:
    _require_interactive(manager, PURPOSE_MANUAL)
    return await _command(manager, {"cmd": "home"}, axis=req.axis)


@router.post("/manual/stop", status_code=200)
async def stop_manual(manager: Manager) -> dict[str, str]:
    """手動運転を終える（quit_interactive。ランナーは残る → セッションを completed にする）。"""
    _require_interactive(manager, PURPOSE_MANUAL)
    await manager.stop()
    return {"status": "ok"}


def _require_interactive(manager: JobManager, purpose: str) -> None:
    cur = manager.current
    if cur is None or cur.spec.kind != "interactive" or cur.spec.purpose != purpose:
        raise HTTPException(status_code=409, detail="手動運転が始まっていません")


# ── タイムスケジュール・シーケンス（段6） ──────────────────────────────


async def _load_schedule_checked(
    schedule_id: str, schedule_repo: ScheduleRepoProtocol, vehicle_id: str, manager: JobManager
) -> dict[str, Any]:
    """スケジュールを読み、走らせられるか検査して、ランナーへ渡す辞書にする（だめなら 404 / 422）。"""
    try:
        schedule = await schedule_repo.get_by_id(schedule_id)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail="schedule_id が UUID 形式ではありません") from exc
    if schedule is None:
        raise HTTPException(status_code=404, detail=f"スケジュール {schedule_id!r} が見つかりません")
    data = schedule_to_dict(schedule)
    problem = check_schedule(schedule_from_dict(data))
    if problem is not None:
        raise HTTPException(status_code=422, detail=problem)
    return data


@router.post("/schedule/start", response_model=JobStartResponse)
async def start_schedule_drive(
    req: StartScheduleRequest,
    manager: Manager,
    store: Store,
    schedule_repo: Annotated[ScheduleRepoProtocol, Depends(get_schedule_repo)],
) -> JobStartResponse:
    """タイムスケジュールを開ループで再生する（手順 1 → 走行前チェック → 再生 → 終了処理）。選んでいる車で走る。

    ボタン名のないイベント（旧データ）・点が無いときは 422。
    """
    vehicle_id = _active_vehicle(store)
    data = await _load_schedule_checked(req.schedule_id, schedule_repo, vehicle_id, manager)
    return _job_response(await manager.start_schedule(vehicle_id, data))


@router.post("/schedule/stop", status_code=200)
async def stop_schedule_drive(manager: Manager) -> dict[str, str]:
    return await stop_drive(manager)


async def _plan_sequence(
    sequence_id: str,
    vehicle_id: str,
    manager: JobManager,
    sequence_repo: SequenceRepoProtocol,
    mode_repo: ModeRepoProtocol,
    schedule_repo: ScheduleRepoProtocol,
) -> tuple[Any, list[PlannedStep]]:
    """シーケンスを読み、各段の参照先を解決して検査する（/sequence/start と /prepare で共通）。"""
    try:
        seq = await sequence_repo.get_by_id(sequence_id)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail="sequence_id が UUID 形式ではありません") from exc
    if seq is None:
        raise HTTPException(status_code=404, detail=f"シーケンス {sequence_id!r} が見つかりません")
    steps: list[PlannedStep] = []
    for no, st in enumerate(seq.steps, start=1):
        if st.kind == "mode":
            mode = await _get_or_none(mode_repo.get_by_id, st.ref_id)
            if mode is None:
                raise HTTPException(
                    status_code=422, detail=f"{no} 段目の走行モード（{st.ref_id}）が見つかりません"
                )
            steps.append(PlannedStep("mode", st.ref_id, mode.name, st.repeat, mode_id=mode.id))
        else:
            try:
                data = await _load_schedule_checked(st.ref_id, schedule_repo, vehicle_id, manager)
            except HTTPException as exc:
                if exc.status_code == 404:
                    exc = HTTPException(
                        status_code=422,
                        detail=f"{no} 段目のタイムスケジュール（{st.ref_id}）が見つかりません",
                    )
                elif exc.status_code in (400, 422):
                    exc = HTTPException(status_code=422, detail=f"{no} 段目: {exc.detail}")
                raise exc from None
            steps.append(PlannedStep("schedule", st.ref_id, str(data["name"]), st.repeat, schedule=data))
    return seq, steps


@router.post("/sequence/start")
async def start_sequence_drive(
    req: StartSequenceRequest,
    manager: Manager,
    store: Store,
    sequence_repo: Annotated[SequenceRepoProtocol, Depends(get_sequence_repo)],
    mode_repo: Annotated[ModeRepoProtocol, Depends(get_mode_repo)],
    schedule_repo: Annotated[ScheduleRepoProtocol, Depends(get_schedule_repo)],
) -> dict[str, Any]:
    """シーケンスを始める。段（走行モード・タイムスケジュール）を順に、繰り返し回数どおり 1 回ずつジョブとして走らせる。

    各段の走らせられるかは始める前に全部検査する（参照先が無い・ボタン名なしなど）。
    どれかの段が 0 以外で終わったら残りは走らせず止める。
    """
    vehicle_id = _active_vehicle(store)
    seq, steps = await _plan_sequence(
        req.sequence_id, vehicle_id, manager, sequence_repo, mode_repo, schedule_repo
    )
    run, first = await manager.start_sequence(
        vehicle_id, seq.id, seq.name, steps, seq.repeat, req.limit_s
    )
    return {
        "sequence_id": run.sequence_id,
        "name": run.name,
        "runs_total": len(run.runs),
        "sequence": run.to_dict(),
        "job": _job_response(first).model_dump(mode="json"),
    }


@router.post("/sequence/stop", status_code=200)
async def stop_sequence_drive(manager: Manager) -> dict[str, str]:
    return await stop_drive(manager)


async def _get_or_none(getter: Any, ref_id: str) -> Any:
    """参照先の取得（UUID 形式でない ID は「見つからない」）。"""
    try:
        return await getter(ref_id)
    except ValueError:
        return None


# ── 走行の準備（開始ポップアップの間に計画を作る。段4） ────────────────


@router.post("/prepare")
async def prepare_drive(
    req: PrepareRequest,
    manager: Manager,
    store: Store,
    mode_repo: Annotated[ModeRepoProtocol, Depends(get_mode_repo)],
    schedule_repo: Annotated[ScheduleRepoProtocol, Depends(get_schedule_repo)],
    sequence_repo: Annotated[SequenceRepoProtocol, Depends(get_sequence_repo)],
) -> dict[str, Any]:
    """走行の準備を始める（モード・シーケンスのモードの段の計画を、ハードに触れない別プロセスで作る）。

    応答: ``{prepare_id, state: planning|ready, done, total}``。準備中・ジョブ実行中・非常停止中・
    （モードを走るのに）未初期化は 409。参照先が無ければ 404。
    """
    vehicle_id = _active_vehicle(store)
    if req.kind == "mode":
        mode = await _get_or_none(mode_repo.get_by_id, req.id)
        if mode is None:
            raise HTTPException(status_code=404, detail=f"走行モード {req.id!r} が見つかりません")
        prep = await manager.start_prepare_mode(vehicle_id, mode.id, mode.name, req.limit_s)
    elif req.kind == "schedule":
        data = await _load_schedule_checked(req.id, schedule_repo, vehicle_id, manager)
        prep = await manager.start_prepare_schedule(vehicle_id, data)
    else:
        seq, steps = await _plan_sequence(req.id, vehicle_id, manager, sequence_repo, mode_repo, schedule_repo)
        prep = await manager.start_prepare_sequence(
            vehicle_id, seq.id, seq.name, steps, seq.repeat, req.limit_s
        )
    return {"prepare_id": prep.prepare_id, "state": prep.state, "done": prep.done, "total": prep.total}


@router.post("/prepare/cancel", status_code=200)
async def prepare_cancel(manager: Manager) -> dict[str, str]:
    """準備を取り消す（計画プロセスを止め、ファイルを消す）。準備が無くても 200。"""
    await manager.cancel_prepare()
    return {"status": "cancelled"}


@router.post("/prepare/go", status_code=200)
async def prepare_go(manager: Manager) -> dict[str, Any]:
    """「はい」: 計画ができていればすぐジョブを始め（``started``）、まだなら ``waiting``（できたら自動で始める）。

    計画が失敗している・準備が無いときは 409。"""
    state, job_id = await manager.prepare_go()
    return {"state": state, "job_id": job_id}
