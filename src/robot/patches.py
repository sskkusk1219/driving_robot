# ruff: noqa: E501
"""core（src/robot/core = tests/research の写し）の挙動を、core を直さずに差し替える所を 1 か所に集めたもの。

どれもモジュール属性・クラス属性の代入で、``install_all()`` が 1 回だけ行う。差し替えた内容は
src/robot/README.md の「差し替え点」の表と同じ（増やしたらそこにも書く）。

| # | 差し替え | 目的 |
|---|---|---|
| 1 | SessionLog → StreamingSessionLog（main・pattern_drive・pre_drive_check の各モジュール属性） | sample/csv イベント |
| 2 | main.write_mode_report → 何もしない | A-2: レポート（md・図）は作らない |
| 3 | hardware.InitReport.add / skip を包む | init_step イベント |
| 4 | main.pre_drive_check / main.shutdown を包む | state イベント・走行前チェックを中断可能に。shutdown は ``set_job_end`` で「原点復帰だけ（home）」「何もしない（keep）」に切り替えられる（常駐ランナー） |
| 5 | button_steps.press_button を包む（ペダル値の同期は**スタブのときだけ**。手順 24 の Ready の前のブレーキ ON は実機・スタブ共通） | スタブ車両のペダルの値を、押す直前の実際のペダルに合わせる／手順 24 の Ready を「ブレーキ ON → スタート短押し」にする |
| 7 | button_steps.ign_off → src 版 | 手順 25: スタートを短押しでなく `buttons.ign_off_press_s`（既定 10 s）押す（ユーザー決定 2026-11-15）。表の 6 は learning_buttons（README） |
| 9 | button_steps.press_button を包む（最も外側。8 の後に入れる） | ボタンの押し始め・離しを button イベントにする（ペダルの図に矩形で出す。段11）。ボタンなしでも出る |
| 10 | `_release_pedals`（core の pattern_drive・mode_drive・excite と src の schedule_job の各モジュール属性）・button_steps.press_shift_n_after_drive・main.shutdown（差し替え 4）を包む | 走行するジョブの非常停止を「減速して停車 → N → スタート 10 秒（IGN-OFF）→ 原点復帰」にする（src/robot/emergency_stop.py。RPD_Phase_20261119_EmergencyStop）。対話・ボタンだけのジョブは従来どおり |
| 11 | build_hardware（core.hardware・core.main・quiet_connect の各モジュール属性）を包む（**スタブのときだけ**） | スタブ車両を実車の走行前と同じ「停止・P」から始める（core は D でクリープ 5 km/h から始め、走行前チェックの停止確認はブレーキを踏む前提。チェックがハードを動かさない決定 2026-11-19 の後は車速確認が通らない） |
| 8 | button_steps.press_button を包む（最も外側）・ensure_buttons_connected → ボタンなしのとき何もしない | `buttons.installed: false`（SG90 を外している。ユーザー決定 2026-11-16 D-15c）: PCA9685 に繋がず・サーボを動かさず、押したことにしてログに残すだけ（押す秒数・押した後の待ちは同じだけ待つ） |
| 12 | `_ModeRun.drive_axis`（mode_drive）・`PatternLoop._drive_or_sense`（pattern_loop）・`_ExciteRun._drive_axis`（excite）を包む | B-2: stop・SIGINT の cancel が Modbus 要求の最中に届き ModbusIOException になっても、cancel 中なら CancelledError に戻して停止の流れに入れる（誤った「通信失敗」ERROR を出さない） |
"""

from __future__ import annotations

import asyncio
import dataclasses
import functools
import math
import sys
import time
from pathlib import Path
from typing import Any

from src.robot import emergency_stop, events, quiet_connect
from src.robot.core import button_steps as core_button_steps
from src.robot.core import hardware as core_hardware
from src.robot.core import main as core_main
from src.robot.core import mode_drive as core_mode_drive
from src.robot.core import pattern_drive as core_pattern_drive
from src.robot.core.button_servo import button_name
from src.robot.core.drive_log import SessionLog
from src.robot.core.term import say
from src.robot.interrupt import StopController
from src.robot.streaming_log import StreamingSessionLog

#: core の press_button（差し替え 5 の前。import 時点の値）
_orig_press_button = core_button_steps.press_button
_installed = False
#: ボタン（SG90）が付いているか。``configure`` が車両の ``buttons.installed`` で決める（既定 True）
_buttons_installed = True
_stop: StopController | None = None  # 走行前チェックを包む wrapper が見る（install_all のたびに更新）


def install_streaming_log() -> int:
    """差し替え 1: SessionLog を名前で取り込んでいる core のモジュールすべてを置き換える。

    Returns:
        置き換えたモジュール数。
    """
    n = 0
    for name, mod in list(sys.modules.items()):
        if not name.startswith("src.robot.core.") or mod is None:
            continue
        if getattr(mod, "SessionLog", None) is SessionLog:
            mod.SessionLog = StreamingSessionLog  # type: ignore[attr-defined]
            n += 1
    return n


def _no_report(rows: Any, cfg: Any, info: Any, results_dir: Path) -> Path:
    """差し替え 2: モード走行のレポート（md・図）を作らない（RPD A-2。ログは DB とログ画面だけ）。"""
    say("レポート（md・図）は作りません（ログは DB のログ画面で見る）")
    return Path(results_dir)


def install_report_skip() -> None:
    core_main.write_mode_report = _no_report  # type: ignore[assignment]


def install_init_events() -> None:
    """差し替え 3: 初期化の 1 項目ごとの結果を init_step イベントにする（say の行はそのまま出る）。"""
    orig_add = core_hardware.InitReport.add
    orig_skip = core_hardware.InitReport.skip

    def add(self: Any, name: str, *, passed: bool, detail: str = "") -> Any:
        result = orig_add(self, name, passed=passed, detail=detail)
        events.emit("init_step", name=name, status="ok" if passed else "ng", detail=detail)
        return result

    def skip(self: Any, name: str, key: str) -> Any:
        result = orig_skip(self, name, key)
        events.emit("init_step", name=name, status="skip", detail=result.detail)
        return result

    core_hardware.InitReport.add = add  # type: ignore[method-assign]
    core_hardware.InitReport.skip = skip  # type: ignore[method-assign]


def install_state_events() -> None:
    """差し替え 4: 走行前チェック（中断可能に包む）と終了処理に state イベントを付ける。"""
    orig_pre = core_main.pre_drive_check
    orig_shutdown = core_main.shutdown

    async def pre_drive_check(ctx: Any, **kwargs: Any) -> None:
        events.emit("state", phase="pre_check", step=kwargs.get("next_step"))
        if _stop is None:
            await orig_pre(ctx, **kwargs)
        else:
            await _stop.run(lambda: orig_pre(ctx, **kwargs))

    async def shutdown(hw: Any) -> None:
        if _job_end == "full":
            events.emit("state", phase="shutdown")
        else:
            events.emit("state", phase="job_end", mode=_job_end)
        await _emergency_before_shutdown(hw)
        if emergency_stop.brake_kept or emergency_stop.buttons_unavailable:
            # ブレーキを踏んだまま終える（最大開度で保つ・停車していてボタンを押せない）: 原点復帰・サーボOFF をすると車が動くので飛ばす
            await emergency_stop.shutdown_keep_servo(hw)
            return
        if _job_end != "full":
            await _job_end_without_disconnect(hw, _job_end)
            return
        await orig_shutdown(hw)

    core_main.pre_drive_check = pre_drive_check  # type: ignore[assignment]
    core_main.shutdown = shutdown  # type: ignore[assignment]


#: 終了処理（core_main.shutdown の差し替え）の範囲。常駐ランナー（--serve）が、ジョブの間だけ切り替える。
#: "full" = core のまま（原点復帰 → サーボ OFF → 切断。既定）／"home" = 原点復帰だけ／"keep" = 何もしない（停車保持のまま）。
#: "home"・"keep" はサーボ ON・接続のまま（初期化の後は基本つなげたままにする。ユーザー決定 2026-11-20）
JOB_END_MODES = ("full", "home", "keep")
_job_end = "full"


def set_job_end(mode: str) -> None:
    """ジョブの終わりの扱いを切り替える（"full" / "home" / "keep"）。ジョブの前に呼ぶ。"""
    global _job_end  # noqa: PLW0603
    if mode not in JOB_END_MODES:
        raise ValueError(f"ジョブの終わりの扱いが不正です: {mode!r}（{', '.join(JOB_END_MODES)}）")
    _job_end = mode


def current_job_end() -> str:
    return _job_end


async def _job_end_without_disconnect(hw: Any, mode: str) -> None:
    """ジョブの終わり（サーボ ON・接続のまま）: "home" は両軸の原点復帰だけ、"keep" は何もしない。"""
    if not getattr(hw, "connected", False):
        return
    if mode == "keep":
        say("ジョブの終わり: 停車保持のまま待機します（原点復帰・サーボOFF・切断はしません）")
        return
    say("走行の終わり: ペダルを原点へ戻します")
    try:
        await asyncio.gather(hw.accel.home_return(), hw.brake.home_return())
    except Exception as exc:  # noqa: BLE001
        say(f"  警告: 原点復帰に失敗しました（{type(exc).__name__}: {exc}）")


async def _emergency_before_shutdown(hw: Any) -> None:
    """差し替え 10: 終了処理の前に、非常停止の流れを走らせる（走行するジョブの中断のとき）。

    中断要求（SIGINT）で手順が打ち切られた場合、ここで流れを走らせる。すでに流れが走っている（自動の
    非常停止の途中で中断が来た）ときは、その終わりを待つだけで、やり直さない・止めない。
    """
    if not emergency_stop.enabled or not getattr(hw, "connected", False):
        return
    if emergency_stop.handled:
        await emergency_stop.wait_if_running()
        return
    if _stop is not None and _stop.requested and emergency_stop.armed():
        if _stop.signal_name in ("stop", "quit", "EOF"):
            # 通常の停止（「中断」「走行終了」）。非常停止ではないので文言を分ける
            await emergency_stop.run(hw, emergency_stop.PLAIN_STOP_REASON)
        else:
            await emergency_stop.run(hw, f"{_stop.signal_name or '中断要求'} による中断")


def _reason_of_current_exception() -> str:
    exc = sys.exception()
    if exc is None:
        return "異常終了"
    return f"{type(exc).__name__}: {exc}"


def install_emergency_stop() -> None:
    """差し替え 10: ペダルを離す関数と N を押す関数を、非常停止の流れに合わせて包む。

    - ``_release_pedals``: 走行するジョブで、中断要求がまだ無いとき → 非常停止の流れ（理由は今の例外の文言）。
      中断要求が来ているときは何もしない（終了処理の前に流れを走らせる）。走行するジョブでなければ元の関数。
    - ``press_shift_n_after_drive``: 流れが済んでいれば押さない（N の二重押しを防ぐ）。
    """
    orig_release = core_pattern_drive._release_pedals  # noqa: SLF001
    orig_press_n = core_button_steps.press_shift_n_after_drive

    async def release_pedals(hw: Any) -> None:
        if not emergency_stop.enabled:
            await orig_release(hw)
            return
        if _stop is not None and _stop.requested:
            return
        await emergency_stop.run(hw, _reason_of_current_exception())

    async def press_shift_n_after_drive(hw: Any, cfg: Any) -> None:
        if emergency_stop.handled:
            say("  非常停止の流れで N まで済んでいるので、ここでは N を押しません")
            return
        await orig_press_n(hw, cfg)

    core_pattern_drive._release_pedals = release_pedals  # type: ignore[assignment]  # noqa: SLF001
    core_mode_drive._release_pedals = release_pedals  # type: ignore[attr-defined]  # noqa: SLF001
    from src.robot import schedule_job  # noqa: PLC0415 - 循環 import を避けて遅延
    from src.robot.core import excite as core_excite  # noqa: PLC0415

    core_excite._release_pedals = release_pedals  # type: ignore[attr-defined]  # noqa: SLF001
    schedule_job._release_pedals = release_pedals  # type: ignore[attr-defined]  # noqa: SLF001
    core_button_steps.press_shift_n_after_drive = press_shift_n_after_drive  # type: ignore[assignment]


def install_cancel_guard() -> None:
    """差し替え 12: 軸を動かす 3 つのメソッドを包み、cancel 中の通信例外を CancelledError に戻す。

    利用者の stop コマンドや SIGINT で内側のタスクを cancel したとき、Modbus の要求の最中だと pymodbus は
    CancelledError を ``ModbusIOException("Request cancelled outside library")`` に変えて投げる。制御ループの
    ``except Exception`` がそれを「アクチュエータ通信に失敗しました」の DriveError（誤った ERROR）にしてしまう。
    子のメソッドの出口で ``raise_if_cancelling`` を呼んで CancelledError に戻せば、gather も CancelledError を
    投げ、``except Exception`` に捕まらず sleep 中に cancel が届いたときと同じ停止の流れに入る
    （ProblemReport_20261120_src全動作チェック B-2）。包み済みなら二重に包まない。
    """
    from src.robot.cancel_guard import raise_if_cancelling  # noqa: PLC0415
    from src.robot.core import excite as core_excite  # noqa: PLC0415
    from src.robot.core import pattern_loop as core_pattern_loop  # noqa: PLC0415

    def wrap(cls: Any, name: str) -> None:
        orig = cls.__dict__[name]
        if getattr(orig, "_cancel_guarded", False):
            return

        @functools.wraps(orig)
        async def guarded(self: Any, *args: Any, **kwargs: Any) -> Any:
            try:
                return await orig(self, *args, **kwargs)
            except Exception as exc:
                raise_if_cancelling(exc)
                raise

        guarded._cancel_guarded = True  # type: ignore[attr-defined]  # noqa: SLF001
        setattr(cls, name, guarded)

    wrap(core_mode_drive._ModeRun, "drive_axis")  # noqa: SLF001
    wrap(core_pattern_loop.PatternLoop, "_drive_or_sense")
    wrap(core_excite._ExciteRun, "_drive_axis")  # noqa: SLF001


def install_stub_button_sync() -> None:
    """差し替え 5: スタブ HW のとき、ボタンを押す直前にスタブ車両の状態へ今のペダルの値を渡す。

    原因: スタブ車両の状態（VehicleStatus）は ``ensure_buttons_connected`` が有効にする（IGN-OFF・P・
    ペダル 0 から始める）が、ペダルの値を受け取るのは CAN 車速を読んだとき（StubVehicle.advance）だけ。
    手順 24 は最初のボタン操作（Ready のスタート短押し）がモードの 0 秒で、有効化の直後・車速を読む前に
    押す（``poll=False``）。そのためブレーキ 0 と見なされ、「ブレーキを踏んでスタート」が成立せず
    IGN-ON(RLY-ON) になり、D に入れず 3 秒で中止になっていた（手順 20〜23 は直前に車速を読むので起きない）。
    スタブでだけ、有効化してから車速を 1 回読んで状態を更新する。実機（``hw.is_real``）は元の関数をそのまま
    呼ぶ（読み取りも足さない）。

    手順 24 の Ready 前のブレーキ ON はここではなく ``brake_on_before_step24``（手順の前。モードの時刻 0 秒より前）。
    """
    async def press_button(hw: Any, cfg: Any, button: str, seconds: float, **kwargs: Any) -> None:
        sim = not hw.is_real and hw.vehicle_status is not None
        if sim:
            await core_hardware.ensure_buttons_connected(hw)  # 有効化（済みなら何もしない）
            await hw.can.read_speed()  # スタブ車両の状態へ今のペダル・車速を渡す
        await _orig_press_button(hw, cfg, button, seconds, **kwargs)

    core_button_steps.press_button = press_button  # type: ignore[assignment]


def brake_on_before_step24(step: core_main.Step) -> core_main.Step:
    """手順 24 の handler の前にブレーキ ON を足す（Ready の準備。ボタンありのときだけ。実機・スタブ共通）。

    手順 24 はモードの時刻 0 秒で Ready（スタート短押し）を押す。走行前チェックがブレーキを踏まなくなったので
    ブレーキ OFF のまま押すと IGN-ON になり D に入れない。press_button の中で踏むと踏む時間（0〜0.5 s）が
    モードの時刻に入って Ready が遅れるため、handler（＝モードの時計）が始まる前に踏み終える。
    ``wrap_step`` より内側に入れるので、このブレーキ操作も中断・非常停止の対象。
    スタブは車両の状態にブレーキを渡すため、有効化と車速 1 回の読み取りも足す。
    """
    orig = step.handler

    async def handler(ctx: Any) -> None:
        if _buttons_installed:  # configure の値（ボタンなしは Ready を押さないので踏まない）
            hw = ctx.require_hardware()
            say("手順 24 の前にブレーキを ON（Ready の準備。モードの時刻 0 秒より前）")
            await core_button_steps.set_brake(hw, ctx.config, on=True)
            if not hw.is_real and hw.vehicle_status is not None:
                await core_hardware.ensure_buttons_connected(hw)
                await hw.can.read_speed()  # スタブ車両の状態へブレーキの値を渡す
        await orig(ctx)

    return dataclasses.replace(step, handler=handler)


def install_stub_start_stopped() -> None:
    """差し替え 11: スタブのとき、車両を停止・P から始める（実機は走行前に P で止まっている）。

    core の build_hardware はスタブ車を「D・クリープ 5 km/h」から始める（走行前チェックでブレーキを
    刻んで止める流れを通すため）。走行前チェックはハードを動かさない決定（2026-11-19）で、止める操作が
    無いため車速確認が通らなかった。スタブでだけ、構築直後に車速を 0 にし、ボタンありなら状態を有効化
    （IGN-OFF・P）する。その後の Ready・D のボタン操作で D に入ればクリープで動き出す。ボタンなしは
    状態を有効にしない（有効にすると D に入れず走れない）ので 0 km/h からクリープで始まる。
    実機（``hw.is_real``）は元の関数の結果をそのまま返す。
    """
    orig_build = core_hardware.build_hardware

    def build_hardware(cfg: Any, hw_mode: str) -> Any:
        hw = orig_build(cfg, hw_mode)
        if not hw.is_real:
            hw.can.speed_kmh = 0.0
            if _buttons_installed and hw.vehicle_status is not None:
                hw.vehicle_status.activate()  # IGN-OFF・P・0 km/h（ensure_buttons_connected の再有効化も同じ状態）
        return hw

    core_hardware.build_hardware = build_hardware  # type: ignore[assignment]
    core_main.build_hardware = build_hardware  # type: ignore[attr-defined]
    quiet_connect.build_hardware = build_hardware  # type: ignore[attr-defined]


IGN_OFF_PRESS_DEFAULT_S = 10.0
#: 手順 25 で押す秒数（configure が起動時に設定から決める）
_ign_off_press_s = IGN_OFF_PRESS_DEFAULT_S


async def ign_off(hw: Any, cfg: Any) -> None:
    """差し替え 7: 手順 25。core の ign_off と同じ流れで、押す秒数だけ ``buttons.ign_off_press_s`` にする。

    停車確認（CAN 車速が ``no_speed_kmh`` 以下）→ ブレーキ ON → スタートを押す。
    """
    await core_button_steps._begin(hw)  # noqa: SLF001
    try:
        speed = await hw.can.read_speed()
    except Exception as exc:
        raise core_button_steps.DriveError(
            f"IGN-OFF の前に CAN 車速を読めません（{type(exc).__name__}: {exc}）"
        ) from exc
    kmh = cfg.buttons.no_speed_kmh
    if speed > kmh:
        raise core_button_steps.DriveError(
            f"停車していません（車速 {speed:.2f} km/h > {kmh:g} km/h）。IGN-OFF を押しません"
        )
    # 設定の src 専用の値は configure で起動時に決めた値を使う（手順2 は保存後に設定を読み直し、
    # 読み直した設定には src_only が付かない。2026-11-16 段10-4 で ボタンなし が効かず N で落ちた）
    seconds = float(_ign_off_press_s)
    await core_button_steps.set_brake(hw, cfg, on=True)
    await core_button_steps.press_button(hw, cfg, core_button_steps.BUTTON_START, seconds)
    core_button_steps.show_stub_status(hw)


def install_ign_off() -> None:
    """差し替え 7: 手順 25 の押し方（core_main.step25_ign_off は button_steps.ign_off を属性で呼ぶ）。"""
    core_button_steps.ign_off = ign_off  # type: ignore[assignment]


def buttons_installed_in(cfg: Any) -> bool:
    """車両設定（src 専用キー ``buttons.installed``）から、ボタン（SG90）が付いているか。既定 True。"""
    return bool(getattr(cfg, "src_only", {}).get("buttons.installed", True))


def configure(cfg: Any) -> None:
    """ジョブの設定を差し替えに伝える（ランナーが設定を読んだ後・手順の前に 1 回呼ぶ）。"""
    global _buttons_installed, _ign_off_press_s  # noqa: PLW0603
    _buttons_installed = buttons_installed_in(cfg)
    _ign_off_press_s = float(
        getattr(cfg, "src_only", {}).get("buttons.ign_off_press_s", IGN_OFF_PRESS_DEFAULT_S)
    )
    if not _buttons_installed:
        say("ボタンなし（buttons.installed: false）: Ready・D・N・IGN-OFF は押したことにするだけ。"
            "PCA9685・サーボには接続しません")


def install_buttons_off() -> None:
    """差し替え 8: ボタンなしの設定のとき、ボタンを押さずにログだけ残す。

    press_button は他の差し替え（5 のスタブのペダル同期・7 の ign_off が呼ぶもの）の外側に被せる。
    ensure_buttons_connected は core_hardware の元の関数と、それを名前で取り込んでいる core の
    モジュール（button_steps・schedule_job）の両方を置き換える（ボタンなしでは PCA9685 に繋がない）。
    """
    inner_press = core_button_steps.press_button
    orig_ensure = core_hardware.ensure_buttons_connected

    async def press_button(hw: Any, cfg: Any, button: str, seconds: float, **kwargs: Any) -> None:
        if _buttons_installed:  # cfg は手順の途中で読み直されることがあるので見ない（configure の値）
            await inner_press(hw, cfg, button, seconds, **kwargs)
            return
        say(f"  ボタンなし: 「{button_name(button)}」を押したことにします（{seconds:g}s 待ちます）")
        if kwargs.get("poll", True):
            await core_button_steps.wait_polling(hw, seconds)
        else:
            await asyncio.sleep(max(0.0, seconds))  # 周期が車速を読んでいる最中（手順 24 の D・Ready）
        if kwargs.get("wait_after", True):
            await core_button_steps.wait_polling(hw, cfg.buttons.after_press_wait_s)

    async def ensure_buttons_connected(hw: Any) -> None:
        if _buttons_installed:
            await orig_ensure(hw)

    core_button_steps.press_button = press_button  # type: ignore[assignment]
    core_hardware.ensure_buttons_connected = ensure_buttons_connected  # type: ignore[assignment]
    core_button_steps.ensure_buttons_connected = ensure_buttons_connected  # type: ignore[assignment]
    from src.robot import schedule_job  # noqa: PLC0415 - 循環 import を避けて遅延

    schedule_job.ensure_buttons_connected = ensure_buttons_connected  # type: ignore[assignment]


def install_button_events() -> None:
    """差し替え 9: press_button（8 のボタンなしの経路も含めた最も外側）に button イベント（start/end）を付ける。

    押す秒数 ``seconds`` だけ「押している」。end は start から ``seconds`` 後（押した後の待ちは含めない。
    途中で例外・中断になっても出す）。``seconds`` が無限（押し続け）のときは start に ``hold: true``、end は離したとき。
    """
    inner_press = core_button_steps.press_button
    core_say = core_button_steps.say

    def hold_aware_say(message: str = "") -> None:
        # core は押す秒数を {seconds:g}s と書くので、押し続け（無限）は「infs 押します」になる。文を直す
        core_say(message.replace(" infs 押します", " 押し続けます（離すまで）"))

    core_button_steps.say = hold_aware_say  # type: ignore[assignment]

    async def press_button(hw: Any, cfg: Any, button: str, seconds: float, **kwargs: Any) -> None:
        if math.isinf(seconds):
            # 押し続け（手動運転のスイッチ。button_hold.py）: 離す（取り消し）まで押す。
            # start は seconds なし・hold:true、end は実際に押した秒数を付けて出す
            events.emit("button", button=button, phase="start", seconds=None, hold=True)
            t_start = time.monotonic()
            try:
                await inner_press(hw, cfg, button, seconds, **kwargs)
            finally:
                events.emit(
                    "button", button=button, phase="end", hold=True,
                    seconds=round(time.monotonic() - t_start, 3),
                )
            return
        sec = round(float(seconds), 3)
        events.emit("button", button=button, phase="start", seconds=sec)
        asyncio.get_running_loop().call_later(
            max(0.0, float(seconds)),
            functools.partial(events.emit, "button", button=button, phase="end", seconds=sec),
        )
        await inner_press(hw, cfg, button, seconds, **kwargs)

    core_button_steps.press_button = press_button  # type: ignore[assignment]


def install_all(stop: StopController) -> None:
    """差し替えを全部行う（差し替えはプロセスで 1 回。2 回目以降は中断の状態 stop だけ更新する）。"""
    global _installed, _stop  # noqa: PLW0603
    _stop = stop
    if _installed:
        return
    install_streaming_log()
    install_report_skip()
    install_init_events()
    install_state_events()
    install_stub_button_sync()
    install_stub_start_stopped()
    install_ign_off()
    install_buttons_off()
    install_button_events()
    install_emergency_stop()
    install_cancel_guard()
    _installed = True
