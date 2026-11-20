# 写したファイル一覧（scripts/port_research.py が作る。手で直さない）

- 写した日: 2026-11-19
- tests の commit: 4c80595+未コミットの変更あり
- ファイル数: 77（__init__.py を除く）/ 合計 55288 行（移植元の行数）
- 写す対象: tests/research/main.py から import で辿れるモジュール全部（関数内の遅延 import も含む）
- 取り込み直し: tests を直したら `.venv/bin/python scripts/port_research.py` を再実行する

| ファイル | 移植元 | 行数 | commit |
|---|---|---:|---|
| actuator_driver.py | tests/research/actuator_driver.py | 616 | 4c80595+未コミットの変更あり |
| app_settings.py | tests/research/app_settings.py | 302 | 4c80595+未コミットの変更あり |
| axis_monitor.py | tests/research/axis_monitor.py | 78 | 4c80595+未コミットの変更あり |
| axis_safety.py | tests/research/axis_safety.py | 103 | 4c80595+未コミットの変更あり |
| button_servo.py | tests/research/button_servo.py | 284 | 4c80595+未コミットの変更あり |
| button_steps.py | tests/research/button_steps.py | 277 | 4c80595+未コミットの変更あり |
| c10_integral.py | tests/research/c10_integral.py | 112 | 4c80595+未コミットの変更あり |
| can_reader.py | tests/research/can_reader.py | 231 | 4c80595+未コミットの変更あり |
| coast_assist.py | tests/research/coast_assist.py | 390 | 4c80595+未コミットの変更あり |
| coast_curve.py | tests/research/coast_curve.py | 139 | 4c80595+未コミットの変更あり |
| config.py | tests/research/config.py | 3157 | 4c80595+未コミットの変更あり |
| creep_curve.py | tests/research/creep_curve.py | 135 | 4c80595+未コミットの変更あり |
| creep_floor.py | tests/research/creep_floor.py | 147 | 4c80595+未コミットの変更あり |
| debug_process23.py | tests/research/debug_process23.py | 659 | 4c80595+未コミットの変更あり |
| dob.py | tests/research/dob.py | 293 | 4c80595+未コミットの変更あり |
| drive_index.py | tests/research/drive_index.py | 210 | 4c80595+未コミットの変更あり |
| drive_log.py | tests/research/drive_log.py | 837 | 4c80595+未コミットの変更あり |
| dynamics_estimation.py | tests/research/dynamics_estimation.py | 365 | 4c80595+未コミットの変更あり |
| excite.py | tests/research/excite.py | 1139 | 4c80595+未コミットの変更あり |
| fel.py | tests/research/fel.py | 721 | 4c80595+未コミットの変更あり |
| fel_pattern.py | tests/research/fel_pattern.py | 508 | 4c80595+未コミットの変更あり |
| ff_bias.py | tests/research/ff_bias.py | 2370 | 4c80595+未コミットの変更あり |
| ff_candidate.py | tests/research/ff_candidate.py | 1141 | 4c80595+未コミットの変更あり |
| ff_model.py | tests/research/ff_model.py | 667 | 4c80595+未コミットの変更あり |
| ff_params.py | tests/research/ff_params.py | 100 | 4c80595+未コミットの変更あり |
| fwd_cv.py | tests/research/fwd_cv.py | 818 | 4c80595+未コミットの変更あり |
| fwd_data.py | tests/research/fwd_data.py | 783 | 4c80595+未コミットの変更あり |
| fwd_final.py | tests/research/fwd_final.py | 459 | 4c80595+未コミットの変更あり |
| fwd_map.py | tests/research/fwd_map.py | 1935 | 4c80595+未コミットの変更あり |
| fwd_model.py | tests/research/fwd_model.py | 455 | 4c80595+未コミットの変更あり |
| fwd_plan.py | tests/research/fwd_plan.py | 4276 | 4c80595+未コミットの変更あり |
| fwd_tool.py | tests/research/fwd_tool.py | 1826 | 4c80595+未コミットの変更あり |
| fwd_torch.py | tests/research/fwd_torch.py | 448 | 4c80595+未コミットの変更あり |
| g_limit.py | tests/research/g_limit.py | 333 | 4c80595+未コミットの変更あり |
| hardware.py | tests/research/hardware.py | 786 | 4c80595+未コミットの変更あり |
| hold_crossing.py | tests/research/hold_crossing.py | 173 | 4c80595+未コミットの変更あり |
| hold_pi.py | tests/research/hold_pi.py | 413 | 4c80595+未コミットの変更あり |
| hold_scene.py | tests/research/hold_scene.py | 440 | 4c80595+未コミットの変更あり |
| horizon_search.py | tests/research/horizon_search.py | 465 | 4c80595+未コミットの変更あり |
| kpi.py | tests/research/kpi.py | 696 | 4c80595+未コミットの変更あり |
| launch_band_step.py | tests/research/launch_band_step.py | 383 | 4c80595+未コミットの変更あり |
| launch_hold.py | tests/research/launch_hold.py | 96 | 4c80595+未コミットの変更あり |
| learning_patterns.py | tests/research/learning_patterns.py | 23 | 4c80595+未コミットの変更あり |
| live_plot.py | tests/research/live_plot.py | 218 | 4c80595+未コミットの変更あり |
| local_gain.py | tests/research/local_gain.py | 1022 | 4c80595+未コミットの変更あり |
| loop_margin.py | tests/research/loop_margin.py | 571 | 4c80595+未コミットの変更あり |
| main.py | tests/research/main.py | 1529 | 4c80595+未コミットの変更あり |
| mode_drive.py | tests/research/mode_drive.py | 5194 | 4c80595+未コミットの変更あり |
| mode_report.py | tests/research/mode_report.py | 1302 | 4c80595+未コミットの変更あり |
| model_gain.py | tests/research/model_gain.py | 267 | 4c80595+未コミットの変更あり |
| pattern_drive.py | tests/research/pattern_drive.py | 2777 | 4c80595+未コミットの変更あり |
| pattern_loop.py | tests/research/pattern_loop.py | 4071 | 4c80595+未コミットの変更あり |
| pedal_arbiter.py | tests/research/pedal_arbiter.py | 270 | 4c80595+未コミットの変更あり |
| pedal_gain.py | tests/research/pedal_gain.py | 221 | 4c80595+未コミットの変更あり |
| pedal_lag.py | tests/research/pedal_lag.py | 676 | 4c80595+未コミットの変更あり |
| pedal_search.py | tests/research/pedal_search.py | 918 | 4c80595+未コミットの変更あり |
| pedal_select.py | tests/research/pedal_select.py | 55 | 4c80595+未コミットの変更あり |
| play_floor.py | tests/research/play_floor.py | 211 | 4c80595+未コミットの変更あり |
| play_onset.py | tests/research/play_onset.py | 326 | 4c80595+未コミットの変更あり |
| pre_check.py | tests/research/pre_check.py | 265 | 4c80595+未コミットの変更あり |
| pre_drive_check.py | tests/research/pre_drive_check.py | 249 | 4c80595+未コミットの変更あり |
| ramp_planner.py | tests/research/ramp_planner.py | 944 | 4c80595+未コミットの変更あり |
| reachability.py | tests/research/reachability.py | 118 | 4c80595+未コミットの変更あり |
| release_gain.py | tests/research/release_gain.py | 200 | 4c80595+未コミットの変更あり |
| research_actuator.py | tests/research/research_actuator.py | 110 | 4c80595+未コミットの変更あり |
| research_types.py | tests/research/research_types.py | 391 | 4c80595+未コミットの変更あり |
| sample_weight.py | tests/research/sample_weight.py | 155 | 4c80595+未コミットの変更あり |
| speed_predictor.py | tests/research/speed_predictor.py | 319 | 4c80595+未コミットの変更あり |
| step2_check.py | tests/research/step2_check.py | 952 | 4c80595+未コミットの変更あり |
| stop_brake_floor.py | tests/research/stop_brake_floor.py | 265 | 4c80595+未コミットの変更あり |
| stop_decel.py | tests/research/stop_decel.py | 175 | 4c80595+未コミットの変更あり |
| stop_end.py | tests/research/stop_end.py | 149 | 4c80595+未コミットの変更あり |
| term.py | tests/research/term.py | 76 | 4c80595+未コミットの変更あり |
| ups_monitor.py | tests/research/ups_monitor.py | 242 | 4c80595+未コミットの変更あり |
| vehicle.py | tests/research/vehicle.py | 103 | 4c80595+未コミットの変更あり |
| vehicle_status.py | tests/research/vehicle_status.py | 277 | 4c80595+未コミットの変更あり |
| wltp_grid.py | tests/research/wltp_grid.py | 881 | 4c80595+未コミットの変更あり |
