@echo off

rem driving_robot production WebUI with real hardware (actuators, CAN, GPIO).
rem Runs inside tmux: keeps running on the Pi even if SSH drops or this window is closed.
rem If session "robot" already exists, reattach to it instead of starting a second instance (-A).
rem Stop: Ctrl+C  /  Detach (keep running): press Ctrl+b, release, then d

echo Open in browser:
echo   http://raspi5-16gb.local:8080/static/index.html
echo   (if the name does not work, use the IP URL shown after connecting)
echo.

ssh -t raspi5_16gb@raspi5-16gb.local "tmux new-session -A -s robot 'set -- $(hostname -I); echo; echo Open in browser:; echo   http://raspi5-16gb.local:8080/static/index.html; echo   http://$1:8080/static/index.html; echo; cd ~/projects/driving_robot && DRIVING_ROBOT_USE_REAL_HW=1 DATABASE_URL=postgresql://localhost/driving_robot .venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8080; echo; echo Stopped. Press Enter to close.; read _'"

pause
