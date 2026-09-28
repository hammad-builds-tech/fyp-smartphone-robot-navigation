#!/bin/bash
# Watchdog: keeps the FYP Android app streaming to the backend.
# Polls /health frames_received; if the counter stalls or the stream is down,
# force-stops and relaunches the app. Also keeps the screen on while on USB.
LOG=/tmp/fyp_phone_watchdog.log
BACKEND=http://127.0.0.1:8000/health
PKG=com.fyp.reconstruction
last=-1
stall=0
echo "$(date +%T) watchdog started" >> $LOG
while true; do
    f=$(curl -s -m 4 $BACKEND | grep -o '"frames_received":[0-9]*' | grep -o '[0-9]*')
    if [ -z "$f" ]; then f=0; fi
    if [ "$f" -le "$last" ]; then
        stall=$((stall+1))
    else
        stall=0
    fi
    # after 3 consecutive stale polls (~21 s), restart the app
    if [ "$stall" -ge 3 ]; then
        echo "$(date +%T) stream stalled at $f frames -> relaunching app" >> $LOG
        adb shell svc power stayon usb >/dev/null 2>&1
        adb shell input keyevent KEYCODE_WAKEUP >/dev/null 2>&1
        adb shell am force-stop $PKG >/dev/null 2>&1
        sleep 1
        adb shell am start -n $PKG/.MainActivity >/dev/null 2>&1
        stall=0
        last=-1
        sleep 8
        continue
    fi
    last=$f
    sleep 7
done
