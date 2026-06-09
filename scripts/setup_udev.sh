#!/usr/bin/env bash
#
# driving_robot: アクチュエータ用 USB-RS485(FTDI) の udev 固定シンボリックリンクを設置する。
#
# USB 列挙順 (ttyUSBn) に依存せず、FTDI シリアル番号でアクセル/ブレーキ軸を固定する。
# 機器固有のシリアルを含む実ルールは生成物（gitignore）。テンプレートは
# config/udev/99-driving-robot-actuators.rules.example。
#
# 使い方:
#   scripts/setup_udev.sh --list                       接続中の FTDI RS485 デバイスを表示
#   sudo scripts/setup_udev.sh <accel_serial> <brake_serial>   ルールを設置して反映
#
set -euo pipefail

RULE_PATH="/etc/udev/rules.d/99-driving-robot-actuators.rules"
FTDI_VENDOR="0403"
FTDI_PRODUCT="6001"

list_devices() {
    echo "接続中の FTDI USB-RS485 デバイス:"
    local found=0
    for dev in /dev/ttyUSB*; do
        [ -e "$dev" ] || continue
        local vid pid model serial
        vid=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_VENDOR_ID=//p')
        pid=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_MODEL_ID=//p')
        if [ "$vid" = "$FTDI_VENDOR" ] && [ "$pid" = "$FTDI_PRODUCT" ]; then
            model=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_MODEL=//p')
            serial=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_SERIAL_SHORT=//p')
            printf "  %-14s serial=%-12s model=%s\n" "$dev" "$serial" "$model"
            found=1
        fi
    done
    [ "$found" = "1" ] || echo "  (FTDI RS485 デバイスが見つかりません)"
}

install_rule() {
    local accel_serial="$1" brake_serial="$2"
    if [ "$(id -u)" -ne 0 ]; then
        echo "エラー: ルール設置には root 権限が必要です（sudo で実行してください）。" >&2
        exit 1
    fi
    if [ "$accel_serial" = "$brake_serial" ]; then
        echo "エラー: accel と brake に同じシリアルが指定されています。" >&2
        exit 1
    fi
    cat > "$RULE_PATH" <<RULES
# driving_robot: IAI アクチュエータ用 USB-RS485(FTDI) の安定シンボリックリンク
# scripts/setup_udev.sh により生成。USB 列挙順に依存せずシリアル番号で軸を固定する。
SUBSYSTEM=="tty", ATTRS{idVendor}=="${FTDI_VENDOR}", ATTRS{idProduct}=="${FTDI_PRODUCT}", ATTRS{serial}=="${accel_serial}", SYMLINK+="actuator_accel", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="${FTDI_VENDOR}", ATTRS{idProduct}=="${FTDI_PRODUCT}", ATTRS{serial}=="${brake_serial}", SYMLINK+="actuator_brake", GROUP="dialout", MODE="0660"
RULES
    echo "設置: $RULE_PATH"
    udevadm control --reload-rules
    udevadm trigger --subsystem-match=tty --action=change
    sleep 1
    echo "シンボリックリンク:"
    ls -l /dev/actuator_accel /dev/actuator_brake 2>&1 || true
}

case "${1:-}" in
    --list|-l)
        list_devices
        ;;
    "" | -h | --help)
        echo "使い方:"
        echo "  $0 --list                                    FTDI RS485 デバイス一覧"
        echo "  sudo $0 <accel_serial> <brake_serial>        udev ルールを設置・反映"
        ;;
    *)
        if [ "$#" -ne 2 ]; then
            echo "エラー: accel と brake の2つのシリアル番号を指定してください。" >&2
            echo "  例: sudo $0 FTBB7KRI FTAQUJJJ" >&2
            exit 1
        fi
        install_rule "$1" "$2"
        ;;
esac
