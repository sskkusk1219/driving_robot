#!/usr/bin/env bash
# scripts/setup_nut.sh
#
# APC Smart-UPS 750 (SUA750JB) を NUT (Network UPS Tools) で管理するための
# セットアップスクリプト。
#
# 前提条件:
#   - APC Smart-UPS 750 がシリアル(DB-9)→USB変換アダプタで接続済み
#   - 変換アダプタのデバイスパスを確認: ls /dev/ttyUSB*
#   - root または sudo 権限があること
#
# 使用方法:
#   # まず UPS のデバイスパスを確認
#   ls /dev/ttyUSB*
#
#   # デバイスパスを指定して実行（デフォルト: /dev/ttyUSB2）
#   sudo bash scripts/setup_nut.sh /dev/ttyUSB2
#
# 確認方法:
#   sudo upsc apcups@localhost         # UPS 状態確認
#   sudo systemctl status nut-server   # サービス状態確認

set -euo pipefail

UPS_PORT="${1:-/dev/ttyUSB2}"
UPS_NAME="apcups"
NUT_USER="monuser"
NUT_PASS="monpassword"

echo "=== NUT セットアップ開始 ==="
echo "  UPS デバイス: ${UPS_PORT}"
echo "  UPS 名: ${UPS_NAME}"
echo ""

# 1. NUT インストール
echo "[1/6] NUT インストール..."
apt-get install -y nut nut-client

# 2. nut.conf（スタンドアロンモード）
echo "[2/6] /etc/nut/nut.conf 設定..."
cat > /etc/nut/nut.conf << EOF
# NUT 動作モード
# standalone: upsd と upsmon を同一マシンで実行
MODE=standalone
EOF

# 3. ups.conf（APC Smart プロトコル / シリアル接続）
echo "[3/6] /etc/nut/ups.conf 設定..."
cat > /etc/nut/ups.conf << EOF
[${UPS_NAME}]
    driver = apcsmart
    port = ${UPS_PORT}
    desc = "APC Smart-UPS 750 (SUA750JB)"
    # APC Smart プロトコル対応: 9600 bps, 8N1
EOF

# 4. upsd.conf（NUT サーバー設定）
echo "[4/6] /etc/nut/upsd.conf 設定..."
cat > /etc/nut/upsd.conf << EOF
# NUT サーバーリスン設定
# ローカルのみ（Raspberry Pi 上の Python から接続）
LISTEN 127.0.0.1 3493
LISTEN ::1 3493
EOF

# 5. upsd.users（接続ユーザー）
echo "[5/6] /etc/nut/upsd.users 設定..."
cat > /etc/nut/upsd.users << EOF
[${NUT_USER}]
    password = ${NUT_PASS}
    actions = SET
    instcmds = ALL
    upsmon master
EOF
chmod 640 /etc/nut/upsd.users

# 6. upsmon.conf（UPS 監視・シャットダウン制御）
echo "[6/6] /etc/nut/upsmon.conf 設定..."
cat > /etc/nut/upsmon.conf << EOF
# UPS 監視設定
# MONITOR <ups>@<host>:<port> <powersupply_count> <username> <password> <type>
MONITOR ${UPS_NAME}@localhost 1 ${NUT_USER} ${NUT_PASS} master

# バッテリー残量の「低」閾値 [%]（この値以下でシャットダウンシーケンス開始）
MINSUPPLIES 1
SHUTDOWNCMD "/sbin/shutdown -h +0"
POLLFREQ 5
POLLFREQALERT 5
HOSTSYNC 15
DEADTIME 15
POWERDOWNFLAG /etc/killpower

# バッテリー警告残量 [%]（デフォルト 20%）
# ※ driving_robot の走行前チェックも 20% 以上を要求している
RBWARNTIME 43200
NOCOMMWARNTIME 300
FINALDELAY 5
EOF

# NUT デバイスへのアクセス権（シリアルポートへの書き込み）
echo ""
echo "=== NUT サービス設定 ==="

# nut ユーザーをシリアルポートグループに追加
usermod -a -G dialout nut || true

# サービスの有効化・起動
systemctl enable nut-driver nut-server nut-monitor 2>/dev/null || \
    systemctl enable nut-driver.service nut-server.service nut-monitor.service 2>/dev/null || true
systemctl restart nut-driver nut-server nut-monitor 2>/dev/null || \
    systemctl restart nut-driver.service nut-server.service nut-monitor.service 2>/dev/null || true

echo ""
echo "=== セットアップ完了 ==="
echo ""
echo "動作確認:"
echo "  sudo upsc ${UPS_NAME}@localhost"
echo "  sudo systemctl status nut-server"
echo ""
echo "期待される出力例:"
echo "  battery.charge: 95"
echo "  ups.status: OL CHRG"
echo ""
echo "UPS デバイスが認識されない場合は以下を確認:"
echo "  ls -la ${UPS_PORT}"
echo "  groups nut"
echo "  journalctl -u nut-driver -n 50"
