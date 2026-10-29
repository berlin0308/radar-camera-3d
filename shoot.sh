#!/bin/bash
# Rig day: record one shot through the NUC's web server, then check every sensor wrote data.
# Usage:  ./shoot.sh <label> [seconds=30] [wait-before-start=5]
# Needs the shared ssh connection (~/.ssh/cm-rover.sock) and the recorder server running on the NUC.
set -u
LABEL="${1:?usage: ./shoot.sh <label> [seconds] [wait]}"
SECS="${2:-30}"
WAIT="${3:-5}"
DEST="/media/rover/e9990bb5-99a1-4186-8539-bba6f0155a77/"   # Tianshu's SSD
R="ssh -o ConnectTimeout=10 -S $HOME/.ssh/cm-rover.sock rover@192.168.8.249"
cd "$(dirname "$0")"

echo "[$LABEL] starting in ${WAIT}s - step behind the rig"; sleep "$WAIT"
# stop any capture left running (e.g. an interrupted earlier shot), otherwise the new start is ignored
$R "curl -s -m 5 -X POST -H 'Content-Type: application/json' -d '{\"action\":\"stop\"}' localhost:5000/command" >/dev/null; sleep 2
$R "curl -s -m 5 -X POST -H 'Content-Type: application/json' -d '{\"action\":\"start\",\"path\":\"$DEST\"}' localhost:5000/command" || exit 1
echo; echo "[$LABEL] recording ${SECS}s ..."; sleep "$SECS"
$R "curl -s -m 5 -X POST -H 'Content-Type: application/json' -d '{\"action\":\"stop\"}' localhost:5000/command"
echo; sleep 4

$R "T=\$(ls -d ${DEST}20*/ | tail -1); echo \"trace: \$(basename \$T)\"; for s in radar lidar camera imu; do echo \"  \$s frames: \$(( \$(stat -c %s \$T/\$s/ts 2>/dev/null || echo 0) / 8 ))\"; done" | tee /tmp/shoot_last.txt
TRACE=$(awk '/^trace:/{print $2}' /tmp/shoot_last.txt)
mkdir -p notes
[ -f notes/shots.csv ] || echo "trace,label,seconds" > notes/shots.csv
echo "$TRACE,$LABEL,$SECS" >> notes/shots.csv
echo "[$LABEL] logged to notes/shots.csv"
