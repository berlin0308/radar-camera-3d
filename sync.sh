#!/bin/bash
# Rig day: pull new traces from the rover NUC, then validate each new one.
# Usage:  ./sync.sh <rig-name> [remote-data-dir] [user]
#   rig-name : NUC hostname (e.g. rover1) or its IP (e.g. 192.168.8.123)
#   remote   : default ~/red-rover/collect/data/  (confirm with Tianshu; may differ if recording to the external SSD /data-a)
#   user     : default rover
# The Mac must be on the rig's wifi. Only timestamped trace folders (2026-...) are pulled, so other people's folders on a shared SSD are skipped.
set -u
NAME="${1:?usage: ./sync.sh <rig-name> [remote-dir] [user]}"
REMOTE="${2:-~/red-rover/collect/data/}"
RUSER="${3:-rover}"
cd "$(dirname "$0")"

case "$NAME" in *.*) HOST="$NAME";; *) HOST="$NAME.local";; esac   # IP or name
rsync -av --partial --progress --include="20[0-9][0-9]-*/***" --exclude="*" "${RUSER}@${HOST}:${REMOTE}" data/raw/ || { echo "rsync failed"; exit 1; }

source .venv/bin/activate
touch data/validated.txt
for t in data/raw/*/; do
  t="${t%/}"
  grep -qxF "$t" data/validated.txt && continue
  [ -f "$t/config.yaml" ] || continue
  echo; echo "================ $t"
  if roverd validate "$t" && python check_valid.py "$t"; then
    echo "$t" >> data/validated.txt
  else
    echo "!! $t FAILED — check before moving on"
  fi
done
df -h . | tail -1 | awk '{print "\nMac free space: " $4}'
