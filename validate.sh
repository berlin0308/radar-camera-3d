#!/bin/bash
# Validate every trace under data/raw/ (use after copying from a disk instead of sync.sh).
# Usage: ./validate.sh            (all traces)
#        ./validate.sh <trace>    (one trace folder name)
cd "$(dirname "$0")"
source .venv/bin/activate
if [ -n "$1" ]; then set -- "data/raw/$1"; else set -- data/raw/*/; fi
for t in "$@"; do
  t="${t%/}"; [ -f "$t/config.yaml" ] || continue
  echo; echo "================ $t"
  roverd validate "$t"
  python check_valid.py "$t"
  echo "-- lidar file sizes (rng/rfl/nir must NOT be 0):"
  ls -l "$t/lidar" | awk 'NR>1{print "   " $5 "\t" $9}'
done
