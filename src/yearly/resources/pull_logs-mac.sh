#!/bin/bash

cd "$(dirname "$0")/../../.." || exit 1
ROBOT=${ROBOT:-lvuser@10.8.46.2}
SSH_OPTS=(-o LogLevel=ERROR -o ConnectTimeout=5)
STAMP=$(date +%m_%d_%H_%M)
mkdir -p robot_logs nt_backups

total=0
for dir in /home/lvuser/logs /u/logs; do
  listing=$(ssh "${SSH_OPTS[@]}" "$ROBOT" "cd $dir 2>/dev/null && find . -type f -exec ls -ln {} \\;") || continue
  need=()
  while read -r _ _ _ _ size _ _ _ path; do
    [ -z "$path" ] && continue
    path=${path#./}
    have=$(stat -f %z "robot_logs/$path" 2>/dev/null || echo -1)
    [ "$have" != "$size" ] && need+=("$path")
  done <<< "$listing"
  [ ${#need[@]} -eq 0 ] && continue
  printf '  %s\n' "${need[@]}"
  ssh "${SSH_OPTS[@]}" "$ROBOT" "cd $dir && tar cf - $(printf '%q ' "${need[@]}")" | tar xf - -C robot_logs
  total=$((total + ${#need[@]}))
done
echo "Copied $total new or changed log files into robot_logs/"

scp "${SSH_OPTS[@]}" -q "$ROBOT:/home/lvuser/networktables.json" "nt_backups/networktables_$STAMP.json" &&
  echo "Saved prefs to nt_backups/networktables_$STAMP.json"
mkdir -p "robot_logs/iclearn/$STAMP"
scp "${SSH_OPTS[@]}" -q "$ROBOT:/home/lvuser/*.iclearn" "robot_logs/iclearn/$STAMP/" 2>/dev/null &&
  echo "Saved learner files to robot_logs/iclearn/$STAMP/" || rmdir "robot_logs/iclearn/$STAMP"
