#!/bin/bash

cd "$(dirname "$0")/../../.." || exit 1
ROBOT=${ROBOT:-lvuser@10.8.46.2}
SSH_OPTS=(-o LogLevel=ERROR -o ConnectTimeout=5)
STAMP=$(date +%m_%d_%H_%M)
mkdir -p robot_logs nt_backups

total=0
pull() {
  local dir=$1 dest=$2 listing size path have
  listing=$(ssh "${SSH_OPTS[@]}" "$ROBOT" "cd $dir 2>/dev/null && find . -type f -exec ls -ln {} \\;") || return
  local need=()
  while read -r _ _ _ _ size _ _ _ path; do
    [ -z "$path" ] && continue
    path=${path#./}
    have=$(stat -f %z "$dest/$path" 2>/dev/null || echo -1)
    [ "$have" != "$size" ] && need+=("$path")
  done <<< "$listing"
  [ ${#need[@]} -eq 0 ] && return
  printf '  %s\n' "${need[@]}"
  mkdir -p "$dest"
  ssh "${SSH_OPTS[@]}" "$ROBOT" "cd $dir && tar cf - $(printf '%q ' "${need[@]}")" | tar xf - -C "$dest"
  total=$((total + ${#need[@]}))
}

pull /home/lvuser/logs robot_logs
pull /u/logs robot_logs
pull /home/lvuser/foresting robot_logs/foresting
pull /u/foresting robot_logs/foresting
echo "Copied $total new or changed log files into robot_logs/"

scp "${SSH_OPTS[@]}" -q "$ROBOT:/home/lvuser/networktables.json" "nt_backups/networktables_$STAMP.json" &&
  echo "Saved prefs to nt_backups/networktables_$STAMP.json"
mkdir -p "robot_logs/iclearn/$STAMP"
scp "${SSH_OPTS[@]}" -q "$ROBOT:/home/lvuser/*.iclearn" "robot_logs/iclearn/$STAMP/" 2>/dev/null &&
  echo "Saved learner files to robot_logs/iclearn/$STAMP/" || rmdir "robot_logs/iclearn/$STAMP"
