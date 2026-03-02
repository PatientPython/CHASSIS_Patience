#!/bin/bash
set -e

unset GIT_DIR
unset GIT_WORK_TREE
unset GIT_COMMON_DIR
unset GIT_INDEX_FILE
unset GIT_OBJECT_DIRECTORY

PAYLOAD="$(cat)"
MESSAGE="$(echo "$PAYLOAD" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("last_assistant_message", ""))' 2>/dev/null || echo "")"
PREFIX="$(printf '%s' "$MESSAGE" | python3 -c 'import sys; s=sys.stdin.read().replace("\n"," ").replace("\r"," ").strip(); print(s[:10])')"

if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if [ -n "$(git status --porcelain 2>/dev/null)" ]; then
    git add -A >/dev/null 2>&1 || true
    git commit -m "CPED:${PREFIX}" >/dev/null 2>&1 || true
  fi
fi

exit 0
