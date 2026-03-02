#!/bin/bash
set -e

unset GIT_DIR
unset GIT_WORK_TREE
unset GIT_COMMON_DIR
unset GIT_INDEX_FILE
unset GIT_OBJECT_DIRECTORY

# Windows compatibility: use 'python' if 'python3' not found
if command -v python3 >/dev/null 2>&1; then
  PYTHON_CMD="python3"
elif command -v python >/dev/null 2>&1; then
  PYTHON_CMD="python"
else
  echo "Error: Python not found" >&2
  exit 1
fi

PAYLOAD="$(cat)"
MESSAGE="$(echo "$PAYLOAD" | "$PYTHON_CMD" -c 'import json,sys; d=json.load(sys.stdin); print(d.get("last_assistant_message", ""))' 2>/dev/null || echo "")"
PREFIX="$(printf '%s' "$MESSAGE" | "$PYTHON_CMD" -c 'import sys; s=sys.stdin.read().replace("\n"," ").replace("\r"," ").strip(); print(s[:10])')"

if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if [ -n "$(git status --porcelain 2>/dev/null)" ]; then
    git add -A >/dev/null 2>&1 || true
    git commit -m "CPED:${PREFIX}" >/dev/null 2>&1 || true
  fi
fi

exit 0
