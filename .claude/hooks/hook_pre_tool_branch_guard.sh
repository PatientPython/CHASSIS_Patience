#!/bin/bash
set -e

unset GIT_DIR
unset GIT_WORK_TREE
unset GIT_COMMON_DIR
unset GIT_INDEX_FILE
unset GIT_OBJECT_DIRECTORY

PAYLOAD="$(cat)"
TOOL_NAME="$(echo "$PAYLOAD" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("tool_name", ""))' 2>/dev/null || echo "")"

if [ -z "$TOOL_NAME" ]; then
  exit 0
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  exit 0
fi

CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "")"
case "$CURRENT_BRANCH" in
  main|master|v1-stable)
    ;;
  *)
    exit 0
    ;;
esac

case "$TOOL_NAME" in
  Edit|Write|MultiEdit|NotebookEdit|Bash|Agent)
    echo "Protected branch '$CURRENT_BRANCH': blocked tool '$TOOL_NAME'. Create/switch to a work branch first (git checkout -b work/<name>)." >&2
    exit 2
    ;;
  *)
    exit 0
    ;;
esac
