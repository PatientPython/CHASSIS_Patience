#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
GUIDE_FILE="$PROJECT_DIR/.claude/hooks/workflow-guide.md"

if [ -f "$GUIDE_FILE" ]; then
  ADDITIONAL_CONTEXT="$(cat "$GUIDE_FILE")"
else
  ADDITIONAL_CONTEXT="Workflow guide is missing at .claude/hooks/workflow-guide.md. Ask user to restore it before using workflow skills."
fi

# Windows compatibility: use 'python' if 'python3' not found
if command -v python3 >/dev/null 2>&1; then
  PYTHON_CMD="python3"
elif command -v python >/dev/null 2>&1; then
  PYTHON_CMD="python"
else
  echo "Error: Python not found" >&2
  exit 1
fi

export ADDITIONAL_CONTEXT PYTHON_CMD
"$PYTHON_CMD" - <<'PY'
import json
import os

print(json.dumps({
    "hookSpecificOutput": {
        "hookEventName": "SessionStart",
        "additionalContext": os.environ.get("ADDITIONAL_CONTEXT", "")
    }
}, ensure_ascii=False))
PY

exit 0
