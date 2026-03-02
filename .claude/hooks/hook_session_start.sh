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

export ADDITIONAL_CONTEXT
python3 - <<'PY'
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
