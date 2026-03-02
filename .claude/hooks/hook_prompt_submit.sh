#!/bin/bash
set -e

unset GIT_DIR
unset GIT_WORK_TREE
unset GIT_COMMON_DIR
unset GIT_INDEX_FILE
unset GIT_OBJECT_DIRECTORY

PAYLOAD="$(cat)"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [ -n "${CLAUDE_PROJECT_DIR:-}" ] && [ -d "$CLAUDE_PROJECT_DIR/.claude" ]; then
  PROJECT_DIR="$CLAUDE_PROJECT_DIR"
fi
POLICY_FILE="$PROJECT_DIR/.claude/hooks/git-harness-agent-policy.md"

PROMPT_TEXT="$(echo "$PAYLOAD" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("prompt", ""))' 2>/dev/null || echo "")"
PROMPT_PREFIX="$(printf '%s' "$PROMPT_TEXT" | python3 -c 'import sys; s=sys.stdin.read().replace("\n"," ").replace("\r"," ").strip(); print(s[:10])')"

if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if [ -n "$(git status --porcelain 2>/dev/null)" ]; then
    git add -A >/dev/null 2>&1 || true
    git commit -m "CPST:${PROMPT_PREFIX}" >/dev/null 2>&1 || true
  fi
fi

if [ -f "$POLICY_FILE" ]; then
  ADDITIONAL_CONTEXT="$(cat "$POLICY_FILE")"
else
  ADDITIONAL_CONTEXT="MANDATORY: Enforce protected-branch read-only mode, guide user to create a work branch with git checkout -b work/<name>, and never use git worktree."
fi

export ADDITIONAL_CONTEXT
python3 - <<'PY'
import json
import os

print(json.dumps({
    "hookSpecificOutput": {
        "hookEventName": "UserPromptSubmit",
        "additionalContext": os.environ.get("ADDITIONAL_CONTEXT", "")
    }
}, ensure_ascii=False))
PY

exit 0
