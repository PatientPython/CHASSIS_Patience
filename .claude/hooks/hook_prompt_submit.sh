#!/bin/bash
PAYLOAD=$(cat)
EVENT=$(echo "$PAYLOAD" | jq -r '.hook_event_name')

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
PROTECTED_BRANCHES=("main" "v1-stable" "master")

if [ "$EVENT" == "UserPromptSubmit" ] && [[ " ${PROTECTED_BRANCHES[*]} " =~ " ${CURRENT_BRANCH} " ]]; then
  echo "[SYS_WARN]: On protected branch '$CURRENT_BRANCH'. Read-only. You MUST create a working branch (git checkout -b) BEFORE editing. DO NOT use worktrees."
fi

if ! git diff-index --quiet HEAD --; then
  git add -A
  if [ "$EVENT" == "UserPromptSubmit" ]; then
      PROMPT=$(echo "$PAYLOAD" | jq -r '.prompt' | head -n 1 | tr -d '\n\r' | cut -c 1-50)
      git commit -m "CP: Checkpoint before prompt: $PROMPT" >/dev/null 2>&1
  else
      git commit -m "CP: Automatic checkpoint" >/dev/null 2>&1
  fi
fi
exit 0
