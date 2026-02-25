#!/bin/bash
PROTECTED_BRANCHES=("main" "v1-stable" "master")
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)

if [[ " ${PROTECTED_BRANCHES[*]} " =~ " ${CURRENT_BRANCH} " ]]; then
  echo "SYSTEM ALERT: You are on a protected branch. Stop and ask the user whether to create a new protected branch or a working branch."
  exit 0
fi

MAP_FILE="$CLAUDE_PROJECT_DIR/.claude/session-git-map.json"
if [ ! -f "$MAP_FILE" ]; then
  echo "{}" > "$MAP_FILE"
fi

SAVED_BRANCH=$(jq -r ".\"$CLAUDE_SESSION_ID\"" "$MAP_FILE" 2>/dev/null)

if [ "$SAVED_BRANCH" == "null" ] || [ -z "$SAVED_BRANCH" ]; then
  jq ".\"$CLAUDE_SESSION_ID\" = \"$CURRENT_BRANCH\"" "$MAP_FILE" > "${MAP_FILE}.tmp" && mv "${MAP_FILE}.tmp" "$MAP_FILE"
elif [ "$SAVED_BRANCH" != "$CURRENT_BRANCH" ]; then
  echo "SYSTEM ALERT: Branch mismatch. Stop and ask the user: 1) Mount this session on the current working branch? 2) Check out the previous working branch? 3) Use /fork-explore to create a new sub-working branch?"
fi
exit 0
