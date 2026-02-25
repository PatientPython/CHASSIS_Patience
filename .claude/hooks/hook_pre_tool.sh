#!/bin/bash
PAYLOAD=$(cat)
TOOL_NAME=$(echo "$PAYLOAD" | jq -r '.tool_name')

if [[ "$TOOL_NAME" == "Bash" || "$TOOL_NAME" == "Edit" || "$TOOL_NAME" == "Write" || "$TOOL_NAME" == "MultiEdit" || "$TOOL_NAME" == "Replace" ]]; then
  PROTECTED_BRANCHES=("main" "v1-stable" "master")
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
  
  if [[ " ${PROTECTED_BRANCHES[*]} " =~ " ${CURRENT_BRANCH} " ]]; then
    echo "Error: Modifying protected branch '$CURRENT_BRANCH' is strictly prohibited." >&2
    exit 2
  fi
fi
exit 0
