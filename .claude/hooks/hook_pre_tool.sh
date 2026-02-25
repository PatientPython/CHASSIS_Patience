#!/bin/bash
PAYLOAD=$(cat)
TOOL_NAME=$(echo "$PAYLOAD" | jq -r '.tool_name')

if [[ "$TOOL_NAME" == "Bash" || "$TOOL_NAME" == "Edit" || "$TOOL_NAME" == "Write" || "$TOOL_NAME" == "MultiEdit" || "$TOOL_NAME" == "Replace" ]]; then
  PROTECTED_BRANCHES=("main" "v1-stable" "master")
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
  
  if [[ " ${PROTECTED_BRANCHES[*]} " =~ " ${CURRENT_BRANCH} " ]]; then
    # Exception to allow essential git branch creation and navigation on protected branches
    if [[ "$TOOL_NAME" == "Bash" ]]; then
      COMMAND=$(echo "$PAYLOAD" | jq -r '.tool_input.command')
      if [[ "$COMMAND" =~ ^[[:space:]]*git[[:space:]]+(checkout|branch|switch|status|log|diff) ]]; then
        exit 0
      fi
    fi
    
    echo "ERR: Protected branch '$CURRENT_BRANCH'. Use 'git checkout -b <branch>'." >&2
    exit 2
  fi
fi
exit 0
