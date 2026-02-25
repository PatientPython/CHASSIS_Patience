#!/bin/bash
PAYLOAD=$(cat)
TOOL_NAME=$(echo "$PAYLOAD" | jq -r '.tool_name')

if [[ "$TOOL_NAME" == "Bash" || "$TOOL_NAME" == "Edit" || "$TOOL_NAME" == "Write" || "$TOOL_NAME" == "MultiEdit" || "$TOOL_NAME" == "Replace" ]]; then
  PROTECTED_BRANCHES=("main" "v1-stable" "master")
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
  
  if [[ " ${PROTECTED_BRANCHES[*]} " =~ " ${CURRENT_BRANCH} " ]]; then
    # Exception to allow git and basic diagnostic commands on protected branches
    if [[ "$TOOL_NAME" == "Bash" ]]; then
      COMMAND=$(echo "$PAYLOAD" | jq -r '.tool_input.command')
      if [[ "$COMMAND" =~ ^[[:space:]]*(git|ls|dir|cat|echo|pwd|tree|find|grep|GIT_CONFIG_PARAMETERS) ]]; then
        exit 0
      fi
    fi
    echo "Error: Modifying protected branch '$CURRENT_BRANCH' is strictly prohibited." >&2
    exit 2
  fi
fi
exit 0
