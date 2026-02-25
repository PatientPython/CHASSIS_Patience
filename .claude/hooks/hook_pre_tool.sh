#!/bin/bash
PAYLOAD=$(cat)
TOOL_NAME=$(echo "$PAYLOAD" | jq -r '.tool_name')

if [[ "$TOOL_NAME" == "Bash" || "$TOOL_NAME" == "Edit" || "$TOOL_NAME" == "Write" || "$TOOL_NAME" == "MultiEdit" || "$TOOL_NAME" == "Replace" ]]; then
  PROTECTED_BRANCHES=("main" "v1-stable" "master")
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
  
  if [[ " ${PROTECTED_BRANCHES[*]} " =~ " ${CURRENT_BRANCH} " ]]; then
    if [[ "$TOOL_NAME" == "Bash" ]]; then
      COMMAND=$(echo "$PAYLOAD" | jq -r '.tool_input.command')
      
      # [FIRST PRINCIPLE]: All modifications must be reversible via a sandbox.
      # 1. Create/Navigate to a sandbox (git checkout -b, git switch, etc.)
      # 2. Perform read-only environment checks (git status, ls, pwd, dir, etc.)
      
      # Strip 'cd' chains and env variable bindings to inspect the true base command
      CMD_CORE=$(echo "$COMMAND" | sed -E 's/^[[:space:]]*//' | sed -E 's/^cd[[:space:]]+[^;&|]+[;&|][[:space:]]*//' | sed -E 's/^([A-Za-z_][A-Za-z0-9_]*=[^[:space:]]+[[:space:]]*)+//')
      EXE=$(echo "$CMD_CORE" | awk '{print $1}')
      
      if [[ "$EXE" == "git" ]]; then
        # Whitelist safe git commands (navigation & sandbox only)
        if echo "$CMD_CORE" | grep -Eq '(^|[[:space:]])(checkout|switch|branch|status|log|diff|show|rev-parse)([[:space:]]|$)'; then
          exit 0
        fi
      elif [[ "$EXE" =~ ^(ls|dir|cat|echo|pwd|tree|find|grep)$ ]]; then
        # Whitelist pure read-only system tools
        exit 0
      fi
      
      echo "Error (Baseline protection): You are on '$CURRENT_BRANCH'." >&2
      echo ">> ACTION REQUIRED: Operations must be reversible. Switch to a working branch via 'git checkout -b <branch>' BEFORE editing. DO NOT use git worktree." >&2
      exit 2
    fi
    
    echo "Error (Baseline protection): Direct file modification on '$CURRENT_BRANCH' is prohibited." >&2
    echo ">> ACTION REQUIRED: Switch to a working branch via 'git checkout -b <branch>' first. DO NOT use git worktree." >&2
    exit 2
  fi
fi
exit 0
