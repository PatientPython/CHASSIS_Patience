#!/bin/bash
if ! git diff-index --quiet HEAD --; then
  git add -A
  PAYLOAD=$(cat)
  EVENT=$(echo "$PAYLOAD" | jq -r '.hook_event_name')
  if [ "$EVENT" == "UserPromptSubmit" ]; then
      PROMPT=$(echo "$PAYLOAD" | jq -r '.prompt' | head -n 1 | cut -c 1-50)
      git commit -m "CP: Checkpoint before prompt: $PROMPT"
  else
      git commit -m "CP: Automatic checkpoint"
  fi
fi
exit 0
