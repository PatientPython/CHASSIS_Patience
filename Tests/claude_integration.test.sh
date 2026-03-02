#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CLAUDE_DIR="$PROJECT_DIR/.claude"

PASS=0
FAIL=0

pass() { echo "[PASS] $1"; PASS=$((PASS+1)); }
fail() { echo "[FAIL] $1"; FAIL=$((FAIL+1)); }

assert_file() {
  local path="$1"
  local label="$2"
  if [ -f "$path" ]; then
    pass "$label"
  else
    fail "$label (missing: $path)"
  fi
}

echo "=== v6 Workflow Integration Tests ==="
echo "Project: $PROJECT_DIR"

assert_file "$CLAUDE_DIR/settings.json" "project settings exists"
assert_file "$CLAUDE_DIR/hooks/hook_session_start.sh" "hook_session_start exists"
assert_file "$CLAUDE_DIR/hooks/hook_prompt_submit.sh" "hook_prompt_submit exists"
assert_file "$CLAUDE_DIR/hooks/hook_stop.sh" "hook_stop exists"
assert_file "$CLAUDE_DIR/hooks/hook_task_complete.sh" "hook_task_complete exists"
assert_file "$CLAUDE_DIR/hooks/hook_pre_tool_branch_guard.sh" "protected-branch guard hook exists"
assert_file "$CLAUDE_DIR/skills/keil-build/SKILL.md" "keil-build skill exists"

PROJECT_DIR="$PROJECT_DIR" python3 - <<'PY' || exit 1
import json
import os
from pathlib import Path

root = Path(os.environ['PROJECT_DIR'])
settings = json.loads((root / '.claude/settings.json').read_text(encoding='utf-8'))
hooks = settings.get('hooks', {})

required = ['SessionStart','UserPromptSubmit','Stop','TaskCompleted','PreToolUse','PostToolUse']
for key in required:
    if key not in hooks:
        raise SystemExit(f"Missing hook event: {key}")

for key in ['UserPromptSubmit','Stop','TaskCompleted']:
    groups = hooks.get(key, [])
    if not groups:
        raise SystemExit(f"No hook group for {key}")
    for group in groups:
        if 'matcher' in group:
            raise SystemExit(f"Unexpected matcher in {key}; should be omitted per official docs")

pre_groups = hooks['PreToolUse']
found_guard = False
for group in pre_groups:
    for handler in group.get('hooks', []):
        if handler.get('command') == '.claude/hooks/hook_pre_tool_branch_guard.sh':
            found_guard = True
if not found_guard:
    raise SystemExit('PreToolUse guard hook not wired in settings')

skill_text = (root / '.claude/skills/keil-build/SKILL.md').read_text(encoding='utf-8')
if '$CLAUDE_PROJECT_DIR/.claude/skills/keil-build/scripts/keil-build.ps1' not in skill_text:
    raise SystemExit('keil-build is not project-level path')

print('SETTINGS_AND_SKILL_OK')
PY
pass "settings schema and project-level keil-build path"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

mkdir -p "$TMP/.claude/hooks"
cp "$CLAUDE_DIR/hooks/"*.sh "$TMP/.claude/hooks/"
cp "$CLAUDE_DIR/hooks/"*.md "$TMP/.claude/hooks/"

cat > "$TMP/.claude/plan-git-SHA.json" <<'JSON'
{
  "git_branch": "",
  "plans": [
    {
      "metadata": {
        "plan_id": "demo-plan",
        "created_at": "2026-03-02T12:00:00+08:00",
        "status": "in_progress"
      },
      "paths": {
        "plan_file_path": "References/PlanPrompt/demo-plan.json"
      },
      "git_sha": {
        "base_plan_sha": "",
        "head_plan_sha": ""
      },
      "tasks": {
        "Task_1": {
          "task_name": "Alpha",
          "status": "pending",
          "completed_at": "",
          "head_sha": ""
        },
        "Task_2": {
          "task_name": "Beta",
          "status": "pending",
          "completed_at": "",
          "head_sha": ""
        }
      }
    }
  ]
}
JSON

pushd "$TMP" >/dev/null
git init -q
git config user.name "Hook Test"
git config user.email "hook@test.local"
git config commit.gpgsign false

echo "init" > README.md
git add -A
git commit -q -m "init"
git checkout -q -B master

SESSION_OUT="$(echo '{"hook_event_name":"SessionStart","source":"startup","model":"claude-sonnet-4-6"}' | CLAUDE_PROJECT_DIR="$TMP" bash ./.claude/hooks/hook_session_start.sh)"
if echo "$SESSION_OUT" | grep -q '"hookEventName": "SessionStart"'; then
  pass "SessionStart emits hookSpecificOutput"
else
  fail "SessionStart output schema"
fi

PROTECTED_OUT="$(echo '{"tool_name":"Edit"}' | bash ./.claude/hooks/hook_pre_tool_branch_guard.sh 2>&1 || true)"
if echo "$PROTECTED_OUT" | grep -q 'Protected branch'; then
  pass "PreToolUse guard blocks risky tool on protected branch"
else
  fail "PreToolUse guard did not block on protected branch"
fi

git checkout -q -b work/test
SAFE_OUT="$(echo '{"tool_name":"Edit"}' | bash ./.claude/hooks/hook_pre_tool_branch_guard.sh 2>&1 || true)"
if [ -z "$SAFE_OUT" ]; then
  pass "PreToolUse guard allows risky tool on work branch"
else
  fail "PreToolUse guard unexpectedly blocked on work branch"
fi

echo "u1" > untracked_prompt.txt
echo '{"hook_event_name":"UserPromptSubmit","prompt":"中文提示词1234567890"}' | CLAUDE_PROJECT_DIR="$TMP" bash ./.claude/hooks/hook_prompt_submit.sh >/dev/null
if git log -1 --pretty=%B | grep -q '^CPST:'; then
  pass "UserPromptSubmit creates CPST checkpoint"
else
  fail "UserPromptSubmit checkpoint missing"
fi

echo "u2" > untracked_stop.txt
echo '{"hook_event_name":"Stop","last_assistant_message":"停止阶段消息123456"}' | bash ./.claude/hooks/hook_stop.sh >/dev/null
if git log -1 --pretty=%B | grep -q '^CPED:'; then
  pass "Stop creates CPED checkpoint (including untracked changes)"
else
  fail "Stop checkpoint missing"
fi

echo "u3" > untracked_task.txt
echo '{"hook_event_name":"TaskCompleted","task_subject":"Alpha","task_description":"desc"}' | bash ./.claude/hooks/hook_task_complete.sh >/dev/null
if git log -1 --pretty=%B | grep -q '^TASK:Alpha'; then
  pass "TaskCompleted creates TASK checkpoint"
else
  fail "TaskCompleted checkpoint missing"
fi

python3 - <<'PY'
import json
from pathlib import Path

path = Path('.claude/plan-git-SHA.json')
data = json.loads(path.read_text(encoding='utf-8'))
plans = data.get('plans', [])
assert plans, 'No plans in plan-git-SHA.json'
plan = plans[-1]
task = plan.get('tasks', {}).get('Task_1', {})
assert task.get('status') == 'completed', 'Task_1 not completed after hook'
assert task.get('head_sha'), 'Task_1 head_sha not set'
assert plan.get('git_sha', {}).get('head_plan_sha'), 'Plan head_plan_sha not set'
print('PLAN_UPDATE_OK')
PY
pass "TaskCompleted updates plan-git-SHA.json"

popd >/dev/null

echo "=== Summary ==="
echo "PASS: $PASS"
echo "FAIL: $FAIL"

if [ "$FAIL" -gt 0 ]; then
  exit 1
fi
