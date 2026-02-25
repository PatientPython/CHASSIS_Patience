#!/bin/bash
# claude_integration.test.sh
# Automated tests for Claude Code Session & Branch Management Architecture
# Execute this script directly or via Claude Code: `bash Tests/claude_integration.test.sh`

# Ensure we are executing from the root of the project
PROJECT_DIR=$(pwd)
if [[ "$PROJECT_DIR" == *"Tests"* ]]; then
    cd ..
    PROJECT_DIR=$(pwd)
fi

export CLAUDE_PROJECT_DIR="$PROJECT_DIR"
MAP_FILE="$PROJECT_DIR/.claude/session-git-map.json"

# Color constants
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

TEST_PASSED=0
TEST_FAILED=0

# Helper function for assertions
assert() {
    local condition=$1
    local description=$2
    local details=$3
    if eval "$condition"; then
        echo -e "[ ${GREEN}PASS${NC} ] $description"
        ((TEST_PASSED++))
    else
        echo -e "[ ${RED}FAIL${NC} ] $description"
        if [ -n "$details" ]; then
            echo -e "         ${RED}Details: ${details}${NC}"
        fi
        ((TEST_FAILED++))
    fi
}

echo "=== Claude Code Hook Integration Tests ==="
echo "Working Directory: $PROJECT_DIR"

# Clean state but back up original
mkdir -p "$PROJECT_DIR/.claude"
if [ -f "$MAP_FILE" ]; then
    cp "$MAP_FILE" "${MAP_FILE}.bak"
fi
echo "{}" > "$MAP_FILE"

# =========================================================
# 1. hook_session_start.sh validation
# =========================================================
echo -e "\n--- 1. Testing Session Start Hook (Working Branch Integrity) ---"

# 1.1 Protected branch check
export MOCK_BRANCH="main"
OUTPUT=$(bash -c '
    function git() { if [[ "$1" == "rev-parse" ]]; then echo "$MOCK_BRANCH"; else command git "$@"; fi; }
    export -f git
    bash "$CLAUDE_PROJECT_DIR/.claude/hooks/hook_session_start.sh"
')
assert "[[ \"\$OUTPUT\" == *\"SYSTEM ALERT: You are on a protected branch.\"* ]]" "Detects protected branch (main) and alerts" "Actual Output: $OUTPUT"

# 1.2 Empty session map - should register
export CLAUDE_SESSION_ID="test-session-new"
export MOCK_BRANCH="working-new"
OUTPUT=$(bash -c '
    function git() { if [[ "$1" == "rev-parse" ]]; then echo "$MOCK_BRANCH"; else command git "$@"; fi; }
    export -f git
    bash "$CLAUDE_PROJECT_DIR/.claude/hooks/hook_session_start.sh"
')
SAVED_BRANCH=$(jq -r ".\"$CLAUDE_SESSION_ID\"" "$MAP_FILE")
assert "[[ \"\$SAVED_BRANCH\" == \"working-new\" ]]" "Saves completely new session to map" "Saved branch was: $SAVED_BRANCH"

# 1.3 Branch mismatch scenario
export MOCK_BRANCH="working-other"
OUTPUT=$(bash -c '
    function git() { if [[ "$1" == "rev-parse" ]]; then echo "$MOCK_BRANCH"; else command git "$@"; fi; }
    export -f git
    bash "$CLAUDE_PROJECT_DIR/.claude/hooks/hook_session_start.sh"
')
assert "[[ \"\$OUTPUT\" == *\"SYSTEM ALERT: Branch mismatch.\"* ]]" "Detects branch mismatch from session map constraints" "Actual Output: $OUTPUT"


# =========================================================
# 2. hook_pre_tool.sh validation
# =========================================================
echo -e "\n--- 2. Testing Pre-Tool Hook (Protected Branch Ejection) ---"

# 2.1 Protected branch + Bash payload -> should exit 2
export MOCK_BRANCH="main"
PAYLOAD='{"tool_name": "Bash"}'
OUTPUT=$(echo "$PAYLOAD" | bash -c '
    function git() { if [[ "$1" == "rev-parse" ]]; then echo "$MOCK_BRANCH"; else command git "$@"; fi; }
    export -f git
    bash "$CLAUDE_PROJECT_DIR/.claude/hooks/hook_pre_tool.sh" 2>&1
    echo "EXIT_CODE=$?"
')
assert "[[ \"\$OUTPUT\" == *\"Error: Modifying protected branch\"* ]]" "Blocks tool usage on protected branch" "Output: $OUTPUT"
assert "[[ \"\$OUTPUT\" == *\"EXIT_CODE=2\"* ]]" "Exits with code 2 to trigger official Claude blockage" ""

# 2.2 Working branch + Bash payload -> should exit 0
export MOCK_BRANCH="working-branch"
OUTPUT=$(echo "$PAYLOAD" | bash -c '
    function git() { if [[ "$1" == "rev-parse" ]]; then echo "$MOCK_BRANCH"; else command git "$@"; fi; }
    export -f git
    bash "$CLAUDE_PROJECT_DIR/.claude/hooks/hook_pre_tool.sh" 2>&1
    echo "EXIT_CODE=$?"
')
assert "[[ \"\$OUTPUT\" == *\"EXIT_CODE=0\"* ]]" "Allows tool usage on working branches safely" "Output: $OUTPUT"


# =========================================================
# 3. hook_prompt_submit.sh validation
# =========================================================
echo -e "\n--- 3. Testing Prompt Submit Hook (Checkpoint Format) ---"

TEST_REPO=$(mktemp -d)
cp "$PROJECT_DIR/.claude/hooks/hook_prompt_submit.sh" "$TEST_REPO/hook_prompt_submit.sh"
cd "$TEST_REPO"
git init >/dev/null 2>&1
git config user.name "Claude Test"
git config user.email "test@example.local"
echo "Initial commit" > file.txt
git add file.txt
git commit -m "Init" >/dev/null 2>&1

# 3.1 Unstaged changes -> auto commit on UserPromptSubmit
echo "Change 1" > file.txt
PAYLOAD='{"hook_event_name": "UserPromptSubmit", "prompt": "Implement the core logic algorithm."}'
echo "$PAYLOAD" | bash hook_prompt_submit.sh >/dev/null 2>&1

LAST_COMMIT_MSG=$(git log -1 --pretty=%B)
assert "[[ \"\$LAST_COMMIT_MSG\" == *\"CP: Checkpoint before prompt: Implement the core logic\"* ]]" "Creates correctly formatted automatic checkpoint commit for prompts" "Commit message: $LAST_COMMIT_MSG"

# 3.2 Unstaged changes -> auto commit on PostToolUse
echo "Change 2" > file.txt
PAYLOAD='{"hook_event_name": "PostToolUse"}'
echo "$PAYLOAD" | bash hook_prompt_submit.sh >/dev/null 2>&1

LAST_COMMIT_MSG=$(git log -1 --pretty=%B)
assert "[[ \"\$LAST_COMMIT_MSG\" == *\"CP: Automatic checkpoint\"* ]]" "Creates generic automatic checkpoint for empty/post-tool events" "Commit message: $LAST_COMMIT_MSG"

cd "$PROJECT_DIR"
rm -rf "$TEST_REPO"


# =========================================================
# 4. Settings & Static Files Verification
# =========================================================
echo -e "\n--- 4. Configuration & Skills Static Checks ---"
SETTINGS_FILE="$PROJECT_DIR/.claude/settings.json"
assert "[ -f \"\$SETTINGS_FILE\" ]" "settings.json exists"
if command -v jq >/dev/null 2>&1 && [ -f "$SETTINGS_FILE" ]; then
    HAS_HOOKS=$(jq '.hooks | length' "$SETTINGS_FILE")
    assert "[[ \$HAS_HOOKS -gt 0 ]]" "settings.json contains active hooks mappings" "Total hook events mapped: $HAS_HOOKS"
fi

SKILLS_DIR="$PROJECT_DIR/.claude/skills"
assert "[ -f \"\$SKILLS_DIR/fork-explore/SKILL.md\" ]" "/fork-explore skill is present"
assert "[ -f \"\$SKILLS_DIR/merge/SKILL.md\" ]" "/merge skill is present"
assert "[ -f \"\$SKILLS_DIR/goto/SKILL.md\" ]" "/goto skill is present"

# Restore original state
if [ -f "${MAP_FILE}.bak" ]; then
    mv "${MAP_FILE}.bak" "$MAP_FILE"
    rm -f "${MAP_FILE}.bak"
else
    rm -f "$MAP_FILE"
fi

# Final Summary
echo -e "\n--- Summary ---"
echo -e "Tests Passed: ${GREEN}$TEST_PASSED${NC}"
echo -e "Tests Failed: ${RED}$TEST_FAILED${NC}"

if [ $TEST_FAILED -eq 0 ]; then
    echo -e "\n${GREEN}ALL INTEGRATION TESTS PASSED. Claude Code is ready for production scaling.${NC}"
    exit 0
else
    echo -e "\n${RED}SOME TESTS FAILED. Please review the output above.${NC}"
    exit 1
fi
