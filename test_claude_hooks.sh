#!/usr/bin/env bash
set -e

echo "=========================================="
echo "    🧪 Testing Claude Code Guardrails"
echo "=========================================="

HOOK_DIR="/c/Users/35766/.claude/hooks/guardrails"
GUARD="$HOOK_DIR/guard-branch.sh"
AUTO_COMMIT="$HOOK_DIR/auto-commit.sh"
SQUASH="$HOOK_DIR/squash-checkpoints.sh"

# Setup temp directory
TEST_DIR="$(mktemp -d)"
cd "$TEST_DIR"
echo "📂 Temporary test directory: $TEST_DIR"

git init -q
git config user.name "Test User"
git config user.email "test@example.com"
git commit -q --allow-empty -m "Initial commit"

# ==========================================
# Test 1: guard-branch.sh on main
# ==========================================
echo -n "Test 1: guard-branch.sh - Protected branch (main) -> "
git checkout -q -b main 2>/dev/null || git branch -M main
if bash "$GUARD" 2>/dev/null; then
    echo "❌ FAILED (Should have blocked)"
else
    echo "✅ PASSED (Blocked correctly)"
fi

# ==========================================
# Test 2: guard-branch.sh on vibe branch
# ==========================================
echo -n "Test 2: guard-branch.sh - Unprotected branch -> "
git checkout -q -b vibe/test-branch
if bash "$GUARD" 2>/dev/null; then
    echo "✅ PASSED (Allowed correctly)"
else
    echo "❌ FAILED (Should have allowed)"
fi

# ==========================================
# Test 3: guard-branch.sh with uncommitted changes
# ==========================================
echo -n "Test 3: guard-branch.sh - Uncommitted changes -> "
echo "hello" > test_file.txt
if bash "$GUARD" 2>/dev/null; then
    echo "❌ FAILED (Should have blocked)"
else
    echo "✅ PASSED (Blocked correctly)"
fi
git add test_file.txt && git commit -q -m "Add test file"

# ==========================================
# Test 4: auto-commit.sh
# ==========================================
echo "Test 4: auto-commit.sh - Auto commit on Write"
echo "modified content" > test_file.txt
# Mock JSON payload from Claude Code
PAYLOAD='{"tool_name": "Edit", "tool_input": {"file_path": "test_file.txt"}}'
echo "$PAYLOAD" | bash "$AUTO_COMMIT"
if git log -1 --oneline | grep -q 'checkpoint: Edit test_file.txt'; then
    echo "✅ PASSED (Auto-committed correctly)"
else
    echo "❌ FAILED (Did not commit properly)"
    git status
fi

# ==========================================
# Test 5: squash-checkpoints.sh (Verify bug with interleaved commits)
# ==========================================
echo "Test 5: squash-checkpoints.sh - Interleaved commits bug test"

# Create a history:
# 1. checkpoint
# 2. regular commit
# 3. checkpoint
# 4. checkpoint
echo "file 1" > file1.txt && echo '{"tool_name": "Write", "tool_input": {"file_path": "file1.txt"}}' | bash "$AUTO_COMMIT"
echo "regular change" > file2.txt && git add file2.txt && git commit -q -m "Regular user commit"
echo "file 3" > file3.txt && echo '{"tool_name": "Write", "tool_input": {"file_path": "file3.txt"}}' | bash "$AUTO_COMMIT"
echo "file 4" > file4.txt && echo '{"tool_name": "Write", "tool_input": {"file_path": "file4.txt"}}' | bash "$AUTO_COMMIT"

echo "Current git history before squash:"
git log --oneline | head -n 6

# Total checkpoints in history will be 4 (from test 4 + 3 here)
# The squash script will do `git reset --soft HEAD~4`
echo "Running squash-checkpoints.sh..."
bash "$SQUASH"

echo "Current git history after squash:"
git log --oneline | head -n 6

echo "Testing .gitignore backup logic in auto-commit.sh..."
# Test 6: backup logic
echo "secret" > .env
echo ".env" > .gitignore
git add .gitignore
git commit -q -m "add gitignore"
PAYLOAD='{"tool_name": "Edit", "tool_input": {"file_path": ".env"}}'
echo "$PAYLOAD" | bash "$AUTO_COMMIT"
if ls .claude/backups/.env.* 1> /dev/null 2>&1; then
    echo "✅ PASSED (Backup created for gitignored file)"
else
    echo "❌ FAILED (Backup not created)"
fi


echo "Cleaning up..."
rm -rf "$TEST_DIR"
