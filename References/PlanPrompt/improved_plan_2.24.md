# Implementation Plan: Claude Code Session & Branch Management Architecture

This plan aims to refactor the Session-Git synchronization workflow, deeply aligning with Claude Code's official extension mechanisms (Skills and Hooks) and its core session-based design philosophy. The goal is to bind Git branches to Claude Code sessions seamlessly, using minimal tokens and leveraging native lifecycle events.

## Proposed Changes

### 1. Branch Philosophy
The architecture defines two distinct types of Git branches:
1. **Protected Branch**: The official codebase (`main`, `v1-stable`, etc.). It cannot be directly modified by AI.
2. **Working Branch**: The branch where AI conversations are mounted. Subsequent conversations will be mounted on the last branch by default, and you need to manually enter commands to switch branches.

### 2. General Rules & Prompt Guidelines (`CLAUDE.md`)
To govern AI behavior at a high level across sessions, the following rule must be added to the project's `CLAUDE.md`:
- **Protected Branch Session Start Rule**: "If I am currently on a protected branch and start a session to ask a question, you must use the built-in `askquestiontools` (or a prompt-based hook) to ask me whether I want to create another protected branch based on this protected branch, or directly start the conversation. If I choose to start the conversation directly, a working branch will be created immediately."

### 3. Data Models & Configuration (Project-Local Scope)
All configurations and scripts will be stored locally within the project to ensure portability and repository-specific encapsulation.
- **Session Mapping File (`.claude/session-git-map.json`)**:
  - Stores mapping between `${CLAUDE_SESSION_ID}` and active working branch names, ensuring sessions and branches are locked 1:1.
- **Configuration Registry ([.claude/settings.json](file:///e:/CODE_project/BalanceSoldier/ChassisControl/CHASSIS_Patience/.claude/settings.json))**:
  - Dedicated project-local configuration file where all custom hooks will be registered under the `"hooks"` key.

### 4. Hooks Implementation (The Guardrails)
Hooks will rely strictly on the official lifecycle events and the structured JSON output / exit codes to guide Claude Code.

#### [NEW] `.claude/scripts/hook_pre_tool.sh`
- **Event**: `PreToolUse` (matcher: `Bash|Edit|Write`)
- **Purpose**: A strict guard against unauthorized modifications to protected branches.
- **Mechanism**: Reads the incoming JSON payload. If targeting a protected branch, it prints a warning to `stderr` and executes an `exit 2` or returns a structured JSON payload (`{"hookSpecificOutput": {"permissionDecision": "deny", "permissionDecisionReason": "Cannot write to protected branch main"}}`).

#### [NEW] `.claude/scripts/hook_prompt_submit.sh` (Checkpoint Control)
- **Event**: `UserPromptSubmit` (matcher: `*`) / `PostToolUse`
- **Purpose**: Silent snapshot check detecting uncommitted changes and creating localized checkpoints.
- **Checkpoint Naming Convention**: Commits must strictly follow the format: `CP: "A brief introduction to the modifications in this submission (focusing on a global perspective to show the role of these modifications in the overall plan)"`. *Rule: Do not write metadata like time or which specific files were modified.*

#### [NEW] `.claude/scripts/hook_session_stop.sh`
- **Event**: `Stop` or `SessionEnd`
- **Purpose**: A `prompt-based` or `agent-based` hook to double-check if all requested Git tasks for the session are successfully committed before the session winds down.

### 5. Skills Implementation (The User Commands)
Skills will be encapsulated into their respective command directories under `.claude/skills/`.

#### [MODIFY] `.claude/skills/goto/SKILL.md` (`/goto`)
- **Frontmatter**: `disable-model-invocation: true`, `user-invocable: true`
- **Behavior**:
  - If the `$ARGUMENTS` is a `commit id`, check out that precise commit.
  - If the `$ARGUMENTS` is a `branch name`, check out the HEAD of that branch.
  - If checking out a detached HEAD, output a warning for visibility.

#### [MODIFY] `.claude/skills/fork-explore/SKILL.md` (`/fork-explore`)
- **Frontmatter**: `context: fork`, `agent: Explore` (optional if heavy analysis required)
- **Behavior**:
  - Conversations under the same working branch follow the official fork design logic. 
  - Forking adds a new sub-branch to the current working branch in Git, maintaining correspondence between the Git commit ID and Claude Session ID.
  - The forked conversation is automatically mounted on this new sub-branch.
  - If the git pointer is not manually modified, subsequent conversations will default to the current branch.

#### [MODIFY] `.claude/skills/merge/SKILL.md` (`/merge`)
- **Frontmatter**: `disable-model-invocation: true`, `allowed-tools: Bash, Read, Write`
- **Behavior**: **Advisory mode only.**
  - Claude is strictly forbidden from executing the merge itself.
  - Generates a block-by-block "merge guide" explaining the reason for each difference.
  - **Output Destination**: The guide must be written to the `References/MergeGuide/` subfolder in the root directory.
  - **Naming Convention**: The file name must cleanly indicate what was merged (e.g., `feat-ui-elements_merged_into_main.md`).
  - Instruct the user to execute the merge manually.

#### [MODIFY] `.claude/skills/branch-status/SKILL.md` (`/branch-status`)
- **Behavior**: We will optimize this as a minimal-token skill using `disable-model-invocation: true` to prevent autonomous firing. We inject the script output directly into the skill prompt using dynamic context (`!`), which prompts Claude to cleanly echo back the graph without prolonged reasoning.

## Code Achievement Guide
Specific instructions mapping the logic into scripts and frontmatter details remain standard.

1. Init local `.claude/settings.json`.
2. Map `CP: "..."` rules into `.claude/skills/` and `hook_pre_tool.sh`.
3. Set `CLAUDE.md` to trigger `askquestion` API on protected branch session starts.

## Verification Plan

### Automated Hooks Verification
1. **Protected Branch Protection**: On `main`, attempting to write code prompts the user via `askquestiontools` (driven by `CLAUDE.md`) or yields an `exit 2` block in execution.
2. **Checkpoint Naming**: Verify Git logs ensure `CP: "..."` global-perspective descriptions without timestamps.

### Manual Skills Verification
1. **Merge Guide Check (`/merge feature-branch`)**: Runs `/merge` resulting in a new markdown file generated strictly inside `References/MergeGuide/`, leaving the git branches unchanged.
2. **Goto Accuracy (`/goto [target]`)**: Verifies it hits detached HEADs or branch heads correctly.
3. **Fork Sub-branching (`/fork-explore`)**: Confirms `--fork-session` creates logically nested Git working branches retaining Session-ID mappings.
