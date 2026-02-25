# Implementation Plan: Claude Code Session & Branch Management Architecture

This plan sets the reference architecture for the Session-Git synchronization workflow, deeply aligning with Claude Code's official extension mechanisms. The primary directive is to ensure robust branch switching mechanisms without risking errors like "null pointer dereferences." Every conversation must correspond to a specific Working Branch.

## 1. Branch Philosophy
The architecture defines two distinct types of Git branches:
1. **Protected Branch**: The official codebase baseline (e.g., `main`, `v1-stable`). AI cannot modify it directly.
2. **Working Branch**: The branch where AI workflows run. Claude Session IDs are locked to specific working branches. Subsequent conversations default to the last used working branch.

*(Note: The global behavioral rules concerning branch switches and prompting have already been securely placed in the user's `memory.md`.)*

## 2. Data Models & State Persistence
To support eventual global installation while preserving project integrity, state is strictly scoped.
- **Session Mapping File (`.claude/session-git-map.json`)**:
  - **Location Requirement**: Must ALWAYS reside in the local project's `.claude/` directory, regardless of where hooks are globally installed.
  - **Purpose**: A strict JSON map linking `${CLAUDE_SESSION_ID}` to an active Working Branch name, ensuring a 1:1 lock.

## 3. Official Hooks Implementation (The Guardrails)
Hooks will rely strictly on the official Claude Code hooks framework using native lifecycle events. Instead of fragile prompt manipulations, we will use structured interactions as officially documented.

### [NEW] `hook_session_start.sh`
- **Event**: `SessionStart` (matcher: `startup|resume`)
- **Execution Mechanism**: `type: "command"`
- **Purpose & Official Usage Tip Pipeline**:
  - **Protected Check**: If Git HEAD is on a Protected Branch, output a strict directive to standard output (stdout): `SYSTEM ALERT: You are on a protected branch. Stop and ask the user whether to create a new protected branch or a working branch.` (Standard output on `SessionStart` is officially injected into Claude's context).
  - **Missing Session Map Check**: If the system detects the user's current Git branch differs from the last saved branch for this Session ID in `session-git-map.json`, the hook will output to stdout: `SYSTEM ALERT: Branch mismatch. Stop and ask the user: 1) Mount this session on the current working branch? 2) Check out the previous working branch? 3) Use /fork-explore to create a new sub-working branch?`

### [NEW] `hook_pre_tool.sh`
- **Event**: `PreToolUse` (matcher: `Bash|Edit|Write`)
- **Execution Mechanism**: `type: "command"`
- **Purpose**: A strict technical lock against Protected Branch modification.
- **Payload Response**: Reads the incoming JSON tool payload via standard input `$(cat)`. If the command modifies a protected branch, it echoes the blockage and crucially `exit 2` to formally block Claude via the official platform mechanism.

### [NEW] `hook_prompt_submit.sh` (Checkpoint Control)
- **Event**: `UserPromptSubmit` (matcher: `*`) / `PostToolUse`
- **Execution Mechanism**: `type: "command"`
- **Purpose**: Creates automatic git commits before specific interactions.
- **Naming Rule Regulation**: Commits MUST adhere to exactly this format: `CP: "A brief introduction to the modifications in this submission (focusing on a global perspective to show the role of these modifications in the overall plan)"`. 
  - **Strict Constraint**: Under no circumstances should the message include metadata such as time, dates, or lists of specifically modified files. Git handles that tracking inherently.

## 4. Skills Implementation (The User Commands)

### [MODIFY] `/goto`
- **Frontmatter**: `disable-model-invocation: true`, `user-invocable: true`
- **Behavior Requirements**:
  - Arguments are cleanly evaluated.
  - If given a Commit ID: Execute `git checkout [commit_id]` to travel back in time (Detached HEAD).
  - If given a Branch Name: Execute `git checkout [branch_name]` (HEAD of branch).

### [MODIFY] `/fork-explore`
- **Frontmatter**: `context: fork`, `agent: Explore`
- **Behavior Requirements**:
  - Represents the official Claude Code `fork-session` branching logic in Git format.
  - Creates a new Git sub-branch nested under the current Working Branch.
  - Generates the new Session ID and maps it to the sub-branch in `session-git-map.json`.
  - Unless the user manually executes Git checkout away from this pointer, subsequent conversations are routed into this sub-branch.

### [MODIFY] `/merge`
- **Frontmatter**: `disable-model-invocation: true`, `allowed-tools: Bash, Read`
- **Behavior Requirements**: **Strictly Advisory Mode**.
  - Claude cannot run `git merge`.
  - It generates a detailed, block-by-block "Merge Guide" explaining each difference in context to the target branch.
  - **Output Placement**: The guide must be written directly to a newly created Markdown file in the project rooted at `References/MergeGuide/`.
  - **Naming Convention**: The file must visually denote what was merged into what (e.g., `feature-A_merged_into_main.md`).

## 5. Reference & Verification Plan for Code Implementation

### Reference Code Achievement Guide
This section dictates precisely how we will script the system to comply with official Claude Code specs.

1. **Config Hook File**: [.claude/settings.json](file:///e:/CODE_project/BalanceSoldier/ChassisControl/CHASSIS_Patience/.claude/settings.json) must be standard JSON defining the "hooks" object dictionary.
2. **Standard I/O Mapping**: For script-based hooks, `.tool_input.command` must be parsed using `jq` from the JSON payload fed to `stdin`.
3. **Session ID Identification**: Scripts must rely on the securely provided environment variable `$CLAUDE_SESSION_ID` injected natively by Claude Code during task execution.

### Verification Matrix
Before the user manually installs this system globally, the following strict checks must pass:

1. **Working Branch Integrity (Null-Pointer Safety)**:
   - Close a session on `working-branch-alpha`. Check out `working-branch-beta` manually in bash.
   - Run `claude`. The `SessionStart` hook successfully triggers the prompt: "Branch mismatch."
2. **Checkpoint Format Check**:
   - Ask Claude to modify code. 
   - Check `git log`. The commit message is solely `CP: "[Global explanation]"` without any automated "Files modified:" boilerplate.
3. **Protected Branch Ejection**:
   - Check out `main`. Ask Claude to delete a file.
   - The official hook interface catches it via Exit Code 2. The action is fully blocked without human intervention.
4. **Merge Guide Generation**:
   - Run `/merge target-branch`.
   - Verify that Git history remains untouched (no merge commit exists), and a nicely formatted `feature_merged_into_target.md` appears exactly in `References/MergeGuide/`.
