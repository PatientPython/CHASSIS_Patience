---
name: fork-explore
description: Create a new sub-branch and start a parallel session
disable-model-invocation: false
user-invocable: true
---

# Fork Explore Skill

This skill synchronizes the official `claude --fork-session` behavior with Git branching.

Please execute the following step-by-step for the user:

1. First, ask the user to choose an action using AskUserQuestion:
   - Option 1: "创建新工作分支" - Create a new working branch
   - Option 2: "进入现有工作分支" - Enter an existing working branch (provide an option like "请在下方输入分支名称" for the branch name input via Other)

2. If user chooses "创建新工作分支":
   a. Ask for the new branch name using AskUserQuestion with "请在下方输入分支名称" option
   b. Run `git checkout -b <new_branch_name>` to create the branch locally
   c. Explicitly instruct the user with the following string to launch the forked session:
      "Please open a new terminal window in this directory and type: claude --fork-session"
   d. Explain that the new session will automatically bind to the new branch via the SessionStart hook

3. If user chooses "进入现有工作分支":
   a. Ask for the branch name OR commit ID using AskUserQuestion with "请在下方输入分支名称或commit ID" option
   b. Run `git checkout <branch_name_or_commit_id>` to switch to the branch or commit
   c. Show the current branch status to confirm
   d. Note: If entering a commit ID directly, the user will be in detached HEAD state - remind them to create a branch if they want to retain changes
