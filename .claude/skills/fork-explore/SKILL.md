---
name: fork-explore
description: Create a new sub-branch and start a parallel session
disable-model-invocation: false
user-invocable: true
---

# Fork Explore Skill

This skill synchronizes the official `claude --fork-session` behavior with Git branching.

Please execute the following step-by-step for the user:

1. Parse $ARGUMENTS to identify the new sub-branch name. If not provided, ask for one.
2. Run `git checkout -b <new_sub_branch>` to create the branch locally.
3. Explicitly instruct the user with the following string to launch the forked session:
   "Please open a new terminal window in this directory and type: claude --fork-session"
4. Explain that the new session will automatically bind to the new branch via the SessionStart hook.
