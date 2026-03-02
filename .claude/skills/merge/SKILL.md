---
name: merge
description: Generate a detailed Merge Guide explaining differences to a target branch
disable-model-invocation: true
allowed-tools: Bash, Read
user-invocable: true
---

# Merge Skill

You are in strictly advisory mode.
You must NOT run `git merge`.

1. First, get all protected branches (branches NOT starting with "work/") using `git branch -a`.
2. If there is only one protected branch, use it directly as the target branch.
3. If there are multiple protected branches, ask the user to select the target branch using AskUserQuestion, showing all protected branches as options.
3. Use Git diff to inspect the differences between HEAD and the selected target branch.
4. Generate a detailed, block-by-block "Merge Guide" explaining each difference in context to the target branch.
5. Write the guide to a new Markdown file at `References/MergeGuide/<current_branch>_merged_into_<target_branch>.md`, and include the following git commands at the very beginning of the file:
   - First, show the recommended merge command in a code block
   - Then show alternative commands (rebase, cherry-pick if applicable)
   - Example format:
     ```bash
     # Recommended: Merge
     git checkout <target_branch>
     git merge <current_branch>

     # Alternative: Rebase
     git checkout <current_branch>
     git rebase <target_branch>
     ```
