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
Target branch is specified in $ARGUMENTS.

1. Use Git diff to inspect the differences between HEAD and the target branch.
2. Generate a detailed, block-by-block "Merge Guide" explaining each difference in context to the target branch.
3. Write the guide to a new Markdown file at `References/MergeGuide/<current_branch>_merged_into_<target_branch>.md`.
