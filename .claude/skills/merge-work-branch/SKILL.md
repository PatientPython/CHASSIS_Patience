---
name: merge-work-branch
description: Generate merge guide for merging current work branch into selected protected branch. Advisory only.
user-invocable: true
---

# Merge Work Branch

## Rules

- Do not run merge automatically.
- Do not use git worktree.
- Generate report to `References/MergeGuide/`.

## Procedure

1. Detect protected target branch (`main`, `master`, `v1-stable` or user-selected protected branch).
2. Analyze `git diff target..HEAD`.
3. Write merge guide:
   - summary of changes
   - risk notes
   - recommended merge command
   - alternatives (rebase/cherry-pick)
4. The merge guide content must be in Chinese.

## Output file

`References/MergeGuide/<current_branch>_into_<target>.md`
