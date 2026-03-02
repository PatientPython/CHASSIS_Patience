---
name: keil-build
description: One-shot Keil compile-fix loop. Run keil-build each round, parse uv4.log, patch minimally, stop at 0 Error(s).
---

## Build command

Run this PowerShell command:
```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\Users\35766\.claude\skills\keil-build\scripts\keil-build.ps1"
```

The script automatically uses the current working directory as workspace.

Build log: `e:\CODE_project\BalanceSoldier\ChassisControl\CHASSIS_Patience\Project\.vscode\uv4.log`

## Execution loop

1. **ALWAYS run build command first** - Do NOT read previous log to check status, must execute fresh build
2. Run build command above
3. Read `uv4.log`
4. If `0 Error(s)` → stop and print "Build clean. 0 Error(s)."
5. Else fix first error, repeat

## Speed rules (strict)
- Do NOT scan the repo before fixing. Use only the file(s) named in the error line.
- Do NOT use WebSearch / WebFetch / broad Grep.
- Fix one primary error category per round.
- Prefer single-file edits.

## Stop conditions
- `0 Error(s)` in log → stop
- Round 8 still failing → stop with summary

## Output format
`[Round N] <error> → <file> → <X> Error(s) remaining`

## Project constraints
- Follow prefix rules: `G/S` scope · `ST/st/EM/F` type · `CH/GB` component
- Do not change business logic — only fix compiler errors
