# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an embedded C project for a **balancing infantry robot chassis control system** (平衡步兵底盘控制系统), developed for the RoboMaster robotics competition (26th season). The project targets an STM32F407VETx microcontroller running FreeRTOS.

## Build System

**Primary Build System: Keil µVision**

- Project file: [Project/Project.uvprojx](Project/Project.uvprojx)
- Target: STM32F407VETx (Cortex-M4, 512KB Flash)
- Compiler: ARM Compiler 5.06 update 7 (build 960)
- Output directory: `../STM32/Output/`

**Clean Build Artifacts:**

```batch
keilkill.bat
```

There is no Makefile or CMake - this is a Keil-only project. Build and flash through Keil µVision IDE.

## Build Commands

**Keil Build (via VS Code Task):**

- Task name: "Keil Build"
- Trigger: `shell: build` or request Copilot to build
- Result: Check `.vscode/uv4.log` for compilation output
- Cache: `.vscode/keil-path-cache.json` stores UV4 path and project path

## Architecture & Data Flow

**Task-Based Architecture (FreeRTOS):**

- **ChassisTask** ([Application/Chassis/Chassis_Task.c](Application/Chassis/Chassis_Task.c)) - Priority 25, 1kHz main control loop
- **ReceiverTask** ([OtherTask/ReceiverTask.c](OtherTask/ReceiverTask.c)) - Priority 30, data reception
- **SendDataTask** ([OtherTask/SendDataTask.c](OtherTask/SendDataTask.c)) - Priority 20, data transmission
- **DebugTask** ([OtherTask/DebugTask.c](OtherTask/DebugTask.c)) - Priority 4, ~100Hz debug output

**Data Pipeline Pattern: Parse -> Process -> Distribute**

1. **Parse**: Raw data from interrupts (CAN/UART) into auxiliary structs (prefix `st`). See [Communication/CAN_Communication.c](Communication/CAN_Communication.c).
2. **Process**: Filtering and control calculations (LQR/VMC) in `ChassisTask`.
3. **Distribute**: Update formal structs (prefix `ST`) for high-level logic. Upper-level code must **only** use formal structs.

## Naming Conventions

Prefix order: `[Scope][Type][Component]_[Name]`

- **Scope**: `G` (Global), `S` (Static)
- **Type**: `ST`/`st` (Formal Struct / Auxiliary Struct), `EM` (Enum), `F` (Flag)
- **Component**: `CH` (Chassis), `GB` (Gimbal)
- **Standard Suffixes**:
  - `FB`: Feedback/Actual value (e.g., `AngleFB`)
  - `Des`: Desired/Target value (e.g., `VelDes`)
  - `ZP`: Zero Point (calibration offset)

Examples: `GFCH_SafeMode` (Global Flag Chassis SafeMode), `GSTCH_Data` (Global Struct Chassis Data)

**Function Prefixes:**

- `_FunctionName`: Helper function used within a single function
- `__FunctionName`: Simple helper (e.g., if condition checks)

## Hardware & Orientation

- **Orientation**: Battery side is **BACK** (电池侧为后方)
- **Motors**:
  - Hub Motors (C620 ESCs): Right = 1, Left = 2
  - Joint Motors (DM): LF = 1, RF = 2, LR = 3, RR = 4
- **Coordinates**:
  - **Pitch**: Back-tilt (抬头) is positive (+)
  - **Yaw**: Counter-clockwise (CCW) from top-view is positive (+)
  - **Roll**: Left-side high is positive (+)

## Core Algorithms

- **Five-Linkage Kinematics**: Leg kinematics calculation, coordinated by `GstCH_LegLinkCal1/2`
- **LQR**: Discrete LQR control for balance, managed in `CH_LQRCal_Process`
- **VMC**: Virtual Model Control for leg forces, managed in `CH_VMCCal_Process`
- **Kalman Filter**: Velocity estimation with adaptive control

All algorithm orchestration happens in [Application/Chassis/Chassis_APIFunction.c](Application/Chassis/Chassis_APIFunction.c).

## Chassis Control Modes

Defined in [Global/GlobalDeclare_Chassis.h](Global/GlobalDeclare_Chassis.h):

- `CHMode_RC_ManualSafe` - Manual safe mode
- `CHMode_RC_AutoSafe` - Auto safe mode
- `CHMode_RC_Standby` - Standby mode
- `CHMode_RC_StandUp` - Stand up mode
- `CHMode_RC_Free` - Free mode (non-following)
- `CHMode_RC_Follow` - Follow mode (follows gimbal)
- `CHMode_RC_SitDown` - Sit down mode
- `CHMode_RC_OffGround` - Off-ground mode
- `CHMode_RC_Jump` - Jump mode

**Adding a New Mode** (6-step checklist in GlobalDeclare_Chassis.h:23):

1. Add enum value in `ChassisMode_EnumTypeDef`
2. Add start time field in `_CH_ModeStartTime_StructTypeDef`
3. Add case in `ChassisModeStartTimeUpdate`
4. Add condition in `ChassisModeChoose_RCControl`
5. Add control strategy in `ChassisControl`
6. Add parameters in `Chassis_ModeChooseParameter_StructTypeDef` if needed

## Key Source Files

| File | Purpose |
|------|---------|
| [Application/Chassis/Chassis_Task.c](Application/Chassis/Chassis_Task.c) | Main 1kHz control loop |
| [Application/Chassis/Chassis_APIFunction.c](Application/Chassis/Chassis_APIFunction.c) | Algorithm orchestration (LQR, VMC, kinematics) |
| [Application/Chassis/Chassis_Stratgy.c](Application/Chassis/Chassis_Stratgy.c) | Mode control strategies |
| [API/Algorithm.c](API/Algorithm.c) | Core algorithms (LQR, VMC, Kalman filters) |
| [Global/GlobalDeclare_Chassis.h](Global/GlobalDeclare_Chassis.h) | Chassis global variables & structs |
| [Communication/CAN_Communication.c](Communication/CAN_Communication.c) | CAN bus handling |
| [User/main.c](User/main.c) | FreeRTOS setup |

## Debugging

- Watch `GSTCH_Data` and `GstCH_IMU2.ST_Rx` in Keil for real-time status
- Frame rate monitoring via `GST_SystemMonitor` (CAN1: ~5000fps, CAN2: ~2000fps, ChassisTask: ~1000fps)
- Use `keilkill.bat` to clean build artifacts if build issues occur

## Communication Frame Rates

- CAN1 Rx: ~5000 fps
- CAN2 Rx: ~2000 fps (hub motors: 1000 each)
- USART1 (Remote Control): ~70 fps
- USART2 (IMU): ~1000 fps
- USART3 (Debug): ~100 fps

## LQR State Vector

The LQR `x_Vector` state vector elements (target - actual):

```
s, s_dot, phi, phi_dot, theta_l, theta_l_dot, theta_r, theta_r_dot, pitch, pitch_dot
(位移, 位移速度, 偏航yaw, 偏航速度, 左腿夹角, 左腿角速度, 右腿夹角, 右腿角速度, 俯仰pitch, 俯仰角速度)
```

The LQR `u_Vector` output:

```
T_wl, T_wr, T_bl, T_br
(左轮扭矩, 右轮扭矩, 左摆杆扭矩, 右摆杆扭矩)
```

## Git Branch Rules & Session Workflow (CRITICAL)

- **Terminology**:
  1. **Protected Branches (`main`, `master`, `v1-stable`)**: Code baseline. AI modification is strictly prohibited.
  2. **Working Branches**: Where AI edits occur. Every active session must map to a working branch to avoid null pointer issues.
- **Workflow**:
  - If you start a session on a Protected Branch, you *cannot* edit code. You MUST use AskUserQuestion to ask: "Create a new protected branch or a working branch?" Then use `Bash(git checkout -b <branch>)` to enter the sandbox.
  - **DO NOT USE `git worktree`**: If blocked by hooks, ALWAYS resolve it by checking out a working branch locally. Never spawn worktrees.

## Claude Workflow v6 (Hooks + Skills)

- Hooks:
  - `SessionStart` -> `.claude/hooks/hook_session_start.sh` injects workflow context from `.claude/hooks/workflow-guide.md`.
  - `UserPromptSubmit` -> `.claude/hooks/hook_prompt_submit.sh` creates `CPST:` checkpoint and injects mandatory git policy from `.claude/hooks/git-harness-agent-policy.md`.
  - `Stop` -> `.claude/hooks/hook_stop.sh` creates `CPED:` checkpoint.
  - `TaskCompleted` -> `.claude/hooks/hook_task_complete.sh` creates `TASK:` checkpoint and updates `.claude/plan-git-SHA.json`.

- Core skills:
  - `brainstorming` -> design note
  - `create-todolist` -> plan JSON + `.claude/plan-context.json` + `.claude/plan-git-SHA.json`
  - `implement-and-verify` -> implement with build evidence
  - `code-review` -> mandatory dispatcher: `spec-reviewer` then `quality-reviewer`
  - `subagent-driven-dev` / `quick-executing-dev` -> execution modes
  - `merge-work-branch` -> advisory merge guide output to `References/MergeGuide/`

- Review outputs:
  - Failure reports under `References/ReviewReport/<plan-id>/`.

- Task status constraints in `.claude/plan-git-SHA.json`:
  - Task: `pending` or `completed`
  - Plan: `in_progress` or `completed`
