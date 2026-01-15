# Balance Infantry Chassis Control Instructions

## Architecture & Data Flow
- **Task-based**: Core logic resides in [Application/Chassis/Chassis_Task.c](Application/Chassis/Chassis_Task.c). Main tasks: `ChassisTask` (1kHz), `ReceiverTask`, `SendDataTask`.
- **Data Pipeline**: Decoupled I/O and Logic using the **Parse -> Process -> Distribute** pattern.
    1. **Parse**: Raw data from interrupts (CAN/UART) into auxiliary structs (prefix `st`). See [Communication/CAN_Communication.c](Communication/CAN_Communication.c).
    2. **Process**: Filtering and control calculations (LQR/VMC) in `ChassisTask`.
    3. **Distribute**: Update formal structs (prefix `ST`) for high-level logic. Upper-level code must **only** use formal structs.
- **System Monitoring**: `GST_SystemMonitor` tracks frame rates (fps) of all communication channels (Hub motors, IMU, etc.).

## Naming Conventions
Follow prefix order: `[Scope][Type][Component]_[Name]`
- **Scope**: `G` (Global), `S` (Static).
- **Type**: `ST`/`st` (Formal Struct / Auxiliary Struct), `EM` (Enum), `F` (Flag).
- **Component**: `CH` (Chassis), `GB` (Gimbal).
- **Standard Suffixes**:
    - `FB`: Feedback/Actual value (e.g., `AngleFB`).
    - `Des`: Desired/Target value (e.g., `VelDes`).
    - `ZP`: Zero Point (calibration offset).
    - `cnt`: Counter / `fps`: Frame Rate.
- **Examples**: `GFCH_SafeMode` (Global Flag Chassis SafeMode), `GSTCH_Data` (Global Struct Chassis Data).

## Coding Patterns
- **Timing**: Use `vTaskDelayUntil` with `GCH_TaskPeriod` for precise periodic execution.
- **Safety First**: Torques must only be applied after a `SafeModeCheck_Temp`. Data resets on safety triggers.
- **Unit Comments**: Explicitly state units in variable definitions (e.g., `float LegLen1Des; // mm`).
- **PID Naming**: `Speed`/`Pos` for velocity/position loops; `Angle`/`Vel`/`Torque` for others.

## Hardware & Orientation (Crucial)
- **Orientation**: Battery side is **BACK**.
- **Motors**:
    - **Hub Motors (C620)**: Right = 1, Left = 2.
    - **Joint Motors (DM)**: LF = 1, RF = 2, LR = 3, RR = 4 (Left-Front, Right-Front, Left-Rear, Right-Rear).
- **Coordinates**:
    - **Pitch**: Back-tilt (抬头) is positive (+).
    - **Yaw**: Counter-clockwise (CCW) from top-view is positive (+).
    - **Roll**: Left-side high is positive (+).

## Core Algorithms
- **Five-Linkage**: Kinematics for legs, coordinated by `GstCH_LegLinkCal1/2`.
- **LQR**: Discrete LQR control for balance, managed in `CH_LQRCal_Process`.
- **VMC**: Virtual Model Control for leg forces, managed in `CH_VMCCal_Process`.
All algorithm orchestration happens in [Application/Chassis/Chassis_APIFunction.c](Application/Chassis/Chassis_APIFunction.c).

## Workflow Tips
- **Build**: Use Keil µVision ([Project/Project.uvprojx](Project/Project.uvprojx)).
- **Debugging**: Watch `GSTCH_Data` and `GstCH_IMU2.ST_Rx` in Keil for real-time status.
- **Adding a Mode**: Follow the 6-step checklist in [Global/GlobalDeclare_Chassis.h](Global/GlobalDeclare_Chassis.h#L23) (Enum -> StartTime -> Strategy Case -> RC Condition).
