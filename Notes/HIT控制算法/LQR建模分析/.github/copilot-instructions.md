# Copilot Instructions for RM25 Balanced Soldier LQR Modeling

## Project Overview
This MATLAB project derives the state-space model (A, B matrices) for a wheeled-legged balanced soldier robot and computes LQR optimal gains (K matrix) fitted over varying leg lengths.

## Key Components
- `BalanceSoldier_LQR_ABMatrix.m`: Symbolic derivation of A/B matrices from kinematics/dynamics.
- `Func1_Get_AB_Matrix.m`: Function to compute A/B for given leg lengths.
- `Func2_Get_K_Matrix.m`: Computes LQR gain K using MATLAB's `lqr(A, B, Q, R)`.
- `FuncLast_Get_K_FittingCoeffcient.m`: Main script looping over leg lengths (0.08-0.38m), computing K, and fitting coefficients using bicubic polynomials.

## State Variables (10)
- s: horizontal displacement
- s_dot: velocity
- phi: yaw angle
- phi_dot: yaw rate
- theta_l, theta_l_dot: left leg angle and rate
- theta_r, theta_r_dot: right leg angle and rate
- pitch, pitch_dot: body pitch and rate

## Inputs (4)
- T_wl, T_wr: left/right wheel torques
- T_bl, T_br: left/right hip torques

## Conventions
- Use symbolic math (syms) for derivations.
- Q matrix weights: diag([650,250,300,80,6000,40,6000,40,200000,100])
- R matrix weights: diag([12,12,1.2,1.2])
- Leg lengths vary from 0.08m to 0.38m in 0.01m steps.
- Fitting: bicubic polynomials for K elements vs. leg lengths.

## Workflows
- Run `FuncLast_Get_K_FittingCoeffcient.m` in MATLAB to generate fitted K coefficients.
- Use `Func3_Fit_LegAndK.m` for polynomial fitting (default degree 3 for two variables).

## Examples
- To get K for specific legs: `K = Func2_Get_K_Matrix(l_l, l_r, Q, R)`
- Fitting loop in `FuncLast_Get_K_FittingCoeffcient.m` stores K elements in matrices like `k11(i,j)` for coefficient fitting.