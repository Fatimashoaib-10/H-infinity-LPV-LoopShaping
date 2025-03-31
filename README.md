# H-infinity-LPV-LoopShaping
# H∞ Loop Shaping Control of an LPV System

## 📌 Overview
This project implements an **H∞ Loop Shaping controller** for a **Linear Parameter-Varying (LPV)** model of an **inverted pendulum**. The aim is to robustly stabilize the system over a range of parameter variations using **dynamic output-feedback controllers** synthesized through **Linear Matrix Inequalities (LMIs)**.

Developed as part of LMIs course at Arizona State University.

---

## 🚀 Key Features

- Developed a polytopic LPV model from nonlinear pendulum dynamics
- Synthesized robust dynamic H∞ controllers via convex optimization
- Implemented **null space-based LMI conditions** to solve for stability
- Evaluated closed-loop performance across multiple scheduling parameter values
- Verified system stability and controller effectiveness using step/impulse responses

--
## 📐 Tools & Technologies

- **MATLAB** (Control System Toolbox, Robust Control Toolbox)
- **YALMIP** for LMI formulation
- **S-Procedure & Bounded Real Lemma** for robust synthesis
- No external solver specified (but compatible with MOSEK, SeDuMi)

---

## 📊 Results

- Successfully synthesized LPV controllers for all parameter vertices
- Achieved low H∞ norm values (as printed in `g = value(g)` and controller performance)
- Closed-loop eigenvalues demonstrate robust stability across variations
- **Step and impulse responses** show fast, stable control behavior with minimal overshoot


## 🧠 What I Learned

- How to model nonlinear dynamics as polytopic LPV systems
- Use of null space and projection operators in robust control
- Controller design tradeoffs between robustness and performance
- Translating theoretical control conditions into solvable LMIs


## 📚 References

- Glover & McFarlane (1989) – H∞ Robust Stabilization
- Apkarian et al. – Self-Scheduled H∞ LPV Control
- Zhou et al. – Robust and Optimal Control

