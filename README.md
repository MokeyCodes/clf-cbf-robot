# Two-Layer Robot Navigation: RRT* Global Planning + CBF-QP Safety Control

## Overview
This project explores safety-constrained control for autonomous navigation using Control Barrier Functions (CBFs) and Quadratic Programming (QP).

I implemented and compared multiple control formulations:
- PID (baseline)
- CLF-CBF-QP
- CBF-QP + Nominal Controller
- CBF-QP + RRT / RRT* (final approach)

The goal is to achieve stable trajectory tracking while ensuring real-time obstacle avoidance in dynamic environments.

---

### CLF-CBF-QP Behavior Comparison

| With Dynamic Obstacles | Without Obstacles |
|------------------------|-------------------|
| <img src="assets/CLF.gif" width="350"/> | <img src="assets/CLF_Fail.gif" width="350"/> |

- Successfully avoids **moving obstacles**
- However, without obstacles, the robot follows a **curved and inefficient trajectory**
- This highlights unintended coupling between **CLF stability constraints** and control inputs

---

### Key Takeaway

While CLF-CBF-QP ensures safety and theoretical stability, it introduces:
- **Directional inefficiency** in unconstrained environments
- **Sensitivity to tuning (e.g., slack variables)**
- Unnecessary complexity for simple navigation tasks

This motivated the transition to a **CBF-QP + nominal controller**, which achieves more direct and consistent goal-reaching behavior.

---

## CBF-QP + Nominal Controller
| <img src="assets/robot_sim.gif" width="350"/> | <img src="assets/robot_sim2.gif" width="350"/> |
- Smooth trajectory to goal
- Reliable avoidance of **dynamic obstacles**
- Minimal tuning required

However, with no global plan the robot is blind to the overall layout — CBF alone can get trapped in local minima in cluttered environments. This motivated adding a global planner.

---

## Two-Layer Architecture: RRT* + CBF-QP

The final approach separates navigation into two layers:

- **RRT* (global planner)** — runs once before the simulation, finds a collision-free path through static obstacles
- **CBF-QP (local safety filter)** — runs every 50ms, deflects the robot away from dynamic obstacles in real time

This decouples the two problems:
- **Large-scale layout** → RRT* handles it offline
- **Dynamic obstacle avoidance** → CBF handles it at runtime

Neither layer can do the other's job. RRT* cannot react in real time, and CBF alone gets stuck without a global plan.

---

### RRT vs RRT*

| | RRT | RRT* |
|---|---|---|
| Waypoints | 55 | 29 |
| Path quality | Jagged, longer | Smooth, shorter |
| Planning time | Faster | Slower per iteration |
| Optimality | Finds any valid path | Continuously improves path |

| RRT (55 waypoints) | RRT* (29 waypoints) |
|---|---|
| <img src="assets/RRT_CBF.gif" width="350"/> | <img src="assets/RRT_Star_CBF.gif" width="350"/> |

RRT* rewires the tree with every new node — choosing the lowest-cost parent within a search radius and updating neighbours if a shorter path exists through the new node. The result is a significantly smoother, shorter path with fewer waypoints for the nominal controller to track.

---

### Stress Test — RRT* + CBF-QP

![Stress Test](assets/Stress_Test_RRT_Star.gif)

- Dense dynamic obstacle environment
- RRT* plans around static obstacles only
- CBF handles all dynamic avoidance at runtime
- **0 collisions** despite high obstacle density

The 0 collision result under stress conditions validates the two-layer approach — RRT* handles the layout CBF alone would get stuck on, while CBF handles the dynamic obstacles RRT* cannot predict.

---

## Key Insight

Adding RRT* as a global planner on top of CBF-QP addresses the core limitation of reactive-only control:

- CBF alone has **no awareness of the global layout** and can get trapped
- RRT* provides a **collision-free reference path** through static obstacles
- CBF then **only needs to handle dynamic deviations** from that path

The combination is robust where neither layer alone would be.

---

## Method Comparison

| Method | Pros | Cons |
|---|---|---|
| PID | Simple baseline | No safety guarantees |
| CLF-CBF-QP | Stability + safety guarantees | Curved paths, difficult tuning |
| CBF-QP | Simple, robust, consistent | No global layout awareness |
| CBF-QP + RRT | Global path + real-time safety | Path quality limited |
| CBF-QP + RRT* | Global path + real-time safety + optimal path | Slower planning |

---

## Approach

### CLF-CBF-QP (Initial)
- Combined Control Lyapunov Function (CLF) and Control Barrier Function (CBF)
- Enforced stability + safety via QP constraints
- Included slack variable for feasibility

➡️ See branch: [`clf_cbf_qp`](https://github.com/MokeyCodes/clf-cbf-robot/tree/clf_cbf_qp)

---

### CBF-QP + Nominal Controller
- Removed CLF component
- Used a **nominal tracking controller** for goal-seeking
- Applied CBF constraints for safety
- Introduced a lookahead point to improve numerical stability and reduce oscillatory behavior near obstacles

This decouples:
- **Goal tracking** (nominal control)
- **Safety enforcement** (CBF)

---

### CBF-QP + RRT* (Final)
- RRT* runs once at start, producing a waypoint list through static obstacles
- Nominal controller tracks waypoints sequentially, advancing when within threshold distance
- CBF-QP filters commands every timestep, deflecting around dynamic obstacles
- Lookahead point ensures CBF detects obstacles early enough for smooth deflection

Key parameters:
- `alpha` — CBF aggressiveness. Lower = tighter safety, earlier intervention
- `l` — lookahead distance. Larger = earlier obstacle detection
- `wp_threshold` — how close before advancing to next waypoint
- `step_size` — RRT* tree growth increment
- `goal_sample_rate` — probability of sampling goal directly (biases tree toward goal)

---

## Tech Stack
- Python
- NumPy
- CVXPY (QP solver)
- Matplotlib (simulation + animation)
- Random (RRT* sampling)

---
