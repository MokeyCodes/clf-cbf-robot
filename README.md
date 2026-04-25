# Two-Layer Robot Navigation: RRT* Global Planning + CBF-QP Safety Control

## Overview
This project explores safety-constrained control for autonomous navigation using Control Barrier Functions (CBFs) and Quadratic Programming (QP).

I implemented and compared multiple control formulations:
- PID (baseline)
- CLF-CBF-QP
- CBF-QP + Nominal Controller
- CBF-QP + RRT / RRT*
- CBF-QP + RRT / RRT* + Dynamic Replanning + Multi-Goal (final approach)

The goal is to achieve stable trajectory tracking while ensuring real-time obstacle avoidance in dynamic environments.

---

## Final Architecture
RRT* replans a collision-free path every frame from the robot's current position. 
CBF-QP filters commands in real-time to deflect dynamic obstacles. 
Supports sequential multi-goal navigation with continuous safety enforcement throughout.
See [Method Comparison](#method-comparison) for the full progression.

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
<img src="assets/robot_sim.gif" width="350"/>  <img src="assets/robot_sim2.gif" width="350"/> 
- Smooth trajectory to goal
- Reliable avoidance of **dynamic obstacles**
- Minimal tuning required

However, with no global plan the robot is blind to the overall layout — CBF alone can get trapped in local minima in cluttered environments. This motivated adding a global planner.

---

## Two-Layer Architecture: RRT* + CBF-QP

The final approach separates navigation into two layers:

- **RRT* (global planner)** — replans from the robot's current position every frame, maintaining a collision-free path through the environment
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
- The addition of dynamic replanning and multi-goal navigation extends this further — see below.

---

## Dynamic Replanning: RRT* Recalculation

<img src="assets/RRT*Recalculation.gif" width="350"/>

In static environments, RRT* runs once at startup. But with dynamic obstacles constantly shifting the navigable space, a fixed plan can become invalid mid-execution.

This extension replans the RRT* path **every frame**, so the robot always has a fresh collision-free route from its current position to the goal. The CBF layer continues to handle real-time micro-corrections between replanning cycles.

Key trade-off: replanning every frame is computationally expensive. In practice, a trigger-based replan (e.g., when the planned path intersects a moved obstacle) would be more efficient — but this demo validates that the architecture supports it.

---

## Multi-Goal Navigation

| Single Objective | Multiple Objectives |
|---|---|
| <img src="assets/multiple_goal.gif" width="350"/> | <img src="assets/multiple_objectives.gif" width="350"/> |

Extended the planner to support a **sequence of goals**. When the robot reaches a waypoint threshold of the current goal, it replans from its current position to the next goal in the queue.

This required:
- A goal queue with index tracking
- Replanning triggered on goal arrival, not just on a timer
- CBF constraints remain active throughout all goal transitions

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
| CBF-QP + RRT* + Replanning | All above + adapts to dynamic layout changes | High compute per frame |
| CBF-QP + RRT* + Multi-Goal | Full pipeline + sequential objectives | Same as above |

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
- Lookahead point shifts the CBF anchor forward, enabling earlier obstacle detection and ensuring ω enters the constraint — the robot can steer away, not just brake

This decouples:
- **Goal tracking** (nominal control)
- **Safety enforcement** (CBF)

---

### CBF-QP + RRT* with Dynamic Replanning & Multi-Goal
- RRT* replans from the robot's current position every frame, keeping the path valid as obstacles move
- Multi-goal support: robot navigates a sequence of goals, replanning to the next on arrival
- CBF-QP continues filtering commands throughout all goal transitions — no gaps in safety enforcement
- Lookahead point shifts the CBF anchor forward, enabling earlier obstacle detection 
and ensuring ω enters the constraint — the robot can steer away, not just brake

Key parameters:
- `alpha` — CBF aggressiveness. Lower = tighter safety, earlier intervention
- `l` — lookahead distance. Larger = earlier obstacle detection. Also ensures ω enters 
the CBF constraint — without it, the QP can only modulate speed, not heading.
- `wp_threshold` — how close before advancing to next waypoint
- `step_size` — RRT* tree growth increment
- `goal_sample_rate` — probability of sampling goal directly (biases tree toward goal)
- `replan_interval` — frames between replanning calls (1 = every frame)
- `goal_arrival_threshold` — distance at which the current goal is considered reached

---

## Tech Stack
- Python
- NumPy
- CVXPY (QP solver)
- Matplotlib (simulation + animation)
- Random (RRT* sampling)

---
