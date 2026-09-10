# Decentralized AMR Coordination & Collision-Avoidance Framework

**Status**: Version 3.0 — High-Resilience Consensus & Deadlock Recovery
**Target Hardware**: Raspberry Pi 4 (4GB) / NVIDIA Jetson Nano (aarch64-unknown-linux-gnu)
**Target Network**: High-noise 2.4GHz/5GHz 802.11ac Wi-Fi with >15% packet drop probability.
**Last Updated**: 2026-09-11

---

## 1. Context, Constraints, and Edge-Case Analysis

**Objective**: Design a completely decentralized coordination and collision-avoidance framework for a multi-robot fleet operating in a dynamic smart warehouse, eliminating the single-point-of-failure inherent in cloud-managed path planning. 

**Hardware & Network Constraints:**
*   **Compute Limits:** Edge devices (RPi4/Jetson Nano) possess limited L2/L3 cache and RAM bandwidth. Complex $O(N^3)$ centralized multi-agent pathfinding (MAPF) algorithms (like CBS - Conflict-Based Search) are computationally intractable locally. 
*   **Network Volatility:** Industrial environments feature heavy RF interference from metal racks and motors. The protocol must assume a base UDP packet loss rate of 15% and periodic split-brain network partitions.
*   **Kinematic Realities:** Theoretical grid-world algorithms assume instant $90^\circ$ turns and infinite deceleration. Physical differential-drive robots suffer from wheel-slip, rotational inertia, and momentum, requiring strict kinematic constraints in the planning space.

---

## 2. Mathematical Formulation & Algorithmic Complexity

### 2.1 Augmented D-SIPP (Safe Interval Path Planning)
Standard A* search on a 3D space-time grid $(x, y, t)$ yields state-space explosion. Safe Interval Path Planning (SIPP) compresses contiguous safe time steps into "intervals." This framework augments SIPP with kinematic rotational constraints.

**Spatial Graph**: $G = (V, E)$ represents the discretized warehouse grid ($256 \times 256$, up to 65,536 nodes).
**Time Discretization**: $T = \lfloor t_{\text{real}} / \Delta t \rfloor, \quad \Delta t = 100\text{ ms}$.

**Augmented Space-Time State**: 
$$s = (v,\; \theta,\; [t_s, t_e])$$
Where $v \in V$, $\theta \in \{0, \frac{\pi}{2}, \pi, \frac{3\pi}{2}\}$, and $[t_s, t_e]$ is the contiguous collision-free time interval.

**Kinematic Transition Cost Function**:
Unlike theoretical MAPF, moving from $(v, \theta)$ to $(v', \theta')$ requires calculating exact rotational delays to prevent temporal collisions with other AMRs expecting the node to be clear.
$$\tau(v, v', \theta, \theta') = T_{\text{rot}}(\theta, \theta') + T_{\text{trans}}(v, v')$$
$$T_{\text{rot}}(\theta, \theta') = \left\lceil \frac{\vert{}\theta' - \theta\vert{}_{\text{shortest arc}}}{\vert{}\Theta\vert{}} \cdot \frac{\pi/2}{\omega_{\max} \cdot \Delta t} \right\rceil \text{ ticks}$$
Where $\omega_{\max}$ is the maximum safe angular velocity (rad/s) accounting for payload inertia.

**Admissible Heuristic**:
To guarantee optimal paths within the D-SIPP expansion, the heuristic must never overestimate the cost.
$$h(s, s_{\text{goal}}) = d_{\text{Manhattan}}(v, v_{\text{goal}}) \cdot T_{\text{trans}}^{\min} + R_{\text{min}}(\theta, v, v_{\text{goal}}, \theta_{\text{goal}}) \cdot T_{90}$$
Where $R_{\text{min}}$ calculates the minimum number of $90^\circ$ rotations required to reach the goal facing the correct vector.

### 2.2 Decentralized Priority Function (Right-of-Way Resolution)
In centralized systems, a manager dictates right-of-way. In this framework, right-of-way at choke points is deterministically calculated by each agent evaluating its own priority against the broadcasted priorities of others.

$$P_i = 10000 \cdot T_{\text{yield}}(i) + \delta \cdot U_{\text{task}}(i) \cdot \frac{100}{D_{\text{goal}}(i) + 1} + \frac{10}{B_i + 1} + \text{ID}_i$$
*   **Starvation Prevention ($T_{\text{yield}}$):** Number of times agent $i$ has yielded to others on the current task. Multiplier is $10000$ to guarantee an agent cannot be permanently gridlocked by higher-tier tasks.
*   **Task Urgency ($U_{\text{task}}$):** Integer $\{0, 1, 2, 3, 4\}$, with $\delta = 50$.
*   **Distance-to-Goal ($D_{\text{goal}}$):** Prioritizes agents closer to completing their tasks to free up warehouse capacity faster.
*   **Absolute Determinism ($\text{ID}_i$):** Global unique MAC or assigned ID ensures a zero-collision tie-break if all other values are perfectly equal.

---

## 3. High-Resilience Decentralized Network Stack (UDP Multicast)

To bypass TCP handshake latency and centralized brokering, communication utilizes UDP multicast on a dedicated local subnet (e.g., `239.255.0.1:9999`). 

### 3.1 Packet Architecture & Segment Compression
Sending full trajectory arrays $(x, y, t)$ across UDP quickly exceeds the 1500-byte Maximum Transmission Unit (MTU), leading to packet fragmentation and high drop rates. 
**Solution**: Trajectories are compressed into `SegmentTable` arrays.
*   **Segment Struct**: `[start_cell (u16), end_cell (u16), t_enter (u32), t_exit (u32), heading (u8)]` = 13 bytes.
*   A 50m straight corridor compresses from 1000 distinct coordinates to a single 13-byte segment. Total payload size for complex paths rarely exceeds 50-80 bytes, safely fitting inside a single, unfragmented UDP datagram.

### 3.2 Implicit-Award Contract Net Protocol (CNP)
Task allocation requires consensus without a leader. 

1. **Blind Triple-Transmit (Micro-Bursting)**: To combat the 15% UDP packet drop rate, any `TASK_CFP` (Call For Proposal) or `TASK_BID` is transmitted 3 times in a rapid 30ms window (10ms spacing).
2. **Implicit Award**: After the bidding window ($T_{\text{bid}} = 500\text{ms}$), all agents evaluate the received bids. The winner determines *itself*.
3. **Split-Brain Quorum Mathematics**:
   If the network physically partitions (e.g., Wi-Fi router failure in one aisle), agents must not double-assign tasks.
   *   *Evaluation Logic*: Agent claims task iff $(U_{\text{self}} = \max(U_{\text{fleet}}))$ AND `(|heard_from| > N/2)`
   *   *Even-Split Edge Case*: In an $N=4$ fleet, a 2/2 partition means `|heard_from| == N/2`. Under strict majority, the task drops. To prevent task starvation, the protocol applies a deterministic tie-breaker: the partition containing the lowest global ID ($\min(\text{ID}_{\text{fleet}})$) achieves quorum; the other defers.

---

## 4. Edge Control Loop & Deadlock Recovery Matrix

The core state machine runs at 10Hz. It is responsible for fusing network intents with physical sensor data (LiDAR, Odometry).

### 4.1 FSM State Diagram

```text
IDLE ──[TASK_AWARD]──→ PLANNING ──[plan found]──→ EXECUTING
  ↑                                                  ↓ 
  │                               ┌──[conflict]──────┘
  │                               ↓ 
  │                     YIELDING_PULLOVER ──[find cell]
  │                               ↓ [no free cell / blocked by agent]
  │                            BLOCKED 
  │                               ↓ [obstacle is dynamic agent]
  │                      KINEMATIC_REVERSE 
  │                               ↓ [reversed 1.5m]
  │                       YIELDING_PARKED
  │                               ↓ [higher-priority agent clears]
  └────────────────────────── REPLANNING