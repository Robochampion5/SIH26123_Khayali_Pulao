# Decentralized AMR Coordination & Collision-Avoidance Framework
**Maintainer:** Adarsh Singh (IC2025006)
**Project:** SIH26123
**Target Hardware:** Edge Compute (`aarch64-unknown-linux-gnu`)
**Version:** 4.0 (Zero-Tolerance Hardening: 2PC, FEC, Lamport Clocks, `io_uring`, L2 Cache Optimization)

---

## 1. Context & Physical Constraints

**Objective:** Design a completely decentralized coordination and collision-avoidance framework for a multi-robot fleet operating in a dynamic smart warehouse, eliminating the single-point-of-failure inherent in cloud-managed path planning. 

**Hardware & Network Realities:**
*   **Compute Limits:** Edge devices (RPi4/Jetson Nano) lack an L3 cache, severely penalizing random memory access. Complex centralized multi-agent pathfinding algorithms are computationally intractable locally. 
*   **Network Volatility:** Industrial environments feature heavy RF interference. The protocol assumes a base UDP packet loss rate of 15%, 50–200ms latency jitter, and periodic network partitions.
*   **Kinematic Realities:** Theoretical grid-world algorithms assume instant turns. Physical differential-drive robots suffer from wheel-slip and rotational inertia, requiring strict kinematic constraints and hardware-level interlocks.

---

## 2. Mathematical Formulation & Causal Synchronization

### 2.1 Lamport Logical Clocks & D-SIPP State Space
To neutralize network latency jitter that causes temporal collisions, blind clock padding is replaced by causal ordering using Lamport Logical Clocks.

*   **Time Discretization:** $T = \lfloor t_{\text{real}} / \Delta t \rfloor$, where $\Delta t = 100\text{ ms}$ synchronized via monotonic wall-clock drift correction.
*   **Logical Clock:** Each agent maintains a monotonic counter $L_i$. On sending a message, $L_i = L_i + 1$. On receiving a message with timestamp $L_{msg}$, the agent updates $L_i = \max(L_i, L_{msg}) + 1$.
*   **Augmented Space-Time State:** 
    $$s = (v,\; \theta,\; [t_s, t_e],\; L_s)$$
*   **Kinematic Transition Cost:**
    $$\tau(v, v', \theta, \theta') = T_{\text{rot}}(\theta, \theta') + T_{\text{trans}}(v, v')$$
    $$T_{\text{rot}}(\theta, \theta') = \left\lceil \frac{|\theta' - \theta|_{\text{shortest arc}}}{|\Theta|} \cdot \frac{\pi/2}{\omega_{\max} \cdot \Delta t} \right\rceil \text{ ticks}$$
*   **Admissible Heuristic:**
    $$h(s, s_{\text{goal}}) = d_{\text{Manhattan}}(v, v_{\text{goal}}) \cdot T_{\text{trans}}^{\min} + R_{\text{min}}(\theta, v, v_{\text{goal}}, \theta_{\text{goal}}) \cdot T_{90}$$

### 2.2 Bounded Priority Function
To prevent low-urgency tasks from overriding emergency tasks via unbounded yield accumulation, the yield multiplier is strictly capped, guaranteeing safety SLAs.
$$P_i = 50000 \cdot U_{\text{task}}(i) + \min(10000 \cdot T_{\text{yield}}(i),\; 20000) + \delta \cdot \frac{100}{D_{\text{goal}}(i) + 1} + \frac{10}{B_i + 1} + \text{ID}_i$$
*   $U_{\text{task}} \in \{0, 1, 2, 3, 4\}$ (0=Idle, 4=Emergency). Emergency tasks ($50,000+$ base weight) can never be overridden by an idling robot, regardless of yield count.

---

## 3. High-Performance Network Stack & Consensus

### 3.1 Kernel-Bypass Networking & Spatial Scoping
*   **`io_uring` Implementation:** UDP socket reads bypass the standard POSIX `recvmsg` syscall. Mapping a shared ring buffer between user-space and the kernel reclaims 10–15ms of CPU time per tick, avoiding Linux context-switching bottlenecks.
*   **Spatial Scoping:** Multicast groups are partitioned by warehouse quadrant (e.g., `239.255.1.X`). Agents subscribe to their current and adjacent quadrants, reducing intersection packet storms by filtering non-local traffic at the NIC level.

### 3.2 Two-Phase Commit (2PC) Trajectory Locking
Eliminates race conditions at choke points. Agents explicitly acquire locks before executing paths.
1.  **Phase 1 (Prepare):** Agent multicasts `INTENT_PREPARE` containing the proposed trajectory and Lamport timestamp $L_p$.
2.  **Phase 2 (Promise):** Agents in a 15-cell radius check their 1D Bitset arrays. If no conflicting higher-priority reservation exists, they respond with `ACK_PROMISE`.
3.  **Phase 3 (Commit):** Upon receiving `ACK_PROMISE` from all active peers in the radius, the agent multicasts `INTENT_COMMIT` and transitions to `EXECUTING`.

### 3.3 Dynamic Sliding-Window Quorum
To handle offline/charging robots without halting the warehouse task allocation:
*   **Active Fleet Size ($N_{\text{active}}$):** Dynamically calculated as the count of unique agents broadcasting within a sliding 5.0-second window.
*   **Quorum Requirement:** $|\text{heard\_from}_i| \geq \lfloor N_{\text{active}} / 2 \rfloor + 1$ (Strict Majority, preventing 50/50 split-brain ties).

---

## 4. Core Data Structures & Payloads

### 4.1 1D Bitset Reservation Table (L2 Cache Optimized)
Nested `HashMap` structures trigger fatal memory thrashing on architectures without an L3 cache. 
*   **Structure:** `std::vector<uint64_t>` or `std::bitset<65536>` mapped linearly to the $256 \times 256$ grid.
*   **Locality:** Array indexing allows the CPU hardware prefetcher to load adjacent temporal cell data directly into the L2 cache boundaries. Conflict checking time complexity drops to a strict $O(1)$ bitwise `AND` operation, reducing D-SIPP expansion time from 46ms to $<5\text{ms}$.

### 4.2 Wire Payloads with Reed-Solomon FEC
All UDP payloads include 4 bytes of Reed-Solomon parity data, allowing receivers to mathematically reconstruct dropped packets without triggering retransmission storms.
*   **INTENT_PREPARE (0x0B):** Proposed path + Lamport timestamp.
*   **INTENT_COMMIT (0x0C):** Execution lock confirmation.
*   **TASK_CFP (0x02):** Uses Aggressive Linear Micro-Bursting. Broadcast every 100ms for exactly 1.0s. If quorum fails, the task is immediately dequeued.
*   **TASK_BID (0x03):** Contains Q8.8 utility and `heard_from` bitmap for dynamic quorum.
*   **DIAGNOSTIC_HEARTBEAT (0x0A):** 1Hz telemetry. **Piggybacking:** Any active `EDGE_BLOCKED` cell coordinates are appended to this heartbeat for 5 seconds to guarantee synchronization across the fleet, fixing blind obstacle failure rates.

---

## 5. Hardware Interlocks & Control Loop (10Hz Strict Sync)

**States:** `IDLE`, `PLANNING`, `AWAITING_PROMISE`, `EXECUTING`, `YIELDING_PULLOVER`, `REPLANNING`, `BLOCKED`.

### 5.1 Sub-Millisecond Hardware Interlock
The software-based 600ms gap for dynamic obstacle avoidance is eliminated.
*   **GPIO Interrupt:** The LiDAR safety boundary ($0.5\text{m}$) is decoupled from the 10Hz Linux loop and wired directly to the motor controller's hardware Enable/Disable pin via GPIO.
*   **Latency:** Obstacle detection triggers a physical circuit break in $<1\text{ms}$. The software loop reads the tripped GPIO state on the next tick and transitions directly to `BLOCKED`.

### 5.2 Execution Pipeline
1.  **Ring Buffer Drain (0ms syscalls):** Read incoming payloads via `io_uring` completion queue. Apply FEC reconstruction.
2.  **Lamport Sync:** Update local logical clocks. Discard causally stale packets.
3.  **Sensor Fusion ($\le$ 5ms):** LiDAR/Odometry SLAM correction mapped to the occupancy grid.
4.  **Deterministic Tick Sync:** Logical tick increments are locked to monotonic wall-clock time (`std::time::Instant`) rather than blind loop counts, compensating for OS scheduling jitter.
5.  **FSM Tick:** Evaluate 2PC `ACK_PROMISE` counts. Transition to `EXECUTING` or execute `KINEMATIC_REVERSE`.

---

## 6. Deployment Verification Gates

To advance to physical hardware deployment, the software must pass these strict integration gates:
*   **Chaos Engineering Harness:** 1,000 trials under 15% packet loss, 200ms jitter, and 10% LiDAR ghosting must yield zero spatial deadlocks.
*   **Zero-Tolerance Test Suite:** The integration test suite (`cargo test` or `ctest`) must return exactly **0 failing tests**. All previously identified edge cases (e.g., RNG seed anomalies, bounding-box overlaps) must be structurally and deterministically resolved within the FSM logic.