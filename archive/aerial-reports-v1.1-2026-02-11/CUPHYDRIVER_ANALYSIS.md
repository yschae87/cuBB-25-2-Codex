# cuphydriver Deep Dive (25.3, Source-Validated) (v1.1)

> **Document Version:** v1.1
> **Last Updated:** 2026-02-11
> **Repository Context:** `cuBB-25-2-Codex` (NVIDIA Aerial CUDA-Accelerated RAN workspace)

## Report Context (Self-Contained)
- This is a standalone technical report for NVIDIA Aerial codepaths in this repository.
- Module names, APIs, and thread/task flows are interpreted from current workspace sources.
- Runtime behavior can vary by deployment YAML, build flags, GPU/NIC topology, and testbench mode.

## Reader Guide
- `Purpose`: what this report/component is intended to explain.
- `Flow`: end-to-end processing order and critical handoffs.
- `Configuration`: major knobs/defaults that affect behavior or performance.
- `Risks`: implementation caveats, operational pitfalls, and validation focus.

## Core Glossary
- `L1`: PHY runtime execution layer (cuPHY and cuPHY-CP driver paths).
- `L2`: Scheduler/control side producing PHY work (FAPI-like command domain).
- `FH`: Fronthaul transport path (O-RAN split data/control planes).
- `SFN/Slot`: 3GPP frame/slot timing identifiers.
- `UE`: User equipment context handled by channel pipelines.

## v1.1 Revision Notes
- Reformatted as a self-contained report with explicit context and reading map.
- Standardized terminology to reduce ambiguity across Aerial modules.
- Expanded report framing to support independent review without external notes.

## Scope and Evidence
Updated from:
- `cuPHY-CP/cuphydriver/src/common/cuphydriver_api.cpp`
- `cuPHY-CP/cuphydriver/src/common/context.cpp`
- `cuPHY-CP/cuphydriver/src/common/worker.cpp`
- `cuPHY-CP/cuphydriver/src/common/task.cpp`
- `cuPHY-CP/cuphydriver/src/downlink/slot_map_dl.cpp`
- `cuPHY-CP/cuphydriver/src/downlink/task_function_dl_aggr.cpp`
- `cuPHY-CP/cuphydriver/src/uplink/slot_map_ul.cpp`
- `cuPHY-CP/cuphydriver/src/uplink/task_function_ul_aggr.cpp`
- `cuPHY-CP/cuphydriver/src/common/mps.cpp`
- `cuPHY-CP/cuphydriver/include/constant.hpp`

## Runtime Role
`cuphydriver` is the L1 runtime engine that:
- Converts `slot_command` into UL/DL task graphs.
- Allocates and recycles slot-scoped objects (slot maps, buffers, aggregators).
- Coordinates worker threads, CUDA streams, FH interactions, and callbacks.

## Lifecycle and Control Flow
### Initialization
`l1_init()` (`cuphydriver_api.cpp`) does:
1. Construct `PhyDriverCtx`.
2. Add per-cell M-plane configs (with mMIMO timing consistency checks).
3. Start context (`PhyDriverCtx::start()`), which creates order entities, HARQ pools, optional metrics/datalake workers, and starts UL/DL/validation/generic workers.

### Steady-state per-slot processing
`l1_enqueue_phy_work()` is the primary orchestrator. It:
- Splits channels into UL/DL paths.
- Acquires `SlotMapDl`/`SlotMapUl` from pools.
- Acquires per-cell DL/UL buffers.
- Acquires per-channel aggregation objects (PDSCH/PDCCH/PBCH/CSI-RS/PUSCH/PUCCH/PRACH/SRS/BFW).
- Acquires one order-entity per UL slot.
- Computes task counts/fanout and enqueues tasks to UL/DL task lists.

### Finalization
`l1_finalize()` deletes `PhyDriverCtx`.
The destructor aggressively tears down workers, buffers, streams, pools, contexts, and optionally emits debug dumps.

## Worker and Task Scheduler Model
### Task lists
`TaskList` uses:
- Per-worker priority queues keyed by worker ID.
- One generic queue for unbound tasks.
- Earliest `ts_exec` priority behavior.

Queue reservations are initialized in `context.cpp` with:
- `TASK_LIST_RESERVE_LENGTH = 64`
- `TASK_LIST_NUM_QUEUES = 32`

### Worker loop
`worker_default()`:
- Locks task list, pops ready task (worker-specific first, then generic).
- Sleeps briefly (`1us`) if no task.
- Runs task and tracks CPU utilization.
- Supports synthetic CPU-stall hooks for stress/testing.

## Slot Object Pools and Synchronization
### Pool sizing
From `constant.hpp`:
- `SLOT_MAP_NUM = 512`
- `SLOT_CMD_NUM = 512`
- `TASK_ITEM_NUM = 2048`
- `ORDER_ENTITY_NUM = 8`

### Synchronization primitives
`SlotMapDl` and `SlotMapUl` rely mostly on atomics plus spin waits:
- Channel/task completion counters.
- U-plane/C-plane readiness flags.
- Slot end and cleanup counters.

Default generic wait threshold:
- `GENERIC_WAIT_THRESHOLD_NS = 4,000,000` (4ms).

`SlotMapDl` wait helpers (`waitDlGpuCommEnd`, `waitDlCompEnd`, etc.) warn/error when waits exceed threshold windows.
Doorbell path additionally uses a ~500us event wait threshold in `task_function_dl_aggr_2_ring_cpu_doorbell()`.

## GPU Context and Stream Strategy
The context builds multiple stream/context partitions across channels:
- UL: PUSCH/PUCCH/PRACH/SRS/order streams.
- DL: PDSCH/PDCCH/PBCH/CSI-RS/BFW/comms streams.

Two modes are supported in `mps.cpp` and `context.cpp`:
- Traditional execution-affinity/MPS-style partitioning (`CU_EXEC_AFFINITY_TYPE_SM_COUNT`).
- Green contexts (`cuGreenCtxCreate`) with device resource splits.

Guardrails:
- Green contexts require CUDA >= 12.4 (fatal otherwise).
- Controller sets `CUDA_DEVICE_MAX_CONNECTIONS=32` before first CUDA call when green contexts are enabled.

## Data Movement and FH Integration
- `FhProxy` is initialized inside `PhyDriverCtx` and NICs are registered from parsed config.
- `prepareUPlanePackets` is called in DL task paths before GPU-comm/doorbell steps.
- Optional H2D copy thread exists; CPU affinity and sched prio are configurable.
- `enable_prepone_h2d_cpy` is currently initialized to `false` in context init.

## Observability and Safety Mechanisms
- NVLOG tags across API/context/task paths.
- PMU metrics optional.
- Optional datalake thread and E3 ports configurable from YAML.
- Optional PTP monitoring thread spawned in `l1_init()` through app config.
- Defensive checks for invalid/missing buffers and unavailable aggregation objects in `l1_enqueue_phy_work()`.

## Code-Level Findings
1. SM allocation bug candidate in context initialization.
- In `context.cpp`, `mps_sm_pbch` is assigned from `ctx_cfg.mps_sm_pdcch` instead of `ctx_cfg.mps_sm_pbch`.
- This can silently mis-partition PBCH resources.

2. Batched memcpy is intentionally disabled in DL slot-map constructor path.
- `SlotMapDl` is constructed with batched memcpy forced off and comment notes thread-safety concerns in fanned-out GPU comm prepare tasks.
- Runtime knob may appear enabled in YAML but is not fully honored in this path.

3. Detached helper threads in destructor.
- H2D copy and UL pcap threads are detached during teardown, not joined.
- Raises shutdown-order and observability risks if those threads access partially torn-down state.

4. Prepone-H2D feature is effectively hard-disabled at context init.
- `enable_prepone_h2d_cpy = false` currently prevents that code path unless toggled elsewhere.

## Practical Implications
- The engine is optimized for throughput and low latency via object pools + lock-light synchronization, but correctness depends on strict lifecycle discipline.
- YAML tuning is highly influential; mis-specified SM budgets, worker counts, or timeout values can destabilize slot execution.
- The identified PBCH SM assignment issue should be prioritized because it directly affects channel-level GPU partitioning.
