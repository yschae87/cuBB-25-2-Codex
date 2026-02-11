# cuBB 25.3 Architecture Analysis (Source-Validated) (v1.1)

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

## Scope
This document refreshes architecture understanding using the current 25.3 workspace (`upgrade-25-3-workspace`) and the active runtime sources.

Primary evidence includes:
- Top-level: `README.md`
- Control plane/runtime: `cuPHY-CP/cuphycontroller`, `cuPHY-CP/cuphyl2adapter`, `cuPHY-CP/cuphydriver`
- GPU PHY kernels/libraries: `cuPHY/src/...`
- MAC stack: `cuMAC`, `cuMAC-CP`
- Python API: `pyaerial`

## Repository Decomposition
From the current tree:
- `cuPHY`: GPU PHY kernels/components (CUDA-heavy, channel pipelines).
- `cuPHY-CP`: runtime/control-plane integration (controller, L2 adapter, driver, FH integration, OAM, data lake).
- `cuMAC`: CUDA-accelerated scheduler APIs and kernels.
- `cuMAC-CP`: MAC control-plane application around cuMAC.
- `pyaerial`: Python bindings and high-level APIs to cuPHY components.
- `testBenches`, `testVectors`, `5GModel`: validation/tooling and vector generation.

Observed source scale snapshot (quick count in this workspace):
- `cuPHY-CP`: ~200 C/C++ source/header files.
- `cuPHY`: ~399 C/C++/CUDA source/header files.

## End-to-End Runtime Path (L2 -> L1 -> FH/GPU)
### 1) Boot and config ingestion
`cuphycontroller` (`cuphycontroller_scf.cpp`) parses YAML, validates environment/hardware, and builds `context_config`.

### 2) Driver startup
`pc_init_phydriver()` -> `l1_init()` creates `PhyDriverCtx`, registers cells, starts worker threads, and initializes OAM.

### 3) L2 adapter and timing loop
In integrated mode, `PHY_group` is started via `cuphyl2adapter`.
`PHY_module` (`nv_phy_module.cpp`) receives FAPI/NVIPC messages, tracks tick/slot state, and calls `PHYDriverProxy::l1_enqueue_phy_work()` when slot commands are complete.

### 4) Slot execution
`l1_enqueue_phy_work()` (`cuphydriver_api.cpp`) splits UL/DL channel commands, binds buffers + aggregation objects, and enqueues tasks to worker queues.

### 5) GPU and FH completion
DL/UL channel kernels execute via cuPHY aggregators; FH prep/compression/doorbell/callback tasks handle transport; indications/callbacks flow back to L2 through adapter callbacks.

## Control Plane vs Data Plane Responsibilities
### Control plane
- YAML parsing/validation (`yamlparser.cpp`)
- Cell lifecycle APIs (`l1_cell_create/start/stop`)
- OAM updates and runtime attributes
- Time/sync policy and slot drop guards in L2 adapter (`check_time_threshold`)

### Data plane
- Per-slot task graph execution (`task_function_dl_aggr.cpp`, `task_function_ul_aggr.cpp`)
- CUDA streams + channel kernels (cuPHY)
- FH packet prep/compression/decompression
- HARQ/order kernel and buffer pools

## Concurrency Model
### Thread domains
- L2 adapter message processing thread (`msg_processing`, `PHY_module::thread_func`).
- UL/DL worker pools in `PhyDriverCtx` (`Worker` threads, affinity-configured).
- Optional helper threads (metrics, datalake, H2D copy, PCAP capture, PTP monitors).

### Scheduling pattern
- Tasks are timestamped and pushed into `TaskList` priority queues.
- Workers poll worker-specific queue first, then generic queue.
- SlotMap objects use atomics/events for fine-grained dependencies instead of coarse global locks.

## GPU Resource Partitioning Model
`cuphydriver` supports two strategies:
- Execution-affinity style SM partitioning.
- CUDA green contexts (`cuGreenCtxCreate`) for stricter partitioning.

Channel-level SM budgets are passed from controller YAML through `context_config` (for example PUSCH/PDSCH/PDCCH/PBCH/SRS/order/gpu-comms budgets).

## cuMAC and cuMAC-CP Placement
### cuMAC
`cuMAC/src/api.h` defines high-capacity scheduler structures and constants (multi-cell/UE-group oriented, GPU-focused scheduler interface).

### cuMAC-CP
`cuMAC-CP/src/main.cpp` + `cumac_cp_handler.cpp` provide:
- Config and IPC setup.
- Message receive/dispatch flow.
- Task ring + scheduler parameter checks.

This stack is parallel to (not embedded in) `cuphydriver`; integration happens via higher-level L2/L1 messaging contracts.

## pyAerial Placement
`pyaerial` wraps cuPHY components for Python workflows:
- pybind C++ bridges under `pyaerial/pybind11`.
- Python API under `pyaerial/src/aerial`.
- Extensive test-vector-based unit tests under `pyaerial/tests`.

It is primarily an API/research/productivity layer, not the production L1 scheduler runtime.

## Key Architectural Observations (25.3)
1. Startup validation is stricter than earlier analyses suggested.
- Hardware presence, NIC/GPU checks, beam ID range checks, and metrics endpoint checks are enforced before runtime.

2. L2 adapter has explicit slot-drop protection.
- If L2+L2A latency exceeds threshold, slot commands are dropped to protect real-time behavior.

3. Task-graph style execution is mature but highly configuration-dependent.
- Worker counts, timeout thresholds, and channel enablement strongly influence task fanout and timing.

4. Resource partitioning is advanced but complex.
- Green-context + per-channel SM budgeting provides strong control, but introduces many deployment-sensitive failure modes.

## Architectural Risk Areas
- Configuration drift: many optional knobs have defaults; sparse YAMLs can silently change behavior.
- Real-time threshold tuning: improper timing thresholds can increase slot drops or timeout noise.
- Feature gating mismatch: some features appear configurable but are partially disabled in runtime paths.
- Cross-component observability: many helper threads and callbacks can complicate shutdown and fault attribution.

## Summary
The 25.3 architecture is a layered real-time system:
- `cuphycontroller` handles environment/config/lifecycle.
- `cuphyl2adapter` handles L2 messaging, timing, and slot command assembly.
- `cuphydriver` performs deterministic per-slot orchestration and GPU/FH execution.
- `cuPHY` supplies channel kernels.
- `cuMAC`/`cuMAC-CP` are adjacent scheduler acceleration/control stacks.
- `pyaerial` provides Python-facing access and test-vector-driven validation for cuPHY components.
