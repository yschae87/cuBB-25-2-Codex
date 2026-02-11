# cuPHY Controller Deep Dive (25.3, Source-Validated) (v1.1)

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
This update is based on direct inspection of:
- `cuPHY-CP/cuphycontroller/examples/cuphycontroller_scf.cpp`
- `cuPHY-CP/cuphycontroller/src/cuphydriver.cpp`
- `cuPHY-CP/cuphycontroller/src/yamlparser.cpp`
- `cuPHY-CP/cuphycontroller/include/yamlparser.hpp`

The focus is the controller/bootstrap layer that converts YAML + runtime settings into a live `PhyDriverCtx` and L2 adapter runtime.

## What the Controller Owns
`cuphycontroller` is responsible for:
- Parsing and validating deployment YAML.
- Building `context_config` for `l1_init`.
- Creating/starting cells in standalone mode.
- Bootstrapping L2 adapter mode (`PHY_group`) in integrated mode.
- Initializing OAM and coordinating shutdown.

It is not the per-slot scheduler; that logic is in `cuphydriver` and `cuphyl2adapter`.

## Startup Sequence (Integrated Mode)
Observed sequence in `main()` (`cuphycontroller_scf.cpp`):
1. Initialize tracing and `curl` global state.
2. Build config name (`cuphycontroller_<profile>.yaml`) and parse it.
3. Apply `low_priority_core` affinity (for low-priority helper threads).
4. Configure PTP monitoring mode:
- `enable_ptp_svc_monitoring` path validates RMS threshold and checks syslog-derived status.
- `enable_rhocp_ptp_events_monitoring` is mutually exclusive with PTP service monitoring and requires publisher/node/consumer fields.
5. Initialize nvlog (`nvlog_fmtlog_init`) and signal handlers.
6. Parse controller YAML via `YamlParser::parse_file()`.
7. If green contexts are enabled, set `CUDA_DEVICE_MAX_CONNECTIONS=32` before first CUDA init call.
8. Call `cudaSetDevice()`, initialize PTI/CUPTI helpers, run PHC/clock sanity checks.
9. Build a large `context_config` object from parser getters and call `pc_init_phydriver()`.
10. In integrated mode: create `PHYDriverProxy`, initialize SCF FAPI, create `PHY_group`, start it, then block on `grp->join()`.
11. On exit: `pc_finalize_phydriver()`, close nvlog, `curl_global_cleanup()`.

## `pc_init_phydriver` and Lifecycle Wrappers
From `src/cuphydriver.cpp`:
- `pc_init_phydriver()` checks non-empty UL cores, DL cores, and worker descriptor list.
- Calls `l1_init(pdh, ctx_cfg)`.
- Immediately initializes OAM (`CuphyOAM::getInstance()->init_everything()`).
- `pc_finalize_phydriver()` wraps `l1_finalize()`.
- `pc_standalone_create_cells()` calls `l1_cell_create` then `l1_cell_start` for each parsed cell.

`pc_start_l1()` is currently a no-op (the `l1_start` path is commented).

## YAML Parsing Responsibilities and Validation Depth
### File-level parsing
`YamlParser::parse_file()`:
- Loads YAML and checks SDK version (`aerial::check_yaml_version`).
- Resolves `l2adapter_filename` relative to configured root.
- Parses:
- `cuphydriver_config` via `parse_cuphydriver_configs()`.
- Cell list via `parse_cell_configs()`.

### Backend metrics address checks
When built with `AERIAL_METRICS`:
- Requires `host:port` format.
- Requires host `127.0.0.1` exactly.
- Requires port in `[1, 65535]` and currently available (bind check).
- Rejects startup on invalid format or occupied port.

### Cell-level validation (`parse_single_cell`)
Notable strict checks:
- Duplicate `cell_id` rejected.
- `ru_type` range checked.
- NIC PCI address must exist under `/sys/bus/pci/devices/<addr>/vendor`.
- Source and destination MAC format validated (`XX:XX:XX:XX:XX:XX`).
- Duplicate destination MAC rejected in non-UE mode.
- Compression method restricted to supported enum values.
- No-compression mode requires 16-bit IQ.
- BFP bit width range checked.
- Optional fields get explicit defaults with NVLOG warnings (for example SRS timing windows, nMaxRxAnt, UL U-plane TX offset).

### Driver-config parsing (`parse_cuphydriver_configs`)
Parses and/or defaults a large set of knobs, including:
- Worker/core layout, scheduling priorities, graph flags.
- UL order timeouts and logging controls.
- MPS/green-context channel SM budgets.
- Datalake and E3 publisher/rep/sub ports.
- PCAP logger and UL packet capture thread settings.
- Beam ID range constraints (static/dynamic ranges, non-overlap, valid bounds).
- Optional tracing/debug flags.

It also validates:
- Each configured GPU exists (`/dev/nvidia<id>`).
- Each NIC exists under PCI sysfs path.
- `cell_group_num` does not exceed YAML cell count.

## Standalone Mode Path
Standalone mode (`standalone=1`) flow:
- Parse standalone launch pattern YAML.
- Create/start cells.
- Optionally simulate L2 by running `pc_l2_adapter` worker (`L2Adapter`) that enqueues slot commands with synthetic SFN/slot progression and busy-wait pacing.

## Code-Level Findings
1. `pc_start_l1()` is effectively stubbed.
- Impact: lifecycle behavior depends on `l1_init` side effects; explicit "start" semantics are currently unclear.

2. `parse_cuphydriver_configs()` has at least one default-value inconsistency.
- For `send_static_bfw_wt_all_cplane`, warning text says default 0, but code assigns 1 in exception path.

3. `parse_cell_configs()` allocates `unique_nic_info` with `new[]` and returns early on parse failure without cleanup.
- Impact: small leak on startup failure path.

4. `parse_eth_addr()` uses `strtok` on `const_cast<char*>(std::string::c_str())`.
- Impact: undefined-behavior risk due mutation through `c_str()` pointer.

5. Standalone simulation pacing uses `wait_ns(l2args->usec - 2 * 1000)`.
- Impact: if `usec <= 2000`, this underflows into a very large wait value.

## Practical Implications
- `cuphycontroller` is now doing substantial "environment gatekeeping" (hardware presence, metrics endpoint sanity, beam ID constraints) before runtime starts.
- Many features rely on defaults when keys are missing; production YAML should be treated as a fully-specified contract, not a sparse override file.
- Green-context deployments must keep the environment ordering (`CUDA_DEVICE_MAX_CONNECTIONS` before first CUDA initialization), which the current startup path does enforce.
