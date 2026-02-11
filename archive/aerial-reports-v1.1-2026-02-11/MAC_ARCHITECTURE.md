# MAC Architecture (v1.1)

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
- Summarizes how the MAC stack in this repo is split between the GPU scheduler library (`cuMAC`) and the control-plane/runtime (`cuMAC-CP`).
- Highlights the main software components, execution flow, and the messaging interface to `cuPHY` / `cuPHY-CP`.

## Top-Level Components
- `cuMAC` (`cuMAC/src`): GPU-accelerated L2 scheduling library. Provides CUDA/CPU implementations for UE selection, PRB allocation, layer selection, and MCS selection. Key headers: `api.h` (limits/constants), `cumac.h` (data structures), and module headers such as `multiCellUeSelection.cuh`, `multiCellScheduler.cuh`, `multiCellLayerSel.cuh`, `mcsSelectionLUT*.{cuh,cpp}`.
- `cuMAC-CP` (`cuMAC-CP/src`, `cuMAC-CP/lib`): Control-plane service that terminates MAC-side NVIPC endpoints, builds per-slot tasks, manages GPU/CPU buffers, and invokes `cuMAC` kernels. Message formats live in `cuMAC-CP/lib/cumac_msg.h`.
- Shared test/support: TensorRT-based optional DRL MCS selection (`mcsSelectionDRL.*`, `trtEngine.*`), CPU fallbacks for all schedulers (`*Cpu.{h,cpp}`), and per-slot simulation contexts (`cumacSubcontext.h`).

## Software Architecture & Flow
1. **Process bring-up (cuMAC-CP)**  
   - `main.cpp` parses YAML (`CONFIG_CUMAC_CONFIG_YAML`), pins optional low-priority cores, initializes NVLOG, and starts `cumac_receiver` (`msg_recv.cpp`).
   - `cumac_cp_configs` (`cumac_cp_configs.cpp/.hpp`) loads threading, GPU selection, buffer sizing, and tuning flags (group buffer, multi-stream, slot concurrency).
2. **IPC setup**  
   - `cumac_receiver` wires `nv::phy_mac_transport_wrapper` and `nv::phy_epoll_context` to the configured per-cell NVIPC endpoints.
   - A lock-free ring (`nv::lock_free_ring_pool`) provides a pool of `cumac_task` objects shared by worker threads and the epoll callback.
3. **Control-plane handshake**  
   - `cumac_cp_handler` responds to CONFIG/START/STOP (and PARAM) requests, caching per-cell static config. After all cells are configured it allocates group-wide GPU/CPU buffers, initializes CUDA streams, and constructs per-ring `cumac_task` objects.
4. **Per-slot scheduling**  
   - For each slot, `CUMAC_SCH_TTI_REQUEST` messages arrive per cell. `sched_slot_data` (ring of `SCHED_SLOT_BUF_NUM` = 4) groups messages by SFN/slot and enforces ordering. `CUMAC_TTI_END` marks slot completeness.
   - The handler copies request payloads + data buffers into the active `cumac_task` (GPU or CPU path), then enqueues the task to the worker ring and signals via semaphore.
5. **Task execution (`cumac_task`)**  
   - `setup()`: Populate `cumac::cumacCellGrpPrms`, `cumac::cumacCellGrpUeStatus`, and `cumac::cumacSchdSol` from incoming data, select GPU/CPU modules, and issue any required host↔device copies (optionally using a contiguous group buffer).
   - `run()`: Invoke requested modules based on `taskBitMask` in order: UE selection (`multiCellUeSelection`), PRB allocation (`multiCellScheduler`), layer selection (`multiCellLayerSel`), MCS selection (`mcsSelectionLUT` or optional `mcsSelectionDRL`). CUDA streams are used when multi-stream is enabled; CPU fallbacks run the `*Cpu` variants.
   - `callback()`: Copies outputs back (UE set, allocSol, layerSelSol, mcsSelSol), emits `CUMAC_SCH_TTI_RESPONSE`, and updates throughput counters.

## cuMAC Scheduler Library (GPU/CPU)
- **Core data structures (in `api.h` / `cumac.h`)**
  - `cumacCellGrpPrms`: Describes a coordinated scheduling group (cells, PRGs, antenna counts, noise variance, precoder/receiver type, allocation type, HARQ, thresholds, etc.) plus pointers to per-cell/per-UE buffers (channel estimates, SINR, masks).
  - `cumacCellGrpUeStatus`: Per-UE state across the group (avg rates, CQI/RI/PMI, buffer sizes, TB error flags, power/noise, priority weights, HARQ metadata).
  - `cumacSchdSol`: Outputs (selected UE set per cell, allocation map, layer selection, MCS per UE) and optional PF metrics.
- **Execution context**
  - `cumacSubcontext` wraps a per-slot simulation/execution context, choosing GPU vs CPU path, precision (FP32/BF16), HARQ/CQI/RI usage, and which modules run (SINR calc, UE selection, PRB allocation, layer/MCS selection). It owns CUDA streams and per-slot buffers.
  - Optional modules: SINR calc (`multiCellSinrCal`), MU-MIMO grouping/sorting (`multiCellMuUeGrp`/`multiCellMuUeSort`), round-robin variants (`roundRobinUeSel`, `roundRobinScheduler`), beamforming and SRS schedulers (`multiCellBeamform`, `multiCellSrsScheduler`, `multiCellWbCqiRepScheduler`), and TensorRT-backed DRL MCS selection (`mcsSelectionDRL`, `trtEngine`).
- **Kernel organization**
  - Each module has GPU and CPU implementations (`*.cuh` / `*Cpu.h`). GPU kernels are launched with descriptors built from the group structures and configured grid/block sizes (`multiCellScheduler` selects kernels based on precision, allocation type, precoding, and HARQ).
  - CPU fallbacks mirror the GPU algorithms to support debug and non-GPU environments.

## cuMAC-CP Runtime
- **Message ingestion**: `cumac_receiver` registers an epoll callback (`start_epoll_loop`) that hands `nv::phy_mac_msg_desc` objects to `cumac_cp_handler`. Messages are not copied until they are assigned to a `cumac_task`, preserving zero-copy semantics on the NVIPC data buffer.
- **Slot state**: `sched_slot_data` tracks the current slot (`ss_sched`), per-cell message cache, and the active `cumac_task`. New slot detection resets state and frees any dropped messages/tasks.
- **Buffer management**: `cumac_cp_handler::initiate_cumac_task` pre-allocates pinned host and device buffers for group- and cell-scoped data. Optional `group_buffer_enable` packs all GPU buffers into a contiguous block to reduce allocations and improve coalesced copies. CPU-only mode is supported via config (`run_in_cpu`).
- **Threading**: Worker thread affinity and count come from YAML (`worker_cores`, `thread_num_per_core`). A semaphore coordinates the producer (epoll thread) and consumers (workers pulling from `task_ring`). CUDA streams per task are created when multi-streaming is enabled.

## Interface to cuPHY / cuPHY-CP
- **Transport**: Uses `nv::phy_mac_transport_wrapper` with NVIPC shared-memory pools. All MAC<->PHY messages use headers defined in `cuMAC-CP/lib/cumac_msg.h`.
- **Control messages**
  - `CUMAC_PARAM_REQUEST/RESPONSE`: Optional parameter exchange (hooks present in `cumac_cp_handler`).
  - `CUMAC_CONFIG_REQUEST/RESPONSE`: Per-cell static config (max UEs/PRGs, antenna counts, allocation type, precoder/receiver type, HARQ/CQI flags, thresholds). Once all cells are configured, `cuMAC-CP` allocates persistent buffers and initializes tasks.
  - `CUMAC_START_REQUEST/RESPONSE`, `CUMAC_STOP_REQUEST/RESPONSE`, `CUMAC_ERROR_INDICATION`: Lifecycle control and error reporting.
- **Per-slot scheduling**
  - Request (`CUMAC_SCH_TTI_REQUEST`): sent by L2 MAC (via NVIPC) to cuMAC-CP once per cell per slot. Payload (`cumac_tti_req_payload_t`) carries cell ID, UL/DL indicator, `taskBitMask`, UE/antenna counts, noise variance, and offsets (`cumac_tti_req_buf_offsets_t`) into the NVIPC data buffer for:
    - UE identity/activity (`CRNTI`, `srsCRNTI`, `nActiveUe`), channel quality (`prgMsk`, `postEqSinr`, `wbSinr`, `sinVal`).
    - Channel/beamforming matrices (`estH_fr` FP32/FP16, `prdMat`, `detMat`).
    - Historical state (`avgRatesActUe`, `prioWeightActUe`, `tbErrLastActUe`, `newDataActUe`, `allocSolLastTxActUe`, `layerSelSolLastTxActUe`, `mcsSelSolLastTxActUe`).
  - Ordering: One `SCH_TTI.req` per cell per slot, then `CUMAC_TTI_END` (also sent by L2). `sched_slot_data` buffers per-cell messages and primes a `cumac_task` once the slot completes.
  - Execution: Worker threads invoke cuMAC modules (UE selection, PRB allocation, layer selection, MCS selection) according to `taskBitMask`, using the copied buffers. Outputs accumulate in `cumacSchdSol`.
  - Response (`CUMAC_SCH_TTI_RESPONSE`): sent by cuMAC-CP to L2 per cell with offsets to `setSchdUePerCellTTI`, `allocSol`, `layerSelSol`, `mcsSelSol` in the NVIPC TX pool.
  - Error path: `CUMAC_TTI_ERROR_INDICATION` and throughput counters capture drops if slots overrun or buffers misalign.
- **Data ownership expectations**
  - PHY/PHY-CP own the NVIPC message and data buffers; `cuMAC-CP` copies into its pinned/device buffers as needed and releases the IPC buffers once safe.
  - Outputs are written back into PHY-owned TX buffers; PHY consumes them to program PDSCH/PUSCH grants and HARQ state in L1.

### CONFIG.req / CONFIG.resp (L2 MAC → cuMAC-CP)
- Sender/receiver: L2 MAC sends CUMAC CONFIG.req per cell directly to cuMAC-CP over NVIPC; cuMAC-CP processes and replies with CONFIG.resp per cell.
- Envelope: `cumac_msg_header_t` (in `cuMAC-CP/lib/cumac_msg.h`), fields `message_count`, `handle_id` (cell id), `type_id=CUMAC_CONFIG_REQUEST`, `body_len`. CONFIG messages do not include `sfn/slot`.
- Payload: `MAC_SCH_CONFIG_REQUEST` from `cuMAC/src/api.h`, one per cell. Fields:
  - `harqEnabledInd`, `mcsSelCqi`
  - `nMaxCell`, `nMaxActUePerCell`, `nMaxSchUePerCell`
  - `nMaxPrg`, `nPrbPerPrg`, `nMaxBsAnt`, `nMaxUeAnt`, `scSpacing`
  - `allocType`, `precoderType`, `receiverType`, `colMajChanAccess`
  - `betaCoeff`, `sinValThr`, `corrThr`, `mcsSelSinrCapThr`, `mcsSelLutType`, `prioWeightStep`
- Handling in `cuMAC-CP` (`cumac_cp_handler::on_config_request`):
  - Stores payload into `cell_configs[cell_id]`, increments `configured_cell_num`.
  - When all cells are configured, `check_config_params()` aggregates to `group_params` (`cumacSchedulerParam`), validates homogeneity across cells with `CHECK_VALUE_EQUAL_ERR`, and allocates per-ring `cumac_task` objects + buffers.
  - `group_params` derivations: sum UE counts (`nUe`, `nActiveUe`), set `nPrbGrp`, antennas, `W=12*scSpacing*nPrbPerPrg`, `numUeSchdPerCellTTI`, precoder/receiver/allocType, HARQ/CQI flags, thresholds, etc.
- Response: `CUMAC_CONFIG_RESPONSE` has only `error_code` (always 0 in current code). Sent immediately after ingesting each CONFIG.req (`tx_send` + `tx_post`). No negative ack path; validation failures only log.
- Expectations for sender (L2/SCF side):
  - Pack CONFIG.req as `[cumac_msg_header_t][MAC_SCH_CONFIG_REQUEST]` with correct `body_len` and per-cell `handle_id`.
  - Send one CONFIG.req per cell before any TTI traffic; buffer allocation happens after the last CONFIG is received.
  - Homogeneous static config across cells is assumed; mismatches will log errors and may mis-size buffers.
- Distinguish from FAPI PHY config: SCF FAPI `CONFIG.req/resp` (for PHY bring-up) are handled in cuPHY-CP and are separate from these CUMAC config messages, which size and configure the scheduler. Both must complete before steady-state TTI traffic.

## TTI Message Sequence (happy path)
```mermaid
sequenceDiagram
    participant L2 as L2 / SCF FAPI
    participant PHY_CP as cuPHY-CP
    participant MAC_CP as cuMAC-CP (epoll)
    participant Worker as cuMAC-CP worker
    participant cuMAC as cuMAC kernels/CPU fallback
    participant cuPHY as cuPHY

    L2->>PHY_CP: FAPI CONFIG.req (per cell)
    PHY_CP-->>L2: FAPI CONFIG.resp

    L2->>MAC_CP: CUMAC CONFIG.req (per cell)
    MAC_CP-->>L2: CUMAC CONFIG.resp (per cell)
    Note over MAC_CP: After last CONFIG.resp<br/>allocate buffers & init tasks (group_params)

    L2->>PHY_CP: FAPI START.req
    PHY_CP-->>L2: FAPI START.resp

    L2->>MAC_CP: CUMAC START.req
    MAC_CP-->>L2: CUMAC START.resp

    loop Each slot
        L2->>MAC_CP: CUMAC SCH_TTI.req (one per cell)
        L2->>MAC_CP: CUMAC TTI_END (slot complete)
        MAC_CP->>Worker: enqueue cumac_task (slot)
        Worker->>cuMAC: setup + run (UE sel, PRB alloc, layer, MCS)
        cuMAC-->>Worker: outputs
        Worker-->>MAC_CP: callback
        MAC_CP-->>L2: CUMAC SCH_TTI.resp (per cell; UE set/alloc/layer/MCS)
        L2->>PHY_CP: FAPI DL_TTI/UL_TTI/TX_DATA.req populated with cuMAC outputs
        PHY_CP->>cuPHY: program grants, execute slot
    end
```

- **L2 → cuMAC-CP:** L2 MAC sends `CUMAC_SCH_TTI_REQUEST` per cell (with NVIPC data buffers) followed by `CUMAC_TTI_END` when the slot is complete.
- **cuMAC-CP slot assembly:** `sched_slot_data` groups messages by `sfn/slot`, caches per-cell requests, and on `TTI_END` enqueues a `cumac_task` to worker threads.
- **cuMAC execution:** Worker sets up group descriptors and runs cuMAC kernels/CPU fallbacks per `taskBitMask` to generate UE set, allocation, layer, and MCS solutions.
- **cuMAC-CP → L2:** Sends `CUMAC_SCH_TTI_RESPONSE` per cell with offsets to `setSchdUePerCellTTI`, `allocSol`, `layerSelSol`, and `mcsSelSol` in the NVIPC TX pool.
- **L2 → cuPHY-CP/cuPHY:** L2 uses the scheduling outputs to populate standard FAPI `DL_TTI.request`, `UL_TTI.request`, and `TX_DATA.request`, which cuPHY-CP executes on cuPHY.

## Notes for Extension/Integration
- Module selection is driven by the `taskBitMask` in `CUMAC_SCH_TTI_REQUEST`; keeping it aligned between cuPHY-CP and cuMAC-CP is critical.
- Group-level parameters assume homogeneous per-cell static config (checked in `check_config_params`); heterogeneous settings require code changes where `CHECK_VALUE_EQUAL_ERR` is used.
- Enable `group_buffer_enable`, `multi_stream_enable`, or `slot_concurrent_enable` in YAML to tune throughput vs. memory footprint when integrating with cuPHY testbeds.
