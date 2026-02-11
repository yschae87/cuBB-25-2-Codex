# Multi-Task / Multi-Thread / Parallel Processing Architecture (v1.1)

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

## 1. Scope & High-Level Component Map

This document describes how cuPHY/cuPHY-CP and cuMAC/cuMAC-CP implement multi-task, multi-thread, and GPU-parallel processing, using downlink PDSCH as an end-to-end example.

**Major components**
- **cuPHY (`cuPHY/src/cuphy/`)**: CUDA kernels and C++ wrappers for 5G NR PHY (PDSCH, PDCCH, PBCH, CSI-RS, UL, etc.).
- **cuPHY-CP (`cuPHY-CP/`)**: Host-side PHY driver, slot maps, fronthaul integration, tick generation, RU emulator.
- **cuMAC (`cuMAC/src/`)**: GPU-accelerated MAC scheduling kernels (multi-cell UE selection, PRB allocation, MCS, etc.).
- **cuMAC-CP (`cuMAC-CP/src/`)**: Control-plane app that receives scheduling requests, dispatches them to cuMAC kernels, and returns decisions.

Parallelism comes from:
- **CUDA kernels & streams** (data-parallel PHY/MAC).
- **Slot-based worker threads** (downlink/uplink tasks).
- **Timer/tick threads** (TTI pacing).
- **CP/UP fronthaul threads** (DPDK/Aerial FH driver).
- **Lock-free queues + atomics** for inter-thread coordination.

When reading code, use the following anchors for deeper inspection:
- Slot maps and DL tasks: `cuPHY-CP/cuphydriver/src/downlink/slot_map_dl.cpp`, `task_function_dl_aggr.cpp`
- PDSCH aggregator: `cuPHY-CP/cuphydriver/src/downlink/phypdsch_aggr.cpp`
- Tick thread: `cuPHY-CP/cuphyl2adapter/lib/nvPHY/nv_tick_generator.cpp`
- MAC CP threading: `cuMAC-CP/src/msg_recv.cpp`, `cumac_task.hpp/.cpp`, `cumac_cp_handler.cpp`

---

## 2. CUDA Parallelism in cuPHY and cuMAC

### 2.1 cuPHY kernels and context

- Kernels for PHY features live under `cuPHY/src/cuphy/` (e.g., modulation, LDPC, channel estimation, PDSCH DMRS).
- `cuphy_context.hpp` and `device.hpp` provide:
  - Cached device index, SM count, and shared memory limits.
  - Helpers to size block/grid for each kernel.
- PTI timing integration (`cuphy_pti.hpp`) adds:
  - Device-side `save_start_time/save_stop_time` that use `atomicAdd` and `__globaltimer()` to capture per-kernel start/stop timestamps and CTA counts.
  - Host-side logging in `task_function_dl_aggr.cpp` (see CUPHY_PTI traces).

Kernels use:
- Thread indices (`threadIdx`, `blockIdx`) for per-PRB/RE/UE work partitioning.
- Coalesced access via row/column-major flags and `tensor_desc.hpp` descriptors.

### 2.2 cuMAC scheduler kernels

In `cuMAC/src/`:
- **Multi-cell schedulers and helpers**:
  - `multiCellUeSelection.cu`, `multiCellScheduler.cu`, `multiCellLayerSel.cu`, `multiCellSinrCal.cu`.
  - `mcsSelectionLUT.cu` for per-UE MCS decisions.
- CPU equivalents (`*Cpu.cpp`) mirror the algorithms to support CPU runs and validation.

Parallel pattern:
- Kernels tile over **[cell, UE, PRB group, antenna]** dimensions.
- Each block/thread processes a slice of UE groups, PRGs, or antenna pairs.
- Precision, HARQ, and column/row-major behavior controlled through parameters in `cumacCellGrpPrms` and flags in `cumac_task`.
- `cumac_pti` timing in GPU kernels is surfaced via CUDA events recorded in `cumac_task` to expose per-stage latency.

---

## 3. Slot Map and Worker Architecture in cuPHY-CP

### 3.1 Slot maps as per-slot DAGs

In `cuphydriver/src/downlink/slot_map_dl.cpp`:
- `SlotMapDl` represents all DL processing for one slot:
  - Aggregated cell list and per-cell output buffers (`aggr_cell_list`, `aggr_dlbuf_list`).
  - Pointers to per-channel aggregators (`aggr_pdsch`, `aggr_pdcch_dl`, `aggr_pbch`, `aggr_csirs`, `aggr_dlbfw`).
  - Atomic flags/counters:
    - `atom_dl_cplane_start`, `atom_dlc_done`, `atom_uplane_prep_done`.
    - `atom_dl_gpu_comm_end`, `atom_dl_comp_end`.
    - Per-cell FHCB completion flags and slot end counters.
    - `atom_cell_fhcb_done[cell]` and channel/task end counters (`atom_dl_channel_end_threads`, `atom_dl_end_threads`) to gate teardown.
- Methods like `waitDLCDone`, `waitFHCBDone`, `waitDlGpuCommEnd`, `waitDlCompEnd` spin on atomics with timeouts to coordinate the per-slot task DAG.
  - Functions warn when exceeding `GENERIC_WAIT_THRESHOLD_NS` to highlight jitter.
  - `cuphyBatchedMemcpyHelper` is configured per-slot (source/dest hint) to batch H2D/D2H traffic.

The slot map encapsulates:
- **C-plane** channel setup/run (PDCCH, PBCH, CSI-RS).
- **PDSCH** setup/run.
- **GPU compression** and fronthaul preparation.
- **FH callbacks** and completion signaling to upper layers.

### 3.2 Tick generator: TTI pacing thread

In `cuphyl2adapter/lib/nvPHY/nv_tick_generator.cpp`:
- `tti_gen::start_slot_indication()` launches a dedicated **timer thread**:
  - Mode 0: **poll method** (`slot_indication_thread_poll_method`) – busy-ish loop, SCHED_FIFO, CPU-affinity, small sleeps (`std::this_thread::sleep_for`) to reduce jitter and OS starvation.
  - Mode 1: **sleep method** (`slot_indication_thread_sleep_method`) – uses `clock_nanosleep` with absolute wake times; calculates next SFN/slot boundary based on GPS/TAI offsets.
  - Mode 2: **timerfd method** (`slot_indication_thread_timer_fd_method`) – timerfd + epoll event loop.
- The tick thread:
  - Is assigned a name `"timer_thread"`, SCHED_FIFO priority (`pthread_setschedparam`), and fixed CPU affinity (`pthread_setaffinity_np`).
  - Calls `slot_indication_handler() -> send_slot_indication()` each TTI.
  - `send_slot_indication()` invokes `module_->tick_received(current_scheduled_ts)`, which leads to DL/UL slot work being scheduled by the PHY driver.
  - Jitter is monitored (`allowed_offset_nsec`) and logged; poll mode yields briefly (`std::this_thread::sleep_for`) to avoid starving the system.
  - `stop_slot_indication()` cancels the thread when reference count drops to zero.

This provides a deterministic TTI clock for all downstream per-slot work.

### 3.3 Downlink task functions and worker threads

In `cuphydriver/src/downlink/task_function_dl_aggr.cpp`:
- Several **task functions** are registered with the PHY driver’s worker framework:
  - `task_work_function_dl_aggr_bfw` – beamforming task.
  - `task_work_function_cplane` – C-plane (PDCCH/PBCH/CSI-RS) setup + run + signaling.
  - `task_work_function_dl_aggr_2` – U-plane PDSCH & fronthaul preparation + metrics + callback.
  - `task_work_function_debug` – debug tracing & PTI stats.
  - `task_work_function_dl_fh_cb` – FH prepare callbacks for each cell.
  - `task_work_function_dl_aggr_2_ring_cpu_doorbell` – CPU doorbell for NIC after compression.
  - `task_work_function_cplane` – waits on per-channel CUDA events to signal completion and triggers compression stream waits.

Each task:
- Runs in a **worker thread** and receives a pointer to `SlotMapDl`.
- Uses NVLOG and timing macros (TI_* tags) for instrumentation.
- Uses **CUDA events** and `non_blocking_event_wait_with_timeout()` to coordinate:
  - Prepare kernel completion.
  - Compression start/stop.
  - TX end events for NIC-specific prepare buffers.
  - PTI logs via `cuphy_pti_get_record_all_activities` in debug task.

**Worker threading model**:
- A configured number of worker threads are created (see PHY driver configuration) and pinned to CPUs close to NICs for predictable latency.
- Tasks can be split by map (slot) and by **first_cell / num_cells** to parallelize across cells.
 - `task_work_function_dl_aggr_bfw` and `task_work_function_dl_aggr_2` both fence on `slot_map->waitDlGpuCommEnd()` / `waitDlCompEnd()` to respect dependencies.

---

## 4. PDSCH Aggregator and GPU Pipeline

### 4.1 PDSCH aggregator object

`cuPHY-CP/cuphydriver/src/downlink/phypdsch_aggr.cpp` defines `PhyPdschAggr`, which wraps the cuPHY PDSCH TX API:

- Constructor:
  - Associates the object with a `phydriver_handle` and a CUDA stream (`s_channel`).
  - Initializes static params (`cuphyPdschStatPrms_t`) and dynamic params (`cuphyPdschDynPrms_t`) with:
    - Number of cells, antennas, numerology, PRBs per BWP, etc.
    - `enableBatchedMemcpy` from driver config.
    - Pointers to PDSCH data inputs (`DataIn`) and output tensors (`DataOut`).
    - `procModeBmsk` computed from driver flags (`PDSCH_PROC_MODE_GRAPHS`, `PDSCH_PROC_MODE_SETUP_ONCE_FALLBACK`).
- `createPhyObj()`:
  - Iterates over active cells (via `PhyDriverCtx` and `Cell`), builds per-cell static descriptors (`cuphyCellStatPrm_t`).
  - When all expected cells are active (`static_params_cell.size() == pdctx->getCellGroupNum()`):
    - Sets stream priority (`cudaStreamGetPriority`).
    - Allocates debug structs, optionally enabling TB size checks.
    - Calls `cuphyCreatePdschTx(&handle, &static_params)` to instantiate the cuPHY PDSCH object.
    - Tracks GPU memory footprint via `static_params.pOutInfo->pMemoryFootprint`.
- `setup()`:
  - For a given `SlotMapDl`, binds:
    - `dyn_params.pCellGrpDynPrm` to L2-provided PDSCH cell group parameters.
    - `DataIn.pTbInput[]` pointing at the TB buffers per cell.
    - `DataOut.pTDataTx[]` pointing at per-cell DL output buffers (GPU buffers attached to `DLOutputBuffer`).
  - Handles “prepone H2D copy” by waiting on a CUDA event if enabled.
  - Calls `cuphySetupPdschTx(handle, &dyn_params, nullptr)` on `s_channel` and records setup time via CUDA events.
  - In AAS/multi-TB cases, maps `cellPrmDynIdx` per cell to set correct output buffer pointer.
- `run()` and `waitRunCompletionGPUEvent()`:
  - Launch PDSCH kernels on `s_channel` and wait on cuPHY’s completion events.
  - `waitRunCompletionGPUEvent` is used by `task_work_function_cplane` to fence before compression.

### 4.2 Compression & fronthaul preparation

After PDSCH/PDCCH/PBCH/CSI-RS have run:

- `task_work_function_dl_aggr_2`:
  - Waits for C-plane tasks (`slot_map->waitDLCDone`) if mMIMO is enabled.
  - **U-plane prepare**:
    - For each cell, uses `FhProxy::prepareUPlanePackets` with RU type, peer ID, CUDA stream, start timestamp, slot indication, per-cell slot info, and the `DLOutputBuffer` for that cell.
    - Uses `cuphyBatchedMemcpyHelper` to batch memory copies.
  - Optionally waits for PDSCH TB H2D copy completion and then triggers `PhyPdschAggr::callback` early to inform L2.
  - Updates per-cell metrics and NIC Tx metrics via `FhProxy::UpdateTxMetricsGpuComm`.
  - Metrics/trace: `slot_map->timings.start_t_dl_uprep/end_t_dl_uprep` and batched memcpy state.

- `task_work_function_debug` and `task_work_function_dl_aggr_bfw`:
  - Query CUDA events on compression and prepare kernels.
  - Use `FhProxy` to stamp GPU timestamps into CQE tracing.
  - Report PTI durations for PREPREP, PREP, TRIGGER phases.

- Compression in `task_work_function_dl_aggr_bfw`:
  - Waits on PDSCH and other channels’ “run completion” events.
  - Waits for GPU comms to finish.
  - Runs compression kernels (`compression_dlbuf->runCompression`).
  - Sets ready flags so NIC/green contexts can pick up compressed IQ.
  - Honors per-NIC prepare events (`prepare_tx_dlbuf_per_nic[i]->getPrePrepareStopEvt()/getTxEndEvt`) before triggering doorbells.

### 4.3 Thread and stream interplay for PDSCH

End-to-end for one DL slot:

1. **Tick thread** emits `SLOT.indication`.
2. L2/FAPI messages are processed into slot commands; a `SlotMapDl` is created and enqueued to DL workers.
3. **DL worker threads** execute:
   - C-plane task: PDCCH/PBCH/CSI-RS setup & run.
   - PDSCH task: `PhyPdschAggr::setup` + `cuphyRunPdschTx`.
4. CUDA streams execute PDSCH and other channels; PTI captures GPU timings.
5. Compression and U-plane prepare tasks (using FhProxy + batched memcpy) run in separate worker tasks/streams.
6. FH callback tasks inform upper layers and mark FHCB done in `SlotMapDl`.
7. `task_work_function_dl_aggr_2_ring_cpu_doorbell` optionally signals NIC doorbells after compression is ready.

---

## 5. MAC Scheduling Pipeline and cuMAC-CP Threading

### 5.1 Threading model in cuMAC-CP

In `cuMAC-CP/src/msg_recv.cpp`:

- **Receiver thread**:
  - Created by `cumac_receiver::start()`.
  - Runs `start_epoll_loop()` on a dedicated thread, with configured name, priority, and CPU affinity.
  - Uses `phy_epoll_context` to watch NVIPC transport FDs and dispatch messages to `on_msg()`.

- **Worker threads**:
  - Created per configured core; `thread_num_per_core` workers per core.
  - Each worker thread:
    - Calls `nvlog_fmtlog_thread_init()`.
    - Sets thread name and CPU affinity.
    - Waits on a semaphore and dequeues `cumac_task` objects from a `lock_free_ring_pool`.
    - For each task: runs `setup()`, `run()`, `callback()`, then frees the task back to the pool.
  - Receiver thread enqueues tasks into the ring and posts the semaphore once per RX burst; workers drain all available tasks before sleeping again.

### 5.2 Task objects, CUDA streams, and buffers

In `cuMAC-CP/src/cumac_task.hpp`/`.cpp`:

- `cumac_task` contains:
  - Flags: `taskBitMask`, `run_in_cpu`, `slot_concurrent_enable`.
  - Timing fields: wall-clock timestamps (`ts_*`) and many CUDA events (`ev_*`) for copy/setup/run/callback.
  - GPU resources: per-task CUDA stream, `cells_buf`, `group_buf`, per-cell descriptors, GPU structs.
  - Scheduler objects: GPU (`multiCellUeSelection`, `multiCellScheduler`, `multiCellLayerSel`, `mcsSelectionLUT`) and CPU equivalents.
  - Input/output arrays for metrics and matrices (`input_estH_fr`, `output_allocSol`, etc.).
  - `calculate_output_data_num()` derives per-buffer sizes from group params; `init_cumac_modules()` instantiates GPU or CPU scheduler objects accordingly.
  - `reset_cumac_task()` zeros timing/flags and optionally clears GPU buffers via `cudaMemset` when not in CPU mode.

In `cumac_cp_handler::initiate_cumac_task()`:
- Allocates pinned host memory and GPU buffers.
- Configures `run_in_cpu` based on config (forcing CPU, forcing GPU, or alternating).
- Optionally creates a CUDA stream per task for multi-stream scheduling.
- Hooks `callback_fun` to `cumac_task_callback`, which sends `SCH_TTI.response` for each cell.
 - Pre-allocates `group_buf` and aligns sections to 16 bytes (`CUMAC_GPU_ALIGN_BYTES`), or uses per-buffer `cudaMalloc` when group buffer is disabled.

### 5.3 Data path CPU vs GPU

In `cumac_cp_handler::on_sch_tti_request()`:

- Validates that per-cell parameters are consistent with global `group_params`.
- Copies per-cell inputs from NVIPC buffers into task buffers:
  - CPU mode: memcpy into CPU buffers.
  - GPU mode (no group buffer): `cudaMemcpyAsync` into GPU buffers.
  - For `estH_fr`, uses a carefully indexed 6D layout to maintain dense multi-cell tensors while preserving cell indexing.
- `cumac_task::run()` launches the scheduling stages (GPU or CPU) and uses CUDA events to time them.
- After run completion, `cumac_task_callback` dispatches TTI responses back via NVIPC.
 - Buffer bounds checking uses `CHECK_VALUE_MAX_ERR` across all fields to guard against overruns.
 - HARQ/MCS/precision toggles (`enableHarq`, `halfPrecision`, `baselineUlMcsInd`, `columnMajor`) set per-task to reduce cross-slot coupling.

**Parallelism**:
- Multiple `cumac_task` objects (slots) may be active across worker threads.
- Each task can run on its own CUDA stream.
- Within a task, kernels parallelize across UE and PRG dimensions.

---

## 6. RU Emulator and Fronthaul Threads

In `cuPHY-CP/ru-emulator`:

- RU emulator runs on top of the Aerial FH driver:
  - One `Peer` per cell in `HYBRID` mode (C-plane by peer, U-plane by flow).
  - TXQs shared across cells per symbol (`share_txqs = true`).
- RX queues per cell:
  - Non-SRS C-plane, SRS C-plane, and U-plane per DL eAxC.
  - Flow rules isolate traffic by MAC/VLAN/eCPRI/eAxC.
- Dedicated CPU threads:
  - Separate cores handle C-plane (SRS and non-SRS) and U-plane RX.
  - Threads map flows to slot/cell structures and feed DU/PHY driver.
 - TXQ sharing assumes aligned symbol timelines across cells; enforced by enqueue order in RU emulator core loop.

This allows high-throughput fronthaul with clear separation of concerns and deterministic per-cell processing.

---

## 7. End-to-End Summary and Tuning Knobs

- **Determinism**:
  - TTI tick thread drives SLOT.indication on a fixed core with SCHED_FIFO.
  - PHY and MAC worker threads are pinned and prioritized.
  - CUDA events and PTI instrumentation provide fine-grained timing.

- **Parallelism**:
  - Inter-slot: multiple slot maps and MAC tasks in flight.
  - Intra-slot: PDSCH and other channels across cells/UEs/PRBs on the GPU.
  - Multi-task: separate tasks for C-plane, PDSCH, compression, U-plane prepare, FH callbacks.

- **Scalability knobs**:
  - Number and placement of DL/UL workers.
  - Group-buffer enablement and size in cuMAC.
  - Batched memcpy vs individual copies.
  - Tick generator mode and jitter thresholds.

- **Robustness**:
  - Atomics and timeouts around wait loops.
  - NVLOG-based diagnostics and PTI stats.
  - Extensive configuration and parameter checks in cuMAC-CP and cuPHY-CP.
  - Signal handlers in cuMAC-CP (`signal_handler.cpp`) log backtraces on faults, aiding crash triage.

This architecture enables low-latency, high-throughput 5G processing by combining GPU data parallelism with carefully scheduled, affinity-pinned CPU threads and explicit synchronization between pipeline stages.
