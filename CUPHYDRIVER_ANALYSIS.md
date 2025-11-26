# cuphydriver Deep Dive

This report covers `cuPHY-CP/cuphydriver`: its role, per-slot orchestration (slot maps, tasks), channel aggregators, synchronization primitives, and key configuration knobs.

## Role & Scope
- Hosts the L1 runtime that executes downlink/uplink pipelines on the GPU via cuPHY.
- Manages slot-level state (`SlotMapDl/Ul`), GPU devices/streams, fronthaul buffers, and task scheduling on pinned CPU workers.
- Integrates with Aerial FH driver/RU emulator for IQ RX/TX and with OAM/metrics for observability.

## Key Structures and Flow

### Slot maps (downlink)
- File: `src/downlink/slot_map_dl.cpp`
- `SlotMapDl` holds:
  - Aggregated cells (`aggr_cell_list`) and DL buffers (`aggr_dlbuf_list`).
  - Channel aggregators: PDSCH/PDCCH/PBCH/CSI-RS, DL BFW, etc.
  - Atomics to gate progress: `atom_dl_cplane_start`, `atom_dlc_done`, `atom_uplane_prep_done`, `atom_dl_gpu_comm_end`, `atom_dl_comp_end`, per-cell `atom_cell_fhcb_done`, channel/task end counters.
  - Timing fields for prepare/compression/trigger and PTI traces.
  - Batched memcpy helper configured per slot (`cuphyBatchedMemcpyHelper`).
- Lifecycle: `reserve()` -> setup by task(s) -> wait/atomics for completion -> `addSlotEndTask()` marks teardown readiness.

### Task functions (downlink)
- File: `src/downlink/task_function_dl_aggr.cpp`
- Tasks executed by worker threads (pinned, often SCHED_FIFO):
  - **C-plane task**: setup/run PDCCH/PBCH/CSI-RS via per-channel aggregators; signals completion events.
  - **PDSCH task**: binds TB inputs/outputs then `cuphySetupPdschTx` + `cuphyRunPdschTx`.
  - **Compression task**: waits on channel completion events and `waitDlGpuCommEnd()`, runs compression kernels, sets ready flags for NIC/green contexts.
  - **U-plane prepare task**: calls `FhProxy::prepareUPlanePackets`, batches memcpy, computes TX start times, updates metrics, optional early PDSCH callback after TB copy.
  - **FH callback task**: invokes `fh_prepare_callback_fn` per cell, sets FHCB atomics.
  - **Doorbell task**: after `waitDlCompEnd()`, waits on prepare events, rings NIC doorbells (DPDK), assumes aligned symbol order for shared TXQs.
  - **Debug task**: emits PTI logs, checks prepare/compression events.
- Dependency fencing via CUDA events and slot-map atomics ensures ordering between tasks without coarse locks.

### Channel aggregators (downlink examples)
- Files: `src/downlink/phypdsch_aggr.cpp`, `phypdcch_aggr.cpp`, `phypbch_aggr.cpp`, `phycsirs_aggr.cpp`.
- Common pattern:
  - Build static descriptors (per cell) and dynamic descriptors (per slot) based on `slot_command` inputs.
  - Map `DataIn/Out` pointers to slot buffers (`DLOutputBuffer`) and set CUDA stream priorities.
  - Call cuPHY setup/run APIs; record CUDA events for setup/run timing; expose debug info (TB sizes, validation).
  - Respect graph vs stream mode based on config (`enable_dl_cuphy_graphs`), pdsch_fallback flags.

### GPU devices / streams
- File: `include/gpudevice.hpp`
  - Manages CUDA context, streams per task/channel, device properties; exposes helpers to set current context and synchronize streams.
  - Sets stream priorities and MPS SM reservations (from YAML: `mps_sm_*` per channel).

### Downlink buffers
- File: `include/dlbuffer.hpp`, `src/downlink/dlbuffer.cpp`
  - Owns device buffers for each cell’s DL grid; tracks descriptors, compression configs, TX message containers, CUDA events (prepare/compression start/stop).
  - Provides getters for events used by tasks (`getPrepareStopEvt`, `getTxEndEvt`, `getAllChannelsDoneEvt`).

### Uplink analogues
- Slot map: `src/uplink/slot_map_ul.cpp` with atomics for UL channel completion (`atom_ul_end_threads`, `atom_ul_channel_end_threads`).
- Tasks: `src/uplink/task_function_ul_*` for PUSCH, PRACH, SRS; use similar event/atomic gating and optional H2D prepone copy threads (`ul_pcap_capture_thread.cpp` for capture, `order_entity.cpp` for order kernels).

### GPU devices / streams / SM partitioning
- File: `include/gpudevice.hpp`, `src/common/context.cpp`:
  - Wraps CUDA init, sets device, creates streams per channel/task, optionally sets stream priority.
  - MPS SM counts per feature pulled from YAML (`mps_sm_*`): pusch/pucch/prach/srs/pdsch/pdcch/pbch/gpu_comms/ul_order; used to limit SM residency for specific streams.
  - Provides sync helpers (`synchronizeStream`) and context setters around kernel launches.

## Synchronization & Instrumentation
- **Atomics**: per-slot counters for channel/task completion; spin-waits with thresholds (`GENERIC_WAIT_THRESHOLD_NS`) log warnings on overruns.
- **CUDA events**: used extensively to fence across streams (channels→compression, prepare→doorbells).
- **PTI**: device-side timestamps (PREPREP/PREP/TRIGGER) collected and logged in debug tasks.
- **Semaphores/locks**: minimal on hot path; workers wake via semaphores; lock-free rings used upstream (NVIPC).

### Key wait thresholds / timeouts (DL examples)
- `waitDLCDone(int num_dlc_tasks)`: warns if `atom_dlc_done` lags for more than `GENERIC_WAIT_THRESHOLD_NS * 2`.
- `waitFHCBDone() / waitCellFHCBDone()`: same threshold; logs at INFO/WARN on slow waits.
- `waitDlGpuCommEnd() / waitDlCompEnd()`: polls with `GENERIC_WAIT_THRESHOLD_NS * 2`; errors if exceeded.
- Doorbell task: wait threshold around 500us before error logging (see `task_function_dl_aggr_2_ring_cpu_doorbell`).
- Debug/prepare waits: `non_blocking_event_wait_with_timeout` uses configurable thresholds (via `pdctx->getcuphy_dl_channel_wait_th()`).
- PDSCH setup errors (`CUPHY_PDSCH_STATUS_UNSUPPORTED_MAX_ER_PER_CB`) skip run; logged with SFN/slot and TB/cell indices.
- H2D prepone copy waits (when enabled) gate `cuphySetupPdschTx` on `event_pdsch_tb_cpy_complete(slot)`, error if timeout (threshold aligns with channel wait th).
- DL wait threshold (`dl_wait_th_ns`) can be set via YAML; used to warn if DL tasks exceed configured budget.

### CUDA events table (downlink hot path)
- **C-plane aggregators**: per-channel run completion events (e.g., `signalRunCompletionEvent`) consumed by `task_work_function_cplane`.
- **PDSCH aggregator**: setup/run events (`start_setup`, `end_setup`, `start_run`, `end_run` in `phypdsch_aggr.cpp`), plus H2D TB copy events when prepone copy enabled.
- **Compression/DL buffers**: `getPrepareStopEvt`, `getTxEndEvt`, `getAllChannelsDoneEvt` used by compression/doorbell tasks; compression kernel events used for timing (`start_t_dl_compression_cuda`/`end_t_dl_compression_cuda`).
- **Batched memcpy helper**: may produce events to sequence host→device copies across streams (used implicitly by helper APIs).
- **Debug task**: reads PTI buffers but also waits on compression/prepare events to extract durations.

## Configuration Influences (from cuphycontroller YAML)
- Graph mode (`enable_dl_cuphy_graphs`, `pdsch_fallback`) impacts aggregator launch mode.
- SM reservations (`mps_sm_pdsch`, `mps_sm_pdcch`, etc.) inform stream priority/SM partitioning in GPU device setup.
- Batched memcpy toggles per slot; green contexts and prepone H2D copy threads influence copy/compute overlap.
- DL wait thresholds, sendCPlane backoffs, and compression/trigger waits control timeout/error logging.
- UL order timeouts: `ul_order_timeout_cpu_ns`, `ul_order_timeout_gpu_ns`, `ul_order_timeout_gpu_srs_ns`, `ul_order_timeout_log_interval_ns` tune RX/order pipelines (logged on breach).
- SM reservations defaults (if missing): e.g., `mps_sm_ul_order` defaults to 16, `mps_sm_gpu_comms` defaults to 8 (warn logged); H2D copy thread disabled by default; `use_batched_memcpy`/`use_green_contexts` default to 0 with warnings.

## Observability / Metrics
- NVLOG tags per module (`NVLOG_TAG_BASE_CUPHY_DRIVER + N`).
- Timing fields in slot maps and tasks; PTI stats; CQE tracing (if enabled).
- Optional PMU metrics and datalake logging controlled via YAML.

## Typical DL Sequence (per slot, with code anchors)
1) **Slot map reserved** (`SlotMapDl::reserve`), cells/buffers assigned.
2) **C-plane task** (`task_work_function_cplane`): per-channel `setup/run`, signal completion events (`signalRunCompletionEvent`); `addSlotChannelEnd`.
3) **PDSCH task** (`PhyPdschAggr::setup/run` in `phypdsch_aggr.cpp`): bind TB inputs/outputs, call cuPHY API; optional pre-H2D waits if copy threads enabled; record CUDA events.
4) **Compression task** (`task_work_function_dl_aggr_bfw`): wait on channel completion events + `waitDlGpuCommEnd`; run `compression_dlbuf->runCompression`; set ready flags for NIC/green contexts.
5) **U-plane prepare** (`task_work_function_dl_aggr_2`): `FhProxy::prepareUPlanePackets`, batched memcpy helper reset/flush, compute TX start, optional early PDSCH callback after TB H2D completion, metrics.
6) **FH callbacks** (`task_work_function_dl_fh_cb`): invoke `fh_prepare_callback_fn`, set `atom_cell_fhcb_done`, `atom_fhcb_done`.
7) **Doorbell** (`task_work_function_dl_aggr_2_ring_cpu_doorbell`): wait `waitDlCompEnd`, wait prepare events, ring NIC; apply wait thresholds.
8) **Slot end**: atomics reach expected counts, slot map released for reuse; timing/metrics logged.

## Typical UL Sequence (high level)
1) Slot map reserved (`SlotMapUl::reserve`), input buffers assigned.
2) RX / order tasks ingest IQ (DPDK/FH) and push into GPU via prepone copy if enabled.
3) UL channel tasks (PUSCH/PUCCH/SRS/PRACH) setup/run, fenced by CUDA events; atomics `atom_ul_channel_end_threads` increment.
4) Order/harq pools managed in `order_entity.cpp`/`harq_pool.cpp`; warnings about lack of locks noted—callers must avoid concurrent mutation.
5) Completion waits (`waitUlEndThreads`, etc.) gate slot teardown and callbacks.

## Error Handling / Timeouts
- Wait loops check thresholds and log `AERIAL_CUPHYDRV_API_EVENT` on overruns (e.g., `waitDLCDone`, `waitFHCBDone`, `waitDlGpuCommEnd`).
- PDSCH setup logs specific errors (e.g., `CUPHY_PDSCH_STATUS_UNSUPPORTED_MAX_ER_PER_CB`) and skips run to avoid misconfig crashes.
- Doorbell waits/timeouts emit NVLOG errors; UL order timeouts configurable via YAML and logged.

## File Anchors
- Slot maps: `src/downlink/slot_map_dl.cpp`, `src/uplink/slot_map_ul.cpp`
- Tasks: `src/downlink/task_function_dl_aggr.cpp`, `src/uplink/task_function_ul_*`
- Aggregators: `src/downlink/phy*.cpp`, `include/phy*.hpp` (PDSCH/PDCCH/PBCH/CSI-RS); `src/uplink/phy*.cpp` for PUSCH/SRS/PUCCH/PRACH
- GPU device/constant: `include/gpudevice.hpp`, `include/constant.hpp`
- Buffers: `include/dlbuffer.hpp`, `src/downlink/dlbuffer.cpp`
- Compression/doorbells: `task_function_dl_aggr.cpp` (prepare/compress/doorbell paths)
- UL capture/order: `src/uplink/ul_pcap_capture_thread.cpp`, `src/uplink/order_entity.cpp`

## Channel-specific notes (PDSCH, PUSCH, SRS)
- **PDSCH (DL)**: `phypdsch_aggr.cpp` builds static/dynamic descriptors, binds TB inputs/outputs to `DLOutputBuffer`, honors `pdsch_fallback` and graph mode. Waits on H2D TB copy events if prepone enabled; errors on `CUPHY_PDSCH_STATUS_UNSUPPORTED_MAX_ER_PER_CB`. Streams/SM counts influenced by `mps_sm_pdsch` (YAML).
- **PUSCH (UL)**: `src/uplink/phypusch_aggr.cpp` (not shown here) mirrors DL pattern for UL: aggregates per-cell PUSCH params, sets up cuPHY PUSCH pipeline (LLR, demod, LDPC decode), fences via CUDA events. Timing/ordering controlled by UL slot map atomics (`atom_ul_channel_end_threads`), optional prepone copy threads, and UL order timeouts (`ul_order_timeout_*`). SM reservations via `mps_sm_pusch`.
- **SRS (UL)**: `physrs_aggr.hpp/cpp` handles SRS channel estimation: configures per-cell SRS parameters, launches SRS kernels, and signals completion into UL slot map. Affects downstream PUSCH equalization if enabled. SM reservations via `mps_sm_srs`; optional SRS-specific timeouts (`ul_srs_aggr3_task_launch_offset_ns`, `ul_order_timeout_gpu_srs_ns`).

## Example config pulls (cuphycontroller_F08.yaml)
- SM reservations:
  - `mps_sm_pusch=108`, `mps_sm_pucch=16`, `mps_sm_prach=2`, `mps_sm_srs=16`, `mps_sm_pdsch=82`, `mps_sm_pdcch=36`, `mps_sm_pbch=36`; `mps_sm_gpu_comms`/`mps_sm_ul_order` not specified → defaults apply (`8` and `16`, logged as warnings).
- Graph/fallback:
  - `enable_dl_cuphy_graphs=1`, `enable_ul_cuphy_graphs=1`, `pdsch_fallback=0`.
- UL order timeouts:
  - `ul_order_timeout_cpu_ns=8_000_000`, `ul_order_timeout_gpu_ns=2_300_000`; `ul_order_timeout_gpu_srs_ns` absent → default 5_200_000 ns; `ul_srs_aggr3_task_launch_offset_ns` absent → default 500_000 ns.
- Copy/overlap features:
  - `use_batched_memcpy`, `use_green_contexts`, `enable_h2d_copy_thread`, `dl_wait_th_ns`, `sendCPlane_*_backoff_ns` not specified → defaults (0/disabled, with NVLOG warnings for missing keys). H2D copy thread affinity/priority defaults to CPU 29 and priority 0 if unspecified.
