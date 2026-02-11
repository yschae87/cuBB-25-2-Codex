# PDSCH End-to-End Processing Chain (FAPI → FH) (v1.1)

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

This document traces the downlink PDSCH path from FAPI ingress to outbound fronthaul packets, with code anchors and parallelism notes.

## 1) FAPI Ingress & Slot Construction
- **Tick trigger**: `cuphyl2adapter/lib/nvPHY/nv_tick_generator.cpp` emits `SLOT.indication` (timer thread with SCHED_FIFO + CPU affinity).
- **FAPI decode**: L2 adapter (`cuphyl2adapter/lib/nvPHY/nv_phy_module.hpp`) parses DL TTI (SCF/FAPI) messages into `slot_command` objects (`cuPHY-CP/gt_common_libs/slot_command/inc.../slot_command.hpp`). PDSCH PDUs populate `pdsch_params` and CSI-RS/DMRS info.
- **Slot map creation**: `cuphydriver` builds a `SlotMapDl` (`cuPHY-CP/cuphydriver/src/downlink/slot_map_dl.cpp`) per slot, embedding all channel params (PDSCH, PDCCH, PBCH, CSI-RS) and allocating DL buffers (`DLOutputBuffer`). Atomics/counters gate per-task completion (`atom_dl_cplane_start`, `atom_uplane_prep_done`, `atom_dl_gpu_comm_end`, etc.).
  - `SlotMapDl::reserve()` marks active; `getTaskTsEnq()` captures enqueue time. Per-cell DL buffers are pulled from pool and tied to CUDA stream(s).

## 2) Task Dispatch & Threading
- **Workers**: DL worker threads (`task_function_dl_aggr.cpp`) are pinned to configured cores, awakened by slot enqueue. Tasks are split by function: C-plane, PDSCH, compression/U-plane, FH callbacks, doorbells.
- **Streams & events**: Each channel gets a CUDA stream; CUDA events fence between stages. PTI (`cuphy_pti.hpp`) can record PREPREP/PREP/TRIGGER intervals at device scope.
  - `task_work_function_cplane/dl_aggr_bfw/dl_aggr_2/...` are scheduled with `Worker` objects; task arguments include first_cell/num_cells to shard work across cells.

## 3) PDSCH Setup (Host)
- **Aggregator**: `PhyPdschAggr` (`cuPHY-CP/cuphydriver/src/downlink/phypdsch_aggr.cpp`) wraps cuPHY APIs.
  - Static params: cells, antennas, numerology, PRBs/BWP, `enableBatchedMemcpy`, stream priority.
  - Dynamic params: per-slot `pCellGrpDynPrm`, TB buffers (`DataIn.pTbInput[]`), output tensors (`DataOut.pTDataTx[]` mapped to `DLOutputBuffer`), procMode bitmask (`PDSCH_PROC_MODE_GRAPHS`, setup-once fallback).
  - Optional pre-H2D TB copies are awaited via CUDA events if copy threads are enabled.
  - `cuphySetupPdschTx()` queues graph/stream work on `s_channel` and records setup events.
  - Cell→buffer mapping: for each cell in `aggr_cell_list`, find `cellPrmDynIdx` and bind `DataOut.pTDataTx[cellPrmDynIdx]` to the correct `DLOutputBuffer` descriptor + device pointer. `procModeBmsk` toggles graph vs stream launches.
  - Setup order: (1) bind inputs/outputs, (2) wait on pre-copy event if enabled, (3) `cudaEventRecord(start_setup)`, (4) `cuphySetupPdschTx`, (5) `cudaEventRecord(end_setup)`.

## 4) PDSCH GPU Execution (cuPHY)
- **DMRS generation** (`cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.cu`):
  - `gridDimY = num_TBs`, `gridDimX = re_blocks_x`; `blockDimX = threads` (power-of-two). Warps sweep contiguous REs, Y-slices spread TBs across SMs. No shared mem.
  - Descriptor copied H2D async; kernel params point at descriptor; PTX function stored in `m_kernelNodeParams.func`.
- **CSI-RS/PDSCH prep** (same file): three kernels
  - K1 zero/init: `blockDimX = block_size`, `gridDimX = ceil(total_offsets/block_size)`.
  - K2 remap: `blockDimX = 32*OFDM_SYMBOLS_PER_SLOT`, `gridDimX = num_ue_groups` (UE-group × symbol tiling).
  - K3 post-process: `blockDimX = num_threads`, `gridDimX = blocks` for remainder/remap.
- **CRC buffer prep** (`cuPHY/src/cuphy/crc/prepareCrcBuffersTest.cu`): per-TB `PdschPerTbParams` define padded offsets; H2D async copy of param array; kernels consume TB spans defined by `cumulativeTbSizePadding`.
- **Other PHY ops** (invoked via graph/launch config): rate-matching, modulation mapping, LDPC encode, DMRS insertion. Partitioning follows RE/PRB tiling similar to DMRS: X dimension for RE/bit spans, Y for TB/UE grouping, with blockDimX tuned for occupancy/coalescing.
  - Graph vs streams: if graphs enabled, kernels are pre-recorded and launched as a graph exec; otherwise, individual launches reuse the populated `m_kernelNodeParams`.
  - Occupancy knobs: `blockDimX` (`threads`), `re_blocks_x`, and TB count jointly determine SM residency; zero shared memory in DMRS avoids SMEM limits.

### 4.a Modulation Mapper (QPSK/16/64/256-QAM) details
- Code: `cuPHY/src/cuphy/modulation_mapper/modulation_mapper.cu`
- Kernel `modulation_mapper(modulationDescr_t* p_desc)`:
  - Grid: `gridDimY = num_TBs`, `gridDimX = ceil(max_num_symbols / blockDimX)`; `blockDimX = min(256, max_num_symbols)`, `blockDimY/Z = 1`; no shared mem.
  - Warps traverse symbols contiguously (`tid` → symbol). Y-slices spread TBs across SMs. Coalesced loads/stores of `modulation_input` and output constellation (`__half2`).
  - Output placement: flat contiguous when `d_params == nullptr`; tensor remap when descriptor present (`output_index_calc` / `bit_index_calc`).
  - Descriptor copied async if `enable_desc_async_copy`; otherwise reused in-pipeline. Kernel function pointer cached via `cudaGetFuncBySymbol` on first use.
  - Bit extraction uses `WORDS_PER_THREAD = 3` (96 bits) to align warp-size (32) with 6-bit symbols (64-QAM) and avoid crossing batch boundaries; QAM traits encode normalization constants.

### 4.b LDPC encode (high level)
- Code: cuPHY error-correction modules (`cuPHY/src/cuphy/error_correction/*`). Invoked via PDSCH pipeline graph nodes (BG1/BG2, various Zc).
- Launching:
  - Grid X set to number of codewords (or pairs for split-x2 variants); blockDims generated per kernel flavor (SM80/SM90, reg-index vs split-index).
  - Descriptor modules (embedded cubins) loaded once; function pointers cached; workspace sized per decoder config (`get_workspace_size`).
- Parallelization:
  - Layered min-sum message passing (C2V/V2C) with register/shared-memory staging. Split-index variants process two codewords/block; reg-index variants one codeword/block.
  - Occupancy bounded by SMEM/register use; Zc/BG determine shared memory and register pressure rather than fixed blockDim tuning.

## 5) Channel Completion & Synchronization
- `task_work_function_cplane` fences on per-channel completion via CUDA events (`waitRunCompletionGPUEvent`) before allowing compression/U-plane tasks to proceed.
- PTI data can be emitted in `task_work_function_debug` to correlate GPU phase timing with CPU scheduling.
  - `waitRunCompletionGPUEvent` gates on cuPHY-generated completion events in the channel stream; errors/NVLOG on timeout or failure; completion triggers `addSlotChannelEnd()` atomics.

## 6) Compression & U-Plane Preparation
- **Compression task** (`task_work_function_dl_aggr_bfw`): waits on channel-completion events, `waitDlGpuCommEnd()`, then runs `compression_dlbuf->runCompression` on a stream. Per-NIC prepare events (`getPrePrepareStopEvt/getTxEndEvt`) are respected before doorbell triggers.
- **U-plane prepare** (`task_work_function_dl_aggr_2`): for each cell, `FhProxy::prepareUPlanePackets` schedules eAxC payload assembly, batching memcpy via `cuphyBatchedMemcpyHelper`. Start TX time derived from slot timing (`Cell::getTtiNsFromMu` minus T1a margin). Metrics are stamped into `slot_map->timings`.
- Optional early PDSCH callback fires after TB H2D copy completion to unblock L2 when data is ready.
  - Sequencing inside `dl_aggr_2`: (a) `waitDLCDone` if mMIMO, (b) compute `start_tx`, (c) per-cell prepare (fills TxMsgContainer, compression configs), (d) batched memcpy `reset()`/`flush()` around prepares, (e) optional early callback, (f) metrics update, (g) `addSlotEndTask()`.
  - Compression task marks `slot_map->timings.start_t_dl_compression_cuda/end_t_dl_compression_cuda` and sets ready flags for NIC consumption if GPU-comm is enabled.

## 7) FH Packetization & Doorbells
- **FH callbacks** (`task_work_function_dl_fh_cb`): invoke `dl_slot_callbacks.fh_prepare_callback_fn` per cell, set `atom_cell_fhcb_done`, then `atom_fhcb_done` to release any waiters.
- **Doorbell task** (`task_work_function_dl_aggr_2_ring_cpu_doorbell`): after `waitDlCompEnd()`, retrieves prepared TX buffers per NIC, waits on prepare events, and rings CPU-side doorbells to NIC/DPDK. Assumes aligned symbol order across cells (TXQs are shared per symbol).
- RU emulator path (if used) mirrors this via `aerial-fh-driver` with separate RXQs per C-plane/SRS/U-plane and shared TXQs per symbol.
  - Doorbell path also records TX-ready timing and enforces `waitThresh` (e.g., 500us) to avoid indefinite waits; on timeout it logs NVLOG errors.
  - RU emulator uses flow rules per RXQ (MAC/VLAN/eCPRI/eAxC) and pinned threads per flow type; TXQs shared per symbol to maintain chronological order.

## 8) Egress & Completion
- Once compression and doorbells are done, NIC/DPDK threads transmit Ethernet/eCPRI IQ packets. Slot-level atomics (`atom_dl_end_threads`, etc.) transition the `SlotMapDl` to inactive, releasing resources for reuse. Timing/metrics are now available for logging and profiling (NVLOG + PTI).

## Parallelism Summary (PDSCH)
- **CPU**: slot-level tasks run on pinned DL workers; separate tasks for C-plane, PDSCH, compression, U-plane, FH callbacks, doorbells. Tick thread drives cadence.
- **GPU**: X-dim tiles RE/bit spans; Y-dim tiles TBs/UE groups. BlockDimX chosen for coalesced loads/stores; minimal shared mem in DMRS/CSI-RS prep. Multiple streams allow overlap of prep, channel exec, compression, and H2D/D2H when enabled.
- **Overlap**: Asynchronous descriptor/TB copies, batched memcpy, and stream events allow concurrent slots when enabled, gated by `wait*` calls in task functions to maintain dependency ordering.
