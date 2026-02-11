# CuPHYDriver: Core PHY Processing Engine Roles and Responsibilities (v1.1)

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

## Executive Summary

The **cuPHYDriver** is the core L1 (Layer 1) PHY processing engine in the NVIDIA Aerial cuBB stack. Unlike cuPHYController which handles configuration and initialization, cuPHYDriver is the actual execution engine that manages GPU/CPU coordination, cell processing, task scheduling, and interfaces with the cuPHY CUDA library for signal processing.

## Table of Contents

1. [Component Overview](#1-component-overview)
2. [Primary Roles](#2-primary-roles)
3. [Architecture Components](#3-architecture-components)
4. [L1 API Implementation](#4-l1-api-implementation)
5. [Slot Command Processing](#5-slot-command-processing)
6. [cuPHY Library Integration](#6-cuphy-library-integration)
7. [Resource Management](#7-resource-management)
8. [Data Flow Pipeline](#8-data-flow-pipeline)
9. [State Management](#9-state-management)
10. [Error Handling and Recovery](#10-error-handling-and-recovery)
11. [Performance Optimization](#11-performance-optimization)
12. [Key Differences: cuPHYDriver vs cuPHYController](#12-key-differences-cuphydriver-vs-cuphycontroller)

---

## 1. Component Overview

### Location
```
/cuPHY-CP/cuphydriver/
├── include/
│   └── cuphydriver_api.hpp         # Main API definitions
├── src/
│   ├── common/
│   │   └── cuphydriver_api.cpp     # Core implementation
│   ├── context/                    # PhyDriverCtx class
│   ├── cell/                       # Cell management
│   ├── worker/                     # Worker thread management
│   └── task/                       # Task scheduling
```

### Primary Purpose
The cuPHYDriver serves as the **execution engine** that:
- Manages all L1 PHY processing operations
- Coordinates CPU and GPU resources
- Implements task scheduling and distribution
- Interfaces with cuPHY CUDA kernels
- Handles fronthaul I/O operations
- Provides callbacks to L2/MAC layer

### Key Characteristics
- **High-performance**: Optimized for sub-millisecond latency
- **Multi-cell capable**: Supports multiple cells per context
- **GPU-accelerated**: Direct CUDA/MPS integration
- **Real-time**: Priority-based task scheduling
- **Fault-tolerant**: Timeout detection and recovery

---

## 2. Primary Roles

### 2.1 L1 PHY Processing Engine
```cpp
// Core processing through cuPHY library
PhyPuschAggr::execute()  // PUSCH decoding
PhyPdschAggr::execute()  // PDSCH encoding
PhyPrachAggr::execute()  // PRACH detection
PhyPucchAggr::execute()  // PUCCH decoding
```

**Responsibilities**:
- Execute PHY layer signal processing
- Manage cuPHY kernel invocations
- Handle channel coding/decoding
- Perform modulation/demodulation
- Execute FFT/IFFT operations

### 2.2 Cell Lifecycle Manager
```cpp
// Complete cell management
l1_cell_create(cell_config)    // Create cell with PHY objects
l1_cell_start(cell_id)          // Activate cell processing
l1_cell_stop(cell_id)           // Deactivate cell
l1_cell_destroy(cell_id)        // Clean up resources
l1_cell_update_cell_config()    // Runtime updates
```

**Responsibilities**:
- Create cell instances with cuPHY objects
- Manage cell state transitions
- Allocate per-cell GPU streams
- Handle cell-specific buffers
- Track cell health status

### 2.3 Task Scheduler and Dispatcher
```cpp
class task_priority_queue {
    bool anyTasksReady(worker_id wid, t_ns time_threshold);
    Task* popReadyTask(worker_id wid, t_ns current_time);
};
```

**Responsibilities**:
- Maintain priority queues for tasks
- Distribute work to available workers
- Enforce timing constraints
- Balance load across workers
- Handle task dependencies

### 2.4 GPU Resource Coordinator
```cpp
class GpuDevice {
    cudaDevice_t cuda_device;
    cudaContext_t cuda_context;
    MpsCtx* ulMpsCtx;      // UL processing context
    MpsCtx* dlMpsCtx;      // DL processing context
    MpsCtx* puschMpsCtx;   // PUSCH specific
    MpsCtx* pdschMpsCtx;   // PDSCH specific
};
```

**Responsibilities**:
- Manage CUDA contexts and devices
- Configure MPS (Multi-Process Service) partitions
- Allocate GPU streams per cell/channel
- Handle GPU memory allocation
- Coordinate kernel launches

### 2.5 Fronthaul I/O Manager
```cpp
// Interface with FH proxy
cell->peer = fh_proxy->addPeerFromMacAddr(mac_src, mac_dst);
fh_proxy->send(cell->peer, dl_buffer);
```

**Responsibilities**:
- Register cells with FH proxy
- Manage IQ sample transmission/reception
- Handle packet ordering (order kernel)
- Coordinate with DPDK/NIC
- Manage compression/decompression

---

## 3. Architecture Components

### 3.1 Core Classes

#### PhyDriverCtx (Main Context)
```cpp
class PhyDriverCtx {
    // Resource pools
    std::array<Task*, TASK_ITEM_NUM> task_item_array;
    std::array<SlotMapUl*, SLOT_MAP_NUM> slot_map_ul_array;
    std::array<SlotMapDl*, SLOT_MAP_NUM> slot_map_dl_array;

    // Worker management
    std::unordered_map<worker_id, Worker*> worker_ul_map;
    std::unordered_map<worker_id, Worker*> worker_dl_map;

    // Cell management
    std::unordered_map<uint16_t, Cell*> cells;

    // GPU management
    std::vector<GpuDevice*> gpu_devices;

    // Aggregation objects
    std::vector<PhyPuschAggr*> aggr_pusch_items;
    std::vector<PhyPdschAggr*> aggr_pdsch_items;
};
```

#### Cell Class
```cpp
class Cell {
    // Cell configuration
    uint16_t cell_id;
    cell_phy_info phy_info;
    cell_mplane_info mplane_info;

    // State management
    std::atomic<cell_status> active;

    // GPU resources
    cudaStream_t stream_ul;
    cudaStream_t stream_dl;
    cudaStream_t stream_order;

    // Buffer pools
    std::vector<DLOutputBuffer*> dlbuf_list;
    std::vector<ULInputBuffer*> ulbuf_st1_list;

    // cuPHY objects
    cuphyCellStatPrm_t cell_static_params;
    PhyPuschAggr* pusch_aggr;
    PhyPdschAggr* pdsch_aggr;
};
```

#### Worker Class
```cpp
class Worker {
    worker_id id;
    std::thread thread;
    uint8_t cpu_affinity;
    uint32_t sched_priority;
    std::atomic<bool> exit_flag;

    void run() {
        while (!exit_flag) {
            Task* task = getNextTask();
            if (task) task->execute();
        }
    }
};
```

#### Task Class
```cpp
class Task {
    uint64_t execution_timestamp;
    TaskType type;  // PUSCH, PDSCH, PRACH, etc.
    Cell* cell;
    void* work_data;

    virtual void execute() = 0;
};
```

### 3.2 Memory Management Structures

#### Buffer Types
```cpp
// Uplink input buffers (from RU)
struct ULInputBuffer {
    void* data;          // IQ samples
    size_t size;
    uint32_t slot_num;
    BufferStage stage;   // ST1, ST2, ST3
};

// Downlink output buffers (to RU)
struct DLOutputBuffer {
    void* data;          // Generated IQ samples
    size_t size;
    uint32_t slot_num;
};

// GPU pinned memory
struct GpuPinnedBuffer {
    void* host_ptr;      // CPU accessible
    void* device_ptr;    // GPU accessible
    size_t size;
};
```

---

## 4. L1 API Implementation

### 4.1 Initialization and Finalization

```cpp
int l1_init(phydriver_handle* pdh, const context_config& ctx_cfg)
{
    // 1. Create PhyDriverCtx instance
    PhyDriverCtx* pdctx = new PhyDriverCtx(ctx_cfg);

    // 2. Initialize GPU devices
    pdctx->initGpuDevices();

    // 3. Create cells from M-plane config
    for (auto& mplane : ctx_cfg.mplane_configs) {
        pdctx->addNewCell(mplane);
    }

    // 4. Start FH proxy
    pdctx->startFhProxy();

    // 5. Return handle
    *pdh = pdctx;
    return 0;
}

int l1_finalize(phydriver_handle pdh)
{
    PhyDriverCtx* pdctx = (PhyDriverCtx*)pdh;

    // 1. Stop all workers
    pdctx->stopAllWorkers();

    // 2. Destroy all cells
    pdctx->destroyAllCells();

    // 3. Clean up GPU resources
    pdctx->cleanupGpu();

    // 4. Delete context
    delete pdctx;
    return 0;
}
```

### 4.2 Cell Management API

```cpp
int l1_cell_create(phydriver_handle pdh, cell_phy_info& info)
{
    PhyDriverCtx* pdctx = (PhyDriverCtx*)pdh;
    Cell* cell = new Cell(info);

    // Allocate PHY objects
    cell->allocateCuphyObjects();

    // Create GPU streams
    cudaStreamCreate(&cell->stream_ul);
    cudaStreamCreate(&cell->stream_dl);
    cudaStreamCreate(&cell->stream_order);

    // Allocate buffers
    cell->allocateBuffers();

    // Register with FH proxy
    cell->registerWithFhProxy(pdctx->fh_proxy);

    pdctx->cells[info.cell_id] = cell;
    return 0;
}

int l1_cell_start(phydriver_handle pdh, uint16_t cell_id)
{
    Cell* cell = pdctx->cells[cell_id];
    cell->active = CELL_ACTIVE;
    return 0;
}
```

### 4.3 Worker Management API

```cpp
int l1_worker_start_generic(
    phydriver_handle pdh,
    phydriverwrk_handle* wh,
    const char* name,
    uint8_t affinity_core,
    uint32_t sched_priority,
    worker_routine wr,
    void* args)
{
    PhyDriverCtx* pdctx = (PhyDriverCtx*)pdh;

    Worker* worker = new Worker();
    worker->name = name;
    worker->cpu_affinity = affinity_core;
    worker->sched_priority = sched_priority;
    worker->routine = wr;
    worker->args = args;

    // Start thread with affinity and priority
    worker->start();

    // Register worker
    pdctx->worker_generic_map[worker->id] = worker;

    *wh = worker;
    return 0;
}
```

---

## 5. Slot Command Processing

### 5.1 Work Enqueuing Pipeline

```cpp
int l1_enqueue_phy_work(
    phydriver_handle pdh,
    slot_command_api::slot_command* sc)
{
    PhyDriverCtx* pdctx = (PhyDriverCtx*)pdh;

    // Stage 1: Parse slot command
    SlotParamsAggr* params = pdctx->getNextSlotCmd();
    params->populate(sc);

    // Stage 2: Split by direction (UL/DL)
    std::vector<Cell*> ul_cells, dl_cells;
    splitCellsByDirection(sc, ul_cells, dl_cells);

    // Stage 3: Allocate buffers
    for (Cell* cell : ul_cells) {
        ULInputBuffer* buf = cell->getNextUlBuffer();
        // Configure buffer for slot
    }

    // Stage 4: Create tasks
    for (auto& channel : sc->channels) {
        Task* task = createTask(channel);
        pdctx->task_queue.push(task);
    }

    // Stage 5: Get aggregation objects
    if (has_pusch) {
        PhyPuschAggr* aggr = pdctx->getNextPuschAggr(params);
        aggr->configure(sc->pusch_params);
    }

    return 0;
}
```

### 5.2 Slot Command Structure

```cpp
struct slot_command {
    // Timing information
    uint16_t sfn;           // System Frame Number
    uint8_t slot;           // Slot within frame

    // Cell groups
    cell_group_t cell_groups;

    // Channel information
    struct {
        bool has_pdsch;
        bool has_pdcch;
        bool has_pusch;
        bool has_pucch;
        bool has_prach;
        bool has_srs;
    } channels;

    // Channel-specific parameters
    pdsch_params_t pdsch;
    pusch_params_t pusch;
    prach_params_t prach;
};
```

### 5.3 Task Priority Management

```cpp
class TaskPriorityQueue {
    // Tasks ordered by execution timestamp
    std::priority_queue<Task*, std::vector<Task*>, TaskComparator> queue;

    struct TaskComparator {
        bool operator()(Task* a, Task* b) {
            return a->execution_timestamp > b->execution_timestamp;
        }
    };

    Task* popReadyTask(uint64_t current_time) {
        if (!queue.empty() &&
            queue.top()->execution_timestamp <= current_time) {
            Task* task = queue.top();
            queue.pop();
            return task;
        }
        return nullptr;
    }
};
```

---

## 6. cuPHY Library Integration

### 6.1 cuPHY Object Management

Each cell maintains cuPHY library objects:

```cpp
struct CellCuphyObjects {
    // Static cell parameters
    cuphyCellStatPrm_t cell_static_params;

    // Channel-specific configurations
    cuphyPuschCellStatPrms_t pusch_params;
    cuphyPucchCellStatPrms_t pucch_params;
    cuphyPrachCellStatPrms_t prach_params;
    cuphyPdschCellStatPrms_t pdsch_params;
    cuphyPdcchCellStatPrms_t pdcch_params;

    // Aggregator objects (execute kernels)
    PhyPuschAggr* pusch_aggr;
    PhyPucchAggr* pucch_aggr;
    PhyPrachAggr* prach_aggr;
    PhyPdschAggr* pdsch_aggr;
    PhyPdcchAggr* pdcch_aggr;
};
```

### 6.2 Aggregator Execution

```cpp
// PUSCH Processing
class PhyPuschAggr {
    void execute(cudaStream_t stream) {
        // Stage 1: Channel estimation
        cuphy_pusch_chest_kernel<<<blocks, threads, 0, stream>>>();

        // Stage 2: Equalization
        cuphy_pusch_equalize_kernel<<<blocks, threads, 0, stream>>>();

        // Stage 3: Demodulation
        cuphy_pusch_demod_kernel<<<blocks, threads, 0, stream>>>();

        // Stage 4: LDPC decoding
        cuphy_ldpc_decode_kernel<<<blocks, threads, 0, stream>>>();
    }
};

// PRACH Detection
class PhyPrachAggr {
    void execute(cudaStream_t stream) {
        // Stage 1: FFT
        cufftExecC2C(fft_plan, input, output, CUFFT_FORWARD);

        // Stage 2: Correlation with Zadoff-Chu sequences
        cuphy_prach_correlate_kernel<<<blocks, threads, 0, stream>>>();

        // Stage 3: Peak detection
        cuphy_prach_detect_kernel<<<blocks, threads, 0, stream>>>();

        // Stage 4: Timing advance calculation
        cuphy_prach_ta_kernel<<<blocks, threads, 0, stream>>>();
    }
};
```

### 6.3 GPU Stream Management

```cpp
// Per-cell streams for isolation
struct CellStreams {
    cudaStream_t stream_ul;       // Uplink processing
    cudaStream_t stream_dl;       // Downlink processing
    cudaStream_t stream_order;    // Packet ordering
};

// Global context streams
struct ContextStreams {
    cudaStream_t stream_order_pd;         // Primary order kernel
    cudaStream_t stream_order_srs_pd;     // SRS order kernel
    cudaStream_t aggr_stream_pusch[4];    // Split PUSCH streams
    cudaStream_t aggr_stream_pdsch;       // PDSCH aggregation
    cudaStream_t H2D_TB_CPY_stream;       // Host-to-device copy
};
```

### 6.4 Memory Transfer Optimization

```cpp
// Batched memory copy for efficiency
class BatchedMemcpyHelper {
    std::vector<cudaMemcpyAsync> pending_copies;

    void addCopy(void* dst, void* src, size_t size, cudaStream_t stream) {
        pending_copies.push_back({dst, src, size, stream});
    }

    void executeBatch() {
        for (auto& copy : pending_copies) {
            cudaMemcpyAsync(copy.dst, copy.src, copy.size,
                           cudaMemcpyHostToDevice, copy.stream);
        }
        pending_copies.clear();
    }
};
```

---

## 7. Resource Management

### 7.1 GPU Resource Allocation

#### MPS Context Management
```cpp
class MpsContext {
    int gpu_id;
    int sm_count;           // Number of SMs reserved
    cudaContext_t context;

    void configure(int num_sms) {
        // Reserve streaming multiprocessors
        cudaDeviceSetLimit(cudaLimitDevRuntimeSyncDepth, num_sms);
    }
};

// SM allocation per workload
MpsAllocation {
    PUSCH: 100 SMs
    PUCCH: 2 SMs
    PRACH: 2 SMs
    SRS: 16 SMs
    PDSCH: 102 SMs
    PDCCH: 10 SMs
    GPU_COMMS: 16 SMs
}
```

#### GPU Memory Pools
```cpp
class GpuMemoryPool {
    std::vector<void*> free_blocks;
    std::vector<void*> used_blocks;
    size_t block_size;

    void* allocate() {
        if (free_blocks.empty()) {
            void* ptr;
            cudaMalloc(&ptr, block_size);
            return ptr;
        }
        void* ptr = free_blocks.back();
        free_blocks.pop_back();
        used_blocks.push_back(ptr);
        return ptr;
    }

    void deallocate(void* ptr) {
        // Move from used to free
        auto it = std::find(used_blocks.begin(), used_blocks.end(), ptr);
        if (it != used_blocks.end()) {
            used_blocks.erase(it);
            free_blocks.push_back(ptr);
        }
    }
};
```

### 7.2 CPU Thread Management

```cpp
class WorkerThreadManager {
    // Worker pools by type
    std::vector<Worker*> ul_workers;    // Uplink workers
    std::vector<Worker*> dl_workers;    // Downlink workers
    std::vector<Worker*> generic_workers; // Generic workers

    void assignWork(Task* task) {
        // Round-robin or least-loaded assignment
        if (task->isUplink()) {
            Worker* worker = getLeastLoadedWorker(ul_workers);
            worker->enqueue(task);
        } else if (task->isDownlink()) {
            Worker* worker = getLeastLoadedWorker(dl_workers);
            worker->enqueue(task);
        }
    }

    Worker* getLeastLoadedWorker(std::vector<Worker*>& pool) {
        return *std::min_element(pool.begin(), pool.end(),
            [](Worker* a, Worker* b) {
                return a->getQueueSize() < b->getQueueSize();
            });
    }
};
```

### 7.3 Buffer Management

```cpp
// Pre-allocated circular buffer pools
template<typename T>
class CircularBufferPool {
    std::vector<T> buffers;
    std::atomic<uint32_t> index;

    CircularBufferPool(size_t count) : buffers(count), index(0) {}

    T& getNext() {
        uint32_t idx = index.fetch_add(1) % buffers.size();
        return buffers[idx];
    }
};

// Buffer pools per cell
struct CellBuffers {
    CircularBufferPool<DLOutputBuffer> dl_buffers{16};
    CircularBufferPool<ULInputBuffer> ul_st1_buffers{16};
    CircularBufferPool<ULInputBuffer> ul_st2_buffers{16};
    CircularBufferPool<ULInputBuffer> ul_st3_buffers{16};
};
```

### 7.4 Memory Footprint Tracking

```cpp
struct MemoryFootprint {
    // CPU memory tracking
    std::atomic<size_t> cpu_regular_memory{0};
    std::atomic<size_t> cpu_pinned_memory{0};

    // GPU memory tracking
    std::atomic<size_t> gpu_regular_memory{0};
    std::atomic<size_t> gpu_pinned_memory{0};

    void allocateCpuRegular(size_t size) {
        cpu_regular_memory += size;
    }

    void allocateGpuPinned(size_t size) {
        gpu_pinned_memory += size;
    }

    void printSummary() {
        LOG(INFO) << "Memory Usage:"
                  << " CPU Regular: " << cpu_regular_memory
                  << " CPU Pinned: " << cpu_pinned_memory
                  << " GPU Regular: " << gpu_regular_memory
                  << " GPU Pinned: " << gpu_pinned_memory;
    }
};
```

---

## 8. Data Flow Pipeline

### 8.1 Uplink Processing Pipeline

```
┌──────────────────────────────────────────────────────────────┐
│                     Uplink Data Flow                          │
├──────────────────────────────────────────────────────────────┤
│                                                                │
│  RU/O-RAN Network                                             │
│       ↓                                                       │
│  FH Driver (DPDK)                                             │
│       ↓                                                       │
│  Order Kernel (Packet Reordering)                             │
│       ↓                                                       │
│  ULInputBuffer (Stage 1/2/3)                                  │
│       ↓                                                       │
│  ┌────────────────────────────────────┐                      │
│  │     cuPHY Kernel Processing        │                      │
│  ├────────────────────────────────────┤                      │
│  │ PUSCH: Chest → Equalize → Demod →  │                      │
│  │        LDPC Decode → CRC Check     │                      │
│  ├────────────────────────────────────┤                      │
│  │ PUCCH: Chest → Equalize → Demod →  │                      │
│  │        UCI Extract                 │                      │
│  ├────────────────────────────────────┤                      │
│  │ PRACH: FFT → Correlate → Detect →  │                      │
│  │        TA Calculate                │                      │
│  └────────────────────────────────────┘                      │
│       ↓                                                       │
│  Results Aggregation                                          │
│       ↓                                                       │
│  L2 Callback (UL.indication)                                  │
│                                                                │
└──────────────────────────────────────────────────────────────┘
```

### 8.2 Downlink Processing Pipeline

```
┌──────────────────────────────────────────────────────────────┐
│                    Downlink Data Flow                         │
├──────────────────────────────────────────────────────────────┤
│                                                                │
│  L2/MAC (Transport Blocks)                                    │
│       ↓                                                       │
│  H2D Memory Copy (to GPU)                                     │
│       ↓                                                       │
│  ┌────────────────────────────────────┐                      │
│  │     cuPHY Kernel Processing        │                      │
│  ├────────────────────────────────────┤                      │
│  │ PDSCH: CRC → LDPC Encode →         │                      │
│  │        Modulation → Layer Map →    │                      │
│  │        Precoding → RE Map          │                      │
│  ├────────────────────────────────────┤                      │
│  │ PDCCH: CRC → Polar Encode →        │                      │
│  │        QPSK Mod → CCE Map          │                      │
│  ├────────────────────────────────────┤                      │
│  │ PBCH: CRC → Polar → QPSK → RE Map  │                      │
│  └────────────────────────────────────┘                      │
│       ↓                                                       │
│  IFFT (to time domain)                                        │
│       ↓                                                       │
│  DLOutputBuffer                                               │
│       ↓                                                       │
│  Compression (BFP)                                            │
│       ↓                                                       │
│  FH Driver (DPDK)                                             │
│       ↓                                                       │
│  RU/O-RAN Network                                             │
│                                                                │
└──────────────────────────────────────────────────────────────┘
```

### 8.3 Timing Diagram

```
Slot N-1        Slot N          Slot N+1       Slot N+2
───┬─────────────┬───────────────┬──────────────┬────────
   │             │               │              │
   │<-- T_proc -->               │              │
   │             │               │              │
   ├─ RX IQ      │               │              │
   ├─ Order      │               │              │
   ├─ Buffer     │               │              │
   ├─ GPU Launch │               │              │
   │  ├─ PUSCH   │               │              │
   │  ├─ PUCCH   │               │              │
   │  └─ PRACH   │               │              │
   ├─ Results    │               │              │
   └─ L2 Callback│               │              │
                 │               │              │
                 ├─ TX Prep      │              │
                 ├─ GPU Launch   │              │
                 │  ├─ PDSCH     │              │
                 │  └─ PDCCH     │              │
                 ├─ IFFT         │              │
                 └─ TX IQ ────────>              │

T_proc < 500μs (processing budget per slot)
```

---

## 9. State Management

### 9.1 Context State Machine

```
┌──────────────┐
│  INACTIVE    │
└──────┬───────┘
       │ l1_init()
       ↓
┌──────────────┐
│   ACTIVE     │←────────┐
└──────┬───────┘         │
       │                 │ Cell operations
       │                 │ Worker operations
       │                 │ Task processing
       │                 │
       │ l1_finalize()   │
       ↓                 │
┌──────────────┐         │
│  FINALIZING  │─────────┘
└──────┬───────┘
       │
       ↓
┌──────────────┐
│  DESTROYED   │
└──────────────┘
```

### 9.2 Cell State Management

```cpp
enum cell_status {
    CELL_ACTIVE = 0,      // Processing enabled
    CELL_INACTIVE = 1,    // Processing disabled
    CELL_UNHEALTHY = 2    // Error state
};

class Cell {
    std::atomic<cell_status> active{CELL_INACTIVE};
    std::atomic<cell_status> active_srs{CELL_INACTIVE};

    void setActive() { active = CELL_ACTIVE; }
    void setInactive() { active = CELL_INACTIVE; }
    void setUnhealthy() { active = CELL_UNHEALTHY; }

    bool canProcess() {
        return active == CELL_ACTIVE;
    }
};
```

### 9.3 Worker State Management

```cpp
class Worker {
    std::atomic<bool> exit_flag{false};
    std::atomic<bool> running{false};

    enum WorkerState {
        CREATED,
        RUNNING,
        IDLE,
        STOPPING,
        STOPPED
    } state;

    void run() {
        state = RUNNING;
        while (!exit_flag) {
            Task* task = getNextTask();
            if (task) {
                task->execute();
            } else {
                state = IDLE;
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
        state = STOPPED;
    }
};
```

### 9.4 Task State Tracking

```cpp
enum TaskState {
    TASK_CREATED,
    TASK_QUEUED,
    TASK_EXECUTING,
    TASK_COMPLETED,
    TASK_FAILED
};

class Task {
    TaskState state{TASK_CREATED};
    uint64_t creation_time;
    uint64_t execution_start;
    uint64_t execution_end;

    void execute() {
        state = TASK_EXECUTING;
        execution_start = getCurrentTime();

        try {
            doWork();
            state = TASK_COMPLETED;
        } catch (...) {
            state = TASK_FAILED;
        }

        execution_end = getCurrentTime();
    }
};
```

---

## 10. Error Handling and Recovery

### 10.1 Timeout Detection

```cpp
class TimeoutMonitor {
    // Configuration
    uint32_t ul_order_timeout_gpu_ns{1000000};    // 1ms GPU timeout
    uint32_t ul_order_timeout_cpu_ns{10000000};   // 10ms CPU timeout
    uint32_t ul_order_timeout_log_interval_ns{100000000}; // 100ms log interval

    // Tracking
    std::atomic<uint32_t> num_consecutive_ok_timeout{0};
    std::atomic<uint32_t> num_consecutive_unhealthy_slots{0};

    void checkTimeout(uint64_t start_time) {
        uint64_t elapsed = getCurrentTime() - start_time;

        if (elapsed > ul_order_timeout_gpu_ns) {
            LOG(WARNING) << "GPU timeout detected: " << elapsed << "ns";
            num_consecutive_ok_timeout++;

            if (num_consecutive_ok_timeout > THRESHOLD) {
                triggerRecovery();
            }
        } else {
            num_consecutive_ok_timeout = 0;  // Reset on success
        }
    }

    void triggerRecovery() {
        LOG(ERROR) << "Triggering L1 recovery";
        // Set recovery mode
        // Skip next N slots
        // Reset state
    }
};
```

### 10.2 Error Recovery Mechanism

```cpp
struct ErrorRecovery {
    // Recovery state
    bool in_recovery{false};
    uint32_t recovery_slots_remaining{0};
    const uint32_t RECOVERY_SLOT_COUNT = 4;

    void startRecovery() {
        in_recovery = true;
        recovery_slots_remaining = RECOVERY_SLOT_COUNT;

        // Clear pending work
        clearAllQueues();

        // Reset GPU state
        resetGpuState();

        // Reset cell state
        resetCellState();
    }

    void processSlot() {
        if (in_recovery) {
            if (--recovery_slots_remaining == 0) {
                in_recovery = false;
                LOG(INFO) << "Recovery complete";
            } else {
                LOG(INFO) << "Recovery: " << recovery_slots_remaining
                         << " slots remaining";
            }
        }
    }

    bool shouldDropWork() {
        return in_recovery;
    }
};
```

### 10.3 Exception Handling

```cpp
// Macro for exception handling
#define PHYDRIVER_TRY_CATCH(stmt)                          \
    try {                                                   \
        stmt;                                               \
    } catch (const std::exception& e) {                    \
        LOG(ERROR) << "Exception: " << e.what();           \
        return -1;                                          \
    } catch (...) {                                        \
        LOG(ERROR) << "Unknown exception";                 \
        return -1;                                          \
    }

// CUDA error checking
#define CUDA_CHECK_PHYDRIVER(stmt)                         \
    do {                                                    \
        cudaError_t result = (stmt);                       \
        if (result != cudaSuccess) {                       \
            LOG(ERROR) << "CUDA error: "                   \
                      << cudaGetErrorString(result);       \
            throw std::runtime_error("CUDA error");        \
        }                                                   \
    } while(0)
```

### 10.4 Health Monitoring

```cpp
class HealthMonitor {
    struct CellHealth {
        uint32_t processed_slots{0};
        uint32_t failed_slots{0};
        uint32_t timeout_count{0};
        float success_rate{100.0f};

        void updateStats() {
            if (processed_slots > 0) {
                success_rate = 100.0f * (processed_slots - failed_slots)
                             / processed_slots;
            }
        }

        bool isHealthy() {
            return success_rate > 95.0f && timeout_count < 10;
        }
    };

    std::unordered_map<uint16_t, CellHealth> cell_health;

    void checkHealth() {
        for (auto& [cell_id, health] : cell_health) {
            if (!health.isHealthy()) {
                LOG(WARNING) << "Cell " << cell_id
                            << " unhealthy: success_rate="
                            << health.success_rate;
                // Trigger cell-specific recovery
            }
        }
    }
};
```

---

## 11. Performance Optimization

### 11.1 Parallel Processing Strategies

#### Multi-Stream Execution
```cpp
// Parallel kernel execution using streams
void executeParallel() {
    // Launch independent kernels on different streams
    kernel1<<<grid1, block1, 0, stream1>>>();
    kernel2<<<grid2, block2, 0, stream2>>>();
    kernel3<<<grid3, block3, 0, stream3>>>();

    // Synchronize when needed
    cudaStreamSynchronize(stream1);
    cudaStreamSynchronize(stream2);
    cudaStreamSynchronize(stream3);
}
```

#### Split Stream Processing
```cpp
// PUSCH split streams for better parallelism
enum PuschSplitStream {
    PHASE1_SPLIT_STREAM1 = 0,
    PHASE1_SPLIT_STREAM2 = 1,
    PHASE2_SPLIT_STREAM1 = 2,
    PHASE2_SPLIT_STREAM2 = 3
};

void processPuschSplit(PhyPuschAggr* aggr) {
    // Phase 1: Channel estimation (split across 2 streams)
    aggr->chest_kernel<<<grid, block, 0, aggr_stream_pusch[0]>>>();
    aggr->chest_kernel<<<grid, block, 0, aggr_stream_pusch[1]>>>();

    // Phase 2: Decoding (split across 2 streams)
    aggr->decode_kernel<<<grid, block, 0, aggr_stream_pusch[2]>>>();
    aggr->decode_kernel<<<grid, block, 0, aggr_stream_pusch[3]>>>();
}
```

### 11.2 Memory Optimization

#### Pinned Memory for Fast Transfers
```cpp
class PinnedMemoryAllocator {
    void* allocatePinned(size_t size) {
        void* ptr;
        CUDA_CHECK(cudaMallocHost(&ptr, size));
        return ptr;
    }

    void freePinned(void* ptr) {
        CUDA_CHECK(cudaFreeHost(ptr));
    }
};
```

#### Zero-Copy Memory Access
```cpp
// GPU direct access to host memory
void* allocateZeroCopy(size_t size) {
    void* host_ptr;
    void* device_ptr;

    // Allocate pinned host memory
    cudaMallocHost(&host_ptr, size);

    // Get device pointer for direct access
    cudaHostGetDevicePointer(&device_ptr, host_ptr, 0);

    return device_ptr;  // GPU can directly access
}
```

### 11.3 CPU Optimization

#### Thread Affinity
```cpp
void setThreadAffinity(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t thread = pthread_self();
    pthread_setaffinity_np(thread, sizeof(cpuset), &cpuset);
}
```

#### Real-time Scheduling
```cpp
void setRealtimePriority(int priority) {
    struct sched_param param;
    param.sched_priority = priority;

    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
}
```

### 11.4 Batching and Pipelining

#### Batched Memory Operations
```cpp
class BatchedOperations {
    void batchedMemcpy(std::vector<MemcpyOp>& ops) {
        // Group operations by stream
        std::map<cudaStream_t, std::vector<MemcpyOp>> stream_ops;

        for (auto& op : ops) {
            stream_ops[op.stream].push_back(op);
        }

        // Execute batched per stream
        for (auto& [stream, batch] : stream_ops) {
            for (auto& op : batch) {
                cudaMemcpyAsync(op.dst, op.src, op.size,
                               cudaMemcpyHostToDevice, stream);
            }
        }
    }
};
```

#### Pipeline Overlap
```cpp
// Overlap compute and memory transfer
void pipelinedExecution() {
    for (int i = 0; i < num_slots; i++) {
        // Stage 1: Copy input for slot i
        cudaMemcpyAsync(d_input[i], h_input[i], size,
                       cudaMemcpyHostToDevice, stream1);

        // Stage 2: Process slot i-1 (if exists)
        if (i > 0) {
            kernel<<<grid, block, 0, stream2>>>(d_input[i-1]);
        }

        // Stage 3: Copy output for slot i-2 (if exists)
        if (i > 1) {
            cudaMemcpyAsync(h_output[i-2], d_output[i-2], size,
                           cudaMemcpyDeviceToHost, stream3);
        }
    }
}
```

---

## 12. Key Differences: cuPHYDriver vs cuPHYController

| Aspect | **cuPHYDriver** | **cuPHYController** |
|--------|-----------------|---------------------|
| **Primary Role** | Core PHY processing engine | Configuration and initialization wrapper |
| **Scope** | Complete L1 implementation | YAML parsing and setup |
| **Execution** | Runs PHY workloads | Configures and starts driver |
| **GPU Interaction** | Direct CUDA/kernel management | None (delegates to driver) |
| **Memory Management** | Handles all buffers/streams | Configuration only |
| **Task Scheduling** | Implements priority queues | None |
| **Worker Management** | Creates and manages threads | Configuration only |
| **Cell Lifecycle** | Complete implementation | Calls driver APIs |
| **Error Handling** | Timeout detection/recovery | Configuration validation |
| **Performance** | Optimization implementation | Configuration parameters |
| **cuPHY Library** | Direct integration | No interaction |
| **State Machine** | Complex state management | Simple initialization |

### Architectural Relationship

```
┌─────────────────────────────────────────┐
│         cuPHYController                 │
│  (Configuration & Initialization)       │
│                                         │
│  • Parse YAML configuration             │
│  • Validate parameters                  │
│  • Call l1_init()                       │
│  • Call l1_cell_create()                │
└────────────────┬────────────────────────┘
                 │ Uses L1 API
                 ↓
┌─────────────────────────────────────────┐
│           cuPHYDriver                   │
│     (Core Processing Engine)            │
│                                         │
│  • PhyDriverCtx management              │
│  • Cell/Worker/Task implementation      │
│  • GPU resource management              │
│  • cuPHY kernel execution               │
│  • Buffer management                    │
│  • Error detection/recovery             │
│  • Performance optimization             │
└────────────────┬────────────────────────┘
                 │ Invokes
                 ↓
┌─────────────────────────────────────────┐
│         cuPHY CUDA Library              │
│    (Signal Processing Kernels)          │
│                                         │
│  • FFT/IFFT                             │
│  • Channel estimation                   │
│  • LDPC encoding/decoding               │
│  • Modulation/demodulation              │
│  • MIMO processing                      │
└─────────────────────────────────────────┘
```

---

## Summary

The **cuPHYDriver** is the execution heart of the NVIDIA Aerial cuBB stack, responsible for:

1. **Complete L1 PHY Processing**: Manages all physical layer operations through cuPHY kernels
2. **Resource Orchestration**: Coordinates GPU/CPU resources with sophisticated scheduling
3. **Cell Management**: Implements complete cell lifecycle with state management
4. **Task Scheduling**: Priority-based task distribution with timing constraints
5. **Memory Management**: Pre-allocated buffer pools with circular recycling
6. **Error Recovery**: Timeout detection with multi-slot recovery mechanism
7. **Performance Optimization**: Multi-stream parallelism, batching, and pipelining
8. **Fronthaul I/O**: Manages IQ sample transmission/reception through FH proxy

While cuPHYController handles configuration and initialization, cuPHYDriver is the actual workhorse that processes millions of IQ samples per second with sub-millisecond latency, making it the critical component for real-time 5G/6G baseband processing.
